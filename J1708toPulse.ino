#include <SoftwareSerial.h>

// 定义J1708消息ID，用于RPM和速度

#define MID_RoadSpeed 144
#define PID_RoadSpeed 84
#define Length_RoadSpeed 1
#define MAX_RoadSpeed 205

#define MID_EngineSpeed 128
#define PID_EngineSpeed 190
#define Length_EngineSpeed 2
#define MAX_EngineSpeed 16383

// 定义PWM输出引脚
const int rpmPin = 5;    // 用于RPM的PWM输出 (Timer0)
const int speedPin = 9;  // 用于速度的PWM输出 (Timer1)

// 软件串口引脚定义
const int j1708RxPin = 10;  // J1708接收引脚
const int j1708TxPin = 11;  // J1708发送引脚（可选，如果需要发送）

// 创建软件串口对象
SoftwareSerial j1708Serial(j1708RxPin, j1708TxPin);  // 使用引脚10作为接收，11作为发送

unsigned long lastReceiveTime = 0;  // 上次接收数据的时间
const int bufferSize = 20;           // 数据包的最大大小
uint8_t j1708Buffer[bufferSize];     // 用于存储接收到的数据
int bufferIndex = 0;                  // 缓冲区索引

// 变量保存RPM和速度数据
float currentRPM = 0;
float currentSpeed = 0;

void setup() {
  // 初始化硬件串口用于调试输出
  Serial.begin(9600);
  Serial.println("J1708 Data Parsing with Hardware PWM Output Started");

  // 初始化软件串口用于接收J1708数据
  j1708Serial.begin(9600);  // 设置与J1708数据速率匹配

  // 设置PWM引脚为输出
  pinMode(rpmPin, OUTPUT);
  pinMode(speedPin, OUTPUT);

  // 初始化定时器0用于控制RPM (D5)
  setupTimer0();
  
  // 初始化定时器1用于控制速度 (D9)
  setupTimer1();
}

void loop() {
  // 检查软件串口是否有数据可读
  if (j1708Serial.available()) {
    lastReceiveTime = millis();  // 更新最后接收时间

    // 读取数据并存入缓冲区
    while (j1708Serial.available() && bufferIndex < bufferSize) {
      j1708Buffer[bufferIndex++] = j1708Serial.read();
    }
  } else {
    // 检查是否超过2毫秒没有接收到数据
    if (millis() - lastReceiveTime > 5) {
      if (bufferIndex > 0) {
        // 处理完整的数据包
        parseJ1708Packet(j1708Buffer, bufferIndex);
        bufferIndex = 0;  // 重置缓冲区索引
      }
    }
  }
  // 根据RPM和速度设置PWM频率
  setPWMFrequencyForRPM(currentRPM);
  setPWMFrequencyForSpeed(currentSpeed);
}

// 解析J1708数据包
void parseJ1708Packet(uint8_t* buffer, int length) {

  if(0 != calculate_checksum(buffer,length)){
    return;
  }
  uint8_t MID = buffer[0];
  uint8_t PID = buffer[1];

  if(MID == MID_RoadSpeed && PID == PID_RoadSpeed){
    currentSpeed = ((float)buffer[2]*MAX_RoadSpeed)/0xFF;
    Serial.print("currentSpeed = ");
    Serial.println(currentSpeed, DEC);

  } else if(MID == MID_EngineSpeed && PID == PID_EngineSpeed) {
    uint16_t temp = buffer[3];
    temp= (temp<<8) + buffer[2];
    currentRPM = ((float)temp*MAX_EngineSpeed)/0XFFFF;
    Serial.print("currentRPM = ");
    Serial.println(currentRPM, DEC);
  } else {
    return;
  }
}

// 设置定时器0用于控制RPM (D5)
void setupTimer0() {
  // 设置定时器0为8位快速PWM模式 (Mode 3)
  TCCR0A = 0b10100011; // WGM01=1, WGM00=1 (Fast PWM, 8-bit), COM0A1/COM0B1=1 (non-inverting)
  TCCR0B = 0b00000001; // WGM02=0 (Fast PWM), CS00=1 (No prescaling)
  OCR0B = 128;         // 初始占空比为50% (PWM0B -> D5)
}

// 设置定时器1用于控制速度 (D9)
void setupTimer1() {
  // 设置定时器1为8位快速PWM模式 (Mode 5)
  TCCR1A = 0b10100001; // WGM10=1, WGM11=0 (Fast PWM, 8-bit), COM1A1/COM1B1=1 (non-inverting)
  TCCR1B = 0b00001001; // WGM12=1 (Fast PWM), CS10=1 (No prescaling)
  OCR1A = 128;         // 初始占空比为50% (PWM1A -> D9)
}

// 设置与RPM对应的PWM频率 (Timer0, D5)
void setPWMFrequencyForRPM(float rpm) {
  if (rpm > 0) {
    // 根据1 RPM = 4脉冲来设置频率
    float frequency = rpm * 4; // 目标频率
    int prescaler = 1;
    long topValue = (16000000 / (prescaler * frequency)) - 1;
    Serial.print("RPM topValue = ");
    Serial.println(topValue, DEC);

    if (topValue < 256) { // 8位定时器能处理
      OCR0B = topValue; // 设定PWM频率 (D5)
    } else {
      OCR0B = 255; // 防止超出范围
    }
  } else {
    // 停止PWM并将引脚设置为低电平
    OCR0B = 0; // 停止PWM (D5)
    digitalWrite(rpmPin, LOW); // 确保引脚输出为低电平
  }
}

// 设置与速度对应的PWM频率 (Timer1, D9)
void setPWMFrequencyForSpeed(float speed) {
  if (speed > 0) {
    // 根据8000脉冲 = 1 km/h来设置频率
    float frequency = speed * 8000; // 目标频率
    int prescaler = 1;
    long topValue = (16000000 / (prescaler * frequency)) - 1;
    Serial.print("Speed topValue = ");
    Serial.println(topValue, DEC);
    if (topValue < 256) {
      OCR1A = topValue; // 设定PWM频率 (D9)
    } else {
      OCR1A = 255; // 防止超出范围
    }
  } else {
    // 停止PWM并将引脚设置为低电平
    OCR1A = 0; // 停止PWM (D9)
    digitalWrite(speedPin, LOW); // 确保引脚输出为低电平
  }
}


uint8_t calculate_checksum(uint8_t* buffer, int length) {
    uint8_t checksum = 0;
    for(int i = 0; i < length; i++) {
      checksum += buffer[i];
    }
    return checksum;
}
