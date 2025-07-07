#include "E720_XD.h"

// 示例掩码数据（96位，12字节，可根据实际需求修改）
// uint8_t mask[12] = {
//     0xE2, 0x80, 0xF3, 0x37,
//     0x20, 0x00, 0xF0, 0x00,
//     0x10, 0x2F, 0xBF, 0xFE};

uint8_t mask[12] = {
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00
};

E720XD rfid(Serial1, 115200);
String hwVer;
String epc;
// 通用等待命令执行完成的函数（模板实现，兼容lambda）
template<typename CmdFunc>
bool waitCmdFinish(CmdFunc sendCmd, const char *msg = nullptr, uint8_t retry = 3, uint8_t WaitTime = 100, unsigned long timeout = 1000) {
  uint8_t cmd = sendCmd();
  unsigned long CmdLastTime = millis();
  uint8_t retryCount = 0;
  while (rfid.loop(WaitTime) != cmd) {
    if (msg)
      Serial.print(msg);
    if (millis() - CmdLastTime > timeout) {
      cmd = sendCmd();
      CmdLastTime = millis();
      retryCount++;
      if (retryCount >= retry) {
        Serial.println("命令执行超时，重试次数超过限制");
        return false;
      }
    }
    delay(WaitTime);
  }
  Serial.println();
  return true;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  rfid.begin();  // 初始化模块串口

  waitCmdFinish(
    [&]() {
      return rfid.inventoryRepeatedlyANT(1, 1);
    },
    "等待多天线配置 ");

  waitCmdFinish(
    [&]() {
      return rfid.setSingleAntenna(2);
    },
    "等待单天线配置 ");

  waitCmdFinish(
    [&]() {
      return rfid.setTransmitContinuousCarrier(true);
    },
    "等待连续载波设置 ");

  waitCmdFinish(
    [&]() {
      return rfid.setSelectParams(
        0,             // Target: 0
        0,             // Action: 0
        rfid.MEM_EPC,  // MemBank: EPC区
        0x20,          // Ptr: 0x20
        96,            // MaskBitLen: 96位
        false,         // Truncate: 不截断
        mask           // 掩码
      );
    },
    "等待标签选择设置 ");

  waitCmdFinish(
    [&]() {
      return rfid.readTagMemory(
        0x00000000,     // 访问密码
        rfid.MEM_USER,  // 读取用户区
        0x0000,         // 起始地址
        0x0002          // 读取1个字（2字节）
      );
    },
    "读取标签 ",
    10,  // 重试次数
    200,
    1000);
    
  waitCmdFinish(
      [&]()
      { return rfid.readTagMemory(
            0x00000000,   // 访问密码
            rfid.MEM_RFU, // 读取保留区
            0x0004,       // 起始地址
            0x0001        // 读取12个字（96位）
        ); },
      "等待标签发光",
      5, // 重试次数
      200,
      500);

  // waitCmdFinish(
  //   [&]() {
  //     return rfid.readTagMemory(
  //       0x00000000,     // 访问密码
  //       rfid.MEM_USER,  // 读取用户区
  //       0x0000,         // 起始地址
  //       0x0002          // 读取1个字（2字节）
  //     );
  //   },
  //   "读取标签 ",
  //   10,  // 重试次数
  //   200,
  //   1000);

  // // 示例：写入4字节数据到用户区（1个word=2字节，写2个word=4字节）
  // uint8_t userData[4] = { 0x12, 0x34, 0x56, 0x78 };  // 示例数据，可根据实际需求修改
  // waitCmdFinish(
  //   [&]() {
  //     return rfid.writeTagMemory(
  //       0x00000000,     // 访问密码
  //       rfid.MEM_USER,  // 用户区
  //       0x0000,         // 起始地址
  //       0x0002,         // 写入2个word（4字节）
  //       userData        // 数据指针
  //     );
  //   },
  //   "写入标签 ",
  //   10,  // 重试次数
  //   200,
  //   1000);

  // waitCmdFinish(
  //   [&]() {
  //     return rfid.readTagMemory(
  //       0x00000000,     // 访问密码
  //       rfid.MEM_USER,  // 读取用户区
  //       0x0000,         // 起始地址
  //       0x0002          // 读取1个字（2字节）
  //     );
  //   },
  //   "读取标签 ",
  //   10,  // 重试次数
  //   200,
  //   1000);

  Serial.print("初始化完毕");
}

unsigned long lastResetTime = 0;
void loop() {
  rfid.loop(100);
  //读取卡片的数据会被存放在以下结构体变量中
  // rfid.CardInfo; // 当前轮询卡信息
  // rfid.CardRead; // 当前读取的卡信息
  if (millis() - lastResetTime > 500) {
    // 读取硬件版本信息
    // rfid.dumpModuleInfo();

    rfid.setMultiplePollingMode(true, 2); //多次轮询

    // rfid.poll(); // 发送单次轮询标签命令
    lastResetTime = millis();
  }
}