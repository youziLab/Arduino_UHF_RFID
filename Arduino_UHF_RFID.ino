#include "E720_XD.h"
// 存入逻辑
// 屏幕 输入 班级 学号 点击存入 ，打开外舵机，关闭内舵机
// 设置2天线，扫描卡片，检测EPC卡号为0，读取TID，写入TID到EPC，扫描EPC，否则，直接写入班级学号到user
// 1. 用户输入与硬件初始化
// - 屏幕输入：班级（class）、学号（student_id）
// - 点击"存入"按钮后触发以下流程

// 2. 舵机控制（打开外舵机，关闭内舵机）
// - 外舵机打开（允许放入物品）
// - 内舵机关闭（防止误取）

// 3. RFID 操作（使用天线2）
// - 设置天线2扫描模式
// - 扫描卡片：
//   - 如果 EPC == 0（新卡或未初始化）：
//     - 读取 TID
//     - 将 TID 写入 EPC 区
//     - 重新扫描 EPC 确认写入成功
//   - 否则（已有数据）：
//     - 直接将 class + student_id 写入 User 区

// 取出逻辑
// 屏幕 输入 班级 学号 点击存入
// 设置1天线，扫描卡片，遍历读取卡片，找到对应的EPC，否则提示未找到，读取发光分区，并打开内舵机，关闭外舵机
// 设置天线2，扫描卡片，检测信号强度，到达阈值，延迟关闭内舵机，打开外舵机、
// 1. 用户输入与触发
// - 屏幕输入：班级（class）、学号（student_id）
// - 点击"取出"按钮后触发以下流程

// 2. RFID 扫描阶段1（使用天线1，寻找匹配卡片）
// - 设置天线1扫描模式
// - 遍历所有卡片，检查 EPC 是否匹配 class + student_id：
//   - 如果匹配：
//     - 读取发光分区（或其他标识）
//     - 打开内舵机（允许取出）
//     - 关闭外舵机（防止外部干扰）
//   - 如果不匹配：
//     - 提示"未找到该卡片"

// 3. RFID 扫描阶段2（使用天线2，检测物品是否被取出）
// - 设置天线2扫描模式
// - 持续检测信号强度（RSSI）：
//   - 当 RSSI < 阈值（物品被取出）：
//     - 延迟一定时间（确保物品完全取出）
//     - 关闭内舵机
//     - 打开外舵机（恢复初始状态）

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
      [&]()
      { return rfid.readTagMemory(
            0x00000000,   // 访问密码
            rfid.MEM_RFU, // 读取保留区
            0x0004,       // 起始地址
            0x0001        // 读取12个字（96位）
        ); },
      "等待标签发光",
      3, // 重试次数
      50,
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
  // rfid.loop(100);
  if (millis() - lastResetTime > 500) {
    // 读取硬件版本信息
    // rfid.dumpModuleInfo();

    // rfid.setMultiplePollingMode(true, 3); //多次轮询

    // rfid.poll(); // 发送单次轮询标签命令
    lastResetTime = millis();
  }
}