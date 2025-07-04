#ifndef E720XD_H
#define E720XD_H

#include <stdint.h>
#include <Arduino.h>

#define DEBUG
#define RX_BUFFER_LENGTH 128
class E720XD{
private:
  HardwareSerial *_serial;
  uint32_t _baudrate; // 串口波特率

  uint8_t _buffer[RX_BUFFER_LENGTH] = {0};

  // 计算校验和
  uint8_t calculateCheckSum(uint8_t *buffer);

  // 将数组转换为uint16_t
  uint16_t arrayToUint16(uint8_t *array);

  // 解析接收到的数据
  bool parseReceivedData();

  // 判断数据是否有效
  bool dataIsValid();

  // 接收数据，带超时参数
  bool receiveData(unsigned long timeOut = 300);

  // 将接收缓冲区内容输出到串口
  void dumpReceiveBufferToSerial();

  // 清空缓冲区
  uint8_t flush();

  // 判断是否有数据可用
  bool dataAvailable();

  // 计算校验和（从指定起始位置到结束位置的累加和）
  uint8_t calculateChecksum(const uint8_t* data, size_t start, size_t end);

  // 发送到读写器的命令和接收到的响应都以数据帧形式传输，例如：
  // 帧头 | 类型 | 命令 | 参数长度(2字节) | 参数 | 校验和 | 帧尾
  //   AA |  00  |  07  |    00 03       | 04 02 05 | 15 | DD
  //
  // 帧总是以0xAA开头
  // Type表示命令(0x00)、响应(0x01)或通知(0x02)
  // Command为指令或响应
  // ParamLength为2字节（高字节在前）参数长度
  // Params为零个或多个参数
  // Checksum为从Type到最后一个参数字节的和的低字节（不包括帧头）
  // 帧总是以0xDD结尾

  // 帧结构中各元素的位置（相对于帧头的偏移）
  enum E720XD_FrameStructure : byte
  {
    E720XD_HeaderPos = 0x00,
    E720XD_TypePos = 0x01,
    E720XD_CommandPos = 0x02,
    E720XD_ParamLengthMSBPos = 0x03,
    E720XD_ParamLengthLSBPos = 0x04,
    E720XD_ParamPos = 0x05,
    // 其他响应元素（参数、校验和、帧尾）的偏移是可变的
    // E720XD_ParamPos = if(E720XD_ParamLengthMSBPos << 8 + E720XD_ParamLengthLSBPos) > 0) { 0x05 } else { null }
    // E720XD_ChecksumPos = 0x05 + (E720XD_ParamLengthMSBPos << 8 + E720XD_ParamLengthLSBPos)
    // E720XD_EndPos = 0x06 + (E720XD_ParamLengthMSBPos << 8 + E720XD_ParamLengthLSBPos)
  };

  // 帧控制字节
  enum E720XD_FrameControl : byte
  {
    E720XD_FrameHeader = 0xBB, // 帧头Header
    E720XD_FrameEnd = 0x7E,    // 帧尾End
  };

  // 帧类型
  enum E720XD_FrameType : byte
  {
    FrameType_Command = 0x00,      // 命令帧 由上位机发送给E720 芯片
    FrameType_Response = 0x01,     // 响应帧 由E720 芯片发回给上位机
    FrameType_Notification = 0x02, // 通知帧 由E720 芯片发回给上位机
  };



public:
  // 命令字定义
  // 共35个
  enum E720XD_Command : byte
  {
    CMD_GetModuleInfo = 0x03,                    // 获取模块信息
    CMD_SinglePollInstruction = 0x22,            // 单次轮询指令
    CMD_MultiplePollInstruction = 0x27,          // 多次轮询指令
    CMD_StopMultiplePoll = 0x28,                 // 停止多次轮询
    CMD_SetSelectParameter = 0x0C,               // 设置选择参数 设置Select 参数指令
    CMD_GetSelectParameter = 0x0B,               // 获取选择参数 获取Select 参数指令
    CMD_SetSendSelectInstruction = 0x12,         // 设置并发送选择指令 设置发送Select 指令
    CMD_ReadLabel = 0x39,                        // 读标签 读标签数据存储区
    CMD_WriteLabel = 0x49,                       // 写标签 写标签数据存储区
    CMD_LockLabel = 0x82,                        // 锁定标签
    CMD_KillTag = 0x65,                          // 销毁标签
    CMD_GetQueryParameters = 0x0D,               // 获取查询参数 获取Query 参数
    CMD_SetQueryParameters = 0x0E,               // 设置查询参数 设置Query 参数
    CMD_SetWorkArea = 0x07,                      // 设置工作区域 设置工作地区
    CMD_SetWorkingChannel = 0xAB,                // 设置工作信道 设置工作信道
    CMD_GetWorkingChannel = 0xAA,                // 获取工作信道 获取工作信道
    CMD_SetAutoFrequencyHopping = 0xAD,          // 设置自动跳频 设置自动跳频
    CMD_AcquireTransmitPower = 0xB7,             // 获取发射功率 获取发射功率
    CMD_SetTransmitPower = 0xB6,                 // 设置发射功率 设置发射功率
    CMD_SetTransmitContinuousCarrier = 0xB0,     // 设置连续载波 设置发射连续载波
    CMD_GetReceiverDemodulatorParameters = 0xF1, // 获取接收解调参数 获取接收解调器参数
    CMD_SetReceiverDemodulatorParameters = 0xF0, // 设置接收解调参数 设置接收解调器参数
    CMD_TestRFInputBlockingSignal = 0xF2,        // 测试RF输入阻塞信号 测试射频输入端阻塞信号
    CMD_TestChannelRSSI = 0xF3,                  // 测试信道RSSI 测试信道RSSI
    CMD_ControlIOPort = 0x1A,                    // 控制IO口 控制IO 端口
    CMD_ModuleSleep = 0x17,                      // 模块休眠
    CMD_SetModuleIdleSleepTime = 0x1D,           // 设置模块空闲休眠时间 设置模块空闲休眠时间
    CMD_ExecutionFailure = 0xFF,                 // 执行失败

    CMD_AntSettingUp = 0x1B, // 设置多天线轮模式和次数

  };

  // 错误码定义
  enum E720XD_ErrorCode : byte
  {
    ERR_CommandError = 0x17,  // 命令错误，命令帧中指令代码错误
    ERR_FHSSFail = 0x20,      // 跳频失败，跳频搜索信道超时，所有信道在这段时间内都被占用
    ERR_InventoryFail = 0x15, // 盘点失败，轮询操作失败，没有标签返回或返回数据CRC校验错误
    ERR_AccessFail = 0x16,    // 访问失败，访问标签失败，可能是访问密码不对
    ERR_ReadFail = 0x09,      // 读失败，读标签数据存储区失败，标签没有返回或返回数据CRC校验错误
    // ERR_ReadError         = 0xA0, // 0xA0 | Error code，读标签数据存储区错误，返回的代码由0xA0位或Error Code得到
    ERR_WriteFail = 0x10, // 写失败，写标签数据存储区失败，标签没有返回或返回数据CRC校验错误
    // ERR_WriteError        = 0xB0, // 0xB0 | Error code，写标签数据存储区错误，返回的代码由0xB0位或Error Code得到
    ERR_LockFail = 0x13, // 锁定失败，锁定标签数据存储区失败，标签没有返回或返回数据CRC校验错误
    // ERR_LockError         = 0xC0, // 0xC0 | Error code，锁定标签数据存储区错误，返回的代码由0xC0位或Error Code得到
    ERR_KillFail = 0x12,           // 销毁失败
    ERR_KillError = 0xD0,          // 0xD0 | Error code，销毁标签数据存储区错误，返回的代码由0xD0位或Error Code得到
    ERR_BlockPermalockFail = 0x14, // BlockPermalock执行失败，标签没有返回或返回数据CRC校验错误
    // ERR_BlockPermalockError = 0xE0, // 0xE0 | Error code，BlockPermalock执行错误，返回的代码由0xE0位或Error Code得到
  };

  //读取分区枚举
  enum E720XD_MemoryBank : byte
  {
    MEM_RFU = 0x00, // RFU 保留
    MEM_EPC = 0x01,
    MEM_TID = 0x02,
    MEM_USER = 0x03
  };

  //卡信息
  struct CardInfo_t {
    uint8_t EPC[12] = {0};    //卡号
    int8_t RSSI;  //信号强度
    uint16_t PC;   //标签自带
    uint8_t ANT;   //读取的天线
  };
    struct CardRead_t {
    uint8_t UL;// PC+EPC 长度
    uint16_t PC;   //标签自带
    uint8_t EPC[12] = {0};    //卡号
    uint8_t Data[96] = {0};   //读取的数据
    uint16_t DataLen;          //读取的数据长度
  };
  CardInfo_t CardInfo; // 当前卡信息
  CardRead_t CardRead; // 当前读取的卡信息

  // 构造函数，传入串口对象与波特率
  E720XD(HardwareSerial &serial, uint32_t baudrate = 115200);
  // 初始化串口通信
  void begin();

  // 主循环处理函数
  uint8_t loop(unsigned long timeOut);

  // 发送获取模块信息到串口 命令
  uint8_t dumpModuleInfo();

  // 设置单天线工作 ANT 工作天线01 或 02
  uint8_t setSingleAntenna(uint8_t ANT);
  // 设置多天线工作，设置指定天线次数
  uint8_t inventoryRepeatedlyANT(uint8_t ANT1,uint8_t ANT2);
  //设置发射连续载波,true为开启，false为关闭
  uint8_t setTransmitContinuousCarrier(bool enable);

  // 发送单次轮询标签 命令
  uint8_t poll();
  // 发送多标签轮询模式，true为开启，false为关闭
  uint8_t setMultiplePollingMode(bool enable=true,uint16_t number = 1);

  //设置Select参数
  //   帧类型Type: 0x00
  // 指令代码Command: 0x0C
  // 指令参数长度PL 0x0013
  // SelParam: 0x01 (Target: 3’b000, Action: 3’b000, MemBank: 2’b01)
  // Ptr: 0x00000020(以bit 为单位，非word) 从EPC 存储位开始
  // Mask 长度MaskLen: 0x60(6 个word，96bits)
  // 是否Truncate: 0x00(0x00 是Disable truncation，0x80 是Enable truncation)
  // Mask: 0x30751FEB705C5904E3D50D70
  // 校验位Checksum: 0xAD
  // SelParam 共1 个Byte，其中Target 占最高3 个bits，Action 占中间3 个bits，MemBank 占最后2 个bits。
  // MemBank 含义如下：
  // 2’b00: 标签RFU 数据存储区
  // 2’b01: 标签EPC 数据存储区
  // 2’b10: 标签TID 数据存储区
  // 2’b11: 标签User 数据存储区
  // Target 和Action 详细含义请参见EPC Gen2 协议。
  // 当Select Mask 长度大于80 bits(5 words)，发送Select 指令会先把场区内所有标签设置成Inventoried Flag
  // 为A，SL Flag 为~SL 的状态，然后再根据所选的Action 进行操作。当Select Mask 长度小于80 bits(5 words)
  // 的时候，不会预先将标签状态通过Select 指令设置成Inventoried Flag 为A，SL Flag 为~SL 的状态。
    uint8_t  setSelectParams(
    uint8_t target,                  // Target: 3 bits
    uint8_t action,                  // Action: 3 bits
    enum E720XD_MemoryBank memBank,  // memBank: 2 bits
    uint32_t ptr,                    // ptr: 指定的起始地址，(以bit 为单位，非word) 从EPC 存储位开始
    uint8_t maskBitLen,              // maskBitLen: 掩码的位长度
    bool truncate,                   // truncate: 是否截断选择参数
    const uint8_t *mask              // mask: 选择的掩码
  );

  // 读取标签数据存储区
  uint8_t readTagMemory(
    uint32_t accessPwd,  // 访问密码
    enum E720XD_MemoryBank memBank, // 读取的存储区
    uint16_t startAddr,  // 起始地址
    uint16_t wordLen     // 读取的字长度
  );

  //写入标签数据存储区
  uint8_t writeTagMemory(
    uint32_t accessPwd,  // 访问密码
    enum E720XD_MemoryBank memBank, // 写入的存储区
    uint16_t startAddr,  // 起始地址
    uint16_t wordLen,     // 写入的字长度
    uint8_t *data        // 写入的数据
  );

  // // 检查是否有新卡出现
  // bool newCardPresent();
  // // 检查是否有卡存在
  // bool isCardPresent();

  // 输出UID到串口
  // void dumpUIDToSerial();
};

#endif