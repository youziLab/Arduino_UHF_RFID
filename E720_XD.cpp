#include "E720_XD.h"

// 构造函数实现
E720XD::E720XD(HardwareSerial &serial, uint32_t baudrate)
    : _serial(&serial), _baudrate(baudrate)
{
}

void E720XD::begin()
{
  _serial->begin(_baudrate);
}

// 计算校验和（从指定起始位置到结束位置的累加和）
uint8_t E720XD::calculateChecksum(const uint8_t *data, size_t start,
                                  size_t end)
{
  uint8_t checksum = 0;
  for (size_t i = start; i <= end; i++)
  {
    checksum += data[i];
  }
  return checksum;
}

/* 命令发送 */

/**
 * 发送获取设备号命令
 */
uint8_t E720XD::dumpModuleInfo()
{
  uint8_t commandFrame[8] = {0};
  commandFrame[0] = E720XD_FrameHeader;
  commandFrame[1] = FrameType_Command;
  commandFrame[2] = CMD_GetModuleInfo;
  commandFrame[3] = 0x00; // 参数长度MSB
  commandFrame[4] = 0x01; // 参数长度LSB
  commandFrame[5] = 0x00; // 参数
  commandFrame[6] = 0x04; // 校验和
  commandFrame[7] = E720XD_FrameEnd;
  _serial->write(commandFrame, 8);
  return CMD_GetModuleInfo;
}

/**
 * 发送单次轮询命令
 */
uint8_t E720XD::poll()
{
  uint8_t commandFrame[7] = {0};
  commandFrame[0] = E720XD_FrameHeader;
  commandFrame[1] = FrameType_Command;
  commandFrame[2] = CMD_SinglePollInstruction;
  commandFrame[3] = 0x00; // 参数长度MSB
  commandFrame[4] = 0x00; // 参数长度LSB
  commandFrame[5] = 0x22; // 校验和
  commandFrame[6] = E720XD_FrameEnd;
  _serial->write(commandFrame, 7);
  return CMD_SinglePollInstruction;
}

// 设置发射连续载波
uint8_t E720XD::setTransmitContinuousCarrier(bool enable)
{
  uint8_t commandFrame[8] = {0};
  commandFrame[0] = E720XD_FrameHeader;
  commandFrame[1] = FrameType_Command;
  commandFrame[2] = CMD_SetTransmitContinuousCarrier;
  commandFrame[3] = 0x00;                                  // 参数长度MSB
  commandFrame[4] = 0x01;                                  // 参数长度LSB
  commandFrame[5] = enable ? 0xFF : 0x00;                  // 参数
  commandFrame[6] = calculateChecksum(commandFrame, 1, 5); // 校验和
  commandFrame[7] = E720XD_FrameEnd;
  _serial->write(commandFrame, 8);
  return CMD_SetTransmitContinuousCarrier;
}

// 设置多次轮询模式
uint8_t E720XD::setMultiplePollingMode(bool enable, uint16_t number)
{
  if (enable)
  {
    uint8_t commandFrame[10] = {0};
    commandFrame[0] = E720XD_FrameHeader;
    commandFrame[1] = FrameType_Command;                     //(0x00)
    commandFrame[2] = CMD_MultiplePollInstruction;           // 0x27
    commandFrame[3] = 0x00;                                  // 参数长度MSB
    commandFrame[4] = 0x03;                                  // 参数长度LSB
    commandFrame[5] = 0x22;                                  // 参数（保留？此命令始终为0x22）
    commandFrame[6] = (number >> 8) & 0xFF;                  // 轮询次数MSB
    commandFrame[7] = number & 0xFF;                         // 轮询次数LSB
    commandFrame[8] = calculateChecksum(commandFrame, 1, 7); // 校验和
    commandFrame[9] = E720XD_FrameEnd;
    _serial->write(commandFrame, 10);
    return CMD_MultiplePollInstruction;
  }
  else
  {
    uint8_t commandFrame[7] = {0};
    commandFrame[0] = E720XD_FrameHeader;
    commandFrame[1] = FrameType_Command;    //(0x00)
    commandFrame[2] = CMD_StopMultiplePoll; // 0x28
    commandFrame[3] = 0x00;                 // 参数长度MSB
    commandFrame[4] = 0x00;                 // 参数长度LSB
    commandFrame[5] = 0x28;                 // 校验和
    commandFrame[6] = E720XD_FrameEnd;
    _serial->write(commandFrame, 7);
    return CMD_StopMultiplePoll;
  }
}

// 设置单根天线
uint8_t E720XD::setSingleAntenna(uint8_t ANT)
{
  uint8_t commandFrame[9] = {0};
  commandFrame[0] = E720XD_FrameHeader;
  commandFrame[1] = FrameType_Command;
  commandFrame[2] = CMD_AntSettingUp;                      // 指令码
  commandFrame[3] = 0x00;                                  // 参数长度MSB
  commandFrame[4] = 0x02;                                  // 参数长度LSB
  commandFrame[5] = 0x01;                                  // 参数数量
  commandFrame[6] = ANT;                                   // 天线编号
  commandFrame[7] = calculateChecksum(commandFrame, 1, 6); // 校验和
  commandFrame[8] = E720XD_FrameEnd;
  _serial->write(commandFrame, 9);
  return CMD_AntSettingUp;
}

// 设置双天线轮询
uint8_t E720XD::inventoryRepeatedlyANT(uint8_t ANT1, uint8_t ANT2)
{
  uint8_t commandFrame[10] = {0};
  commandFrame[0] = E720XD_FrameHeader;
  commandFrame[1] = FrameType_Command;
  commandFrame[2] = CMD_AntSettingUp;                      // 指令码
  commandFrame[3] = 0x00;                                  // 参数长度MSB
  commandFrame[4] = 0x03;                                  // 参数长度LSB
  commandFrame[5] = 0x02;                                  // 参数数量
  commandFrame[6] = ANT1;                                  // 天线1编号
  commandFrame[7] = ANT2;                                  // 天线2编号
  commandFrame[8] = calculateChecksum(commandFrame, 1, 7); // 校验和
  commandFrame[9] = E720XD_FrameEnd;
  _serial->write(commandFrame, 10);
  return CMD_AntSettingUp;
}

// 设置Select参数
uint8_t E720XD::setSelectParams(
    uint8_t target,                 // Target: 3 bits
    uint8_t action,                 // Action: 3 bits
    enum E720XD_MemoryBank memBank, // memBank: 2 bits
    uint32_t ptr,                   // ptr: 指定的起始地址，(以bit 为单位，非word) 从EPC 存储位开始
    uint8_t maskBitLen,             // maskBitLen: 掩码的位长度
    bool truncate,                  // truncate: 是否截断选择参数
    const uint8_t *mask             // mask: 选择的掩码
)
{
  // 组装SelParam: Target(7:5), Action(4:2), MemBank(1:0)
  uint8_t selParam = ((target & 0x07) << 5) | ((action & 0x07) << 2) | (memBank & 0x03);

  // 计算掩码字节长度（maskBitLen为bit，向上取整到byte）
  uint8_t maskByteLen = (maskBitLen + 7) / 8;
  // 参数长度 = SelParam(1) + Ptr(4) + MaskLen(1) + Truncate(1) + Mask(n)
  uint16_t paramLen = 1 + 4 + 1 + 1 + maskByteLen;

  // 帧总长度 = 帧头(1) + 类型(1) + 命令(1) + 参数长度(2) + 参数(paramLen) + 校验(1) + 帧尾(1)
  uint8_t frameLen = 1 + 1 + 1 + 2 + paramLen + 1 + 1;
  uint8_t *commandFrame = new uint8_t[frameLen];
  uint8_t idx = 0;
  commandFrame[idx++] = E720XD_FrameHeader;     // 帧头
  commandFrame[idx++] = FrameType_Command;      // 类型
  commandFrame[idx++] = CMD_SetSelectParameter; // 指令码(0x0C)
  commandFrame[idx++] = (paramLen >> 8) & 0xFF; // 参数长度MSB
  commandFrame[idx++] = paramLen & 0xFF;        // 参数长度LSB
  commandFrame[idx++] = selParam;               // SelParam
  commandFrame[idx++] = (ptr >> 24) & 0xFF;     // Ptr MSB
  commandFrame[idx++] = (ptr >> 16) & 0xFF;
  commandFrame[idx++] = (ptr >> 8) & 0xFF;
  commandFrame[idx++] = ptr & 0xFF;             // Ptr LSB
  commandFrame[idx++] = maskBitLen;             // MaskLen (bit)
  commandFrame[idx++] = truncate ? 0x80 : 0x00; // Truncate
  // 拷贝掩码
  for (uint8_t i = 0; i < maskByteLen; ++i)
  {
    commandFrame[idx++] = mask ? mask[i] : 0x00;
  }
  // 校验和
  commandFrame[idx] = calculateChecksum(commandFrame, 1, frameLen - 3); // 校验和(不含帧头和帧尾)
  idx++;
  commandFrame[idx] = E720XD_FrameEnd; // 帧尾

  _serial->write(commandFrame, frameLen);
  delete[] commandFrame;
  return CMD_SetSelectParameter;
}

uint8_t E720XD::readTagMemory(
    uint32_t accessPwd,             // 访问密码
    enum E720XD_MemoryBank memBank, // 读取的存储区
    uint16_t startAddr,             // 起始地址
    uint16_t wordLen                // 读取的字长度
)
{
  // 参数长度 = AccessPwd(4) + MemBank(1) + StartAddr(2) + WordLen(2) = 9
  uint16_t paramLen = 9;
  uint8_t commandFrame[16] = {0}; // 16字节

  commandFrame[0] = E720XD_FrameHeader;     // 帧头
  commandFrame[1] = FrameType_Command;      // 类型 0x00
  commandFrame[2] = CMD_ReadLabel;          // 指令码 Command
  commandFrame[3] = (paramLen >> 8) & 0xFF; // 参数长度MSB
  commandFrame[4] = paramLen & 0xFF;        // 参数长度LSB

  // Access Password 4字节，高字节在前
  commandFrame[5] = (accessPwd >> 24) & 0xFF;
  commandFrame[6] = (accessPwd >> 16) & 0xFF;
  commandFrame[7] = (accessPwd >> 8) & 0xFF;
  commandFrame[8] = accessPwd & 0xFF;

  // MemBank 1字节
  commandFrame[9] = memBank & 0xFF;

  // StartAddr 2字节，高字节在前
  commandFrame[10] = (startAddr >> 8) & 0xFF;
  commandFrame[11] = startAddr & 0xFF;

  // WordLen 2字节，高字节在前
  commandFrame[12] = (wordLen >> 8) & 0xFF;
  commandFrame[13] = wordLen & 0xFF;

  // 校验和（从Type到最后一个参数）
  commandFrame[14] = calculateChecksum(commandFrame, 1, 14);
  commandFrame[15] = E720XD_FrameEnd; // 帧尾

  _serial->write(commandFrame, 16);
  return CMD_ReadLabel;
}

// 写入标签数据存储区
uint8_t E720XD::writeTagMemory(
    uint32_t accessPwd,  // 访问密码
    enum E720XD_MemoryBank memBank, // 写入的存储区
    uint16_t startAddr,  // 起始地址
    uint16_t wordLen,    // 写入的字长度
    uint8_t *data        // 写入的数据
)
{
  // 参数长度 = AccessPwd(4) + MemBank(1) + StartAddr(2) + WordLen(2) + DataLen = 9 + wordLen*2
  uint16_t paramLen = 9 + wordLen * 2;
  uint16_t frameLen = 1 + 1 + 1 + 2 + paramLen + 1 + 1; // 帧头+类型+命令+参数长度+参数+校验和+帧尾

  uint8_t *commandFrame = new uint8_t[frameLen];
  uint16_t idx = 0;
  commandFrame[idx++] = E720XD_FrameHeader;     // 帧头
  commandFrame[idx++] = FrameType_Command;      // 类型 0x00
  commandFrame[idx++] = CMD_WriteLabel;         // 指令码 Command
  commandFrame[idx++] = (paramLen >> 8) & 0xFF; // 参数长度MSB
  commandFrame[idx++] = paramLen & 0xFF;        // 参数长度LSB

  // Access Password 4字节，高字节在前
  commandFrame[idx++] = (accessPwd >> 24) & 0xFF;
  commandFrame[idx++] = (accessPwd >> 16) & 0xFF;
  commandFrame[idx++] = (accessPwd >> 8) & 0xFF;
  commandFrame[idx++] = accessPwd & 0xFF;

  // MemBank 1字节
  commandFrame[idx++] = memBank & 0xFF;

  // StartAddr 2字节，高字节在前
  commandFrame[idx++] = (startAddr >> 8) & 0xFF;
  commandFrame[idx++] = startAddr & 0xFF;

  // WordLen 2字节，高字节在前
  commandFrame[idx++] = (wordLen >> 8) & 0xFF;
  commandFrame[idx++] = wordLen & 0xFF;

  // Data 区，长度为 wordLen*2
  for (uint16_t i = 0; i < wordLen * 2; ++i) {
    commandFrame[idx++] = data[i];
  }

  // 校验和（从Type到最后一个数据字节）
  commandFrame[idx] = calculateChecksum(commandFrame, 1, frameLen - 3);
  idx++;
  commandFrame[idx] = E720XD_FrameEnd; // 帧尾

  _serial->write(commandFrame, frameLen);
  delete[] commandFrame;
  return CMD_WriteLabel;
}

/*命令接收处理*/

// 计算校验和
uint8_t E720XD::calculateCheckSum(uint8_t *buffer)
{
  // 提取参数长度
  uint16_t paramLength = buffer[3];
  paramLength <<= 8;
  paramLength += buffer[4];

  // 校验和为所有参数字节与前4个控制字节之和（从1开始，不包括帧头）
  uint16_t check = 0;
  for (uint16_t i = 1; i < paramLength + 4 + 1; i++)
  {
    check += buffer[i];
  }
  // 只返回低8位
  return (check & 0xff);

  /*
  // 另一种常见校验和算法
  uint16_t paramLength = *(buffer+3);
  paramLength <<=8;
  paramLength += *(buffer+4);

  uint16_t sum = 0;
  for (int i=1; i<4+paramLength; i++) {
    sum += buffer[i];
  }
  return -sum;
  */
}

// 判断接收到的数据是否有效
bool E720XD::dataIsValid()
{
  // Serial.println("检查数据有效性");
  // dumpReceiveBufferToSerial();
  uint8_t CRC = calculateCheckSum(_buffer);

  // 注意
  // 不能一行写完，否则指针引用会出错
  // uint16_t paramLength = _buffer[3]<<8 + _buffer[4];
  uint16_t paramLength = _buffer[3];
  paramLength <<= 8;
  paramLength += _buffer[4];
  uint8_t CRCpos = 5 + paramLength;

  // Serial.print(CRC, HEX);
  // Serial.print(":");
  // Serial.println(_buffer[CRCpos], HEX);
  return (CRC == _buffer[CRCpos]);
}

// 判断是否有数据可读
bool E720XD::dataAvailable()
{
  // Serial.println("检查数据可用性");
  return _serial->available() > 0;
}

/*
 * 注意：Arduino的Serial.flush()方法只清除输出缓冲区，不会清除输入缓冲区！
 */
// 清空串口输入缓冲区
uint8_t E720XD::flush()
{
  uint8_t bytesDiscarded = 0;
  while (_serial->available())
  {
    _serial->read();
    bytesDiscarded++;
  }
  return bytesDiscarded;
}

// 读取串口接收的数据帧
// 可能是命令响应，也可能是通知（如自动轮询模式下）
// 在超时时间内读取到完整帧返回true
bool E720XD::receiveData(unsigned long timeOut)
{
  // Serial.println("接收数据");
  unsigned long startTime = millis();
  uint8_t bytesReceived = 0;
  // 清空缓冲区
  // memset(_buffer, 0, sizeof _buffer);
  for (int i = 0; i < RX_BUFFER_LENGTH; i++)
  {
    _buffer[i] = 0;
  }
  while ((millis() - startTime) < timeOut)
  {
    while (_serial->available())
    {
      uint8_t b = _serial->read();
      if (bytesReceived > RX_BUFFER_LENGTH - 1)
      {
        Serial.print("错误: 超出最大缓冲区长度!");
        flush();
        return false;
      }
      else
      {
        _buffer[bytesReceived] = b;
      }
      bytesReceived++;
      if (b == E720XD_FrameEnd)
      {
        break;
      }
    }
  }
  if (bytesReceived > 1 && _buffer[0] == E720XD_FrameHeader && _buffer[bytesReceived - 1] == E720XD_FrameEnd)
  {
    // flush();
    return true;
  }
  else
  {
    // flush();
    return false;
  }
  // flush();
  return false;
}

// 主循环处理函数，处理接收到的数据帧
uint8_t E720XD::loop(unsigned long timeOut)
{
  // 是否有新数据接收？
  if (dataAvailable())
  {
    // 尝试接收一帧完整数据
    if (receiveData(timeOut))
    {
      if (dataIsValid())
      {
        // 如果接收到完整有效数据帧，进行解析
        switch (_buffer[E720XD_CommandPos])
        {
        case CMD_GetModuleInfo: // 获取模块信息
#ifdef DEBUG
          Serial.println("获取模块信息");
#endif
          for (uint8_t i = 0; i < RX_BUFFER_LENGTH - 8; i++)
          {
            Serial.print((char)_buffer[6 + i]);
            // 当只剩下CRC和帧尾时停止
            if (_buffer[8 + i] == E720XD_FrameEnd)
            {
              break;
            }
          }

          break;

        case CMD_AntSettingUp: // 天线设置
#ifdef DEBUG
          Serial.println("天线设置");
#endif
          break;
        case CMD_SinglePollInstruction: // 单次轮询指令

          // 示例成功响应
          // AA 02 22 00 11 C7 30 00 E2 80 68 90 00 00 50 0E 88 C6 A4 A7 11 9B 29 DD
          // AA:帧头
          // 02:指令码
          // 22:命令参数
          // 00 11:指令数据长度 (0x11 = 17字节)
          // C7：RSSI信号强度
          // 30 00: 标签PC码（工厂寄存器码）
          // E2 80 68 90 00 00 50 0E 88 C6 A4 A7：EPC码
          // 11 9B:CRC校验
          // 29: 校验
          // DD:帧尾

          memset(&CardInfo, 0, sizeof(CardInfo)); // 先将结构体全部清零

          // 直接使用memcpy将EPC码复制到结构体中
          memcpy(CardInfo.EPC, &_buffer[8], 12);

          // 解析RSSI
          // CardInfo.RSSI = _buffer[5];
          CardInfo.RSSI = (int8_t)_buffer[5];
          // 解析PC码
          CardInfo.PC = (_buffer[6] << 8) | _buffer[7];
          // 解析天线编号
          CardInfo.ANT = _buffer[25 - 3];

#ifdef DEBUG
          Serial.print("单次轮询指令");
          Serial.println("检测到卡片:");
          Serial.print("EPC: ");
          for (int i = 0; i < 12; i++)
          {
            Serial.print(CardInfo.EPC[i], HEX);
            Serial.print(" ");
          }
          Serial.println();
          Serial.print("RSSI: ");
          Serial.print(_buffer[5], HEX);
          Serial.println(CardInfo.RSSI);
          Serial.print("PC: ");
          Serial.println(CardInfo.PC, HEX);
          Serial.print("ANT: ");
          Serial.println(CardInfo.ANT);
#endif
          break;
        case CMD_ReadLabel:                                      // 读取标签指令
          memset(&CardRead, 0, sizeof(CardRead));                // 先将结构体全部清零
          CardRead.UL = _buffer[5];                              // 写入PC+EPC长度
          uint16_t paramLength = (_buffer[3] << 8) | _buffer[4]; // 参数长度
          CardRead.DataLen = paramLength - CardRead.UL - 1;   // 数据长度

          CardRead.PC = (_buffer[6] << 8) | _buffer[7];                     // 读取PC码
          memcpy(CardRead.EPC, &_buffer[8], CardRead.UL - 2);               // 直接使用memcpy将EPC码复制到结构体中
          memcpy(CardRead.Data, &_buffer[5 + CardRead.UL + 1], CardRead.DataLen); // 读取数据

#ifdef DEBUG
          Serial.println("读取标签存储指令");
          Serial.print("PC: ");
          Serial.println(CardRead.PC, HEX); // 打印PC码
          Serial.print("EPC: ");
          for (int i = 0; i < CardRead.UL - 2;
               i++)
          {
            Serial.print(CardRead.EPC[i], HEX);
            Serial.print(" ");
          }
          Serial.println();
          Serial.print("Data: ");
          for (int i = 0; i < CardRead.DataLen; i++)
          {
            Serial.print(CardRead.Data[i], HEX);
            Serial.print(" ");
          }
          Serial.println();
          Serial.print("UL: ");
          Serial.println(CardRead.UL);

#endif
          break;
        case CMD_ExecutionFailure: // 执行失败
          switch (_buffer[E720XD_ParamPos])
          {
          case ERR_CommandError:
            Serial.println("命令错误");
            break;
          case ERR_InventoryFail:
            // 这不一定是“失败”——只是没有卡片在范围内
            Serial.println("未检测到卡片!");
            //             // 如果之前有uid
            //             // if (memcmp(uid, blankUid, sizeof uid) != 0)
            //             {
            // #ifdef DEBUG
            //               Serial.print("卡片移除 : ");
            //               //  dumpUIDToSerial();
            //               Serial.println("");
            // #endif
            //               //  memset(uid, 0, sizeof uid);
            //             }

            break;
          case ERR_AccessFail:
            Serial.println("访问失败");
            break;
          case ERR_ReadFail:
            Serial.println("读取失败");
            break;
          case ERR_WriteFail:
            Serial.println("写入失败");
            break;
          default:
            Serial.print("失败码 ");
            Serial.println(_buffer[E720XD_ParamPos], HEX);
            break;
          }
          break;
        }

        return _buffer[E720XD_CommandPos]; // 返回命令类型
      }
    }
  }
}