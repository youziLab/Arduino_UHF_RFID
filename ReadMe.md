# Arduino 串口 高频 RFID 通信测试程序

## 适用硬件

- 淘宝商铺：UHF物联科技  
- 商品链接：[点击查看](https://item.taobao.com/item.htm?id=654792682777&spm=tbpc.boughtlist.suborder_itemtitle.1.21272e8dtIV74y)  
- 说明：双天线，支持天线切换的 RFID 读卡模块，有概率兼容其他厂家模块，请谨慎核对通信协议后使用
  
---

## 功能简介


- 支持天线切换、标签轮询、标签读写、掩码过滤等常用 RFID 操作
- 典型应用流程（存入/取出）已在 `RfidTest.ino` 文件注释中详细说明
- 所有支持的命令和参数详见 [`E720_XD.h`](./E720_XD.h) 文件

---

## 快速开始

1. **硬件连接**  
   使用Arduino mega 2560 的 Serial1 串口与 E720 RFID 模块通信，Serial0为串口显示输出
   将 RFID 模块通过串口（如 Serial1）连接到主控板。

2. **编译上传**  
   使用 Arduino IDE 打开本项目，编译并上传 `RfidTest.ino`。

3. **串口监视器**  
   打开串口监视器（115200 波特率），可查看初始化和操作日志。

---

## 主要接口说明

- `E720XD` 类：封装了所有 RFID 通信命令
- 常用方法：
  - `begin()`：初始化串口
  - `setSingleAntenna(ANT)`：设置单天线
  - `inventoryRepeatedlyANT(ANT1, ANT2)`：设置双天线轮询
  - `setSelectParams(...)`：设置标签掩码过滤
  - `readTagMemory(...)` / `writeTagMemory(...)`：读写标签存储区
  - `loop()`：处理接收数据帧

详细参数和用法请查阅 [`E720_XD.h`](./E720_XD.h) 注释。

---

## 注意事项

- **动态长度数据处理**  
  部分动态长度的发送和接收命令（如 96 位/128 位 EPC）目前仅实现了静态长度处理。  
  如需支持动态长度，请参考 `setSelectParams` 函数的掩码处理逻辑进行扩展。

- **BUG 提示**  
  动态长度相关处理可能存在未覆盖的 BUG，使用前请充分测试。

- **其他注意事项**
    本项目仅作测试使用，生产环境请谨慎使用。 
---

## 参考

- [E720 RFID 模块官方手册](https://item.taobao.com/item.htm?id=654792682777)
- 项目代码注释

---

如有问题欢迎反馈或 PR！

