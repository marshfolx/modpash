# Modpash

Modpash 是一个用于 Modbus 协议通信的 C++ 库，适用于嵌入式系统和 Arduino 平台。它提供了客户端和服务器端的功能，支持非阻塞串口通信，可以轻松集成到各种项目中。

## 特性

- 支持 Modbus RTU 协议
- 非阻塞串口通信
- CRC16 校验
- 跨平台支持，适用于 Arduino 和其他嵌入式系统

## 示例

- `example/basic_client.cpp` - 基本的 Modbus 客户端示例
- `example/demo_server.cpp` - Modbus 服务器示例
- `example/non_block_serial_echo.cpp` - 非阻塞串口回显示例

## 依赖

- Arduino 开发环境
- `scheduler_basic` 库
- `oled_basic` 库（仅限 OLED 示例）

## 安装

1. 下载并安装 Arduino IDE。
2. 安装所需的库（如 `scheduler_basic` 和 `oled_basic`）。
3. 将 Modpash 库文件夹复制到 Arduino 的 libraries 文件夹中。
4. 重启 Arduino IDE。

## 使用方法

请参考示例代码以了解如何使用 Modpash 进行 Modbus 通信。

## 许可证

本项目使用 MPL 许可证。详情请查看 LICENSE 文件。
