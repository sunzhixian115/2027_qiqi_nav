#include "red.h"

// ============================================================
// SerialReceiver 实现
// ============================================================

void SerialReceiver::readSerialData()
{
  if (!serial_port_.isOpen())
  {
    // 串口可能在启动后才连接，静默等待，避免周期性刷屏。
    return;
  }

  size_t bytes_available =
    serial_port_.available();

  if(bytes_available == 0)
  {
    return;
  }

  std::vector<uint8_t> read_data(bytes_available);

  size_t bytes_read =
    serial_port_.read(
      read_data.data(),
      bytes_available);

  buffer_.insert(
    buffer_.end(),
    read_data.begin(),
    read_data.begin() + bytes_read);

  processBuffer();
}

void SerialReceiver::processBuffer()
{
  constexpr size_t FRAME_LENGTH =
    sizeof(SerialInFrame);

  while(buffer_.size() >= FRAME_LENGTH)
  {
    auto header_pos =
      std::find(
        buffer_.begin(),
        buffer_.end(),
        0x5A);

    if(header_pos == buffer_.end())
    {
      buffer_.clear();

      return;
    }

    if(header_pos != buffer_.begin())
    {
      buffer_.erase(
        buffer_.begin(),
        header_pos);
    }

    if(buffer_.size() < FRAME_LENGTH)
    {
      return;
    }

    const uint8_t* p = buffer_.data();

    // ======================================================
    // 帧头校验
    // ======================================================

    uint8_t header;
    std::memcpy(&header, p + 0, sizeof(header));

    if(header != 0x5A)
    {
      buffer_.erase(buffer_.begin());

      continue;
    }

    // ======================================================
    // CRC 校验（对整帧 22 字节验证）
    // ======================================================

    if (!verify_CRC16_check_sum(p, FRAME_LENGTH)) {
      // 打印原始帧数据用于调试
      std::string hex_dump;
      for (size_t i = 0; i < FRAME_LENGTH; ++i) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%02X ", p[i]);
        hex_dump += buf;
      }
      RCLCPP_WARN(
        rclcpp::get_logger("serial"),
        "CRC校验失败, 原始数据: %s", hex_dump.c_str());
      buffer_.erase(buffer_.begin());
      continue;
    }

    // dump接收帧(hex) - 只在前2次打印，之后每500次打印一次
    {
      static int rx_count = 0;
      rx_count++;
      if (rx_count <= 2 || rx_count % 500 == 0) {
        std::string hex;
        for (size_t i = 0; i < FRAME_LENGTH; ++i) {
          char buf[8];
          snprintf(buf, sizeof(buf), "%02X ", p[i]);
          hex += buf;
        }
        RCLCPP_INFO(rclcpp::get_logger("serial"),
          "RX#%d(%zu字节): %s", rx_count, FRAME_LENGTH, hex.c_str());
      }
    }

    // 精简协议字段（CRC 位于偏移 20，由上面的校验处理）
    std::memcpy(&start_flag_,     p + 1,  sizeof(start_flag_));
    std::memcpy(&enemy_distance_, p + 2,  sizeof(enemy_distance_));
    std::memcpy(&enemy_yaw_,      p + 6,  sizeof(enemy_yaw_));
    std::memcpy(&my_hp_,          p + 10, sizeof(my_hp_));
    std::memcpy(&second_,         p + 14, sizeof(second_));
    std::memcpy(&bullet_,         p + 16, sizeof(bullet_));
    std::memcpy(&attack_mession_, p + 18, sizeof(attack_mession_));

    buffer_.erase(
      buffer_.begin(),
      buffer_.begin() + FRAME_LENGTH);
  }
}
