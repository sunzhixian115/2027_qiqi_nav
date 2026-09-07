#include "red.h"

// ============================================================
// SerialReceiver 实现
// ============================================================

void SerialReceiver::readSerialData()
{
  if (!serial_port_.isOpen())
  {
    std::cout << "打不开啊" << std::endl;
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
    // CRC 校验（对整帧含CRC的149字节验证）
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

    // ======================================================
    // 逐字段拷贝
    // ======================================================

    std::memcpy(&start_flag_,      p + 1,  sizeof(uint8_t));
    std::memcpy(&enemy_distance_,  p + 2,  sizeof(float));
    std::memcpy(&enemy_yaw_,       p + 6,  sizeof(float));
    std::memcpy(&my_hp_,           p + 10, sizeof(float));
    std::memcpy(&second_,          p + 14, sizeof(uint16_t));
    std::memcpy(&bullet_,          p + 16, sizeof(uint16_t));

    std::memcpy(&rune_mode_,       p + 18, sizeof(bool));
    std::memcpy(&attack_mession_,  p + 19, sizeof(uint16_t));

    // 己方机器人
    std::memcpy(&my_hero_x_,       p + 21, sizeof(float));
    std::memcpy(&my_hero_y_,       p + 25, sizeof(float));
    std::memcpy(&my_hero_hp_,      p + 29, sizeof(int));
    std::memcpy(&my_engineer_x_,   p + 33, sizeof(float));
    std::memcpy(&my_engineer_y_,   p + 37, sizeof(float));
    std::memcpy(&my_engineer_hp_,  p + 41, sizeof(int));
    std::memcpy(&my_infantry3_x_,  p + 45, sizeof(float));
    std::memcpy(&my_infantry3_y_,  p + 49, sizeof(float));
    std::memcpy(&my_infantry3_hp_, p + 53, sizeof(int));
    std::memcpy(&my_infantry4_x_,  p + 57, sizeof(float));
    std::memcpy(&my_infantry4_y_,  p + 61, sizeof(float));
    std::memcpy(&my_infantry4_hp_, p + 65, sizeof(int));

    // 敌方机器人
    std::memcpy(&enemy_hero_x_,       p + 69, sizeof(float));
    std::memcpy(&enemy_hero_y_,       p + 73, sizeof(float));
    std::memcpy(&enemy_hero_hp_,      p + 77, sizeof(int));
    std::memcpy(&enemy_engineer_x_,   p + 81, sizeof(float));
    std::memcpy(&enemy_engineer_y_,   p + 85, sizeof(float));
    std::memcpy(&enemy_engineer_hp_,  p + 89, sizeof(int));
    std::memcpy(&enemy_infantry3_x_,  p + 93, sizeof(float));
    std::memcpy(&enemy_infantry3_y_,  p + 97, sizeof(float));
    std::memcpy(&enemy_infantry3_hp_, p + 101, sizeof(int));
    std::memcpy(&enemy_infantry4_x_,  p + 105, sizeof(float));
    std::memcpy(&enemy_infantry4_y_,  p + 109, sizeof(float));
    std::memcpy(&enemy_infantry4_hp_, p + 113, sizeof(int));
    std::memcpy(&enemy_sentry_x_,     p + 117, sizeof(float));
    std::memcpy(&enemy_sentry_y_,     p + 121, sizeof(float));
    std::memcpy(&enemy_sentry_hp_,    p + 125, sizeof(int));

    // 场地状态
    std::memcpy(&base_exposed_,      p + 129, sizeof(bool));
    std::memcpy(&energy_activated_,  p + 130, sizeof(bool));

    // 前哨站/基地血量
    std::memcpy(&outpost_hp_,        p + 131, sizeof(int));
    std::memcpy(&base_hp_,           p + 135, sizeof(int));


    std::memcpy(&if_fly_,           p + 139, sizeof(uint8_t));
    std::memcpy(&gimble_status_,    p + 140, sizeof(uint8_t));
    
    if(start_flag_!= 4)
    {
      gimble_status_ = 0;
    }

    buffer_.erase(
      buffer_.begin(),
      buffer_.begin() + FRAME_LENGTH);
  }
}

