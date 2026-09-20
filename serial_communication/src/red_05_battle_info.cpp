#include "red.h"

void RefereeSystemNode::updateBattleInfo()
{
  enemy_distance_ = serial_receiver_.enemy_distance_;
  enemy_yaw_ = serial_receiver_.enemy_yaw_;
  my_hp_ = serial_receiver_.my_hp_;
  second_ = serial_receiver_.second_;
  bullet_ = serial_receiver_.bullet_;
  attack_mession_ = serial_receiver_.attack_mession_;

  base_interfaces_demo::msg::Target msg;
  msg.header.header.stamp = now();
  msg.header.header.frame_id = "chassis";
  msg.distance = enemy_distance_;
  msg.yaw = enemy_yaw_;
  track_target_pub_->publish(msg);

}
