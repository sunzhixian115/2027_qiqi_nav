#include "red.h"

void RefereeSystemNode::decisionTick()
 {
  //   // 电控说开始我在开始
  //   if(serial_receiver_.start_flag_ != 4)
  //   {
  //     serial_receiver_.gimble_status_=0;
  //     return;
  //   }

  //   // 获得本车区域
  //  const int current_region = getRegion(robot_pos_);
    
  //   // 每个tick重置,只有真正到达时才设1
  //   if_arrive_base_point_ = 0;
  //   attack_target_ = 0;


  //   // 调试日志 - 追踪决策状态
  //   RCLCPP_INFO_THROTTLE(
  //     get_logger(),
  //     *get_clock(),
  //     5000,//5秒输出一次
  //     "[决策] 区域=%d | HP=%.0f 弹药=%d | 追击=%s | 雷达点=(%.1f,%.1f) | 位置=(%.1f,%.1f)",
  //     current_region,
  //     serial_receiver_.my_hp_, serial_receiver_.bullet_,
  //     chase_locked_ ? "是" : "否",
  //     radar_saved_point_.x, radar_saved_point_.y,
  //     robot_pos_.x, robot_pos_.y);
    

  //   //如果是基地模式的话
  //   if(decision_mode_ == DecisionMode::BASE_MODE)
  //   {

  //     // ========================================================
  //     // 补给
  //     // ========================================================

  //     // 进入补给：HP低 或 弹药低
  //     if(!need_heal_ &&
  //         (serial_receiver_.my_hp_ <= 150.0f ||
  //         serial_receiver_.bullet_ < 50))
  //     {
  //       need_heal_ = true;
  //     }
  //     // 解除补给：HP满 且 弹药够
  //     if(need_heal_ &&
  //         serial_receiver_.my_hp_ >= 390.0f &&
  //         serial_receiver_.bullet_ > 50)
  //     {
  //       need_heal_ = false;
  //     }
  //     // 执行补给
  //     if(need_heal_)
  //     {
  //       current_target_point_ = supply_point_;
  //       publishGoal(supply_point_);
  //       return;
  //     }

  //     // ========================================================
  //     // 云台手给我的标记 默认0巡航 1是前哨站爆了 2是猥琐 3是绕基地
  //     // ========================================================

  //     //云台手不给插点，直接跳过不处理
  //     if(serial_receiver_.gimble_status_==1)//如果给了1,那我就跑的进攻性一点
  //     {
  //       if(chase_locked_)//进攻性的跑，优先追击
  //       {
  //         if(
  //           getRegion(chase_point_) ==
  //           current_region)
  //         {
  //           current_target_point_ =
  //             chase_point_;
  //           RCLCPP_INFO(get_logger(), "追击ing");
  //           if(getDistance(chase_point_,robot_pos_)<1.5)
  //           {
  //             RCLCPP_INFO(get_logger(), "到达追击点");
  //             return;
  //           }
  //           publishGoal(chase_point_);
  //           return;
  //         }
  //         // 不同区域 → 不追，继续往下判断
  //       }

  //       no_out_poset_Cruise();//没有追击目标，就巡航

  //       return;
  //     }


  //     else if(serial_receiver_.gimble_status_==2)//如果给了2,我就猥琐的扫描
  //     {
  //       publishGoal(base_point_);
  //       if(getDistance(base_point_,robot_pos_)<0.5)
  //       {
  //         RCLCPP_INFO(get_logger(), "到达基地点");
  //         if_arrive_base_point_=1;
  //         return;
  //       }
  //       return;
  //     }


  //     else if(serial_receiver_.gimble_status_==3)//如果给了3,那我就绕基地点
  //     {
  //       //绕基地跑，优先追击
  //       if(chase_locked_)
  //       {
  //         if(
  //           getRegion(chase_point_) ==
  //           current_region)
  //         {
  //           current_target_point_ =
  //             chase_point_;
  //           RCLCPP_INFO(get_logger(), "追击ing");
  //           if(getDistance(chase_point_,robot_pos_)<2.5)
  //           {
  //             RCLCPP_INFO(get_logger(), "到达追击点");
  //             return;
  //           }
  //           publishGoal(chase_point_);
  //           return;
  //         }
  //         // 不同区域 → 不追，继续往下判断
  //       }

  //       baseCruise();
  //       return;
  //     }




  //     // ========================================================
  //     // 锁定前哨站
  //     // ========================================================
  //     if(serial_receiver_.attack_mession_==10)//如果索到前哨就不动了
  //     {
  //       return;
  //     }


  //     // ========================================================
  //     // 飞机是否起飞
  //     // ========================================================

  //     if(if_fly_==1)//起飞了
  //     {
  //       //到基地点猥琐的扫描
  //       publishGoal(base_point_);
  //       if(getDistance(base_point_,robot_pos_)<0.5)
  //       {
  //         RCLCPP_INFO(get_logger(), "到达基地点");
  //         if_arrive_base_point_=1;
  //         return;
  //       }
  //       return;

  //     }


  //     // ========================================================
  //     // 追击
  //     // ========================================================
  //     if(chase_locked_)
  //     {
  //       if(
  //         getRegion(chase_point_) == current_region)
  //       {
  //         current_target_point_ =
  //           chase_point_;
  //         RCLCPP_INFO(get_logger(), "追击ing");
  //         if(getDistance(chase_point_,robot_pos_)<1.5)
  //         {
  //           RCLCPP_INFO(get_logger(), "到达追击点");
  //           return;
  //         }
  //         publishGoal(chase_point_);
  //         return;
  //       }
  //       // 不同区域 → 不追，继续往下判断
  //     }



  //     // ========================================================
  //     // 雷达点巡逻 (数据已在 updateBattleInfo 中缓存)
  //     // ========================================================

  //     if (radar_active_)// 雷达点有效才导航
  //     {
  //       publishGoal(radar_saved_point_);// 导航到雷达点

  //       if(getDistance(radar_saved_point_, robot_pos_) < arrive_tolerance_)// 如果到了
  //       {
  //         if(!radar_arrived_)// 刚到达
  //         {
  //           radar_arrive_time_ = this->now();
  //           radar_arrived_ = true;//标记刚到
  //           RCLCPP_INFO(get_logger(), "已到达雷达点，开始计时");
  //         }
  //         else if(this->now() - radar_arrive_time_ >= rclcpp::Duration(10s))
  //         {
  //           RCLCPP_INFO(get_logger(), "雷达点10秒结束");
  //           radar_arrived_ = false;
  //           radar_saved_point_.x = -1.0f;
  //           radar_saved_point_.y = -1.0f;
  //           radar_active_ = false;
  //         }
  //       }
  //       return;
  //     }



  //     // ========================================================
  //     // 基地巡逻
  //     // ========================================================
      
  //     homeCruise();
      

  //   }
    
  //   else
  //   {
  //     // ========================================================
  //     // 第一 补给
  //     // ========================================================

  //     // 进入补给：HP低 或 弹药低
  //     if(!need_heal_ &&
  //         (serial_receiver_.my_hp_ <= 150.0f ||
  //         serial_receiver_.bullet_ < 50))
  //     {
  //       need_heal_ = true;
  //     }
  //     // 解除补给：HP满 且 弹药够
  //     if(need_heal_ &&
  //         serial_receiver_.my_hp_ >= 390.0f &&
  //         serial_receiver_.bullet_ > 50)
  //     {
  //       need_heal_ = false;
  //     }
  //     // 执行补给
  //     if(need_heal_)
  //     {
  //       // current_target_point_ = supply_point_;
  //       // publishGoal(supply_point_);
  //       go_to_base();//下高地然后在基地区回补给区
  //       return;
  //     }
  //     // ========================================================
  //     // 第二 追击
  //     // ========================================================
  //     if(chase_locked_)
  //     {
  //       if(
  //         getRegion(chase_point_) ==
  //         current_region)
  //       {
  //         current_target_point_ =
  //           chase_point_;
  //         RCLCPP_INFO(get_logger(), "追击ing");
  //         if(getDistance(chase_point_,robot_pos_)<2.5)
  //         {
  //           RCLCPP_INFO(get_logger(), "到达追击点");
  //           return;
  //         }
  //         publishGoal(chase_point_);
  //         return;
  //       }
  //       // 不同区域 → 不追，继续往下判断
  //     }



  //     // ========================================================
  //     // 第三 雷达点巡逻
  //     // ========================================================

  //     // 找同区域最近敌人，有效时保存到缓存
  //     if (radar_active_)// 雷达点有效才导航
  //     {
  //       publishGoal(radar_saved_point_);// 导航到雷达点

  //       if(getDistance(radar_saved_point_, robot_pos_) < arrive_tolerance_)// 如果到了
  //       {
  //         if(!radar_arrived_)// 刚到达
  //         {
  //           radar_arrive_time_ = this->now();
  //           radar_arrived_ = true;//标记刚到
  //           RCLCPP_INFO(get_logger(), "已到达雷达点，开始计时");
  //         }
  //         else if(this->now() - radar_arrive_time_ >= rclcpp::Duration(10s))
  //         {
  //           RCLCPP_INFO(get_logger(), "雷达点10秒结束");
  //           radar_arrived_ = false;
  //           radar_saved_point_.x = -1.0f;
  //           radar_saved_point_.y = -1.0f;
  //           radar_active_ = false;
  //         }
  //       }
  //       return;
  //     }



  //     // ========================================================
  //     // 第四 高地巡逻
  //     // ========================================================

  //     highCruise();

  //   }


}


void RefereeSystemNode::homeCruise()//基地区前哨站巡航
{
  int choice = ( (420 - second_ ) / 30)%2;
  
  if(choice == 1)
  {
    publishGoal(attack_point_1_);
  }
  else
  {
    publishGoal(attack_point_2_);
  }

  return;
}

void RefereeSystemNode::no_out_poset_Cruise()//前哨站巡航
{
  int choice = ( (420 - second_ ) / 30)%2;
  
  if(choice == 1)
  {
    publishGoal(no_outpost_point_1_);
  }
  else
  {
    publishGoal(no_outpost_point_2_);
  }

  return;
}

void RefereeSystemNode::highCruise()//高地巡航
{
  int choice = ( (420 - second_ ) / 30)%3;
  if(choice == 1)
  {
    publishGoal(high_point_1_);
  }
  else if(choice == 2)
  {
    publishGoal(high_point_2_);
  }
  else
  {
    publishGoal(high_point_3_);
  }
}

void RefereeSystemNode::baseCruise()//基地巡航
{
  int choice = ( (420 - second_ ) / 5)%4;//五秒一点

  if(choice == 1)//基地点1
  {
    publishGoal(home_point_1_);//之后去参数服务器加点
  }
  else if(choice == 2)//基地点2
  {
    publishGoal(home_point_2_);
  }
  else if(choice == 3)//基地点3
  {
    publishGoal(home_point_3_);
  }
  else//基地点4
  {
    publishGoal(home_point_4_);
  }
}

void RefereeSystemNode::go_to_high()//上高地
{

  switch(stage_)
  {
    case Stage::IDLE:
    {
      publishGoal(pre_point_);
      stage_ = Stage::GO_PRE_POINT;
      if_turn_ = true;
      break;
    }

    case Stage::GO_PRE_POINT:
    {
      if(
        getDistance(robot_pos_, pre_point_) <
        arrive_tolerance_)
      {
        publishGoal(z1_);
        stage_ = Stage::GO_Z1;
      }
      break;
    }

    case Stage::GO_Z1:
    {
      if(
        getDistance(robot_pos_, z1_) <
        arrive_tolerance_)
      {
        publishGoal(z2_);
        stage_ = Stage::GO_Z2;
      }
      break;
    }

    case Stage::GO_Z2:
    {
      if(
        getDistance(robot_pos_, z2_) <
        arrive_tolerance_)
      {
        publishGoal(back_point_);
        stage_ = Stage::GO_BACK_POINT;
      }
      break;
    }

    case Stage::GO_BACK_POINT:
    {
      if(
        getDistance(robot_pos_, back_point_) <
        arrive_tolerance_)
      {
        stage_ = Stage::FINISHED;
      }
      break;
    }

    case Stage::FINISHED:
    {
      stage_ = Stage::IDLE;
      decision_mode_ =DecisionMode::HIGH_MODE;
      if_turn_ = false;
      break;
    }

    default:
      break;
  }
}

void RefereeSystemNode::go_to_base()//返回基地
{

  switch(stage_)
  {
    case Stage::IDLE:
    {
      publishGoal(back_point_);
      stage_ = Stage::GO_BACK_POINT;
      if_turn_ = true;
      break;
    }

    case Stage::GO_BACK_POINT:
    {
      if(
        getDistance(robot_pos_, back_point_) <
        arrive_tolerance_)
      {
        publishGoal(z2_);
        stage_ = Stage::GO_Z2;
      }
      break;
    }

    case Stage::GO_Z2:
    {
      if(
        getDistance(robot_pos_, z2_) <
        arrive_tolerance_)
      {
        publishGoal(z1_);
        stage_ = Stage::GO_Z1;
      }
      break;
    }

    case Stage::GO_Z1:
    {
      if(
        getDistance(robot_pos_, z1_) <
        arrive_tolerance_)
      {
        publishGoal(pre_point_);
        stage_ = Stage::GO_PRE_POINT;
      }
      break;
    }

    case Stage::GO_PRE_POINT:
    {
      if(
        getDistance(robot_pos_, pre_point_) <
        arrive_tolerance_)
      {
        stage_ = Stage::FINISHED;
      }
      break;
    }
    case Stage::FINISHED:
    {
      stage_ = Stage::IDLE;
      if_turn_ = false;
      decision_mode_ =DecisionMode::BASE_MODE;
      break;
    }

    default:
      break;
  }
}






