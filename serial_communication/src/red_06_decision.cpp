#include "red.h"

void RefereeSystemNode::decisionTick()
 {
  //   // 电控说开始我在开始
  //   if(serial_receiver_.start_flag_ != 4)
  //   {
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
  //     "[决策] 区域=%d | HP=%.0f 弹药=%d | 追击=%s | 位置=(%.1f,%.1f)",
  //     current_region,
  //     serial_receiver_.my_hp_, serial_receiver_.bullet_,
  //     chase_locked_ ? "是" : "否",
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
  //     // 锁定前哨站
  //     // ========================================================
  //     if(serial_receiver_.attack_mession_==10)//如果索到前哨就不动了
  //     {
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



