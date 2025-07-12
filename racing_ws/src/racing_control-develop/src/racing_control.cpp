// Copyright (c) 2022，Horizon Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "racing_control/racing_control.h"

#include <unistd.h>
#include <chrono> // 用于 std::chrono 时间库

const double LOOP_RATE_HZ = 30.0;

// RacingControlNode 类的构造函数
RacingControlNode::RacingControlNode(const std::string& node_name,const rclcpp::NodeOptions& options)
  : rclcpp::Node(node_name, options) {
  
  auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(avoidance_hold_duration_));
  avoidance_duration_tolerance_ = rclcpp::Duration(duration_ns);

  if (!msg_process_) {
    msg_process_ = std::make_shared<std::thread>(
        std::bind(&RacingControlNode::MessageProcess, this));
  }

  // === 参数声明和获取 ===
  this->declare_parameter<std::string>("pub_control_topic", pub_control_topic_);
  this->get_parameter<std::string>("pub_control_topic", pub_control_topic_);

  // 巡线参数
  this->declare_parameter<float>("follow_linear_speed", follow_linear_speed_);
  this->get_parameter<float>("follow_linear_speed", follow_linear_speed_);
  this->declare_parameter<float>("follow_angular_ratio", follow_angular_ratio_);
  this->get_parameter<float>("follow_angular_ratio", follow_angular_ratio_);
  this->declare_parameter<float>("line_confidence_threshold", line_confidence_threshold_);
  this->get_parameter<float>("line_confidence_threshold", line_confidence_threshold_);

  // 避障参数
  this->declare_parameter<float>("avoid_linear_speed", avoid_linear_speed_);
  this->get_parameter<float>("avoid_linear_speed", avoid_linear_speed_);
  this->declare_parameter<float>("avoid_angular_ratio", avoid_angular_ratio_);
  this->get_parameter<float>("avoid_angular_ratio", avoid_angular_ratio_);
  this->declare_parameter<float>("obstacle_confidence_threshold", obstacle_confidence_threshold_);
  this->get_parameter<float>("obstacle_confidence_threshold", obstacle_confidence_threshold_);
  this->declare_parameter<int>("bottom_threshold", bottom_threshold_);
  this->get_parameter<int>("bottom_threshold", bottom_threshold_);

  // 新增：特殊状态下的参数
  this->declare_parameter<float>("cruise_linear_speed", cruise_linear_speed_);
  this->get_parameter<float>("cruise_linear_speed", cruise_linear_speed_);
  this->declare_parameter<float>("recovering_linear_speed", recovering_linear_speed_);
  this->get_parameter<float>("recovering_linear_speed", recovering_linear_speed_);
  this->declare_parameter<float>("recovering_angular_ratio", recovering_angular_ratio_);
  this->get_parameter<float>("recovering_angular_ratio", recovering_angular_ratio_);

  this->declare_parameter<double>("avoidance_hold_duration", avoidance_hold_duration_);
  this->get_parameter<double>("avoidance_hold_duration", avoidance_hold_duration_);

  // === 订阅者和发布者创建 ===
  point_subscriber_ =
    this->create_subscription<ai_msgs::msg::PerceptionTargets>(
      "racing_track_center_detection", rclcpp::SensorDataQoS(),
      std::bind(&RacingControlNode::subscription_callback_point, this, std::placeholders::_1)); 

  target_subscriber_ =
    this->create_subscription<ai_msgs::msg::PerceptionTargets>(
      "racing_obstacle_detection", rclcpp::SensorDataQoS(),
      std::bind(&RacingControlNode::subscription_callback_target, this, std::placeholders::_1)); 
  
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(pub_control_topic_, 5);
  
  RCLCPP_INFO(rclcpp::get_logger("RacingControlNode"), "RacingControlNode initialized!");
}

// 析构函数
RacingControlNode::~RacingControlNode(){
  if (msg_process_ && msg_process_->joinable()) {
    process_stop_ = true;
    msg_process_->join();
    msg_process_ = nullptr;
  }
}

// 中线消息回调
void RacingControlNode::subscription_callback_point(const ai_msgs::msg::PerceptionTargets::SharedPtr point_msg){
  RCLCPP_DEBUG(this->get_logger(), "Received a track center message.");
  {
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    latest_point_msg_ = point_msg;
  }
}

// 障碍物消息回调
void RacingControlNode::subscription_callback_target(const ai_msgs::msg::PerceptionTargets::SharedPtr targets_msg){
  RCLCPP_DEBUG(this->get_logger(), "Received an obstacle message.");
  {
    std::unique_lock<std::mutex> lock(point_target_mutex_);
    latest_targets_msg_ = targets_msg;
  }
}

// 核心控制逻辑线程
void RacingControlNode::MessageProcess(){
  rclcpp::Rate loop_rate(LOOP_RATE_HZ);

  while(process_stop_ == false){
    // 动态获取参数
    this->get_parameter<double>("avoidance_hold_duration", avoidance_hold_duration_);
    auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(avoidance_hold_duration_));
    avoidance_duration_tolerance_ = rclcpp::Duration(duration_ns);

    std::unique_lock<std::mutex> lock(point_target_mutex_);
    auto current_line_msg = latest_point_msg_;
    auto current_obstacle_msg = latest_targets_msg_;
    lock.unlock();

    if (!current_line_msg) {
        RCLCPP_INFO(this->get_logger(), "Waiting for track center message...");
        loop_rate.sleep();
        continue;
    }

    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;

    // --- 状态机决策 ---
    bool obstacle_detected_and_close = false;
    ai_msgs::msg::Target relevant_obstacle_target;

    // 1. 检查障碍物，这具有最高优先级
    if (current_obstacle_msg && !current_obstacle_msg->targets.empty()) {
        for(const auto &target : current_obstacle_msg->targets){
            if(target.type == "construction_cone" && !target.rois.empty()){
                float obstacle_conf = target.rois[0].confidence;
                int bottom = target.rois[0].rect.y_offset + target.rois[0].rect.height;
                if (obstacle_conf >= obstacle_confidence_threshold_ && bottom >= bottom_threshold_) {
                    obstacle_detected_and_close = true;
                    relevant_obstacle_target = target;
                    current_state_ = State::OBSTACLE_AVOIDING;
                    last_avoidance_time_ = this->get_clock()->now();
                    RCLCPP_INFO(this->get_logger(), "Close obstacle detected! Switching to OBSTACLE_AVOIDING.");
                    break;
                }
            }
        }
    }
    
    // 2. 根据当前状态执行动作
    switch(current_state_) {

      case State::OBSTACLE_AVOIDING:
        if (obstacle_detected_and_close) {
          ObstaclesAvoiding(relevant_obstacle_target);
        } else {
          rclcpp::Time now = this->get_clock()->now();
          if (last_avoidance_time_.seconds() > 0 && now > last_avoidance_time_ && (now - last_avoidance_time_) < avoidance_duration_tolerance_) {
            // 在避障后的短暂保持期内，继续执行最后一次避障动作，确保完全避开
            RCLCPP_INFO(this->get_logger(), "In post-avoidance hold period, continuing last avoidance move.");
            twist_msg.linear.x = avoid_linear_speed_; 
            twist_msg.angular.z = last_avoidance_angular_z_; // 使用最后一次的角速度
            publisher_->publish(twist_msg);
          } else {
            // 保持期结束，切换到寻找赛道状态
            current_state_ = State::RECOVERING_LINE;
            RCLCPP_INFO(this->get_logger(), "Avoidance complete. Switching to RECOVERING_LINE.");
          }
        }
        break;

      case State::RECOVERING_LINE:
        RCLCPP_INFO(this->get_logger(), "In RECOVERING_LINE state, trying to find the track.");
        // 检查是否已找到高置信度的赛道线
        if (!current_line_msg->targets.empty() && !current_line_msg->targets[0].points.empty() &&
            !current_line_msg->targets[0].points[0].confidence.empty() && 
            current_line_msg->targets[0].points[0].confidence[0] >= line_confidence_threshold_) {
          // 切换回巡线状态
          current_state_ = State::LINE_FOLLOWING;
          RCLCPP_INFO(this->get_logger(), "High confidence line found! Switching back to LINE_FOLLOWING.");
        } else {
          // 未找到，执行“反向转弯”寻找赛道
          twist_msg.linear.x = follow_linear_speed_;
          // last_avoidance_angular_z_ < 0 表示向左转避障，现在需要向右转(>0)寻找
          // last_avoidance_angular_z_ > 0 表示向右转避障，现在需要向左转(<0)寻找
          twist_msg.angular.z = -1.5 * std::copysign(recovering_angular_ratio_, last_avoidance_angular_z_);
          publisher_->publish(twist_msg);
        }
        break;

      case State::LINE_FOLLOWING:
        // 在巡线状态，也要检查障碍物，因为障碍物可能突然出现
        if (obstacle_detected_and_close) { // 如果在巡线时发现障碍物，立即切换
            current_state_ = State::OBSTACLE_AVOIDING;
            ObstaclesAvoiding(relevant_obstacle_target);
        } 
        else {
          float line_confidence = current_line_msg->targets[0].points[0].confidence[0];
          LineFollowing(current_line_msg->targets[0], line_confidence);
        }
        /*{
            // 没有障碍物，正常巡线
            if (!current_line_msg->targets.empty() && !current_line_msg->targets[0].points.empty() &&
                !current_line_msg->targets[0].points[0].point.empty() && !current_line_msg->targets[0].points[0].confidence.empty()) {
                
                float line_confidence = current_line_msg->targets[0].points[0].confidence[0];
                if (line_confidence >= line_confidence_threshold_) {
                  // 高置信度，正常巡线
                  LineFollowing(current_line_msg->targets[0], line_confidence);
                } else {
                  // === 新逻辑：低置信度，慢速直行 ===
                  RCLCPP_WARN(this->get_logger(), "Line confidence (%f) is low. Cruising straight.", line_confidence);
                  twist_msg.linear.x = cruise_linear_speed_;
                  twist_msg.angular.z = 0.0;
                  publisher_->publish(twist_msg);
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Line message is invalid or empty. Cruising straight.");
                twist_msg.linear.x = cruise_linear_speed_;
                twist_msg.angular.z = 0.0;
                publisher_->publish(twist_msg);
            }
        }*/
        
        break;
      
      default: // 包括 STOP 状态
        publisher_->publish(twist_msg); // 发布停止指令
        break;
    }
    
    loop_rate.sleep();
  }
}

// 巡线控制函数
void RacingControlNode::LineFollowing(const ai_msgs::msg::Target &line_target, float line_confidence){
  if (line_target.points.empty() || line_target.points[0].point.empty()) { return; }
  
  int x = static_cast<int>(line_target.points[0].point[0].x);
  int y = static_cast<int>(line_target.points[0].point[0].y);
  float center_offset = static_cast<float>(x) - 320.0f;
  if (std::abs(center_offset) < 5.0f) { center_offset = 0.0f; }
  
  auto twist_msg = geometry_msgs::msg::Twist();
  float line_y_relative = (static_cast<float>(y) - 256.0f) / (480.0f - 256.0f);
  line_y_relative = std::max(0.0f, std::min(1.0f, line_y_relative));
  float angular_z = follow_angular_ratio_ * (center_offset / 320.0f) * line_y_relative; 

  twist_msg.linear.x = follow_linear_speed_;
  twist_msg.angular.z = angular_z;
  publisher_->publish(twist_msg);
  RCLCPP_INFO(this->get_logger(), "Line Following -> X:%d, Y:%d, Ang_Z: %f, Lin_X: %f", x, y, angular_z, follow_linear_speed_);
}

// 避障控制函数
void RacingControlNode::ObstaclesAvoiding(const ai_msgs::msg::Target &target){
  if (target.rois.empty() || target.rois[0].rect.width == 0) {
      RCLCPP_ERROR(this->get_logger(), "CRITICAL: ObstaclesAvoiding called with invalid ROI data!");
      return; 
  }

  auto twist_msg = geometry_msgs::msg::Twist();
  int center_x = target.rois[0].rect.x_offset + target.rois[0].rect.width / 2;
  float obstacle_center_offset = static_cast<float>(center_x) - 320.0f; 

  float angular_z_avoid = 0.0f;
  
  // 如果障碍物在中心，强制向一个固定方向转（例如，左转，角速度为负）
  /*
  if (std::abs(obstacle_center_offset) < 30.0f) {
      angular_z_avoid = -1.0f * std::abs(avoid_angular_ratio_);
  } else {
      // 障碍物在右侧(offset>0)，向左转(angular<0)；在左侧则相反。
      // 所以角速度方向与偏移量符号相反。
      angular_z_avoid = -1.0f * avoid_angular_ratio_ * (obstacle_center_offset / 320.0f);
  }
  */
  
  if(obstacle_center_offset < 5.0f && obstacle_center_offset >= 0) obstacle_center_offset = 5.0f;
  else if(obstacle_center_offset > -5.0f && obstacle_center_offset <0) obstacle_center_offset = -5.0f; 

  angular_z_avoid = -1.0f * avoid_angular_ratio_ * std::min(4.0f, (320.0f / obstacle_center_offset));

  // 限制最大角速度，防止转向过快
  //angular_z_avoid = std::max(-2.0f, std::min(2.0f, angular_z_avoid));

  // --- 新逻辑：记录这次避障的转向角速度 ---
  last_avoidance_angular_z_ = angular_z_avoid;

  twist_msg.linear.x = avoid_linear_speed_;
  twist_msg.angular.z = angular_z_avoid;
  publisher_->publish(twist_msg);
  RCLCPP_INFO(this->get_logger(), "Obstacles Avoiding -> CenterX:%d, Ang_Z: %f, Lin_X: %f", center_x, angular_z_avoid, avoid_linear_speed_);
}

// 主函数
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv); // 初始化ROS 2
  // 使用合适的节点名 "racing_control" 创建并运行节点
  rclcpp::spin(std::make_shared<RacingControlNode>("racing_control"));
  rclcpp::shutdown(); // 关闭ROS 2
  RCLCPP_WARN(rclcpp::get_logger("racing_control"), "Pkg exit.");
  return 0;
}