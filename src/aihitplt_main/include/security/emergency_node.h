#ifndef EMERGENCY_NODE_H
#define EMERGENCY_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <queue>
#include <string>

class EmergencyNode : public rclcpp::Node {
public:
    EmergencyNode();

private:
    // 回调函数
    void grade_callback(const std_msgs::msg::String::SharedPtr msg);
    
    // 核心逻辑
    void update_grade_history(const std::string& grade);
    void enter_emergency_mode();
    void exit_emergency_mode();
    
    // 状态变量
    bool emergency_mode_{false};
    std::string current_grade_{"A"};
    std::queue<std::string> grade_history_;
    int trigger_count_{0};
    int recovery_count_{0};
    
    // 常量
    const int HISTORY_MAXLEN = 3;
    const int TRIGGER_THRESHOLD = 3;
    const int RECOVERY_THRESHOLD = 3;
    
    // ROS通信
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr grade_sub_;
};

#endif
