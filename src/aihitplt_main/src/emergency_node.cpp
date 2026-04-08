#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <aihitplt_yolo/DetectResult.h>
#include <string>
#include <deque>
#include <algorithm>

class EmergencyNode {
private:
    ros::NodeHandle nh_;
    
    // 状态变量
    bool emergency_mode_ = false;
    bool is_scanning_ = false;
    bool alert_playing_ = false;
    bool pending_continue_ = false;
    bool current_point_alert_triggered_ = false;
    
    // 等级历史
    std::deque<std::string> grade_history_;
    int trigger_count_ = 0;
    int recovery_count_ = 0;
    const int trigger_threshold_ = 3;
    const int recovery_threshold_ = 3;
    
    // 检测历史
    std::deque<bool> detection_history_;
    const int detection_threshold_ = 3;
    int alert_count_ = 0;
    const int alert_max_count_ = 3;
    
    // 巡查点位
    int current_point_ = 0;
    const int max_points_ = 10;
    
    // 冷却时间
    ros::Time cooldown_until_;
    
    // ROS通信
    ros::Publisher cmd_vel_pub_;
    ros::Publisher emergency_state_pub_;
    ros::Publisher ui_command_pub_;
    ros::Publisher pan_tilt_pub_;
    
    ros::Subscriber grade_sub_;
    ros::Subscriber detect_sub_;
    ros::Subscriber dialog_response_sub_;
    
    // 定时器
    ros::Timer scan_timer_;
    ros::Timer alert_timer_;
    ros::Timer detection_timer_;
    ros::Timer move_timer_;
    
public:
    EmergencyNode() {
        // 初始化发布器
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        emergency_state_pub_ = nh_.advertise<std_msgs::Bool>("/emergency_state", 10);
        ui_command_pub_ = nh_.advertise<std_msgs::String>("/emergency_ui_command", 10);
        pan_tilt_pub_ = nh_.advertise<std_msgs::String>("/pan_tilt_camera/preset_control", 10);
        
        // 初始化订阅器
        grade_sub_ = nh_.subscribe("/environment_grade", 10, &EmergencyNode::gradeCallback, this);
        detect_sub_ = nh_.subscribe("/pan_tilt_camera/DetectMsg", 10, &EmergencyNode::detectCallback, this);
        dialog_response_sub_ = nh_.subscribe("/emergency_dialog_response", 10, &EmergencyNode::dialogCallback, this);
        
        ROS_INFO("Emergency Node started");
    }
    
    void gradeCallback(const std_msgs::String::ConstPtr& msg) {
        // 冷却检查
        if (ros::Time::now() < cooldown_until_) return;
        
        // 解析数据
        std::string data = msg->data;
        size_t comma = data.find(',');
        if (comma == std::string::npos) return;
        
        std::string grade = data.substr(0, comma);
        
        // 更新历史
        grade_history_.push_back(grade);
        if (grade_history_.size() > 3) grade_history_.pop_front();
        
        bool is_c_grade = (grade == "C");
        
        // 如果在等待恢复巡检，忽略等级变化
        if (pending_continue_) return;
        
        if (!emergency_mode_) {
            if (is_c_grade) {
                trigger_count_++;
            } else {
                trigger_count_ = 0;
            }
            
            if (trigger_count_ >= trigger_threshold_) {
                enterEmergencyMode();
            }
        } else {
            if (!is_scanning_ && !pending_continue_) {
                if (!is_c_grade) {
                    recovery_count_++;
                } else {
                    recovery_count_ = 0;
                }
                
                if (recovery_count_ >= recovery_threshold_) {
                    exitEmergencyMode();
                }
            }
        }
    }
    
    void detectCallback(const aihitplt_yolo::DetectResult::ConstPtr& msg) {
        // 不在巡查中、正在播报警报、等待恢复、或已触发警报，忽略检测
        if (!is_scanning_ || alert_playing_ || pending_continue_ || current_point_alert_triggered_) {
            return;
        }
        
        // 检测火源/烟雾
        if (msg->detected && msg->box_count > 0 && 
            (msg->class_name == "Fire" || msg->class_name == "Smoke") &&
            msg->confidence > 0.7) {
            
            ROS_WARN("检测到 %s, 置信度: %.2f", msg->class_name.c_str(), msg->confidence);
            
            detection_history_.push_back(true);
            if (detection_history_.size() > detection_threshold_) {
                detection_history_.pop_front();
            }
            
            // 检查是否连续检测到
            bool all_true = true;
            for (bool b : detection_history_) {
                if (!b) { all_true = false; break; }
            }
            
            // 连续检测到阈值次数，触发警报
            if (detection_history_.size() == detection_threshold_ && all_true) {
                ROS_WARN("连续%d次检测到%s，触发警报", detection_threshold_, msg->class_name.c_str());
                current_point_alert_triggered_ = true;
                triggerAlert(msg->class_name);
            }
        } else {
            detection_history_.clear();
        }
    }
    
    void dialogCallback(const std_msgs::String::ConstPtr& msg) {
        // 处理用户对话框响应
        if (msg->data == "yes") {
            cooldown_until_ = ros::Time::now() + ros::Duration(60.0);
            pending_continue_ = false;
            detection_history_.clear();
            exitEmergencyMode();
        } else {
            ROS_INFO("停止巡检");
            pending_continue_ = false;
            exitEmergencyMode();
        }
    }
    
    void enterEmergencyMode() {
        if (emergency_mode_) return;
        
        ROS_WARN("环境异常！进入应急模式");
        emergency_mode_ = true;
        trigger_count_ = 0;
        recovery_count_ = 0;
        current_point_ = 0;
        
        // 停止机器人
        geometry_msgs::Twist stop_cmd;
        for (int i = 0; i < 3; i++) {
            cmd_vel_pub_.publish(stop_cmd);
            ros::Duration(0.1).sleep();
        }
        
        // 发布应急状态
        std_msgs::Bool state_msg;
        state_msg.data = true;
        emergency_state_pub_.publish(state_msg);
        
        // 通知UI播放语音
        std_msgs::String cmd;
        cmd.data = "play_emergency_voice";
        ui_command_pub_.publish(cmd);
        
        // 2秒后启动巡查
        scan_timer_ = nh_.createTimer(ros::Duration(2.0), 
                                       boost::bind(&EmergencyNode::startCameraScan, this, _1), 
                                       true);
    }
    
    void startCameraScan(const ros::TimerEvent&) {
        if (emergency_mode_ && !is_scanning_ && !pending_continue_) {
            ROS_INFO("启动应急相机巡查");
            is_scanning_ = true;
            current_point_ = 0;
            current_point_alert_triggered_ = false;
            
            // 通知UI启动云台巡查
            std_msgs::String cmd;
            cmd.data = "start_pan_tilt_scan";
            ui_command_pub_.publish(cmd);
            
            // 发送预置点1
            goToPresetPoint(1);
        }
    }
    
    void goToPresetPoint(int preset_num) {
        std_msgs::String preset;
        preset.data = "go," + std::to_string(preset_num);
        pan_tilt_pub_.publish(preset);
        
        // 3秒后开始检测
        detection_timer_ = nh_.createTimer(ros::Duration(3.0), 
                                            boost::bind(&EmergencyNode::startPointDetection, this, _1), 
                                            true);
    }
    
    void startPointDetection(const ros::TimerEvent&) {
        if (!is_scanning_ || pending_continue_) return;
        
        detection_history_.clear();
        
        // 3秒后移动到下一点
        move_timer_ = nh_.createTimer(ros::Duration(3.0), 
                                       boost::bind(&EmergencyNode::moveToNextPoint, this, _1), 
                                       true);
    }
    
    void moveToNextPoint(const ros::TimerEvent&) {
        if (!is_scanning_ || pending_continue_) return;
        
        current_point_++;
        current_point_alert_triggered_ = false;
        
        if (current_point_ > max_points_) {
            // 巡查完成
            ROS_INFO("应急巡查完成");
            is_scanning_ = false;
            current_point_ = 0;
            
            // 发送休眠指令
            goToPresetPoint(40);
            
            // 检查是否需要退出应急模式
            bool has_c = false;
            for (const auto& g : grade_history_) {
                if (g == "C") has_c = true;
            }
            
            if (!has_c && emergency_mode_) {
                ROS_INFO("环境已恢复正常，退出应急模式");
                ros::Timer timer = nh_.createTimer(ros::Duration(1.0), 
                                                    boost::bind(&EmergencyNode::exitEmergencyMode, this, _1), 
                                                    true);
            }
            return;
        }
        
        // 发送下一个预置点
        goToPresetPoint(current_point_);
    }
    
    void triggerAlert(const std::string& class_name) {
        alert_playing_ = true;
        is_scanning_ = false;
        pending_continue_ = true;
        detection_history_.clear();
        alert_count_ = 0;
        
        ROS_WARN("触发警报，停止巡查");
        
        // 播报警报
        playAlertVoice();
        
        // 通知UI显示对话框
        std_msgs::String cmd;
        cmd.data = "show_alert_dialog:" + class_name;
        ui_command_pub_.publish(cmd);
    }
    
    void playAlertVoice() {
        if (alert_playing_) return;
        
        alert_playing_ = true;
        alert_count_++;
        
        // 异步播放语音
        system("aplay /home/aihit/aihitplt_ws/src/aihitplt_main/scripts/security_app/voice/fire_alarm.wav &");
        
        // 设置定时器检查播放完成
        alert_timer_ = nh_.createTimer(ros::Duration(1.5), 
                                        boost::bind(&EmergencyNode::checkAlertStatus, this, _1));
    }
    
    void checkAlertStatus(const ros::TimerEvent&) {
        static int wait_count = 0;
        wait_count++;
        
        if (wait_count >= 2) {
            alert_timer_.stop();
            wait_count = 0;
            alert_playing_ = false;
            
            if (alert_count_ < alert_max_count_) {
                // 继续播报
                playAlertVoice();
            }
            // 播报完成，等待UI响应
        }
    }
    
    void exitEmergencyMode(const ros::TimerEvent& = ros::TimerEvent()) {
        if (!emergency_mode_) return;
        
        ROS_INFO("========== 退出应急模式 ==========");
        
        emergency_mode_ = false;
        is_scanning_ = false;
        recovery_count_ = 0;
        pending_continue_ = false;
        current_point_alert_triggered_ = false;
        current_point_ = 0;
        
        // 发送休眠指令
        std_msgs::String preset;
        preset.data = "go,40";
        pan_tilt_pub_.publish(preset);
        
        // 发布状态
        std_msgs::Bool state_msg;
        state_msg.data = false;
        emergency_state_pub_.publish(state_msg);
        
        // 通知UI退出应急模式
        std_msgs::String cmd;
        cmd.data = "exit_emergency_mode";
        ui_command_pub_.publish(cmd);
        
        // 停止语音
        system("pkill aplay");
        
        ROS_INFO("应急模式已退出");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "emergency_handler");
    EmergencyNode node;
    ros::spin();
    return 0;
}