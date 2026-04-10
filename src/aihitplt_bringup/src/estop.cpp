#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"

ros::Publisher pub;
geometry_msgs::Twist zero;
bool e_stop_active = false;

void callback(const std_msgs::Bool::ConstPtr& msg) {
    e_stop_active = msg->data;
    if (e_stop_active) {
        pub.publish(zero);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "estop_vel_con");
    ros::NodeHandle nh;
    

    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true); 
    ros::Subscriber sub = nh.subscribe("/e_stop", 1, callback);
    
    ros::spin();
    return 0;
}