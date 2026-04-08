#include "ros/ros.h"
#include "aihitplt_mag_follower/data.h"
#include "aihitplt_mag_follower/netdata.h"
#include "serial/serial.h"
#include <iostream>
#include <vector>
#include <signal.h>
#include <string.h>

std::vector<uint8_t> buff;
serial::Serial ser;
std::string port;
double RequestCycle;
int Baudrate, ID, Mode;
aihitplt_mag_follower::data Data;
aihitplt_mag_follower::netdata NetData;

// 控制变量
ros::Time last_print_time;
const double PRINT_INTERVAL = 0.05; // 0.05秒打印一次（20Hz）
bool data_valid = false;
int error_count = 0;
const int MAX_ERRORS_BEFORE_RESET = 10;

// 最新有效数据
uint16_t latest_switch_value = 0;
int8_t latest_left_offset = 0;
int8_t latest_straight_offset = 0;
int8_t latest_right_offset = 0;
ros::Time latest_data_time;
uint16_t last_printed_switch_value = 0;
int8_t last_printed_left_offset = 0;
int8_t last_printed_straight_offset = 0;
int8_t last_printed_right_offset = 0;

/*************************************************
 * 信号处理函数，防止程序崩溃
 ***********************************************/
void signal_handler(int sig) {
    ROS_ERROR("程序收到信号: %d，安全退出", sig);
    if(ser.isOpen()) {
        ser.close();
    }
    exit(1);
}

/*************************************************
 * 计算校验和
 ***********************************************/
uint8_t CalculateChecksum(uint8_t *data, size_t len) {
    uint8_t sum = 0;
    for(size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

/*************************************************
 * 简化输出：显示 8 个位置状态 + 偏移
 ***********************************************/
void PrintSimpleStatus(uint16_t io, int8_t left, int8_t straight, int8_t right)
{
    ros::Time current_time = ros::Time::now();
    
    // 检查数据是否有变化
    bool data_changed = (io != last_printed_switch_value) || 
                       (left != last_printed_left_offset) ||
                       (straight != last_printed_straight_offset) ||
                       (right != last_printed_right_offset);
    
    // 控制打印频率：有变化立即打印，无变化按频率打印
    if (!data_changed && (current_time - last_print_time).toSec() < PRINT_INTERVAL) {
        return;
    }
    
    last_print_time = current_time;
    last_printed_switch_value = io;
    last_printed_left_offset = left;
    last_printed_straight_offset = straight;
    last_printed_right_offset = right;
    
    // 清屏并显示最新数据
    std::cout << "\033[2J\033[1;1H"; // 清屏并移动光标到左上角
    std::cout << "=== 磁导航传感器 ==" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    
    // 显示二进制状态（从左到右：S1到S8）
    std::cout << "传感器状态: " << std::endl;
    for (int i = 0; i < 8; i++) {
        bool detected = (io >> i) & 0x01;
        std::cout << (detected ? "1" : "0");
    }
    
    // 显示传感器状态条（从左到右：S1到S8）
    std::cout << "传感器条:   [";
    for (int i = 0; i < 8; i++) {
        bool detected = (io >> i) & 0x01;
        std::cout << (detected ? "■" : "□");
    }
    std::cout << "]" << std::endl;
    
    // 显示传感器编号（从左到右：S1到S8）
    std::cout << "传感器编号:  ";
    for (int i = 0; i < 8; i++) {
        std::cout << "S" << (i+1) << " ";
    }
    std::cout << std::endl;
    
    // 显示每个传感器状态（从左到右：S1到S8）
    std::cout << "详细状态:   ";
    for (int i = 0; i < 8; i++) {
        bool detected = (io >> i) & 0x01;
        std::cout << (detected ? "●" : "○") << "  ";
    }
    std::cout << std::endl;
    
    // 显示偏移量（转换为实际毫米数）
    std::cout << "偏移信息: ";
    std::cout << "左:" << (int)left << "(" << (int)left * 5 << "mm) ";
    std::cout << "中:" << (int)straight << "(" << (int)straight * 5 << "mm) ";
    std::cout << "右:" << (int)right << "(" << (int)right * 5 << "mm)" << std::endl;
    
    // 显示状态
    if (io == 0) {
        std::cout << "运行状态: \033[31m未检测到磁条\033[0m" << std::endl;
    } else {
        std::cout << "运行状态: \033[32m正常跟踪磁条\033[0m" << std::endl;
    }
    
    // 显示数据延迟
    double delay = (current_time - latest_data_time).toSec();
    std::cout << "数据延迟: " << (delay * 1000) << "ms" << std::endl;
    
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "按 Ctrl+C 退出" << std::endl;
}

/*************************************************
 * 定时器回调：在透传模式下不发送数据
 ***********************************************/
void timercallback(const ros::TimerEvent &)
{
    // 透传模式下传感器主动发送数据，不需要查询
}

/*************************************************
 * 快速解析IGK-G408透传模式数据
 * 只处理最新的数据帧，丢弃积压的旧数据
 ***********************************************/
bool ParseLatestIGK_G408_Data(uint8_t *buffer, size_t buffer_size)
{
    // 从缓冲区末尾向前查找最新的完整数据帧
    int frames_found = 0;
    for (int i = buffer_size - 23; i >= 0; i--) {
        // 检查帧头
        if (buffer[i] != 0xDD) {
            continue;
        }
        
        // 检查是否有完整的23字节帧
        if (i + 23 > buffer_size) {
            continue;
        }
        
        // 检查校验和
        uint8_t calculated_checksum = CalculateChecksum(&buffer[i], 22);
        uint8_t received_checksum = buffer[i + 22];
        
        if (calculated_checksum != received_checksum) {
            // 校验和错误，继续查找前一个帧
            if (error_count < 3 && frames_found == 0) {
                ROS_WARN_THROTTLE(2.0, "校验和错误: 计算=%02X, 接收=%02X", calculated_checksum, received_checksum);
            }
            error_count++;
            continue;
        }
        
        // 解析数据帧
        uint8_t *frame = &buffer[i];
        
        // 解析开关量 (2字节)
        uint16_t switch_value = (frame[1] << 8) | frame[2];
        
        // 解析偏移量 (有符号字节)
        int8_t left_offset = (int8_t)frame[3];
        int8_t straight_offset = (int8_t)frame[4];
        int8_t right_offset = (int8_t)frame[5];
        
        // 更新最新数据
        latest_switch_value = switch_value;
        latest_left_offset = left_offset;
        latest_straight_offset = straight_offset;
        latest_right_offset = right_offset;
        latest_data_time = ros::Time::now();
        
        // 更新ROS消息
        Data.io = switch_value;
        Data.offset_left = left_offset;
        Data.offset_straight = straight_offset;
        Data.offset_right = right_offset;
        Data.has_magnet = (switch_value != 0);
        Data.all_on = (switch_value == 0x00FF);
        Data.online = true;
        Data.header.stamp = latest_data_time;
        Data.header.frame_id = "IGK-G408-MAG-Sensor";
        
        // 重置错误计数
        error_count = 0;
        data_valid = true;
        frames_found++;
        
        // 找到第一个有效帧就返回，确保实时性
        ROS_DEBUG_THROTTLE(5.0, "成功解析数据帧，开关量: 0x%04X", switch_value);
        return true;
    }
    
    return false;
}

/*************************************************
 * 数据接收和处理 - 优化版本
 ***********************************************/
void ProcessReceivedDataOptimized(uint8_t *buf, size_t len)
{
    if (buf == nullptr || len == 0) {
        return;
    }

    // 使用静态缓冲区累积数据，但限制大小
    static std::vector<uint8_t> cumulative_buffer;
    const size_t MAX_BUFFER_SIZE = 200; // 最大200字节，约8帧数据
    
    // 添加新数据
    cumulative_buffer.insert(cumulative_buffer.end(), buf, buf + len);
    
    // 如果缓冲区太大，从末尾保留最新数据
    if (cumulative_buffer.size() > MAX_BUFFER_SIZE) {
        size_t excess = cumulative_buffer.size() - MAX_BUFFER_SIZE;
        cumulative_buffer.erase(cumulative_buffer.begin(), cumulative_buffer.begin() + excess);
        if (excess > 50) {
            ROS_WARN_THROTTLE(2.0, "数据积压，丢弃 %zu 字节旧数据", excess);
        }
    }
    
    // 尝试解析最新的数据帧
    bool parsed = ParseLatestIGK_G408_Data(cumulative_buffer.data(), cumulative_buffer.size());
    
    if (parsed) {
        // 成功解析后，可以清空大部分缓冲区，只保留最后46字节（可能包含下一帧）
        if (cumulative_buffer.size() > 46) {
            cumulative_buffer.erase(cumulative_buffer.begin(), cumulative_buffer.end() - 46);
        }
    } else {
        // 如果没有找到有效帧，清理过旧的数据
        if (cumulative_buffer.size() > 100) {
            cumulative_buffer.erase(cumulative_buffer.begin(), cumulative_buffer.end() - 50);
        }
    }
    
    // 打印当前状态（受频率控制和变化检测控制）
    PrintSimpleStatus(latest_switch_value, latest_left_offset, latest_straight_offset, latest_right_offset);
}

/*************************************************
 * 主函数
 ***********************************************/
int main(int argc, char *argv[])
{
    // 设置信号处理
    signal(SIGSEGV, signal_handler);
    signal(SIGABRT, signal_handler);
    signal(SIGILL, signal_handler);
    
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "mag_sensor");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    nh_private.param("Port", port, std::string("/dev/ttyUSB0"));
    nh_private.param("RequestCycle", RequestCycle, 1.0);
    nh_private.param("Baudrate", Baudrate, 115200);
    nh_private.param("ID", ID, 1);
    nh_private.param("Mode", Mode, 0);

    // 初始化变量
    last_print_time = ros::Time::now();
    latest_data_time = ros::Time::now();
    error_count = 0;
    data_valid = false;

    ros::Publisher pub;
    if (Mode)
        pub = nh.advertise<aihitplt_mag_follower::netdata>("mag_sensor", 10);
    else
        pub = nh.advertise<aihitplt_mag_follower::data>("mag_sensor", 10);

    ROS_INFO("IGK-G408 磁导航传感器初始化...");
    ROS_INFO("工作模式: RS485透传模式");
    ROS_INFO("端口号: %s, 波特率: %d", port.c_str(), Baudrate);
    ROS_INFO("显示频率: 20Hz (数据变化时立即更新)");

    // 初始化数据
    Data.io = 0;
    Data.offset_left = 0;
    Data.offset_straight = 0;
    Data.offset_right = 0;
    Data.has_magnet = false;
    Data.all_on = false;
    Data.online = false;
    Data.header.frame_id = "IGK-G408-MAG-Sensor";

    // 串口初始化
    try
    {
        ser.setPort(port);
        ser.setBaudrate(Baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(10); // 短超时
        ser.setTimeout(to);
        ser.open();
        
        if(ser.isOpen()){
            ser.flushInput();
            ser.flushOutput();
            ROS_INFO_STREAM("串口已成功打开: " << port);
        } else {
            ROS_ERROR("串口打开失败");
            return -1;
        }
    }
    catch (std::exception &e)
    {
        ROS_ERROR_STREAM("初始化异常: " << e.what());
        return -1;
    }

    // 创建定时器
    ros::Timer timer1 = nh.createTimer(ros::Duration(RequestCycle), timercallback);

    // 提高处理频率
    ros::Rate rate(100); // 100Hz处理频率
    uint8_t revBuf[256] = {0};

    // 清屏并显示初始信息
    std::cout << "\033[2J\033[1;1H";
    std::cout << "=== IGK-G408 磁导航传感器启动完成 ===" << std::endl;
    std::cout << "开始接收数据..." << std::endl;

    while (ros::ok())
    {
        if (ser.isOpen() && ser.available())
        {
            size_t len = ser.available();
            if(len > sizeof(revBuf)) {
                len = sizeof(revBuf);
            }
            
            memset(revBuf, 0, sizeof(revBuf));
            size_t actual_len = ser.read(revBuf, len);

            if(actual_len > 0) {
                ProcessReceivedDataOptimized(revBuf, actual_len);
            }
        }

        // 发布数据
        if (Mode)
            pub.publish(NetData);
        else
            pub.publish(Data);

        rate.sleep();
        ros::spinOnce();
    }

    // 清理资源
    if(ser.isOpen()) {
        ser.close();
    }
    
    std::cout << "\n磁导航传感器节点正常退出" << std::endl;
    return 0;
}
