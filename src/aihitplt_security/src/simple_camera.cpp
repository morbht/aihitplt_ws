// simple_camera.cpp
#include <iostream>
#include <cstring>

// 简单的函数声明（避免复杂依赖）
extern "C" {
    // 初始化
    bool init_camera() {
        std::cout << "初始化相机" << std::endl;
        return true;
    }
    
    // 登录
    int login_camera(const char* ip, const char* user, const char* pwd) {
        std::cout << "登录相机: " << ip << ", 用户: " << user << std::endl;
        return 1; // 模拟成功
    }
    
    // 启动预览
    bool start_preview(int user_id) {
        std::cout << "启动预览, UserID: " << user_id << std::endl;
        return true;
    }
    
    // 抓图
    bool capture_picture(int user_id, const char* filename) {
        std::cout << "抓图保存到: " << filename << std::endl;
        return true;
    }
    
    // 云台控制
    bool ptz_control(int user_id, int command, int stop, int speed) {
        const char* commands[] = {"上", "下", "左", "右", "放大", "缩小"};
        std::cout << "云台控制: " << commands[command-21] << ", 速度: " << speed << std::endl;
        return true;
    }
    
    // 清理
    void cleanup() {
        std::cout << "清理资源" << std::endl;
    }
}
