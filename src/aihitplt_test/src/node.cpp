#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
#include <ctime>
#include <cstdio>

int main() {
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(10));
        
        // 读取CPU占用率
        FILE* fp = popen("top -bn1 | grep 'Cpu(s)' | awk '{print $2}' | cut -d'%' -f1", "r");
        char cpu[10];
        fgets(cpu, sizeof(cpu), fp);
        pclose(fp);
        
        // 获取时间
        time_t now = time(nullptr);
        char time_str[100];
        strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", localtime(&now));
        
        // 终端打印
        printf("[%s] CPU: %s%%\n", time_str, cpu);
        
        // 写入文件
        std::ofstream file("cpu.log", std::ios::app);
        file << "[" << time_str << "] CPU: " << cpu << "%\n";
        file.close();
    }
    return 0;
}