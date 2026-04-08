#!/bin/bash

# 配置文件路径 - 根据实际情况调整
CONFIG_FILE="/etc/NetworkManager/system-connections/Hotspot.nmconnection"

# 自动请求sudo权限
request_sudo() {
    if [ "$EUID" -ne 0 ]; then
        echo "需要root权限来修改网络配置"
        echo "请输入您的密码以继续..."
        
        # 重新以sudo执行当前脚本
        exec sudo "$0" "$@"
    fi
}

# 检查文件是否存在
check_config_file() {
    if [ ! -f "$CONFIG_FILE" ]; then
        echo "错误：热点配置文件 $CONFIG_FILE 不存在！"
        echo "当前可用的连接文件："
        ls -la "/etc/NetworkManager/system-connections/" 2>/dev/null || echo "无法访问目录"
        exit 1
    fi
}

# 获取当前热点名称
get_current_ssid() {
    if [ -f "$CONFIG_FILE" ]; then
        grep '^ssid=' "$CONFIG_FILE" | cut -d= -f2- 2>/dev/null
    else
        echo "未知"
    fi
}

# 检查并处理自定义热点服务
handle_custom_hotspot_service() {
    local new_ssid="$1"
    
    if [ -f "/etc/systemd/system/aihitplt_hotspot.service" ]; then
        echo "⚠️  检测到自定义热点服务，正在更新配置..."
        
        # 备份原服务文件
        sudo cp /etc/systemd/system/aihitplt_hotspot.service /etc/systemd/system/aihitplt_hotspot.service.backup.$(date +%Y%m%d_%H%M%S)
        
        # 更新服务文件中的SSID
        sudo sed -i "s/ssid aihitplt/ssid $new_ssid/g" /etc/systemd/system/aihitplt_hotspot.service
        sudo sed -i "s/con delete aihitplt/con delete $new_ssid/g" /etc/systemd/system/aihitplt_hotspot.service
        sudo sed -i "s/con down aihitplt/con down $new_ssid/g" /etc/systemd/system/aihitplt_hotspot.service
        sudo sed -i "s/con mod Hotspot connection.id aihitplt/con mod Hotspot connection.id $new_ssid/g" /etc/systemd/system/aihitplt_hotspot.service
        
        # 重新加载systemd配置
        sudo systemctl daemon-reload
        echo "✅ 已更新自定义热点服务配置"
    fi
}

# 启动热点函数
start_hotspot() {
    local ssid="$1"
    
    echo "尝试启动热点: $ssid"
    
    # 方法1: 使用现有的连接
    if nmcli connection show | grep -q "$ssid"; then
        echo "使用现有连接启动热点..."
        if nmcli connection up "$ssid"; then
            echo "✅ 热点启动成功"
            return 0
        fi
    fi
    
    # 方法2: 使用通用的热点名称
    echo "尝试使用通用热点配置..."
    if nmcli connection up Hotspot 2>/dev/null; then
        echo "✅ 热点启动成功"
        return 0
    fi
    
    # 方法3: 创建新的热点
    echo "创建新的热点..."
    if nmcli device wifi hotspot ifname wlp4s0 ssid "$ssid" password 12345678; then
        echo "✅ 热点创建并启动成功"
        return 0
    fi
    
    echo "❌ 所有启动方法都失败了"
    return 1
}

# 开启开机自启动
enable_autostart() {
    clear
    echo "-------------------------------------"
    echo "  正在开启热点开机自启动..."
    echo "-------------------------------------"
    local conn_id=$(grep '^id=' "$CONFIG_FILE" | cut -d= -f2-)

    # 1. 开启 NetworkManager 层的自启
    if [ -n "$conn_id" ]; then
        nmcli connection modify "$conn_id" connection.autoconnect yes 2>/dev/null
        echo "✅ NetworkManager 自启动: 已开启 (连接ID: $conn_id)"
    fi

    # 2. 开启 custom systemd 服务的自启 (如果存在)
    if [ -f "/etc/systemd/system/aihitplt_hotspot.service" ]; then
        sudo systemctl enable aihitplt_hotspot.service 2>/dev/null
        echo "✅ 系统服务 (aihitplt_hotspot) 自启动: 已开启"
    fi

    echo ""
    read -p "按回车键返回菜单..."
    show_menu
}

# 关闭开机自启动
disable_autostart() {
    clear
    echo "-------------------------------------"
    echo "  正在关闭热点开机自启动..."
    echo "-------------------------------------"
    local conn_id=$(grep '^id=' "$CONFIG_FILE" | cut -d= -f2-)

    # 1. 关闭 NetworkManager 层的自启
    if [ -n "$conn_id" ]; then
        nmcli connection modify "$conn_id" connection.autoconnect no 2>/dev/null
        echo "✅ NetworkManager 自启动: 已关闭 (连接ID: $conn_id)"
    fi

    # 2. 关闭 custom systemd 服务的自启 (如果存在)
    if [ -f "/etc/systemd/system/aihitplt_hotspot.service" ]; then
        sudo systemctl disable aihitplt_hotspot.service 2>/dev/null
        echo "✅ 系统服务 (aihitplt_hotspot) 自启动: 已关闭"
    fi

    echo ""
    read -p "按回车键返回菜单..."
    show_menu
}

# 显示当前配置
show_config() {
    clear
    echo "-------------------------------------"
    echo "  当前热点配置信息"
    echo "-------------------------------------"
    
    if [ -f "$CONFIG_FILE" ]; then
        echo "配置文件: $CONFIG_FILE"
        echo ""
        echo "主要配置项："
        
        # 显示SSID和ID
        local current_ssid=$(grep '^ssid=' "$CONFIG_FILE" | cut -d= -f2-)
        local current_id=$(grep '^id=' "$CONFIG_FILE" | cut -d= -f2-)
        echo "SSID: $current_ssid"
        echo "ID: $current_id"
        
        # 显示模式
        mode_line=$(grep '^mode=' "$CONFIG_FILE")
        if [ -n "$mode_line" ]; then
            echo "模式: $(echo "$mode_line" | cut -d= -f2-)"
        fi
        
        # 显示频段
        band_line=$(grep '^band=' "$CONFIG_FILE")
        if [ -n "$band_line" ]; then
            echo "频段: $(echo "$band_line" | cut -d= -f2-)"
        fi
        
        # 显示密码（隐藏）
        psk_line=$(grep '^psk=' "$CONFIG_FILE")
        if [ -n "$psk_line" ]; then
            echo "密码: ********"
        fi

        echo "-------------------------------------"
        echo "自启动状态："
        
        # 检查 NM 的自启动状态
        if [ -n "$current_id" ]; then
            autoconnect=$(nmcli -f connection.autoconnect c show "$current_id" 2>/dev/null | awk '{print $2}')
            if [ "$autoconnect" = "yes" ]; then
                echo "NetworkManager 自启动: [开启]"
            else
                echo "NetworkManager 自启动: [关闭]"
            fi
        fi

        # 检查 Systemd 的自启动状态
        if [ -f "/etc/systemd/system/aihitplt_hotspot.service" ]; then
            service_status=$(systemctl is-enabled aihitplt_hotspot.service 2>/dev/null)
            if [ "$service_status" = "enabled" ]; then
                echo "系统服务 (aihitplt_hotspot): [开启]"
            else
                echo "系统服务 (aihitplt_hotspot): [关闭]"
            fi
        fi
        
    else
        echo "配置文件不存在"
    fi
    
    echo "-------------------------------------"
    echo "当前活跃连接:"
    nmcli connection show --active | grep -E "(wifi|热点)" || echo "  无活跃热点"
    
    echo ""
    read -p "按回车键返回菜单..."
    show_menu
}

# 修改 SSID 函数
modify_ssid() {
    clear
    current_ssid=$(get_current_ssid)
    echo "-------------------------------------"
    echo "  修改热点 WiFi 名称"
    echo "-------------------------------------"
    echo "当前热点名称: $current_ssid"
    echo ""

    # 输入新名称
    while true; do
        echo "命名规则："
        echo "- 建议以 'aihitplt' 开头"
        echo "- 例如：aihitplt, aihitplt_01, aihitplt_room1"
        echo "- 支持字母、数字、下划线，长度1-32字符"
        echo ""
        read -p "请输入新热点名称 (输入 0 返回菜单): " new_ssid
        
        # 如果输入 0，返回菜单
        if [ "$new_ssid" = "0" ]; then
            show_menu
            return
        fi

        # 验证输入是否为空
        if [ -z "$new_ssid" ]; then
            echo "错误：热点名称不能为空！"
            continue
        fi

        # 验证名称长度和字符
        if [[ "$new_ssid" =~ ^[a-zA-Z0-9_-]{1,32}$ ]]; then
            # 检查是否与当前名称相同
            if [ "$new_ssid" = "$current_ssid" ]; then
                echo "错误：新名称与当前名称相同！"
                continue
            fi
            break
        else
            echo "错误：热点名称只能包含字母、数字、下划线、连字符，且长度不超过32个字符！"
        fi
    done

    # 备份原配置文件
    backup_file="${CONFIG_FILE}.backup.$(date +%Y%m%d_%H%M%S)"
    cp "$CONFIG_FILE" "$backup_file"
    echo "已备份原配置文件到: $backup_file"

    # 修改 SSID
    sed -i "s/^ssid=.*$/ssid=$new_ssid/" "$CONFIG_FILE"
    
    # 修改 id 字段（如果有且与SSID相关）
    if grep -q "^id=.*$current_ssid" "$CONFIG_FILE"; then
        current_id=$(grep '^id=' "$CONFIG_FILE" | cut -d= -f2-)
        new_id=$(echo "$current_id" | sed "s/$current_ssid/$new_ssid/")
        sed -i "s/^id=$current_id$/id=$new_id/" "$CONFIG_FILE"
    elif grep -q "^id=Hotspot" "$CONFIG_FILE"; then
        # 如果id是Hotspot，也改为新的SSID
        sed -i "s/^id=Hotspot$/id=$new_ssid/" "$CONFIG_FILE"
    fi
    
    echo "已修改热点名称为: $new_ssid"

    # 处理自定义热点服务
    handle_custom_hotspot_service "$new_ssid"

    # 询问是否立即应用更改
    while true; do
        echo ""
        read -p "是否立即应用更改并启动热点？[y-立即应用 / n-暂不应用]: " apply_choice
        
        # 输入验证
        case $apply_choice in
            y|Y|yes|YES)
                apply_choice=1
                break
                ;;
            n|N|no|NO)
                apply_choice=0
                break
                ;;
            *)
                echo "错误：请输入 y 或 n！"
                ;;
        esac
    done

    if [ "$apply_choice" = "1" ]; then
        # 显示操作信息
        echo -e "\n即将执行以下操作："
        echo "1. 重启 NetworkManager 服务"
        echo "2. 启动热点 '$new_ssid'"
        echo -e "\n注意：网络连接可能会暂时中断！"
        
        # 倒计时5秒
        for i in {5..1}; do
            echo -ne "操作将在 ${i} 秒后执行...\r"
            sleep 1
        done
        
        # 执行重启
        echo -e "\n重启 NetworkManager 服务..."
        systemctl restart NetworkManager
        sleep 5
        
        # 启动热点
        echo "启动热点 '$new_ssid'..."
        if start_hotspot "$new_ssid"; then
            echo "✅ 热点启动成功！"
        else
            echo "❌ 热点启动失败，请手动执行以下命令："
            echo "   nmcli connection up \"$new_ssid\""
            echo "或"
            echo "   nmcli device wifi hotspot ifname wlp4s0 ssid \"$new_ssid\" password 12345678"
        fi
        
        # 显示状态验证
        echo ""
        echo "当前活跃连接状态:"
        nmcli connection show --active | grep -E "(wifi|热点)" || echo "  无活跃热点"
        
        echo ""
        echo "✅ 修改完成！"
        echo "新的热点信息："
        echo "名称: $new_ssid"
        echo "请使用新名称搜索并连接热点。"
        
        exit 0
    else
        echo "未应用更改，新热点名称将在下次系统启动或手动执行以下命令后生效："
        echo "  nmcli connection up \"$new_ssid\""
        read -p "按回车键返回菜单..."
        show_menu
    fi
}

# 主菜单函数
show_menu() {
    clear
    current_ssid=$(get_current_ssid)
    echo "-------------------------------------"
    echo "  热点 WiFi 名称修改工具"
    echo "-------------------------------------"
    echo "  当前热点名称: $current_ssid"
    echo "  配置文件: $(basename "$CONFIG_FILE")"
    echo "-------------------------------------"
    echo "  1. 修改热点 WiFi 名称"
    echo "  2. 查看当前配置"
    echo "  3. 启动热点"
    echo "  4. 停止热点"
    echo "  5. 开启热点自启动"
    echo "  6. 关闭热点自启动"
    echo "  7. 退出"
    echo "-------------------------------------"
    read -p "请输入选项 [1-7]: " choice

    case $choice in
        1) modify_ssid ;;
        2) show_config ;;
        3) 
            current_ssid=$(get_current_ssid)
            echo "启动热点: $current_ssid"
            if start_hotspot "$current_ssid"; then
                echo "✅ 热点启动成功！"
            else
                echo "❌ 热点启动失败"
            fi
            read -p "按回车键返回菜单..."
            show_menu 
            ;;
        4) 
            echo "停止热点..."
            nmcli connection down "$(get_current_ssid)" 2>/dev/null || nmcli connection down Hotspot 2>/dev/null
            echo "✅ 热点已停止"
            read -p "按回车键返回菜单..."
            show_menu 
            ;;
        5) enable_autostart ;;
        6) disable_autostart ;;
        7) exit 0 ;;
        *) echo "无效选项，请重新输入！"; sleep 1; show_menu ;;
    esac
}

# 主程序入口
main() {
    # 自动请求sudo权限
    request_sudo "$@"
    
    # 检查配置文件
    check_config_file
    
    # 显示主菜单
    show_menu
}

# 启动主程序
main "$@"
