#!/bin/bash

# 定义录音文件路径
RECORD_FILE="/home/aihit/aihitplt_test/test_$(date +%Y%m%d_%H%M%S).wav"
MUSIC_FILE="/home/aihit/Music/test_music.wav"

# 创建录音目录
mkdir -p /home/aihit/aihitplt_test

# 全局变量存储选择的设备
SELECTED_RECORD_DEVICE=""
SELECTED_PLAY_DEVICE=""
SELECTED_RECORD_NAME=""
SELECTED_PLAY_NAME=""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 获取系统默认设备
get_default_input() {
    # 尝试多种方法获取默认输入设备
    if pactl info 2>/dev/null | grep -q "Default Source"; then
        pactl info | grep "Default Source" | cut -d: -f2 | sed 's/^[[:space:]]*//'
    else
        echo "default"
    fi
}

get_default_output() {
    # 尝试多种方法获取默认输出设备
    if pactl info 2>/dev/null | grep -q "Default Sink"; then
        pactl info | grep "Default Sink" | cut -d: -f2 | sed 's/^[[:space:]]*//'
    else
        echo "default"
    fi
}

# 检测所有音频设备（与系统设置同步）
detect_audio_devices() {
    clear
    echo -e "${CYAN}正在检测音频设备...${NC}"
    echo
    
    # 保存设备到数组
    declare -g INPUT_DEVICES_INDEX=()
    declare -g INPUT_DEVICES_NAME=()
    declare -g INPUT_DEVICES_ID=()
    declare -g OUTPUT_DEVICES_INDEX=()
    declare -g OUTPUT_DEVICES_NAME=()
    declare -g OUTPUT_DEVICES_ID=()
    
    local input_count=0
    local output_count=0
    
    echo -e "${GREEN}=== 输入设备（麦克风）===${NC}"
    echo -e "${YELLOW}方法1: ALSA设备列表${NC}"
    
    # ALSA输入设备
    while IFS= read -r line; do
        if [[ $line =~ card[[:space:]]+([0-9]+):[[:space:]]+.*device[[:space:]]+([0-9]+): ]]; then
            card_num="${BASH_REMATCH[1]}"
            device_num="${BASH_REMATCH[2]}"
            device_name=$(echo "$line" | sed 's/.*device [0-9]*: //')
            
            INPUT_DEVICES_INDEX[$input_count]="plughw:$card_num,$device_num"
            INPUT_DEVICES_NAME[$input_count]="$device_name (ALSA)"
            INPUT_DEVICES_ID[$input_count]="alsa:$card_num,$device_num"
            
            echo -e "  [$((input_count+1))] ${INPUT_DEVICES_INDEX[$input_count]} - ${INPUT_DEVICES_NAME[$input_count]}"
            ((input_count++))
        fi
    done < <(arecord -l 2>/dev/null)
    
    echo -e "\n${YELLOW}方法2: PulseAudio源列表${NC}"
    # PulseAudio输入设备
    if command -v pactl &> /dev/null; then
        while IFS= read -r line; do
            if [[ $line =~ ^[0-9]+\.[[:space:]]+([^[:space:]]+)[[:space:]]+.*$ ]]; then
                source_name="${BASH_REMATCH[1]}"
                if [[ $source_name != "alsa_input"* ]] && [[ $source_name != "monitor" ]]; then
                    # 获取设备描述
                    description=$(pactl list sources | grep -A2 "Name: $source_name" | grep "Description:" | cut -d: -f2 | sed 's/^[[:space:]]*//')
                    if [ -z "$description" ]; then
                        description="$source_name"
                    fi
                    
                    INPUT_DEVICES_INDEX[$input_count]="$source_name"
                    INPUT_DEVICES_NAME[$input_count]="$description (PulseAudio)"
                    INPUT_DEVICES_ID[$input_count]="pulse:$source_name"
                    
                    echo -e "  [$((input_count+1))] ${INPUT_DEVICES_INDEX[$input_count]} - ${INPUT_DEVICES_NAME[$input_count]}"
                    ((input_count++))
                fi
            fi
        done < <(pactl list sources short 2>/dev/null | grep "input")
    fi
    
    echo -e "\n${GREEN}=== 输出设备（扬声器）===${NC}"
    echo -e "${YELLOW}方法1: ALSA设备列表${NC}"
    
    # ALSA输出设备
    while IFS= read -r line; do
        if [[ $line =~ card[[:space:]]+([0-9]+):[[:space:]]+.*device[[:space:]]+([0-9]+): ]]; then
            card_num="${BASH_REMATCH[1]}"
            device_num="${BASH_REMATCH[2]}"
            device_name=$(echo "$line" | sed 's/.*device [0-9]*: //')
            
            OUTPUT_DEVICES_INDEX[$output_count]="plughw:$card_num,$device_num"
            OUTPUT_DEVICES_NAME[$output_count]="$device_name (ALSA)"
            OUTPUT_DEVICES_ID[$output_count]="alsa:$card_num,$device_num"
            
            echo -e "  [$((output_count+1))] ${OUTPUT_DEVICES_INDEX[$output_count]} - ${OUTPUT_DEVICES_NAME[$output_count]}"
            ((output_count++))
        fi
    done < <(aplay -l 2>/dev/null)
    
    echo -e "\n${YELLOW}方法2: PulseAudio接收器列表${NC}"
    # PulseAudio输出设备
    if command -v pactl &> /dev/null; then
        while IFS= read -r line; do
            if [[ $line =~ ^[0-9]+\.[[:space:]]+([^[:space:]]+)[[:space:]]+.*$ ]]; then
                sink_name="${BASH_REMATCH[1]}"
                if [[ $sink_name != "alsa_output"* ]]; then
                    # 获取设备描述
                    description=$(pactl list sinks | grep -A2 "Name: $sink_name" | grep "Description:" | cut -d: -f2 | sed 's/^[[:space:]]*//')
                    if [ -z "$description" ]; then
                        description="$sink_name"
                    fi
                    
                    OUTPUT_DEVICES_INDEX[$output_count]="$sink_name"
                    OUTPUT_DEVICES_NAME[$output_count]="$description (PulseAudio)"
                    OUTPUT_DEVICES_ID[$output_count]="pulse:$sink_name"
                    
                    echo -e "  [$((output_count+1))] ${OUTPUT_DEVICES_INDEX[$output_count]} - ${OUTPUT_DEVICES_NAME[$output_count]}"
                    ((output_count++))
                fi
            fi
        done < <(pactl list sinks short 2>/dev/null)
    fi
    
    # 显示系统默认设备
    echo -e "\n${CYAN}=== 系统默认设备 ===${NC}"
    echo -e "默认输入: $(get_default_input)"
    echo -e "默认输出: $(get_default_output)"
    
    INPUT_COUNT=$input_count
    OUTPUT_COUNT=$output_count
    
    echo
}

# 选择录音设备
select_record_device() {
    clear
    echo -e "${CYAN}选择录音设备（麦克风）${NC}"
    echo
    
    detect_audio_devices
    
    if [ $INPUT_COUNT -eq 0 ]; then
        echo -e "${RED}未检测到任何输入设备！${NC}"
        echo -e "${YELLOW}请检查：${NC}"
        echo "1. 麦克风是否已连接"
        echo "2. 系统音频设置中是否识别到设备"
        echo "3. 尝试重新拔插设备"
        read -p "按回车键返回..." dummy
        return
    fi
    
    echo -e "${GREEN}请选择输入设备：${NC}"
    
    # 显示所有设备
    for ((i=0; i<INPUT_COUNT; i++)); do
        current=$((i+1))
        echo -e "  [$current] ${INPUT_DEVICES_NAME[$i]}"
        echo -e "      设备ID: ${INPUT_DEVICES_INDEX[$i]}"
    done
    
    echo -e "  [0] 使用系统默认设备"
    
    while true; do
        read -p "请输入选择 [0-$INPUT_COUNT]: " choice
        
        if [[ $choice == "0" ]]; then
            SELECTED_RECORD_DEVICE="default"
            SELECTED_RECORD_NAME="系统默认输入设备"
            echo -e "${GREEN}已选择: 系统默认输入设备${NC}"
            break
        elif [[ $choice =~ ^[0-9]+$ ]] && [ $choice -ge 1 ] && [ $choice -le $INPUT_COUNT ]; then
            idx=$((choice-1))
            SELECTED_RECORD_DEVICE="${INPUT_DEVICES_INDEX[$idx]}"
            SELECTED_RECORD_NAME="${INPUT_DEVICES_NAME[$idx]}"
            echo -e "${GREEN}已选择: ${SELECTED_RECORD_NAME}${NC}"
            echo -e "设备ID: ${SELECTED_RECORD_DEVICE}"
            break
        else
            echo -e "${RED}无效选择，请重新输入${NC}"
        fi
    done
    
    # 测试设备是否可用
    echo
    echo -e "${YELLOW}正在测试设备是否可用...${NC}"
    if [[ $SELECTED_RECORD_DEVICE == "default" ]] || [[ $SELECTED_RECORD_DEVICE == pulse:* ]]; then
        echo -e "${GREEN}PulseAudio设备，跳过直接测试${NC}"
    else
        # 尝试录音1秒测试
        test_file="/tmp/test_rec_$(date +%s).wav"
        timeout 2 arecord -D "$SELECTED_RECORD_DEVICE" -f S16_LE -r 16000 -c 1 -d 1 "$test_file" &>/dev/null
        
        if [ $? -eq 0 ] && [ -f "$test_file" ]; then
            echo -e "${GREEN}✓ 设备测试通过${NC}"
            rm -f "$test_file"
        else
            echo -e "${YELLOW}⚠ 设备测试失败，可能仍可使用${NC}"
        fi
    fi
    
    echo
    read -p "按回车键继续..." dummy
}

# 选择播放设备
select_play_device() {
    clear
    echo -e "${CYAN}选择播放设备（扬声器）${NC}"
    echo
    
    detect_audio_devices
    
    if [ $OUTPUT_COUNT -eq 0 ]; then
        echo -e "${RED}未检测到任何输出设备！${NC}"
        echo -e "${YELLOW}请检查：${NC}"
        echo "1. 扬声器是否已连接"
        echo "2. 系统音频设置中是否识别到设备"
        echo "3. 尝试重新拔插设备"
        read -p "按回车键返回..." dummy
        return
    fi
    
    echo -e "${GREEN}请选择输出设备：${NC}"
    
    # 显示所有设备
    for ((i=0; i<OUTPUT_COUNT; i++)); do
        current=$((i+1))
        echo -e "  [$current] ${OUTPUT_DEVICES_NAME[$i]}"
        echo -e "      设备ID: ${OUTPUT_DEVICES_INDEX[$i]}"
    done
    
    echo -e "  [0] 使用系统默认设备"
    
    while true; do
        read -p "请输入选择 [0-$OUTPUT_COUNT]: " choice
        
        if [[ $choice == "0" ]]; then
            SELECTED_PLAY_DEVICE="default"
            SELECTED_PLAY_NAME="系统默认输出设备"
            echo -e "${GREEN}已选择: 系统默认输出设备${NC}"
            break
        elif [[ $choice =~ ^[0-9]+$ ]] && [ $choice -ge 1 ] && [ $choice -le $OUTPUT_COUNT ]; then
            idx=$((choice-1))
            SELECTED_PLAY_DEVICE="${OUTPUT_DEVICES_INDEX[$idx]}"
            SELECTED_PLAY_NAME="${OUTPUT_DEVICES_NAME[$idx]}"
            echo -e "${GREEN}已选择: ${SELECTED_PLAY_NAME}${NC}"
            echo -e "设备ID: ${SELECTED_PLAY_DEVICE}"
            break
        else
            echo -e "${RED}无效选择，请重新输入${NC}"
        fi
    done
    
    # 测试设备是否可用
    echo
    echo -e "${YELLOW}正在测试设备是否可用...${NC}"
    if [[ $SELECTED_PLAY_DEVICE == "default" ]] || [[ $SELECTED_PLAY_DEVICE == pulse:* ]]; then
        echo -e "${GREEN}PulseAudio设备，跳过直接测试${NC}"
    else
        # 生成测试音并播放
        echo -e "播放测试音..."
        if speaker-test -D "$SELECTED_PLAY_DEVICE" -t sine -f 1000 -l 1 &>/dev/null; then
            echo -e "${GREEN}✓ 设备测试通过${NC}"
        else
            echo -e "${YELLOW}⚠ 设备测试失败，可能仍可使用${NC}"
        fi
    fi
    
    echo
    read -p "按回车键继续..." dummy
}

# 检查并释放音频设备
release_audio_device() {
    echo -e "${YELLOW}检查音频设备占用情况...${NC}"
    
    # 如果是PulseAudio设备，确保PulseAudio运行
    if [[ $SELECTED_RECORD_DEVICE == pulse:* ]] || [[ $SELECTED_RECORD_DEVICE == "default" ]]; then
        if ! pgrep pulseaudio >/dev/null; then
            echo -e "${GREEN}启动PulseAudio...${NC}"
            pulseaudio --start
            sleep 1
        fi
        return 0
    fi
    
    # 对于ALSA设备，检查占用
    if [[ $SELECTED_RECORD_DEVICE =~ plughw:([0-9]+),([0-9]+) ]]; then
        card=${BASH_REMATCH[1]}
        device=${BASH_REMATCH[2]}
        
        # 检查设备文件是否存在
        dev_path="/dev/snd/pcmC${card}D${device}c"
        if [ -e "$dev_path" ]; then
            # 查找占用进程
            occupied=$(sudo fuser -v "$dev_path" 2>/dev/null | grep -v "COMMAND")
            
            if [ -n "$occupied" ]; then
                echo -e "${RED}发现占用音频设备的进程：${NC}"
                echo "$occupied"
                echo
                
                read -p "是否终止这些进程？[y/N]: " kill_choice
                if [[ $kill_choice =~ [yY] ]]; then
                    sudo fuser -k "$dev_path"
                    echo -e "${GREEN}已终止占用进程${NC}"
                    sleep 1
                else
                    echo -e "${YELLOW}请手动关闭占用音频设备的程序后重试${NC}"
                    return 1
                fi
            else
                echo -e "${GREEN}音频设备未被占用${NC}"
            fi
        fi
        
        # 暂停PulseAudio以防止冲突
        if pgrep pulseaudio >/dev/null; then
            echo -e "${YELLOW}暂停PulseAudio以避免冲突...${NC}"
            pulseaudio --kill
            sleep 1
        fi
    fi
    
    return 0
}

# 恢复PulseAudio
restore_pulseaudio() {
    if ! pgrep pulseaudio >/dev/null; then
        echo -e "${GREEN}恢复PulseAudio服务...${NC}"
        pulseaudio --start
        sleep 1
    fi
}

# 简单的进度条显示函数
show_progress() {
    local duration=$1
    local message=$2
    local bar_length=50
    
    echo -ne "${message} ["
    
    for i in $(seq 1 $duration); do
        # 计算进度百分比
        local progress=$((i * bar_length / duration))
        local remaining=$((bar_length - progress))
        
        # 绘制进度条
        printf "%${progress}s" | tr ' ' '█'
        printf "%${remaining}s" | tr ' ' '░'
        
        echo -ne "] ${i}/${duration}秒\r"
        sleep 1
    done
    
    echo
}

# 测试麦克风 - 修复版
test_microphone() {
    clear
    echo -e "${CYAN}麦克风测试${NC}"
    echo
    
    # 如果没有选择设备，先选择
    if [ -z "$SELECTED_RECORD_DEVICE" ]; then
        echo -e "${YELLOW}未选择录音设备，请先选择设备${NC}"
        select_record_device
        if [ -z "$SELECTED_RECORD_DEVICE" ]; then
            return
        fi
    fi
    
    echo -e "${GREEN}设备信息：${NC}"
    echo -e "  名称: ${SELECTED_RECORD_NAME}"
    echo -e "  设备ID: ${SELECTED_RECORD_DEVICE}"
    echo
    
    # 设置录音参数
    local record_seconds=5
    local sample_rate=16000
    local channels=1
    
    echo -e "将录制 ${record_seconds} 秒音频到："
    echo -e "  $RECORD_FILE"
    echo
    
    # 检查并释放设备
    if ! release_audio_device; then
        echo -e "${RED}无法释放音频设备${NC}"
        read -p "按回车键返回..." dummy
        return
    fi
    
    echo -e "${YELLOW}准备录音（${record_seconds}秒）...${NC}"
    echo -e "${GREEN}请对着麦克风说话${NC}"
    
    # 先完成倒计时，再开始录音
    echo -e "${CYAN}倒计时开始：${NC}"
    for i in {5..1}; do
        echo -e "${YELLOW}$i...${NC}"
        sleep 1
    done
    
    echo -e "${GREEN}开始录音！${NC}"
    echo
    
    # 根据设备类型选择不同的录音方法
    if [[ $SELECTED_RECORD_DEVICE == "default" ]] || [[ $SELECTED_RECORD_DEVICE == pulse:* ]]; then
        # 使用PulseAudio录音
        echo -e "${YELLOW}使用PulseAudio录音...${NC}"
        
        if [[ $SELECTED_RECORD_DEVICE == "default" ]]; then
            pa_device=$(get_default_input)
        else
            pa_device=${SELECTED_RECORD_DEVICE#pulse:}
        fi
        
        # 使用parec录音 - 确保只录5秒
        if command -v parec &>/dev/null; then
            echo -e "正在录音（5秒）..."
            
            # 在后台开始录音
            timeout $((record_seconds+1)) parec --device="$pa_device" --format=s16le --rate=$sample_rate --channels=$channels "$RECORD_FILE" &
            local record_pid=$!
            
            # 显示进度条
            show_progress $record_seconds "录音进度"
            
            # 等待录音完成
            wait $record_pid 2>/dev/null
            
        else
            # 如果没有parec，使用arecord
            echo -e "${YELLOW}使用arecord录音...${NC}"
            echo -e "正在录音（5秒）..."
            
            # 显示进度条
            (
                show_progress $record_seconds "录音进度"
            ) &
            local progress_pid=$!
            
            # 开始录音
            arecord -f S16_LE -r $sample_rate -c $channels -d $record_seconds "$RECORD_FILE" 2>/dev/null
            
            # 等待进度条显示完成
            wait $progress_pid 2>/dev/null
        fi
    else
        # 使用ALSA直接录音
        echo -e "${YELLOW}使用ALSA录音...${NC}"
        
        # 尝试不同的参数
        local success=0
        for format in "S16_LE" "S32_LE"; do
            for rate in 16000 44100 48000; do
                echo -e "尝试参数: $format, ${rate}Hz, ${channels}声道..."
                
                # 显示进度条
                (
                    show_progress $record_seconds "录音进度"
                ) &
                local progress_pid=$!
                
                # 开始录音
                arecord -D "$SELECTED_RECORD_DEVICE" -f "$format" -r $rate -c $channels -d $record_seconds "$RECORD_FILE" 2>/dev/null
                local record_result=$?
                
                # 等待进度条显示完成
                wait $progress_pid 2>/dev/null
                
                if [ $record_result -eq 0 ] && [ -f "$RECORD_FILE" ] && [ -s "$RECORD_FILE" ]; then
                    echo -e "${GREEN}✓ 录音成功${NC}"
                    success=1
                    break 2
                else
                    echo -e "${YELLOW}参数失败，尝试其他参数...${NC}"
                    rm -f "$RECORD_FILE"
                fi
            done
        done
        
        if [ $success -eq 0 ]; then
            echo -e "${RED}所有参数尝试均失败${NC}"
        fi
    fi
    
    # 恢复PulseAudio
    restore_pulseaudio
    
    # 检查录音结果
    if [ -f "$RECORD_FILE" ] && [ -s "$RECORD_FILE" ]; then
        file_size=$(du -h "$RECORD_FILE" | cut -f1)
        
        # 使用soxi检查音频时长，如果不可用则使用替代方法
        if command -v soxi &>/dev/null; then
            duration=$(soxi -D "$RECORD_FILE" 2>/dev/null)
        else
            # 简单的替代方法：根据文件大小估算时长
            local bytes_per_sec=$((sample_rate * 2 * channels))  # 16-bit = 2字节
            local file_bytes=$(stat -c%s "$RECORD_FILE" 2>/dev/null || wc -c < "$RECORD_FILE")
            duration=$(echo "scale=2; $file_bytes / $bytes_per_sec" | bc 2>/dev/null || echo "未知")
        fi
        
        echo -e "\n${GREEN}✓ 录音完成！${NC}"
        echo -e "  文件大小: $file_size"
        echo -e "  录音时长: ${duration}秒"
        
        # 询问是否播放
        while true; do
            echo
            echo -e "${CYAN}是否播放录音？${NC}"
            echo -e "  1. 立即播放"
            echo -e "  2. 稍后播放"
            echo -e "  3. 返回主菜单"
            
            read -p "请选择 [1-3]: " play_choice
            
            case $play_choice in
                1)
                    play_recording
                    ask_delete
                    return
                    ;;
                2)
                    echo -e "${YELLOW}录音文件已保存: $RECORD_FILE${NC}"
                    return
                    ;;
                3)
                    return
                    ;;
                *)
                    echo -e "${RED}无效选择！${NC}"
                    ;;
            esac
        done
    else
        echo -e "\n${RED}✗ 录音失败！${NC}"
        echo -e "${YELLOW}可能的原因：${NC}"
        echo "1. 麦克风未连接或已断开"
        echo "2. 麦克风被其他程序占用"
        echo "3. 麦克风权限问题"
        echo "4. 设备参数不匹配"
        
        echo -e "\n${YELLOW}建议：${NC}"
        echo "1. 检查系统设置中的音频输入设备"
        echo "2. 尝试重新拔插麦克风"
        echo "3. 选择其他录音设备"
        
        read -p "按回车键返回..." dummy
    fi
}

# 播放录音
play_recording() {
    echo -e "\n${CYAN}播放录音${NC}"
    
    if [ ! -f "$RECORD_FILE" ]; then
        echo -e "${RED}录音文件不存在！${NC}"
        read -p "按回车键返回..." dummy
        return
    fi
    
    # 如果没有选择播放设备，先选择
    if [ -z "$SELECTED_PLAY_DEVICE" ]; then
        echo -e "${YELLOW}未选择播放设备，请先选择设备${NC}"
        select_play_device
        if [ -z "$SELECTED_PLAY_DEVICE" ]; then
            return
        fi
    fi
    
    echo -e "${GREEN}播放设备：${NC}"
    echo -e "  名称: ${SELECTED_PLAY_NAME}"
    echo -e "  设备ID: ${SELECTED_PLAY_DEVICE}"
    echo -e "  文件: $(basename "$RECORD_FILE")"
    echo
    
    # 根据设备类型选择播放方法
    if [[ $SELECTED_PLAY_DEVICE == "default" ]] || [[ $SELECTED_PLAY_DEVICE == pulse:* ]]; then
        # 使用PulseAudio播放
        echo -e "${YELLOW}使用PulseAudio播放...${NC}"
        
        if [[ $SELECTED_PLAY_DEVICE == "default" ]]; then
            pa_device=$(get_default_output)
        else
            pa_device=${SELECTED_PLAY_DEVICE#pulse:}
        fi
        
        # 使用pacat播放
        if command -v pacat &>/dev/null; then
            echo -e "正在播放..."
            pacat --device="$pa_device" "$RECORD_FILE"
        else
            # 如果没有pacat，使用aplay
            echo -e "正在播放..."
            aplay -q "$RECORD_FILE"
        fi
    else
        # 使用ALSA直接播放
        echo -e "${YELLOW}使用ALSA播放...${NC}"
        echo -e "正在播放..."
        aplay -D "$SELECTED_PLAY_DEVICE" -q "$RECORD_FILE"
    fi
    
    if [ $? -eq 0 ]; then
        echo -e "\n${GREEN}✓ 播放完成！${NC}"
    else
        echo -e "\n${RED}✗ 播放失败！${NC}"
    fi
}

# 测试扬声器
test_speaker() {
    clear
    echo -e "${CYAN}扬声器测试${NC}"
    echo
    
    # 如果没有选择播放设备，先选择
    if [ -z "$SELECTED_PLAY_DEVICE" ]; then
        echo -e "${YELLOW}未选择播放设备，请先选择设备${NC}"
        select_play_device
        if [ -z "$SELECTED_PLAY_DEVICE" ]; then
            return
        fi
    fi
    
    echo -e "${GREEN}设备信息：${NC}"
    echo -e "  名称: ${SELECTED_PLAY_NAME}"
    echo -e "  设备ID: ${SELECTED_PLAY_DEVICE}"
    echo
    
    # 测试选项
    echo -e "${CYAN}选择测试方式：${NC}"
    echo -e "  1. 播放测试音频文件"
    echo -e "  2. 播放系统提示音"
    echo -e "  3. 播放频率扫描音"
    echo -e "  4. 声道测试（左/右）"
    
    read -p "请选择 [1-4]: " test_choice
    
    case $test_choice in
        1)
            test_with_audio_file
            ;;
        2)
            test_with_beep
            ;;
        3)
            test_frequency_sweep
            ;;
        4)
            test_channels
            ;;
        *)
            echo -e "${RED}无效选择！${NC}"
            ;;
    esac
    
    read -p "按回车键返回主菜单..." dummy
}

# 使用音频文件测试
test_with_audio_file() {
    if [ ! -f "$MUSIC_FILE" ]; then
        echo -e "${YELLOW}测试音频文件不存在: $MUSIC_FILE${NC}"
        echo -e "请将测试音频文件（WAV格式）放在该位置"
        test_with_beep
        return
    fi
    
    echo -e "${GREEN}播放测试音频文件...${NC}"
    
    if [[ $SELECTED_PLAY_DEVICE == "default" ]] || [[ $SELECTED_PLAY_DEVICE == pulse:* ]]; then
        if [[ $SELECTED_PLAY_DEVICE == "default" ]]; then
            pa_device=$(get_default_output)
        else
            pa_device=${SELECTED_PLAY_DEVICE#pulse:}
        fi
        
        if command -v pacat &>/dev/null; then
            pacat --device="$pa_device" "$MUSIC_FILE"
        else
            aplay -q "$MUSIC_FILE"
        fi
    else
        aplay -D "$SELECTED_PLAY_DEVICE" -q "$MUSIC_FILE"
    fi
}

# 使用提示音测试
test_with_beep() {
    echo -e "${GREEN}播放提示音测试...${NC}"
    
    for i in 1 2 3; do
        echo -e "${YELLOW}提示音 $i (1000Hz, 0.5秒)${NC}"
        
        if [[ $SELECTED_PLAY_DEVICE == "default" ]] || [[ $SELECTED_PLAY_DEVICE == pulse:* ]]; then
            speaker-test -t sine -f 1000 -l 1 -c 2 &>/dev/null
        else
            speaker-test -D "$SELECTED_PLAY_DEVICE" -t sine -f 1000 -l 1 -c 2 &>/dev/null
        fi
        
        sleep 0.3
    done
    
    echo -e "${GREEN}提示音测试完成！${NC}"
}

# 频率扫描测试
test_frequency_sweep() {
    echo -e "${GREEN}频率扫描测试（从100Hz到5000Hz）${NC}"
    echo -e "${YELLOW}按Ctrl+C停止测试${NC}"
    
    frequencies=(100 250 500 750 1000 1500 2000 3000 4000 5000)
    
    for freq in "${frequencies[@]}"; do
        echo -e "播放 ${freq}Hz..."
        
        if [[ $SELECTED_PLAY_DEVICE == "default" ]] || [[ $SELECTED_PLAY_DEVICE == pulse:* ]]; then
            speaker-test -t sine -f $freq -l 1 &>/dev/null
        else
            speaker-test -D "$SELECTED_PLAY_DEVICE" -t sine -f $freq -l 1 &>/dev/null
        fi
        
        sleep 0.5
    done
    
    echo -e "${GREEN}频率扫描完成！${NC}"
}

# 声道测试
test_channels() {
    echo -e "${GREEN}声道测试${NC}"
    
    echo -e "${YELLOW}左声道测试...${NC}"
    if [[ $SELECTED_PLAY_DEVICE == "default" ]] || [[ $SELECTED_PLAY_DEVICE == pulse:* ]]; then
        speaker-test -t sine -f 1000 -l 2 -c 2 -s 1 &>/dev/null
    else
        speaker-test -D "$SELECTED_PLAY_DEVICE" -t sine -f 1000 -l 2 -c 2 -s 1 &>/dev/null
    fi
    
    sleep 0.5
    
    echo -e "${YELLOW}右声道测试...${NC}"
    if [[ $SELECTED_PLAY_DEVICE == "default" ]] || [[ $SELECTED_PLAY_DEVICE == pulse:* ]]; then
        speaker-test -t sine -f 1500 -l 2 -c 2 -s 2 &>/dev/null
    else
        speaker-test -D "$SELECTED_PLAY_DEVICE" -t sine -f 1500 -l 2 -c 2 -s 2 &>/dev/null
    fi
    
    echo -e "${GREEN}声道测试完成！${NC}"
}

# 询问是否删除录音
ask_delete() {
    if [ ! -f "$RECORD_FILE" ]; then
        return
    fi
    
    while true; do
        echo
        echo -e "${CYAN}是否删除录音文件？${NC}"
        echo -e "  文件: $(basename "$RECORD_FILE")"
        echo -e "  大小: $(du -h "$RECORD_FILE" | cut -f1)"
        echo
        echo -e "  1. 删除文件"
        echo -e "  2. 保留文件"
        
        read -p "请选择 [1-2]: " delete_choice
        
        case $delete_choice in
            1)
                rm -v "$RECORD_FILE"
                echo -e "${GREEN}文件已删除${NC}"
                sleep 1
                return
                ;;
            2)
                echo -e "${YELLOW}文件已保留: $RECORD_FILE${NC}"
                return
                ;;
            *)
                echo -e "${RED}无效输入！${NC}"
                ;;
        esac
    done
}

# 显示系统音频状态
show_system_status() {
    clear
    echo -e "${CYAN}系统音频状态${NC}"
    echo
    
    # 显示PulseAudio状态
    if command -v pactl &>/dev/null; then
        echo -e "${GREEN}PulseAudio状态：${NC}"
        if pgrep pulseaudio >/dev/null; then
            echo -e "  ✓ 正在运行"
            echo -e "  默认输入: $(get_default_input)"
            echo -e "  默认输出: $(get_default_output)"
        else
            echo -e "  ✗ 未运行"
        fi
        echo
    fi
    
    # 显示ALSA设备
    echo -e "${GREEN}ALSA设备：${NC}"
    echo -e "${YELLOW}输入设备：${NC}"
    arecord -l 2>/dev/null || echo "  无输入设备"
    echo
    echo -e "${YELLOW}输出设备：${NC}"
    aplay -l 2>/dev/null || echo "  无输出设备"
    
    echo
    read -p "按回车键返回..." dummy
}

# 检查系统音频服务
check_audio_services() {
    clear
    echo -e "${CYAN}检查音频服务${NC}"
    echo
    
    # 检查PulseAudio
    echo -e "${GREEN}1. PulseAudio服务：${NC}"
    if pgrep pulseaudio >/dev/null; then
        echo -e "  ✓ 正在运行 (PID: $(pgrep pulseaudio))"
        echo -e "  重启命令: pulseaudio -k && pulseaudio --start"
    else
        echo -e "  ✗ 未运行"
        echo -e "  启动命令: pulseaudio --start"
    fi
    
    echo
    
    # 检查ALSA
    echo -e "${GREEN}2. ALSA状态：${NC}"
    if [ -c "/dev/snd/controlC0" ]; then
        echo -e "  ✓ ALSA驱动已加载"
    else
        echo -e "  ✗ ALSA驱动未加载"
    fi
    
    echo
    
    # 检查用户权限
    echo -e "${GREEN}3. 用户权限检查：${NC}"
    if groups $USER | grep -q audio; then
        echo -e "  ✓ 用户在audio组中"
    else
        echo -e "  ✗ 用户不在audio组中"
        echo -e "  添加命令: sudo usermod -a -G audio $USER"
    fi
    
    echo
    
    # 提供修复选项
    echo -e "${CYAN}修复选项：${NC}"
    echo -e "  1. 重启PulseAudio"
    echo -e "  2. 重新加载ALSA"
    echo -e "  3. 添加到audio组（需要sudo密码）"
    echo -e "  4. 返回"
    
    read -p "请选择 [1-4]: " fix_choice
    
    case $fix_choice in
        1)
            echo -e "${YELLOW}重启PulseAudio...${NC}"
            pulseaudio -k
            sleep 1
            pulseaudio --start
            sleep 1
            echo -e "${GREEN}完成！${NC}"
            ;;
        2)
            echo -e "${YELLOW}重新加载ALSA驱动...${NC}"
            sudo alsa force-reload
            sleep 1
            echo -e "${GREEN}完成！${NC}"
            ;;
        3)
            echo -e "${YELLOW}添加用户到audio组...${NC}"
            sudo usermod -a -G audio $USER
            echo -e "${GREEN}完成！需要重新登录生效${NC}"
            ;;
        4)
            return
            ;;
    esac
    
    read -p "按回车键继续..." dummy
}

# 主菜单函数
main_menu() {
    while true; do
        clear
        echo -e "${CYAN}╔═══════════════════════════════════════╗${NC}"
        echo -e "${CYAN}║      aihitplt 音频测试工具           ║${NC}"
        echo -e "${CYAN}╠═══════════════════════════════════════╣${NC}"
        echo -e "${CYAN}║${NC} 当前设备状态："
        
        if [ -n "$SELECTED_RECORD_NAME" ]; then
            echo -e "${CYAN}║${NC}  🎤 输入: ${GREEN}$SELECTED_RECORD_NAME${NC}"
        else
            echo -e "${CYAN}║${NC}  🎤 输入: ${YELLOW}未选择${NC}"
        fi
        
        if [ -n "$SELECTED_PLAY_NAME" ]; then
            echo -e "${CYAN}║${NC}  🔊 输出: ${GREEN}$SELECTED_PLAY_NAME${NC}"
        else
            echo -e "${CYAN}║${NC}  🔊 输出: ${YELLOW}未选择${NC}"
        fi
        
        if [ -f "$RECORD_FILE" ]; then
            echo -e "${CYAN}║${NC}  💾 录音文件: $(basename "$RECORD_FILE")"
        fi
        
        echo -e "${CYAN}╠═══════════════════════════════════════╣${NC}"
        echo -e "${CYAN}║${NC}  1. 选择录音设备（麦克风）"
        echo -e "${CYAN}║${NC}  2. 选择播放设备（扬声器）"
        echo -e "${CYAN}║${NC}  3. 测试麦克风（录音测试）"
        echo -e "${CYAN}║${NC}  4. 测试扬声器（播放测试）"
        echo -e "${CYAN}║${NC}  5. 检测所有音频设备"
        echo -e "${CYAN}║${NC}  6. 显示系统音频状态"
        echo -e "${CYAN}║${NC}  7. 检查音频服务"
        echo -e "${CYAN}║${NC}  8. 删除录音文件"
        echo -e "${CYAN}║${NC}  9. 退出"
        echo -e "${CYAN}╚═══════════════════════════════════════╝${NC}"
        echo
        
        read -p "请选择操作 [1-9]: " choice

        case $choice in
            1) select_record_device ;;
            2) select_play_device ;;
            3) test_microphone ;;
            4) test_speaker ;;
            5) detect_audio_devices; read -p "按回车键继续..." dummy ;;
            6) show_system_status ;;
            7) check_audio_services ;;
            8) 
                if [ -f "$RECORD_FILE" ]; then
                    rm -v "$RECORD_FILE"
                    echo -e "${GREEN}文件已删除${NC}"
                else
                    echo -e "${YELLOW}没有找到录音文件${NC}"
                fi
                sleep 1
                ;;
            9) 
                restore_pulseaudio
                echo -e "${GREEN}退出程序，再见！${NC}"
                exit 0
                ;;
            *) 
                echo -e "${RED}无效输入，请重新选择！${NC}"
                sleep 1
                ;;
        esac
    done
}

# 启动脚本
echo -e "${CYAN}音频测试工具启动...${NC}"
echo -e "${YELLOW}检测系统音频设备...${NC}"

# 确保PulseAudio运行
if ! pgrep pulseaudio >/dev/null; then
    echo -e "${GREEN}启动PulseAudio...${NC}"
    pulseaudio --start
    sleep 1
fi

# 自动选择系统默认设备
SELECTED_RECORD_DEVICE="default"
SELECTED_RECORD_NAME="系统默认输入设备"
SELECTED_PLAY_DEVICE="default"
SELECTED_PLAY_NAME="系统默认输出设备"

echo -e "${GREEN}已选择系统默认设备${NC}"
sleep 1

# 启动主菜单
main_menu
