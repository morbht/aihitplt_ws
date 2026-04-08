#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 麦克风阵列测试程序 – 布局精简版

import tkinter as tk
from tkinter import ttk, messagebox
import serial.tools.list_ports
import threading
import subprocess
import os
import time
import yaml
import glob
import psutil
import numpy as np
from pathlib import Path
import signal

class MicTester:
    def __init__(self, root):
        self.root = root
        self.root.title("麦克风阵列测试")
        self.root.geometry("500x500")   # 高度略微压缩

        # 串口相关
        self.serial_port = None
        self.serial_connected = False
        self.serial_thread = None
        self.stop_serial_thread = False

        # ROS 相关
        self.ros_process = None
        self.ros_running = False
        self.ros_pid = None
        self.gnome_terminal_pid = None  # 保存gnome-terminal的进程ID

        # 录音状态
        self.recording = False
        self.recording_time = 0
        self.record_duration = 5

        # 协议常量
        self.SYNC_HEAD = 0xA5
        self.MSG_TYPE_FEEDBACK = 0x04

        # 配置文件路径
        self.pkg_name = "aihitplt_hardware_test"
        self.config_dir = os.path.expanduser(f"/home/aihit/aihitplt_ws/src/{self.pkg_name}/config")
        self.config_file = os.path.join(self.config_dir, "mic_array_port.yaml")

        # 音频保存路径
        self.audio_save_dir = os.path.expanduser(f"~/aihitplt_ws/src/{self.pkg_name}/audio")
        Path(self.audio_save_dir).mkdir(parents=True, exist_ok=True)

        # 创建界面
        self.create_widgets()

        # 加载保存的串口
        self.load_saved_port()

        # 绑定关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    # -------------------- 界面生成 --------------------
    def create_widgets(self):
        main_frame = ttk.Frame(self.root, padding="15")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 1. 串口连接
        frame1 = ttk.LabelFrame(main_frame, text="设备连接", padding=10)
        frame1.pack(fill=tk.X, pady=(0, 5))

        # 串口选择部分 - 单行布局
        port_select_frame = ttk.Frame(frame1)
        port_select_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(port_select_frame, text="设备:").pack(side=tk.LEFT, padx=(0, 5))
        
        # 缩短设备选择栏
        self.port_combo = ttk.Combobox(port_select_frame, width=25, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(0, 15))
        
        # 刷新按钮 - 与设备选择在同一行
        self.refresh_btn = ttk.Button(
            port_select_frame, 
            text="刷新", 
            command=self.refresh_ports,
            width=8
        )
        self.refresh_btn.pack(side=tk.LEFT, padx=(0, 15))
        
        # 连接按钮 - 与设备选择在同一行
        self.connect_btn = ttk.Button(
            port_select_frame, 
            text="连接",
            command=self.toggle_serial_connection,
            width=8
        )
        self.connect_btn.pack(side=tk.LEFT)

        # 2. 设备信息
        frame2 = ttk.LabelFrame(main_frame, text="设备信息", padding=10)
        frame2.pack(fill=tk.BOTH, expand=True, pady=(0, 5))
        self.info_text = tk.Text(frame2, height=6, width=50, state=tk.DISABLED)
        self.info_text.pack(fill=tk.BOTH, expand=True)

        # 3. 测试工具
        frame3 = ttk.LabelFrame(main_frame, text="测试工具", padding=10)
        frame3.pack(fill=tk.X, pady=(0, 5))
        tool_frame = ttk.Frame(frame3)
        tool_frame.pack(fill=tk.X, expand=True)
        tool_frame.columnconfigure(0, weight=1)
        tool_frame.columnconfigure(1, weight=1)
        tool_frame.columnconfigure(2, weight=1)

        self.record_btn = ttk.Button(tool_frame, text="开始录音(5秒)", command=self.start_recording, state="disabled")
        self.record_btn.grid(row=0, column=0, padx=5, pady=10, sticky="ew")
        self.get_audio_btn = ttk.Button(tool_frame, text="获取音频", command=self.get_audio_files, state="disabled")
        self.get_audio_btn.grid(row=0, column=1, padx=5, pady=10, sticky="ew")
        self.clean_cache_btn = ttk.Button(tool_frame, text="清理缓存", command=self.clean_audio_cache, state="disabled")
        self.clean_cache_btn.grid(row=0, column=2, padx=5, pady=10, sticky="ew")

        self.record_status_var = tk.StringVar(value="就绪")
        ttk.Label(tool_frame, textvariable=self.record_status_var).grid(row=1, column=0, columnspan=3, pady=(5, 0))

        # 4. 底部合并区域：串口配置 + ROS 启动（上下紧贴，按钮同高）
        bottom_frame = ttk.Frame(main_frame)
        bottom_frame.pack(fill=tk.X, pady=(0, 5))

        # 左侧：串口配置
        frame4 = ttk.LabelFrame(bottom_frame, text="设备配置", padding=10)
        frame4.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        self.save_btn = ttk.Button(frame4, text="保存设备号", command=self.save_serial_port)
        # 让按钮高度与 ROS 按钮一致
        self.save_btn.pack(fill=tk.BOTH, expand=True, pady=10)

        # 右侧：ROS 启动
        frame5 = ttk.LabelFrame(bottom_frame, text="ROS启动", padding=10)
        frame5.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=(5, 0))
        self.launch_btn = tk.Button(frame5, text="启动launch文件",
                                    command=self.toggle_ros_launch,
                                    font=("Arial", 10),
                                    bg="lightgray",
                                    fg="black")
        self.launch_btn.pack(fill=tk.BOTH, expand=True, pady=10)

        # 保存默认按钮颜色（用于恢复）
        tmp = tk.Button(self.root)
        self.default_bg = tmp.cget("bg")
        self.default_fg = tmp.cget("fg")
        tmp.destroy()

        # 状态栏
        self.status_var = tk.StringVar(value="就绪")
        status_bar = ttk.Label(main_frame, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)

        # 初始按钮状态
        self.update_button_states()
        self.refresh_ports()

    def update_button_states(self):
        """更新按钮状态"""
        if self.serial_connected:
            self.save_btn.config(state="disabled")
            self.launch_btn.config(state="disabled")
            self.record_btn.config(state="normal")
            self.get_audio_btn.config(state="normal")
            self.clean_cache_btn.config(state="normal")
        elif self.ros_running:
            self.connect_btn.config(state="disabled")
            self.refresh_btn.config(state="disabled")
            self.save_btn.config(state="disabled")
            self.record_btn.config(state="disabled")
            self.get_audio_btn.config(state="disabled")
            self.clean_cache_btn.config(state="disabled")
        else:
            self.save_btn.config(state="normal")
            self.launch_btn.config(state="normal")
            self.connect_btn.config(state="normal")
            self.refresh_btn.config(state="normal")
            self.record_btn.config(state="disabled")
            self.get_audio_btn.config(state="disabled")
            self.clean_cache_btn.config(state="disabled")

    def refresh_ports(self):
        """刷新设备列表"""
        current_selection = self.port_combo.get()
        ports = []
        for p in serial.tools.list_ports.comports():
            if not any(f'ttyS{i}' in p.device for i in range(32)):
                ports.append(p.device)
        
        other_devices = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*') + glob.glob('/dev/aihitplt*')
        for device in other_devices:
            if device not in ports:
                ports.append(device)
        
        ports.sort()
        self.port_combo['values'] = ports
        
        if current_selection and current_selection in ports:
            self.port_combo.set(current_selection)
        elif ports:
            self.port_combo.current(0)
        
        self.update_status(f"找到 {len(ports)} 个设备")

    def toggle_serial_connection(self):
        if self.serial_connected:
            self.disconnect_serial()
        else:
            self.connect_serial()

    def connect_serial(self):
        port = self.port_combo.get()
        if not port:
            messagebox.showwarning("警告", "请选择设备")
            return
        
        if not port.startswith('/dev/'):
            port = f'/dev/{port}'
        
        if not os.path.exists(port):
            messagebox.showerror("错误", f"设备 {port} 不存在")
            return
        
        try:
            if self.serial_port:
                self.serial_port.close()
            
            self.serial_port = serial.Serial(port, 115200, timeout=0.1)
            self.serial_connected = True
            self.stop_serial_thread = False
            
            self.connect_btn.config(text="关闭")
            self.update_button_states()
            
            self.serial_thread = threading.Thread(target=self.read_serial_data, daemon=True)
            self.serial_thread.start()
            
            self.update_status(f"已连接到设备: {port}")
            self.update_info_display()
            
            self.root.after(500, self.auto_query_version)
            
        except Exception as e:
            msg = str(e)
            if "Permission denied" in msg:
                msg = f"权限不足，请运行: sudo chmod 666 {port}"
            messagebox.showerror("连接失败", f"无法连接设备:\n{msg}")
            self.update_status("连接失败")

    def disconnect_serial(self):
        if self.serial_connected:
            self.serial_connected = False
            self.stop_serial_thread = True
            
            if self.recording:
                self.stop_recording()
            
            if self.serial_port:
                self.serial_port.close()
                self.serial_port = None
            
            if self.serial_thread and self.serial_thread.is_alive():
                self.serial_thread.join(timeout=1.0)
            
            self.connect_btn.config(text="连接")
            self.update_button_states()
            
            self.record_status_var.set("就绪")
            self.update_status("已断开设备连接")
            self.update_info_display()

    def read_serial_data(self):
        while self.serial_connected and not self.stop_serial_thread:
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    first_byte = self.serial_port.read(1)
                    if len(first_byte) == 1 and first_byte[0] == self.SYNC_HEAD:
                        header = self.serial_port.read(6)
                        if len(header) == 6:
                            msg_type = header[1]
                            msg_len = header[2] + (header[3] << 8)
                            content = self.serial_port.read(msg_len)
                            checksum = self.serial_port.read(1)
                            if len(content) == msg_len and len(checksum) == 1:
                                if msg_type == self.MSG_TYPE_FEEDBACK:
                                    try:
                                        feedback_text = content.decode('utf-8', errors='ignore')
                                        self.root.after(0, lambda text=feedback_text: self.handle_feedback(text))
                                    except:
                                        pass
            except Exception as e:
                if not self.stop_serial_thread:
                    print(f"设备读取错误: {e}")
                break
            time.sleep(0.01)

    def handle_feedback(self, feedback_text):
        feedback = feedback_text.strip()
        if not feedback:
            return
        if "aiui_event" in feedback or "ivw" in feedback or "wakeup" in feedback.lower():
            self.update_info_display_text(f"语音唤醒事件:\n{feedback}")
        elif "error" in feedback.lower() or "fail" in feedback.lower():
            self.update_info_display_text(f"错误信息:\n{feedback}")
            self.update_status("设备返回错误")
        elif "version" in feedback.lower() or "firmware" in feedback.lower():
            self.update_status("固件版本信息已接收")
        elif len(feedback) > 50 and "{" in feedback and "}" in feedback:
            self.update_info_display_text(f"设备信息:\n{feedback}")

    def update_info_display(self):
        self.info_text.config(state=tk.NORMAL)
        self.info_text.delete(1.0, tk.END)
        if self.serial_connected:
            port = self.port_combo.get()
            self.info_text.insert(tk.END, f"状态: 已连接\n设备: {port}\n连接时间: {time.strftime('%H:%M:%S')}\n")
        elif self.ros_running:
            self.info_text.insert(tk.END, f"状态: ROS运行中\nPID: {self.ros_pid}\n启动时间: {time.strftime('%H:%M:%S')}\n")
        else:
            self.info_text.insert(tk.END, "状态: 未连接\n请连接设备或启动ROS\n")
        self.info_text.config(state=tk.DISABLED)

    def update_info_display_text(self, text):
        self.info_text.config(state=tk.NORMAL)
        self.info_text.insert(tk.END, f"\n{text}")
        self.info_text.see(tk.END)
        self.info_text.config(state=tk.DISABLED)

    def auto_query_version(self):
        if not self.serial_connected or not self.serial_port:
            return
        try:
            version_cmd = b'{"type": "version"}'
            msg = self.msg_packet(version_cmd)
            self.serial_port.write(msg)
            self.update_status("查询固件版本")
        except Exception as e:
            print(f"查询版本失败: {e}")

    # ========================== 修改点 1：修复了协议包头 ==========================
    def msg_packet(self, content):
        sync_head = 0xA5
        user_id = 0x01
        msg_type = 0x05
        msg_id_l = 0x01   # 官方协议里的 MsgId
        msg_id_h = 0x00
        msg_len = len(content)
        msg_len_l = msg_len & 0xFF
        msg_len_h = (msg_len >> 8) & 0xFF
        
        # 校验和加上了 msg_id_l 和 msg_id_h
        check_sum = sum([sync_head, user_id, msg_type, msg_len_l, msg_len_h, msg_id_l, msg_id_h]) + sum(content)
        check_code = ((~check_sum) & 0xFF) + 1
        return bytes([sync_head, user_id, msg_type, msg_len_l, msg_len_h, msg_id_l, msg_id_h]) + content + bytes([check_code])
    # ==============================================================================

    # ========================== 修改点 2：修复了录音逻辑 ==========================
    def start_recording(self):
        if not self.serial_connected:
            messagebox.showwarning("警告", "请先连接设备")
            return
        try:
            # 1. 必须先设置波束(手动唤醒)，否则硬件不生成 iat.pcm！
            beam_cmd = b'{"type": "manual_wakeup","content": {"beam": 1}}'
            msg = self.msg_packet(beam_cmd)
            self.serial_port.write(msg)
            time.sleep(0.2)
            
            # 2. 清理缓存
            clean_cmd = b'{"type":"clean_pcm"}'
            msg = self.msg_packet(clean_cmd)
            self.serial_port.write(msg)
            time.sleep(0.2)
            
            # 3. 开始录音
            record_cmd = b'{"type":"dump_audio","content":{"debug":3}}'
            msg = self.msg_packet(record_cmd)
            self.serial_port.write(msg)
            
            self.recording = True
            self.recording_time = 0
            self.record_duration = 5
            self.record_btn.config(state="disabled", text="录音中...")
            self.record_status_var.set(f"录音中: 0/{self.record_duration}秒")
            self.get_audio_btn.config(state="disabled")
            self.clean_cache_btn.config(state="disabled")
            self.update_status("已发送唤醒并开始录音 (5秒)")
            self.update_info_display_text("已触发波束1唤醒，开始录音 (5秒)...")
            self.update_record_timer()
        except Exception as e:
            messagebox.showerror("录音失败", f"开始录音失败:\n{e}")
    # ==============================================================================

    def update_record_timer(self):
        if self.recording and self.recording_time < self.record_duration:
            self.recording_time += 1
            self.record_status_var.set(f"录音中: {self.recording_time}/{self.record_duration}秒")
            if self.recording_time >= self.record_duration:
                self.stop_recording()
            else:
                self.root.after(1000, self.update_record_timer)

    def stop_recording(self):
        try:
            if self.serial_connected:
                record_cmd = b'{"type":"dump_audio","content":{"debug":0}}'
                msg = self.msg_packet(record_cmd)
                self.serial_port.write(msg)
            self.recording = False
            self.record_btn.config(state="normal", text="开始录音(5秒)")
            self.record_status_var.set(f"录音完成: {self.recording_time}秒")
            self.get_audio_btn.config(state="normal")
            self.clean_cache_btn.config(state="normal")
            self.update_status("录音完成")
            self.update_info_display_text(f"录音完成，时长: {self.recording_time}秒")
        except Exception as e:
            print(f"停止录音错误: {e}")
            self.recording = False
            self.record_btn.config(state="normal", text="开始录音(5秒)")
            self.record_status_var.set("录音失败")

    def get_audio_files(self):
        if not self.serial_connected:
            messagebox.showwarning("警告", "请先连接设备")
            return
        try:
            result = subprocess.run(["adb", "devices"], capture_output=True, text=True, timeout=5)
            if "device" not in result.stdout:
                messagebox.showwarning("ADB未连接", "未检测到ADB设备连接\n请确保设备通过USB连接并启用ADB调试")
                return
            device_files = {"origin": "/data/build/origin.pcm", "iat": "/data/build/iat.pcm"}
            success_files = []
            for file_type, device_path in device_files.items():
                try:
                    check_cmd = ["adb", "shell", f"ls {device_path}"]
                    check_result = subprocess.run(check_cmd, capture_output=True, text=True, timeout=5)
                    if "No such file" in check_result.stderr:
                        self.update_info_display_text(f"{file_type}.pcm 文件不存在: {device_path}")
                        continue
                    temp_pcm = f"/tmp/{file_type}.pcm"
                    self.update_info_display_text(f"正在拉取 {file_type}.pcm...")
                    pull_cmd = ["adb", "pull", device_path, temp_pcm]
                    pull_result = subprocess.run(pull_cmd, capture_output=True, text=True, timeout=10)
                    if pull_result.returncode != 0:
                        self.update_info_display_text(f"拉取 {file_type}.pcm 失败")
                        continue
                    wav_path = os.path.join(self.audio_save_dir, f"{file_type}.wav")
                    self.update_info_display_text(f"正在转换为 {file_type}.wav...")
                    file_size = os.path.getsize(temp_pcm)
                    if file_size == 0:
                        self.update_info_display_text(f"{file_type}.pcm 文件为空")
                        os.remove(temp_pcm)
                        continue
                    with open(temp_pcm, 'rb') as f:
                        pcm_data = f.read()
                    audio_data = np.frombuffer(pcm_data, dtype=np.int16)
                    if len(audio_data) == 0:
                        self.update_info_display_text(f"{file_type}.pcm 无有效数据")
                        os.remove(temp_pcm)
                        continue
                    try:
                        from scipy.io import wavfile
                        wavfile.write(wav_path, 16000, audio_data)
                    except ImportError:
                        if subprocess.run(["which", "sox"], capture_output=True).returncode == 0:
                            cmd = f"sox -t raw -r 16000 -e signed -b 16 -c 1 {temp_pcm} {wav_path}"
                        elif subprocess.run(["which", "ffmpeg"], capture_output=True).returncode == 0:
                            cmd = f"ffmpeg -f s16le -ar 16000 -ac 1 -i {temp_pcm} {wav_path} -y"
                        else:
                            wav_path = wav_path.replace('.wav', '.pcm')
                            cmd = f"cp {temp_pcm} {wav_path}"
                        subprocess.run(cmd, shell=True, check=True)
                    if os.path.exists(temp_pcm):
                        os.remove(temp_pcm)
                    if os.path.exists(wav_path):
                        file_size = os.path.getsize(wav_path)
                        success_files.append(f"{file_type}.wav ({file_size//1024}KB)")
                        self.update_info_display_text(f"已保存: {file_type}.wav ")
                except Exception as e:
                    self.update_info_display_text(f"处理 {file_type} 失败: {str(e)[:50]}")
            if success_files:
                message = f"音频获取成功！\n\n保存到: {self.audio_save_dir}\n\n文件列表:\n" + "\n".join(success_files)
                messagebox.showinfo("成功", message)
                self.update_status(f"已获取 {len(success_files)} 个音频文件")
            else:
                messagebox.showwarning("获取失败", "未能获取任何音频文件\n请检查设备路径和ADB连接")
                self.update_status("音频获取失败")
        except subprocess.TimeoutExpired:
            messagebox.showerror("超时", "ADB操作超时")
            self.update_status("ADB超时")
        except Exception as e:
            messagebox.showerror("错误", f"获取音频失败:\n{str(e)[:100]}")
            self.update_status(f"获取失败: {str(e)[:30]}")

    def clean_audio_cache(self):
        if not self.serial_connected:
            messagebox.showwarning("警告", "请先连接设备")
            return
        try:
            cmd = b'{"type":"clean_pcm"}'
            msg = self.msg_packet(cmd)
            self.serial_port.write(msg)
            self.update_status("已发送清空音频缓存命令")
            self.update_info_display_text("已发送清空音频缓存命令")
        except Exception as e:
            messagebox.showerror("发送失败", f"清空音频缓存失败:\n{e}")

    def save_serial_port(self):
        port = self.port_combo.get()
        if not port:
            messagebox.showwarning("警告", "请先选择设备")
            return
        
        if not port.startswith('/dev/'):
            port = f'/dev/{port}'
        
        try:
            os.makedirs(self.config_dir, exist_ok=True)
            config = {
                'port': port, 
                'baudrate': 115200, 
                'last_updated': time.strftime("%Y-%m-%d %H:%M:%S")
            }
            
            with open(self.config_file, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
            
            messagebox.showinfo("保存成功", 
                                f"设备 {port} 已保存成功\n" 
                                f"配置文件: {self.config_file}")
            
            self.update_status(f"已保存设备: {port}")
            
        except Exception as e:
            messagebox.showerror("保存失败", f"保存设备失败:\n{str(e)}")

    def load_saved_port(self):
        if not os.path.exists(self.config_file):
            return
        try:
            with open(self.config_file, 'r') as f:
                config = yaml.safe_load(f)
            if config and 'port' in config:
                saved_port = config['port']
                if saved_port.startswith('//dev/'):
                    saved_port = saved_port.replace('//dev/', '/dev/')
                elif not saved_port.startswith('/dev/'):
                    saved_port = f'/dev/{saved_port}'
                self.port_combo.set(saved_port)
                self.refresh_ports()
                self.update_status(f"已加载保存的设备: {saved_port}")
        except Exception as e:
            print(f"加载保存的设备失败: {e}")

    def toggle_ros_launch(self):
        if not self.ros_running:
            self.start_ros_launch()
        else:
            self.stop_ros_launch()

    def start_ros_launch(self):
        """启动ROS launch文件 - 使用gnome-terminal打开新终端"""
        try:
            if self.serial_connected:
                self.disconnect_serial()
            
            port = self.port_combo.get()
            if not port:
                messagebox.showwarning("警告", "请先选择设备")
                return
            
            if not port.startswith('/dev/'):
                port = f'/dev/{port}'
            
            roslaunch_cmd = f'roslaunch aihitplt_hardware_test aihitplt_mic_init.launch'
            
            print(f"启动命令: {roslaunch_cmd}")
            print(f"使用设备: {port}")
            
            cmd = [
                'gnome-terminal',
                '--title=麦克风阵列 - ROS Launch',
                '--geometry=80x24+100+100',
                '--',
                'bash', '-c',
                f'{roslaunch_cmd}'
            ]
            
            self.ros_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True
            )
            
            self.ros_pid = self.ros_process.pid
            self.ros_running = True
            
            self._get_gnome_terminal_pid()
            
            self.launch_btn.config(
                text="关闭launch文件",
                bg="green",
                fg="white"
            )
            self.update_button_states()
            
            self.update_status(f"已启动ROS launch文件，使用设备: {port}")
            self.update_info_display()
            
        except FileNotFoundError:
            messagebox.showerror("启动失败", "未找到gnome-terminal。")
            self.update_status("启动失败: 未找到gnome-terminal")
        except Exception as e:
            messagebox.showerror("启动失败", f"无法启动ROS launch文件:\n{e}")
            self.update_status(f"启动失败: {e}")

    def _get_gnome_terminal_pid(self):
        try:
            time.sleep(0.5) 
            cmd = f"ps aux | grep 'gnome-terminal.*aihitplt_mic_init' | grep -v grep | head -1"
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
            if result.stdout:
                parts = result.stdout.split()
                if len(parts) > 1:
                    self.gnome_terminal_pid = int(parts[1])
                    print(f"找到gnome-terminal进程ID: {self.gnome_terminal_pid}")
        except Exception as e:
            print(f"获取gnome-terminal进程ID失败: {e}")

    def kill_process_tree(self, pid):
        try:
            process = psutil.Process(pid)
            children = process.children(recursive=True)
            
            for child in children:
                try:
                    child.terminate()
                except:
                    pass
            
            gone, alive = psutil.wait_procs(children, timeout=3)
            
            for child in alive:
                try:
                    child.kill()
                except:
                    pass
            
            try:
                process.terminate()
                process.wait(timeout=3)
            except:
                try:
                    process.kill()
                except:
                    pass
                    
        except psutil.NoSuchProcess:
            pass
        except Exception as e:
            print(f"终止进程树时出错: {e}")

    def stop_ros_launch(self):
        if self.ros_running:
            try:
                if self.gnome_terminal_pid:
                    try:
                        print(f"尝试关闭gnome-terminal进程: {self.gnome_terminal_pid}")
                        os.kill(self.gnome_terminal_pid, signal.SIGTERM)
                        time.sleep(0.5)
                        
                        if psutil.pid_exists(self.gnome_terminal_pid):
                            os.kill(self.gnome_terminal_pid, signal.SIGKILL)
                            time.sleep(0.5)
                    except ProcessLookupError:
                        print("gnome-terminal进程已不存在")
                    except Exception as e:
                        print(f"关闭gnome-terminal进程失败: {e}")
                
                if self.ros_pid:
                    self.kill_process_tree(self.ros_pid)
                
                self._kill_ros_processes()
                
                self._close_by_window_title("麦克风阵列 - ROS Launch")
                
            except Exception as e:
                print(f"停止ROS进程时出错: {e}")
                
            finally:
                self.ros_running = False
                self.ros_process = None
                self.ros_pid = None
                self.gnome_terminal_pid = None
                
                self.launch_btn.config(
                    text="启动launch文件",
                    bg=self.default_bg,
                    fg=self.default_fg
                )
                self.update_button_states()
                
                self.update_status("ROS进程已停止")
                self.update_info_display()

    def _kill_ros_processes(self):
        try:
            launch_files_to_kill = [
                'aihitplt_mic_init.launch'
            ]
            
            for launch_file in launch_files_to_kill:
                subprocess.run(['pkill', '-f', launch_file], 
                            stdout=subprocess.DEVNULL, 
                            stderr=subprocess.DEVNULL)
            
            time.sleep(1)
            
        except Exception as e:
            print(f"终止ROS进程时出错: {e}")

    def _close_by_window_title(self, title):
        try:
            cmd = f"wmctrl -l | grep '{title}'"
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
            if result.stdout:
                lines = result.stdout.strip().split('\n')
                for line in lines:
                    if line:
                        window_id = line.split()[0]
                        subprocess.run(['wmctrl', '-ic', window_id], 
                                    stdout=subprocess.DEVNULL, 
                                    stderr=subprocess.DEVNULL)
                        print(f"通过wmctrl关闭窗口: {window_id}")
        except Exception as e:
            print(f"通过窗口标题关闭窗口失败: {e}")

    def update_status(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.status_var.set(f"[{timestamp}] {message}")
        print(f"[状态] {message}")

    def on_closing(self):
        if self.serial_connected:
            self.disconnect_serial()
        
        if self.ros_running:
            self.stop_ros_launch()
        
        time.sleep(0.5)
        self.root.destroy()

def main():
    if 'DISPLAY' not in os.environ:
        os.environ['DISPLAY'] = ':0'
    os.environ['QT_X11_NO_MITSHM'] = '1'
    
    root = tk.Tk()
    
    window_width = 500
    window_height = 500
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    x = (screen_width - window_width) // 2
    y = (screen_height - window_height) // 2
    root.geometry(f"{window_width}x{window_height}+{x}+{y}")
    
    MicTester(root)
    root.mainloop()

if __name__ == "__main__":
    import serial
    main()