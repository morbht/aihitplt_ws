#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
音频设备测试工具 - Python版本
支持麦克风录音和扬声器播放测试
"""

import os
import sys
import time
import subprocess
import threading
import wave
import json
from datetime import datetime
from pathlib import Path
import pyaudio
import numpy as np
import soundfile as sf
import sounddevice as sd
from colorama import init, Fore, Style
import psutil

# 初始化颜色输出
init(autoreset=True)

class AudioTester:
    def __init__(self):
        self.home_dir = Path.home()
        self.record_dir = self.home_dir / "aihitplt_test"
        self.record_dir.mkdir(exist_ok=True)
        self.music_dir = self.home_dir / "Music"
        
        self.record_file = self.record_dir / "test.wav"
        self.music_file = self.music_dir / "test_music.wav"
        self.config_file = self.home_dir / ".audio_tester_config.json"
        
        # 音频参数
        self.sample_rate = 44100
        self.channels = 1  # 单声道录音
        self.chunk_size = 1024
        self.record_seconds = 5
        
        # 设备信息
        self.selected_input = None
        self.selected_output = None
        self.input_devices = []
        self.output_devices = []
        
        # 加载配置
        self.load_config()
        
        # 初始化PyAudio
        try:
            self.p = pyaudio.PyAudio()
            self.refresh_devices()
        except Exception as e:
            print(f"{Fore.RED}初始化PyAudio失败: {e}")
            print(f"请安装所需库: pip install pyaudio sounddevice soundfile numpy colorama psutil")
            sys.exit(1)
    
    def load_config(self):
        """加载保存的设备配置"""
        if self.config_file.exists():
            try:
                with open(self.config_file, 'r') as f:
                    config = json.load(f)
                    self.selected_input = config.get('input_device')
                    self.selected_output = config.get('output_device')
                    self.sample_rate = config.get('sample_rate', 44100)
            except:
                pass
    
    def save_config(self):
        """保存设备配置"""
        config = {
            'input_device': self.selected_input,
            'output_device': self.selected_output,
            'sample_rate': self.sample_rate
        }
        with open(self.config_file, 'w') as f:
            json.dump(config, f, indent=2)
    
    def refresh_devices(self):
        """刷新音频设备列表"""
        self.input_devices = []
        self.output_devices = []
        
        # 使用PyAudio获取设备信息
        for i in range(self.p.get_device_count()):
            device_info = self.p.get_device_info_by_index(i)
            
            # 检查是否为输入设备（麦克风）
            if device_info['maxInputChannels'] > 0:
                self.input_devices.append({
                    'index': i,
                    'name': device_info['name'],
                    'channels': device_info['maxInputChannels'],
                    'sample_rate': int(device_info['defaultSampleRate']),
                    'host_api': self.p.get_host_api_info_by_index(device_info['hostApi'])['name']
                })
            
            # 检查是否为输出设备（扬声器）
            if device_info['maxOutputChannels'] > 0:
                self.output_devices.append({
                    'index': i,
                    'name': device_info['name'],
                    'channels': device_info['maxOutputChannels'],
                    'sample_rate': int(device_info['defaultSampleRate']),
                    'host_api': self.p.get_host_api_info_by_index(device_info['hostApi'])['name']
                })
        
        # 同时使用sounddevice获取更详细的信息
        try:
            sd_devices = sd.query_devices()
            print(f"{Fore.CYAN}系统检测到 {len(sd_devices)} 个音频设备")
        except:
            pass
    
    def display_devices(self):
        """显示所有音频设备"""
        print(f"\n{Fore.YELLOW}{'='*60}")
        print(f"{'音频设备列表':^60}")
        print(f"{'='*60}{Style.RESET_ALL}")
        
        # 输入设备
        print(f"\n{Fore.GREEN}🎤 输入设备（麦克风）:")
        print(f"{'-'*60}")
        for i, dev in enumerate(self.input_devices, 1):
            selected = " ✓" if self.selected_input == dev['index'] else ""
            print(f"{i:2d}. [{dev['index']}] {dev['name']}")
            print(f"     采样率: {dev['sample_rate']} Hz, 声道: {dev['channels']}, API: {dev['host_api']}{selected}")
        
        # 输出设备
        print(f"\n{Fore.BLUE}🔊 输出设备（扬声器）:")
        print(f"{'-'*60}")
        for i, dev in enumerate(self.output_devices, 1):
            selected = " ✓" if self.selected_output == dev['index'] else ""
            print(f"{i:2d}. [{dev['index']}] {dev['name']}")
            print(f"     采样率: {dev['sample_rate']} Hz, 声道: {dev['channels']}, API: {dev['host_api']}{selected}")
        
        print()
    
    def select_input_device(self):
        """选择输入设备"""
        if not self.input_devices:
            print(f"{Fore.RED}未检测到输入设备！")
            return
        
        self.display_devices()
        
        while True:
            try:
                choice = input(f"{Fore.CYAN}选择输入设备编号 [1-{len(self.input_devices)}]，或按回车使用默认: ")
                
                if choice.strip() == '':
                    # 选择默认设备
                    default_input = self.p.get_default_input_device_info()
                    for dev in self.input_devices:
                        if dev['name'] == default_input['name']:
                            self.selected_input = dev['index']
                            break
                    else:
                        self.selected_input = self.input_devices[0]['index'] if self.input_devices else None
                    break
                
                choice_idx = int(choice) - 1
                if 0 <= choice_idx < len(self.input_devices):
                    self.selected_input = self.input_devices[choice_idx]['index']
                    break
                else:
                    print(f"{Fore.RED}无效的选择！")
            except ValueError:
                print(f"{Fore.RED}请输入数字！")
        
        if self.selected_input is not None:
            dev = next(d for d in self.input_devices if d['index'] == self.selected_input)
            print(f"{Fore.GREEN}已选择输入设备: {dev['name']}")
            self.save_config()
    
    def select_output_device(self):
        """选择输出设备"""
        if not self.output_devices:
            print(f"{Fore.RED}未检测到输出设备！")
            return
        
        self.display_devices()
        
        while True:
            try:
                choice = input(f"{Fore.CYAN}选择输出设备编号 [1-{len(self.output_devices)}]，或按回车使用默认: ")
                
                if choice.strip() == '':
                    # 选择默认设备
                    default_output = self.p.get_default_output_device_info()
                    for dev in self.output_devices:
                        if dev['name'] == default_output['name']:
                            self.selected_output = dev['index']
                            break
                    else:
                        self.selected_output = self.output_devices[0]['index'] if self.output_devices else None
                    break
                
                choice_idx = int(choice) - 1
                if 0 <= choice_idx < len(self.output_devices):
                    self.selected_output = self.output_devices[choice_idx]['index']
                    break
                else:
                    print(f"{Fore.RED}无效的选择！")
            except ValueError:
                print(f"{Fore.RED}请输入数字！")
        
        if self.selected_output is not None:
            dev = next(d for d in self.output_devices if d['index'] == self.selected_output)
            print(f"{Fore.GREEN}已选择输出设备: {dev['name']}")
            self.save_config()
    
    def check_audio_processes(self):
        """检查占用音频设备的进程"""
        print(f"\n{Fore.YELLOW}🔍 检查音频设备占用情况...")
        
        # 查找可能占用音频的进程
        audio_keywords = ['pulseaudio', 'alsa', 'arecord', 'aplay', 'audacity', 'chrome', 'firefox']
        audio_processes = []
        
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                proc_info = proc.info
                name = proc_info['name'].lower()
                cmdline = ' '.join(proc_info['cmdline'] or []).lower()
                
                for keyword in audio_keywords:
                    if keyword in name or keyword in cmdline:
                        audio_processes.append(proc)
                        break
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        
        if audio_processes:
            print(f"{Fore.RED}发现可能占用音频设备的进程:")
            for proc in audio_processes:
                try:
                    print(f"  PID {proc.pid}: {proc.name()} - {proc.cmdline()[:100]}")
                except:
                    print(f"  PID {proc.pid}: {proc.name()}")
            
            choice = input(f"\n{Fore.YELLOW}是否终止这些进程？[y/N]: ").lower()
            if choice == 'y':
                for proc in audio_processes:
                    try:
                        proc.terminate()
                        print(f"{Fore.GREEN}已终止进程 {proc.pid}")
                    except:
                        print(f"{Fore.RED}无法终止进程 {proc.pid}")
                time.sleep(1)
        else:
            print(f"{Fore.GREEN}未发现明显占用音频设备的进程")
        
        return True
    
    def test_microphone(self):
        """测试麦克风录音"""
        print(f"\n{Fore.CYAN}{'='*60}")
        print(f"{'麦克风测试':^60}")
        print(f"{'='*60}{Style.RESET_ALL}")
        
        # 如果没有选择设备，先选择
        if self.selected_input is None:
            print(f"{Fore.YELLOW}未选择输入设备，请先选择设备")
            self.select_input_device()
            if self.selected_input is None:
                return
        
        # 检查设备占用
        self.check_audio_processes()
        
        # 获取设备信息
        device_info = self.p.get_device_info_by_index(self.selected_input)
        print(f"\n📱 设备信息:")
        print(f"  名称: {device_info['name']}")
        print(f"  采样率: {device_info['defaultSampleRate']} Hz")
        print(f"  最大输入声道: {device_info['maxInputChannels']}")
        
        # 测试不同参数
        sample_rates = [16000, 44100, 48000]
        channels_list = [1, 2]
        
        print(f"\n🎤 开始录音测试（{self.record_seconds}秒）...")
        print(f"请对着麦克风说话{Style.BRIGHT}（测试录音中...）{Style.RESET_ALL}")
        
        # 尝试不同的参数组合
        for sr in sample_rates:
            for channels in channels_list:
                if channels > device_info['maxInputChannels']:
                    continue
                
                print(f"\n尝试参数: {sr}Hz, {channels}声道...")
                
                try:
                    # 创建临时文件名
                    temp_file = self.record_dir / f"test_{sr}_{channels}ch.wav"
                    
                    # 设置录音参数
                    audio_format = pyaudio.paInt16
                    
                    # 创建录音流
                    stream = self.p.open(
                        format=audio_format,
                        channels=channels,
                        rate=sr,
                        input=True,
                        input_device_index=self.selected_input,
                        frames_per_buffer=self.chunk_size
                    )
                    
                    print(f"{Fore.YELLOW}录音开始（{self.record_seconds}秒）...")
                    frames = []
                    
                    # 显示进度
                    def show_progress():
                        for i in range(self.record_seconds):
                            print(f"\r进度: [{('█' * (i+1)).ljust(self.record_seconds, '░')}] {i+1}/{self.record_seconds}秒", end='')
                            time.sleep(1)
                    
                    # 启动进度显示线程
                    progress_thread = threading.Thread(target=show_progress)
                    progress_thread.daemon = True
                    progress_thread.start()
                    
                    # 录音
                    for _ in range(0, int(sr / self.chunk_size * self.record_seconds)):
                        data = stream.read(self.chunk_size, exception_on_overflow=False)
                        frames.append(data)
                    
                    stream.stop_stream()
                    stream.close()
                    
                    print(f"\n{Fore.GREEN}录音完成！")
                    
                    # 保存WAV文件
                    wf = wave.open(str(temp_file), 'wb')
                    wf.setnchannels(channels)
                    wf.setsampwidth(self.p.get_sample_size(audio_format))
                    wf.setframerate(sr)
                    wf.writeframes(b''.join(frames))
                    wf.close()
                    
                    # 检查文件
                    file_size = os.path.getsize(temp_file) / 1024  # KB
                    print(f"文件大小: {file_size:.1f} KB")
                    
                    # 更新主录音文件
                    if file_size > 1:  # 大于1KB才认为是有效录音
                        self.record_file = temp_file
                        self.sample_rate = sr
                        
                        # 播放测试录音
                        choice = input(f"\n{Fore.CYAN}是否播放录音？[Y/n]: ").lower()
                        if choice != 'n':
                            self.play_recording()
                        
                        # 删除其他测试文件
                        self.cleanup_temp_files(keep_file=str(temp_file))
                        
                        self.save_config()
                        return True
                    
                except Exception as e:
                    print(f"{Fore.RED}参数 {sr}Hz/{channels}ch 失败: {e}")
                    continue
        
        print(f"{Fore.RED}所有参数尝试均失败！")
        return False
    
    def play_recording(self):
        """播放录音文件"""
        if not self.record_file.exists():
            print(f"{Fore.RED}录音文件不存在！")
            return
        
        print(f"\n{Fore.BLUE}播放录音文件: {self.record_file.name}")
        
        if self.selected_output is None:
            print(f"{Fore.YELLOW}未选择输出设备，请先选择设备")
            self.select_output_device()
            if self.selected_output is None:
                return
        
        try:
            # 使用soundfile读取音频文件
            data, samplerate = sf.read(str(self.record_file))
            
            print(f"音频信息: {samplerate}Hz, {len(data)/samplerate:.1f}秒")
            
            # 使用sounddevice播放
            print(f"{Fore.GREEN}正在播放...")
            sd.play(data, samplerate, device=self.selected_output)
            sd.wait()  # 等待播放完成
            print(f"{Fore.GREEN}播放完成！")
            
        except Exception as e:
            print(f"{Fore.RED}播放失败: {e}")
            
            # 尝试使用PyAudio播放
            print(f"{Fore.YELLOW}尝试使用PyAudio播放...")
            try:
                wf = wave.open(str(self.record_file), 'rb')
                
                stream = self.p.open(
                    format=self.p.get_format_from_width(wf.getsampwidth()),
                    channels=wf.getnchannels(),
                    rate=wf.getframerate(),
                    output=True,
                    output_device_index=self.selected_output
                )
                
                data = wf.readframes(self.chunk_size)
                while data:
                    stream.write(data)
                    data = wf.readframes(self.chunk_size)
                
                stream.stop_stream()
                stream.close()
                wf.close()
                print(f"{Fore.GREEN}播放完成！")
            except Exception as e2:
                print(f"{Fore.RED}PyAudio播放也失败: {e2}")
    
    def test_speaker(self):
        """测试扬声器"""
        print(f"\n{Fore.CYAN}{'='*60}")
        print(f"{'扬声器测试':^60}")
        print(f"{'='*60}{Style.RESET_ALL}")
        
        if self.selected_output is None:
            print(f"{Fore.YELLOW}未选择输出设备，请先选择设备")
            self.select_output_device()
            if self.selected_output is None:
                return
        
        device_info = self.p.get_device_info_by_index(self.selected_output)
        print(f"\n📱 设备信息:")
        print(f"  名称: {device_info['name']}")
        print(f"  采样率: {device_info['defaultSampleRate']} Hz")
        print(f"  最大输出声道: {device_info['maxOutputChannels']}")
        
        # 选项
        print(f"\n{Fore.CYAN}选择测试方式:")
        print(f"1. 播放测试音频文件（如果存在）")
        print(f"2. 播放系统提示音")
        print(f"3. 播放正弦波测试音")
        print(f"4. 播放白噪声")
        
        try:
            choice = input(f"\n{Fore.CYAN}请选择 [1-4]: ")
            
            if choice == '1':
                self.play_test_audio()
            elif choice == '2':
                self.play_system_beep()
            elif choice == '3':
                self.play_sine_wave()
            elif choice == '4':
                self.play_white_noise()
            else:
                print(f"{Fore.RED}无效选择！")
                
        except KeyboardInterrupt:
            print(f"\n{Fore.YELLOW}测试中断")
    
    def play_test_audio(self):
        """播放测试音频文件"""
        if self.music_file.exists():
            print(f"\n{Fore.GREEN}找到测试音频: {self.music_file}")
            
            # 读取并播放
            try:
                data, samplerate = sf.read(str(self.music_file))
                print(f"正在播放 {samplerate}Hz 音频...")
                
                sd.play(data, samplerate, device=self.selected_output)
                sd.wait()
                print(f"{Fore.GREEN}播放完成！")
            except Exception as e:
                print(f"{Fore.RED}播放失败: {e}")
        else:
            print(f"{Fore.YELLOW}测试音频文件不存在: {self.music_file}")
            print(f"请将测试音频文件放在: {self.music_file}")
    
    def play_system_beep(self):
        """播放系统提示音"""
        print(f"\n{Fore.YELLOW}播放3次提示音...")
        
        for i in range(3):
            print(f"提示音 {i+1}...")
            
            # 生成440Hz正弦波（A4音符）
            duration = 0.5  # 0.5秒
            t = np.linspace(0, duration, int(self.sample_rate * duration), False)
            tone = np.sin(2 * np.pi * 440 * t)
            
            # 添加淡入淡出
            fade = 0.01
            fade_samples = int(self.sample_rate * fade)
            tone[:fade_samples] *= np.linspace(0, 1, fade_samples)
            tone[-fade_samples:] *= np.linspace(1, 0, fade_samples)
            
            # 播放
            sd.play(tone, self.sample_rate, device=self.selected_output)
            sd.wait()
            
            time.sleep(0.3)
        
        print(f"{Fore.GREEN}提示音播放完成！")
    
    def play_sine_wave(self):
        """播放正弦波测试音"""
        print(f"\n{Fore.CYAN}正弦波频率测试")
        print(f"按 Ctrl+C 停止测试")
        
        frequencies = [250, 500, 1000, 2000, 4000]  # 测试不同频率
        duration = 2  # 每个频率2秒
        
        try:
            for freq in frequencies:
                print(f"\n播放 {freq}Hz 正弦波 ({duration}秒)...")
                
                t = np.linspace(0, duration, int(self.sample_rate * duration), False)
                tone = 0.5 * np.sin(2 * np.pi * freq * t)
                
                sd.play(tone, self.sample_rate, device=self.selected_output)
                sd.wait()
                
                time.sleep(0.5)
                
        except KeyboardInterrupt:
            print(f"\n{Fore.YELLOW}测试中断")
            sd.stop()
    
    def play_white_noise(self):
        """播放白噪声"""
        print(f"\n{Fore.CYAN}播放白噪声（10秒）...")
        print(f"按 Ctrl+C 提前停止")
        
        try:
            duration = 10
            noise = np.random.uniform(-0.1, 0.1, int(self.sample_rate * duration))
            
            # 淡入淡出
            fade = 0.1
            fade_samples = int(self.sample_rate * fade)
            noise[:fade_samples] *= np.linspace(0, 1, fade_samples)
            noise[-fade_samples:] *= np.linspace(1, 0, fade_samples)
            
            sd.play(noise, self.sample_rate, device=self.selected_output)
            sd.wait()
            
            print(f"{Fore.GREEN}白噪声播放完成！")
            
        except KeyboardInterrupt:
            print(f"\n{Fore.YELLOW}测试中断")
            sd.stop()
    
    def cleanup_temp_files(self, keep_file=None):
        """清理临时文件"""
        pattern = self.record_dir / "test_*_*ch.wav"
        for file in self.record_dir.glob("test_*_*ch.wav"):
            if keep_file and str(file) == keep_file:
                continue
            try:
                file.unlink()
                print(f"{Fore.YELLOW}已清理: {file.name}")
            except:
                pass
    
    def delete_recording(self):
        """删除录音文件"""
        print(f"\n{Fore.RED}{'='*60}")
        print(f"{'删除录音文件':^60}")
        print(f"{'='*60}{Style.RESET_ALL}")
        
        # 列出所有录音文件
        wav_files = list(self.record_dir.glob("*.wav"))
        
        if not wav_files:
            print(f"{Fore.YELLOW}没有找到录音文件")
            return
        
        print(f"\n找到 {len(wav_files)} 个录音文件:")
        for i, file in enumerate(wav_files, 1):
            size = os.path.getsize(file) / 1024
            print(f"{i:2d}. {file.name} ({size:.1f} KB)")
        
        try:
            choice = input(f"\n{Fore.RED}删除哪个文件？[1-{len(wav_files)}]，或输入 'all' 删除所有: ")
            
            if choice.lower() == 'all':
                for file in wav_files:
                    file.unlink()
                    print(f"{Fore.GREEN}已删除: {file.name}")
            else:
                idx = int(choice) - 1
                if 0 <= idx < len(wav_files):
                    wav_files[idx].unlink()
                    print(f"{Fore.GREEN}已删除: {wav_files[idx].name}")
                else:
                    print(f"{Fore.RED}无效的选择！")
                    
        except (ValueError, KeyboardInterrupt):
            print(f"{Fore.YELLOW}操作取消")
    
    def show_status(self):
        """显示当前状态"""
        print(f"\n{Fore.CYAN}{'='*60}")
        print(f"{'当前状态':^60}")
        print(f"{'='*60}{Style.RESET_ALL}")
        
        # 输入设备
        if self.selected_input is not None:
            try:
                dev = self.p.get_device_info_by_index(self.selected_input)
                print(f"🎤 输入设备: {dev['name']}")
            except:
                print(f"🎤 输入设备: 索引 {self.selected_input}")
        else:
            print(f"🎤 输入设备: {Fore.RED}未选择")
        
        # 输出设备
        if self.selected_output is not None:
            try:
                dev = self.p.get_device_info_by_index(self.selected_output)
                print(f"🔊 输出设备: {dev['name']}")
            except:
                print(f"🔊 输出设备: 索引 {self.selected_output}")
        else:
            print(f"🔊 输出设备: {Fore.RED}未选择")
        
        # 录音文件
        wav_count = len(list(self.record_dir.glob("*.wav")))
        print(f"💾 录音文件: {wav_count} 个")
        
        if self.record_file.exists():
            size = os.path.getsize(self.record_file) / 1024
            print(f"📁 主录音文件: {self.record_file.name} ({size:.1f} KB)")
    
    def run(self):
        """运行主程序"""
        # 检查依赖
        try:
            import pyaudio
            import numpy as np
            import soundfile as sf
            import sounddevice as sd
            from colorama import init, Fore, Style
        except ImportError as e:
            print(f"{Fore.RED}缺少依赖库: {e}")
            print(f"请运行: pip install pyaudio sounddevice soundfile numpy colorama psutil")
            return
        
        # 显示欢迎信息
        print(f"\n{Fore.CYAN}{'='*60}")
        print(f"{'Python 音频测试工具':^60}")
        print(f"{'='*60}{Style.RESET_ALL}")
        
        while True:
            self.show_status()
            
            print(f"\n{Fore.CYAN}{'主菜单':^60}")
            print(f"{'-'*60}")
            print(f"1. 选择输入设备（麦克风）")
            print(f"2. 选择输出设备（扬声器）")
            print(f"3. 测试麦克风（录音）")
            print(f"4. 测试扬声器（播放）")
            print(f"5. 刷新设备列表")
            print(f"6. 显示所有设备")
            print(f"7. 删除录音文件")
            print(f"8. 检查音频进程")
            print(f"9. 清理临时文件")
            print(f"0. 退出")
            print(f"{'-'*60}")
            
            try:
                choice = input(f"{Fore.GREEN}请选择操作 [0-9]: ")
                
                if choice == '0':
                    print(f"\n{Fore.YELLOW}退出程序...")
                    break
                elif choice == '1':
                    self.select_input_device()
                elif choice == '2':
                    self.select_output_device()
                elif choice == '3':
                    self.test_microphone()
                elif choice == '4':
                    self.test_speaker()
                elif choice == '5':
                    self.refresh_devices()
                    print(f"{Fore.GREEN}设备列表已刷新！")
                elif choice == '6':
                    self.display_devices()
                elif choice == '7':
                    self.delete_recording()
                elif choice == '8':
                    self.check_audio_processes()
                elif choice == '9':
                    self.cleanup_temp_files()
                else:
                    print(f"{Fore.RED}无效选择！")
                
                input(f"\n{Fore.YELLOW}按回车键继续...")
                
            except KeyboardInterrupt:
                print(f"\n{Fore.YELLOW}\n程序被中断")
                break
            except Exception as e:
                print(f"{Fore.RED}错误: {e}")
                import traceback
                traceback.print_exc()
                input(f"\n按回车键继续...")
        
        # 清理
        self.p.terminate()
        print(f"{Fore.GREEN}程序结束")

def main():
    """主函数"""
    tester = AudioTester()
    tester.run()

if __name__ == "__main__":
    # 检查是否以root运行（通常不需要）
    if os.geteuid() == 0:
        print(f"{Fore.YELLOW}警告：不建议以root用户运行音频程序")
        choice = input(f"{Fore.YELLOW}是否继续？[y/N]: ").lower()
        if choice != 'y':
            sys.exit(0)
    
    main()
