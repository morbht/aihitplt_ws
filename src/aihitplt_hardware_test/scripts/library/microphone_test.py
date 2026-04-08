#!/usr/bin/env python3
import rospy
import subprocess
import threading
import time
import os
import signal
import numpy as np
import select

def optimized_volume_display():
    """优化的音量显示版本"""
    print("启动麦克风测试与音量显示...")
    print("按 Ctrl+C 停止")
    
    try:
        # 启动arecord和aplay
        arecord_proc = subprocess.Popen(
            ['arecord', '-f', 'S16_LE', '-r', '16000', '-c', '1', '-t', 'raw'],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=1024
        )
        
        aplay_proc = subprocess.Popen(
            ['aplay', '-f', 'S16_LE', '-r', '16000', '-c', '1', '-t', 'raw'],
            stdin=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        )
        
        print("音频流运行中...")
        
        chunk_size = 512
        display_interval = 0.2  # 显示间隔（秒）
        last_display = time.time()
        
        while not rospy.is_shutdown():
            # 使用select检查是否有数据可读
            if select.select([arecord_proc.stdout], [], [], 0.1)[0]:
                data = arecord_proc.stdout.read(chunk_size)
                
                if data:
                    # 传递给扬声器
                    aplay_proc.stdin.write(data)
                    
                    # 计算音量
                    audio_data = np.frombuffer(data, dtype=np.int16)
                    if len(audio_data) > 0:
                        volume = np.sqrt(np.mean(audio_data.astype(np.float64)**2))
                        
                        # 定期显示
                        current_time = time.time()
                        if current_time - last_display >= display_interval:
                            if volume > 50:  # 静音阈值
                                level = min(20, int(20 * volume / 2000))
                                bars = "█" * level + " " * (20 - level)
                                print(f"音量: [{bars}] {volume:.0f}    ", end='\r')
                            else:
                                print("音量: [                    ] 静音    ", end='\r')
                            last_display = current_time
                else:
                    break
            else:
                time.sleep(0.01)
                
    except KeyboardInterrupt:
        print("\n\n停止测试")
    except Exception as e:
        print(f"\n错误: {e}")
    finally:
        # 清理进程
        for proc in [arecord_proc, aplay_proc]:
            try:
                proc.terminate()
                proc.wait(timeout=1)
            except:
                try:
                    proc.kill()
                except:
                    pass

rospy.init_node('microphone_test', anonymous=True)
# 运行
optimized_volume_display()