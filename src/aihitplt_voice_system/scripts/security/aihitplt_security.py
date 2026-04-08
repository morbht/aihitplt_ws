#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
科大讯飞语音合成ROS节点
实现文本转语音(TTS)功能，支持离线合成和播放
"""

import os
import sys
import time
import struct
import subprocess
from ctypes import *
import rospy
from std_msgs.msg import String, Int32


class AihitTTSNode:
    """语音合成节点类"""
    
    # 常量定义
    SDK_LIB_PATH = '/home/aihit/aihitplt_ws/src/aihitplt_voice_system/libs/x64/libmsc.so'
    BASE_DIR = '/home/aihit/aihitplt_ws/src/aihitplt_voice_system/Linux_xtts_exp1227_0daad079'
    RES_PATH = os.path.join(BASE_DIR, 'bin', 'msc', 'res', 'xtts')
    VOICE_FILE = '/home/aihit/aihitplt_ws/src/aihitplt_voice_system/wav/security/voice.wav'
    
    # WAV文件格式常量
    SAMPLE_RATE = 16000
    CHANNELS = 1
    BITS_PER_SAMPLE = 16
    
    def __init__(self):
        """初始化ROS节点"""
        rospy.init_node('security_tts_node', anonymous=True)
        
        # 加载配置参数
        self._load_params()
        
        # 设置环境变量
        self._setup_environment()
        
        # 加载SDK库
        self._load_sdk()
        
        # 初始化ROS通信
        self._init_ros_comms()
        
        # 登录讯飞SDK
        if not self._login():
            rospy.logerr("讯飞SDK登录失败")
            sys.exit(1)
        
        rospy.loginfo("语音合成节点启动成功")
        
        # 合成文本输入
        self.synthesize_and_play("未检测到异常火源情况，解除应急模式")
        
        rospy.spin()
    
    def _load_params(self):
        """加载ROS参数"""
        self.appid = rospy.get_param('~appid', '0daad079')
        self.work_dir = rospy.get_param('~work_dir', 
            '/home/aihit/aihitplt_ws/src/aihitplt_voice_system/bin/msc')
    
    def _setup_environment(self):
        """设置系统环境变量"""
        lib_path = '/home/aihit/aihitplt_ws/src/aihitplt_voice_system/libs/x64'
        os.environ['LD_LIBRARY_PATH'] = f"{lib_path}:{os.environ.get('LD_LIBRARY_PATH', '')}"
        os.environ['MSC_CFG_PATH'] = self.work_dir
    
    def _load_sdk(self):
        """加载讯飞SDK动态库"""
        try:
            self.msc = CDLL(self.SDK_LIB_PATH)
            rospy.loginfo(f"加载SDK成功: {self.SDK_LIB_PATH}")
        except Exception as e:
            rospy.logerr(f"加载SDK失败: {e}")
            sys.exit(1)
        
        # 定义函数原型
        self._define_function_prototypes()
    
    def _define_function_prototypes(self):
        """定义SDK函数参数和返回类型"""
        # MSPLogin
        self.msc.MSPLogin.argtypes = [c_char_p, c_char_p, c_char_p]
        self.msc.MSPLogin.restype = c_int
        
        # MSPLogout
        self.msc.MSPLogout.argtypes = []
        self.msc.MSPLogout.restype = c_int
        
        # QTTSSessionBegin
        self.msc.QTTSSessionBegin.argtypes = [c_char_p, POINTER(c_int)]
        self.msc.QTTSSessionBegin.restype = c_char_p
        
        # QTTSTextPut
        self.msc.QTTSTextPut.argtypes = [c_char_p, c_char_p, c_uint, c_char_p]
        self.msc.QTTSTextPut.restype = c_int
        
        # QTTSAudioGet
        self.msc.QTTSAudioGet.argtypes = [c_char_p, POINTER(c_int), POINTER(c_int), POINTER(c_int)]
        self.msc.QTTSAudioGet.restype = c_void_p
        
        # QTTSSessionEnd
        self.msc.QTTSSessionEnd.argtypes = [c_char_p, c_char_p]
        self.msc.QTTSSessionEnd.restype = c_int
    
    def _init_ros_comms(self):
        """初始化ROS发布订阅"""
        self.sub = rospy.Subscriber('/voice/aihitplt_nlu_topic', String, self.callback)
        self.pub = rospy.Publisher('/voice/aihitplt_tts_topic', Int32, queue_size=10)
    
    def _login(self):
        """登录讯飞SDK"""
        login_params = f"appid = {self.appid}, work_dir = ."
        rospy.loginfo(f"登录参数: {login_params}")
        
        ret = self.msc.MSPLogin(None, None, login_params.encode('utf-8'))
        if ret != 0:
            rospy.logerr(f"登录失败: {ret}")
            return False
        
        return True
    
    def synthesize_and_play(self, text):
        """合成并播放语音"""
        if self._synthesize(text, self.VOICE_FILE):
            self._play_audio(self.VOICE_FILE)
    
    def _synthesize(self, text, filename):
        """合成语音到文件"""
        try:
            # 检查资源文件
            xiaoyan_jet = os.path.join(self.RES_PATH, 'xiaoyan.jet')
            common_jet = os.path.join(self.RES_PATH, 'common.jet')
            
            if not all(os.path.exists(p) for p in [xiaoyan_jet, common_jet]):
                rospy.logerr("资源文件不存在")
                return False
            
            # 构建合成参数
            params = (
                f"engine_type = purextts,"
                f"voice_name = xiaoyan,"
                f"text_encoding = UTF8,"
                f"tts_res_path = fo|{xiaoyan_jet};fo|{common_jet},"
                f"sample_rate = {self.SAMPLE_RATE},"
                f"speed = 50,"
                f"volume = 50,"
                f"pitch = 50,"
                f"rdn = 2"
            )
            
            return self._synthesize_basic(text, filename, params)
            
        except Exception as e:
            rospy.logerr(f"合成异常: {e}")
            return False
    
    def _synthesize_basic(self, text, filename, params):
        """基础合成函数"""
        try:
            # 开始会话
            err_code = c_int(0)
            session_id = self.msc.QTTSSessionBegin(params.encode('utf-8'), byref(err_code))
            
            if err_code.value != 0 or not session_id:
                rospy.logerr(f"会话创建失败: {err_code.value}")
                return False
            
            # 发送文本
            text_bytes = text.encode('utf-8')
            ret = self.msc.QTTSTextPut(session_id, text_bytes, len(text_bytes), None)
            
            if ret != 0:
                rospy.logerr(f"文本发送失败: {ret}")
                self.msc.QTTSSessionEnd(session_id, b"TextPutError")
                return False
            
            # 获取音频数据
            audio_data = self._get_audio_data(session_id)
            
            # 结束会话
            self.msc.QTTSSessionEnd(session_id, b"Normal")
            
            if not audio_data:
                return False
            
            # 保存为WAV文件
            self._save_wav_file(filename, audio_data)
            
            return True
            
        except Exception as e:
            rospy.logerr(f"合成过程异常: {e}")
            return False
    
    def _get_audio_data(self, session_id):
        """获取音频数据"""
        audio_data = bytearray()
        
        while True:
            audio_len = c_int(0)
            synth_status = c_int(0)
            err_code = c_int(0)
            
            data_ptr = self.msc.QTTSAudioGet(
                session_id,
                byref(audio_len),
                byref(synth_status),
                byref(err_code)
            )
            
            if err_code.value != 0:
                rospy.logerr(f"获取音频失败: {err_code.value}")
                break
            
            if data_ptr and audio_len.value > 0:
                data = string_at(data_ptr, audio_len.value)
                audio_data.extend(data)
            
            if synth_status.value == 2:  # 数据结束
                break
            
            time.sleep(0.01)
        
        return bytes(audio_data) if audio_data else None
    
    def _create_wav_header(self, data_size=0):
        """创建WAV文件头"""
        header = bytearray(44)
        byte_rate = self.SAMPLE_RATE * self.CHANNELS * self.BITS_PER_SAMPLE // 8
        block_align = self.CHANNELS * self.BITS_PER_SAMPLE // 8
        
        # RIFF头
        header[0:4] = b'RIFF'
        struct.pack_into('<I', header, 4, data_size + 36)
        header[8:12] = b'WAVE'
        
        # fmt块
        header[12:16] = b'fmt '
        struct.pack_into('<I', header, 16, 16)           # fmt块大小
        struct.pack_into('<H', header, 20, 1)            # PCM格式
        struct.pack_into('<H', header, 22, self.CHANNELS)
        struct.pack_into('<I', header, 24, self.SAMPLE_RATE)
        struct.pack_into('<I', header, 28, byte_rate)
        struct.pack_into('<H', header, 32, block_align)
        struct.pack_into('<H', header, 34, self.BITS_PER_SAMPLE)
        
        # data块
        header[36:40] = b'data'
        struct.pack_into('<I', header, 40, data_size)
        
        return header
    
    def _save_wav_file(self, filename, audio_data):
        """保存WAV文件"""
        try:
            header = self._create_wav_header(len(audio_data))
            
            with open(filename, 'wb') as f:
                f.write(header)
                f.write(audio_data)
            
            rospy.loginfo(f"文件保存成功: {filename} ({len(audio_data)} 字节)")
            
        except Exception as e:
            rospy.logerr(f"保存文件失败: {e}")
    
    def _play_audio(self, filename):
        """播放音频文件"""
        if not os.path.exists(filename) or os.path.getsize(filename) <= 44:
            rospy.logerr("音频文件无效")
            return
        
        # 通知ASR节点
        self.pub.publish(Int32(0))
        
        try:
            result = subprocess.run(['aplay', filename], capture_output=True, text=True)
            
            if result.returncode != 0:
                rospy.logwarn(f"aplay播放失败，尝试play命令")
                subprocess.run(['play', filename], stderr=subprocess.DEVNULL)
            
            rospy.loginfo("音频播放完成")
            
        except Exception as e:
            rospy.logerr(f"播放失败: {e}")
    
    def callback(self, msg):
        """ROS消息回调"""
        rospy.loginfo(f"收到文本: {msg.data}")
        self.synthesize_and_play(msg.data)
    
    def shutdown(self):
        """节点关闭清理"""
        try:
            self.msc.MSPLogout()
            rospy.loginfo("SDK已登出")
        except:
            pass


if __name__ == '__main__':
    try:
        node = AihitTTSNode()
        rospy.on_shutdown(node.shutdown)
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"节点启动异常: {e}")