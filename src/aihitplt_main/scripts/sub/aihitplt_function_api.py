#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import time
import cv2
import subprocess
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int16, String, Bool, Float32
from playsound import playsound

class aihitplt_function_api:
    def __init__(self):
        # 初始化变量
        self.round_panel_state = False
        self.round_panel_detecting = False
        self.round_panel_last_state = False
        self.round_panel_detected = False
        
        self.yolo_process = None
        self.yolo_running = False
        
        # 创建发布者
        self.spray_control_pub_data = rospy.Publisher("/spray_control", Bool, queue_size=10)

        # 旋钮屏幕按钮状态订阅
        self.round_panel_sub = rospy.Subscriber("/round_panel", Bool, self.round_panel_callback)
        
        rospy.loginfo("aihitplt_function_api initialized")
        rospy.sleep(0.5)

    def pan_cam_image_save(self, path, topic_name): 
        """
        云台相机话题保存
        Args:
            path：保存路径，例如"/home/aihit/aihitplt_ws/src"
            topic: 相机图像话题名称，例如"/pan_tilt_camera/image"
        """
        cam0_path = path 
        bridge = CvBridge()
        cv_img = rospy.wait_for_message(topic_name, Image)
        cv_img = bridge.imgmsg_to_cv2(cv_img, "bgr8")
        cv2.imwrite(cam0_path, cv_img)
        rospy.loginfo(f"Image saved to: {cam0_path}")
        return True
    
    def yolo_data(self, data):
        """
        控制YOLO检测的启动/停止
        """
        if data == 1:
            if self.yolo_running:
                rospy.logwarn("YOLO检测已经在运行")
                return True
            
            try:
                self.yolo_process = subprocess.Popen([
                    "roslaunch", "aihitplt_yolov4_tiny", "yolodetect_deepcam.launch"
                ])
                
                self.yolo_running = True
                rospy.loginfo("YOLO检测系统启动中...")
                rospy.sleep(3)
                rospy.loginfo("YOLO检测系统启动完成")
                return True
                
            except Exception as e:
                rospy.logerr(f"启动YOLO检测失败: {e}")
                self.yolo_running = False
                return False
                
        elif data == 0:
            if not self.yolo_running:
                rospy.logwarn("YOLO检测未在运行")
                return True
            
            try:
                subprocess.call(["rosnode", "kill", "/YoloDetect_deepcam"])
                subprocess.call(["rosnode", "kill", "/msgToimg_deepcam"])
                
                if self.yolo_process:
                    self.yolo_process.terminate()
                    self.yolo_process.wait(timeout=5)
                
                self.yolo_running = False
                self.yolo_process = None
                rospy.loginfo("YOLO检测系统已停止")
                return True
                
            except Exception as e:
                rospy.logerr(f"停止YOLO检测失败: {e}")
                return False
        else:
            rospy.logwarn(f"未知的YOLO命令: {data}")
            return False
    
    def get_yolo_result(self, timeout=10.0):
        """
        获取YOLO检测结果
        """
        if not self.yolo_running:
            rospy.logwarn("YOLO检测未启动")
            return []
        
        try:
            from aihitplt_msgs.msg import TargetArray
            msg = rospy.wait_for_message("DetectMsg", TargetArray, timeout=timeout)
            
            results = []
            for target in msg.data:
                result = {
                    'class': target.frame_id,
                    'confidence': float(target.scores),
                    'x': float(target.ptx),
                    'y': float(target.pty),
                    'width': float(target.distw),
                    'height': float(target.disth),
                    'center_x': float(target.centerx),
                    'center_y': float(target.centery)
                }
                results.append(result)
            
            rospy.loginfo(f"获取到{len(results)}个检测结果")
            return results
            
        except rospy.ROSException as e:
            if "timeout" in str(e):
                rospy.logwarn(f"等待YOLO结果超时 ({timeout}秒)")
            else:
                rospy.logerr(f"获取YOLO结果异常: {e}")
            return []
        except Exception as e:
            rospy.logerr(f"处理YOLO结果失败: {e}")
            return []
    
    def play_sound(self, path):
        """
        音频播放
        Args:
            path : 播放音频文件路径
        """
        try:
            playsound(path)
            rospy.loginfo(f"Audio played successfully: {path}")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to play audio {path}: {e}")
            return False
    
    def spray_control(self, data):
        """
        控制喷雾消毒模块
        Args:
            data：布尔值
        """
        try:
            if not isinstance(data, bool): 
                rospy.logwarn(f"Invalid spray control data type: {type(data)}, expected bool")
                return False            
            
            rospy.sleep(0.2)
            msg = Bool()
            msg.data = data
            self.spray_control_pub_data.publish(msg)
            action = "开启" if data else "关闭"
            rospy.loginfo(f"喷雾消毒{action}指令已发送")            
            return True
        except Exception as e:
            rospy.logerr(f"喷雾控制失败: {e}")
            return False

    def round_panel_callback(self, msg):
        """
        圆形屏幕按钮状态回调函数
        """
        self.round_panel_state = msg.data
        
        if self.round_panel_detecting and not self.round_panel_last_state and msg.data:
            rospy.loginfo("检测到圆形屏点击事件")
            self.round_panel_detected = True
        
        self.round_panel_last_state = msg.data
    
    def wait_for_round_panel_click(self, timeout=None):
        """
        等待圆形屏点击事件
        """
        rospy.loginfo("开始检测圆形屏状态...")
        
        self.round_panel_detected = False
        self.round_panel_last_state = self.round_panel_state
        self.round_panel_detecting = True
        
        start_time = time.time()
        
        try:
            while not rospy.is_shutdown():
                if self.round_panel_detected:
                    self.round_panel_detecting = False
                    return True
                
                if timeout is not None and (time.time() - start_time) > timeout:
                    rospy.logwarn(f"圆形屏检测超时 ({timeout}秒)")
                    self.round_panel_detecting = False
                    return False
                
                rospy.sleep(0.1)
                
        except Exception as e:
            rospy.logerr(f"圆形屏检测异常: {e}")
            self.round_panel_detecting = False
            return False
        
        self.round_panel_detecting = False
        return False
    
    def get_round_panel_state(self):
        """
        获取当前圆形屏状态
        """
        return self.round_panel_state
    
    def is_round_panel_detecting(self):
        """
        检查是否正在检测圆形屏
        """
        return self.round_panel_detecting
    
    def stop_round_panel_detection(self):
        """
        停止圆形屏检测
        """
        if self.round_panel_detecting:
            self.round_panel_detecting = False
            rospy.loginfo("已停止圆形屏检测")
            return True
        return False

if __name__ == "__main__":
    try:
        rospy.init_node("aihitplt_function_api", anonymous=True)
        api = aihitplt_function_api()
        rospy.loginfo("启动成功")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass