#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
from datetime import datetime

class AGVControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("AGV控制系统 - Ubuntu 20.04")
        self.root.geometry("1200x900")
        
        # 设置样式
        self.setup_styles()
        
        # 创建主容器
        self.main_container = ttk.Frame(root)
        self.main_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 创建分页控件
        self.create_notebook()
        
        # 创建状态栏
        self.create_status_bar()
        
    def setup_styles(self):
        """设置样式"""
        style = ttk.Style()
        style.theme_use('clam')
        
        # 自定义颜色
        style.configure('TNotebook.Tab', padding=[10, 5])
        style.configure('Red.TButton', background='#ff6b6b', foreground='white')
        style.configure('Green.TButton', background='#51cf66', foreground='white')
        style.configure('Blue.TButton', background='#339af0', foreground='white')
        style.configure('Disabled.TButton', background='#adb5bd', foreground='white')
        
    def create_notebook(self):
        """创建分页控件"""
        notebook = ttk.Notebook(self.main_container)
        notebook.pack(fill=tk.BOTH, expand=True)
        
        # AGV页面
        agv_frame = ttk.Frame(notebook)
        notebook.add(agv_frame, text='AGV控制')
        self.create_agv_page(agv_frame)
        
        # 上装页面
        upper_frame = ttk.Frame(notebook)
        notebook.add(upper_frame, text='上装模块')
        self.create_upper_page(upper_frame)
    
    def create_agv_page(self, parent):
        """创建AGV控制页面"""
        # 主容器 - 使用PanedWindow实现左右分割
        main_pane = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        main_pane.pack(fill=tk.BOTH, expand=True)
        
        # 左侧功能区
        left_frame = ttk.Frame(main_pane)
        main_pane.add(left_frame, weight=2)
        
        # 右侧可视化区域
        right_frame = ttk.Frame(main_pane, relief=tk.RAISED, borderwidth=2)
        main_pane.add(right_frame, weight=3)
        
        # 创建左侧功能面板
        self.create_agv_left_panel(left_frame)
        
        # 创建右侧可视化面板
        self.create_agv_right_panel(right_frame)
    
    def create_agv_left_panel(self, parent):
        """创建AGV左侧功能面板"""
        # IP地址设置
        ip_frame = ttk.LabelFrame(parent, text="IP地址设置", padding=10)
        ip_frame.pack(fill=tk.X, padx=5, pady=(0, 10))
        
        ttk.Label(ip_frame, text="AGV IP地址:").grid(row=0, column=0, padx=(0, 5))
        self.ip_entry = ttk.Entry(ip_frame, width=20)
        self.ip_entry.grid(row=0, column=1, padx=(0, 10))
        self.ip_entry.insert(0, "192.168.1.100")
        
        ttk.Button(ip_frame, text="连接", command=self.connect_agv).grid(row=0, column=2)
        
        # 控制按钮区域
        control_frame = ttk.LabelFrame(parent, text="AGV下位机通信控制", padding=10)
        control_frame.pack(fill=tk.X, padx=5, pady=(0, 10))
        
        # 方向控制按钮
        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=0, column=0, columnspan=3, pady=10)
        
        # 创建方向控制按钮网格
        directions = [
            ('', '↑', ''),
            ('←', '停止', '→'),
            ('', '↓', '')
        ]
        
        positions = [(1, 0), (0, 1), (1, 1), (1, 2), (2, 1)]
        buttons = ['↑', '←', '停止', '→', '↓']
        
        for i, (btn_text, pos) in enumerate(zip(buttons, positions)):
            btn = ttk.Button(button_frame, text=btn_text, width=5,
                            command=lambda t=btn_text: self.control_agv(t))
            btn.grid(row=pos[0], column=pos[1], padx=2, pady=2)
            setattr(self, f'direction_{btn_text}', btn)
        
        # 传感器状态显示
        sensor_frame = ttk.LabelFrame(parent, text="传感器状态可视化", padding=10)
        sensor_frame.pack(fill=tk.X, padx=5, pady=(0, 10))
        
        sensors = [
            ("超声波", self.toggle_sensor),
            ("防跌落", self.toggle_sensor),
            ("防碰撞", self.toggle_sensor),
            ("急停", self.toggle_sensor),
            ("自定义按钮", self.toggle_sensor)
        ]
        
        for i, (name, cmd) in enumerate(sensors):
            btn = ttk.Button(sensor_frame, text=name, command=cmd)
            btn.grid(row=i//3, column=i%3, padx=5, pady=5, sticky='ew')
        
        # 功能测试区域
        test_frame = ttk.LabelFrame(parent, text="功能测试", padding=10)
        test_frame.pack(fill=tk.X, padx=5, pady=(0, 10))
        
        test_buttons = [
            ("启动键盘控制", self.toggle_keyboard),
            ("打开深度相机", self.toggle_depth_camera),
            ("打开激光雷达", self.toggle_lidar),
            ("旋钮屏幕测试", self.toggle_knob_test),
            ("磁导航传感器测试", self.toggle_mag_sensor),
            ("麦克风测试", self.toggle_microphone),
            ("WIFI名称修改", self.modify_wifi)
        ]
        
        for i, (text, cmd) in enumerate(test_buttons):
            btn = ttk.Button(test_frame, text=text, command=cmd)
            btn.grid(row=i//2, column=i%2, padx=5, pady=5, sticky='ew')
            setattr(self, f'test_btn_{i}', btn)
        
        # 状态指示灯
        status_frame = ttk.Frame(parent)
        status_frame.pack(fill=tk.X, padx=5, pady=10)
        
        self.connection_status = tk.Label(status_frame, text="● 未连接", fg="red")
        self.connection_status.pack(side=tk.LEFT, padx=(0, 20))
        
        self.emergency_status = tk.Label(status_frame, text="急停状态: 正常", fg="green")
        self.emergency_status.pack(side=tk.LEFT)
    
    def create_agv_right_panel(self, parent):
        """创建AGV右侧可视化面板"""
        # 标题
        title_label = ttk.Label(parent, text="AGV可视化界面", font=('Arial', 14, 'bold'))
        title_label.pack(pady=(10, 5))
        
        # 可视化显示区域
        canvas_frame = ttk.Frame(parent, relief=tk.SUNKEN, borderwidth=2)
        canvas_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 创建模拟的可视化画布
        self.visual_canvas = tk.Canvas(canvas_frame, bg='#f8f9fa')
        self.visual_canvas.pack(fill=tk.BOTH, expand=True)
        
        # 在画布上绘制AGV示意图
        self.draw_agv_schematic()
        
        # 状态信息显示
        info_frame = ttk.Frame(parent)
        info_frame.pack(fill=tk.X, padx=10, pady=(0, 10))
        
        ttk.Label(info_frame, text="速度:").pack(side=tk.LEFT, padx=(0, 5))
        self.speed_label = ttk.Label(info_frame, text="0 m/s")
        self.speed_label.pack(side=tk.LEFT, padx=(0, 20))
        
        ttk.Label(info_frame, text="方向:").pack(side=tk.LEFT, padx=(0, 5))
        self.direction_label = ttk.Label(info_frame, text="停止")
        self.direction_label.pack(side=tk.LEFT, padx=(0, 20))
        
        ttk.Label(info_frame, text="电池:").pack(side=tk.LEFT, padx=(0, 5))
        self.battery_label = ttk.Label(info_frame, text="100%")
        self.battery_label.pack(side=tk.LEFT)
    
    def draw_agv_schematic(self):
        """在画布上绘制AGV示意图"""
        width = self.visual_canvas.winfo_reqwidth() or 400
        height = self.visual_canvas.winfo_reqheight() or 300
        
        # 清空画布
        self.visual_canvas.delete("all")
        
        # 绘制AGV主体
        center_x = width // 2
        center_y = height // 2
        agv_width = 200
        agv_height = 150
        
        # AGV车身
        self.visual_canvas.create_rectangle(
            center_x - agv_width//2, center_y - agv_height//2,
            center_x + agv_width//2, center_y + agv_height//2,
            fill='#4dabf7', outline='#339af0', width=2
        )
        
        # 轮子
        wheel_radius = 15
        positions = [
            (center_x - agv_width//3, center_y + agv_height//2 + 5),
            (center_x + agv_width//3, center_y + agv_height//2 + 5),
            (center_x - agv_width//3, center_y - agv_height//2 - 5),
            (center_x + agv_width//3, center_y - agv_height//2 - 5)
        ]
        
        for x, y in positions:
            self.visual_canvas.create_oval(
                x - wheel_radius, y - wheel_radius,
                x + wheel_radius, y + wheel_radius,
                fill='#495057', outline='#343a40'
            )
        
        # 传感器位置示意
        sensor_points = [
            (center_x, center_y - agv_height//2 - 10, "超声波", "red"),
            (center_x - agv_width//2 - 10, center_y, "防碰撞", "orange"),
            (center_x + agv_width//2 + 10, center_y, "防碰撞", "orange"),
            (center_x, center_y + agv_height//2 + 10, "防跌落", "yellow")
        ]
        
        for x, y, name, color in sensor_points:
            self.visual_canvas.create_oval(
                x - 8, y - 8, x + 8, y + 8,
                fill=color, outline='black'
            )
            self.visual_canvas.create_text(x, y - 15, text=name, font=('Arial', 8))
        
        # 添加方向箭头
        self.visual_canvas.create_line(
            center_x, center_y - 30,
            center_x, center_y - 60,
            arrow=tk.LAST, width=3, fill='#51cf66'
        )
    
    def create_upper_page(self, parent):
        """创建上装模块页面"""
        # 使用Canvas和Scrollbar实现滚动
        canvas = tk.Canvas(parent)
        scrollbar = ttk.Scrollbar(parent, orient=tk.VERTICAL, command=canvas.yview)
        
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # 页面标题
        title_label = ttk.Label(scrollable_frame, 
                               text="上装模块控制面板", 
                               font=('Arial', 16, 'bold'))
        title_label.pack(pady=(10, 20))
        
        # 创建模块按钮
        modules = [
            ("迎宾模块和送餐模块急停按钮测试", self.toggle_module),
            ("安防模块传感器系统测试", self.toggle_module),
            ("安放模块工业云台相机测试", self.toggle_module),
            ("喷雾模块硬件系统测试", self.toggle_module),
            ("AI套件传感器系统测试", self.toggle_module),
            ("AI套件麦克风阵列测试", self.toggle_module),
            ("工业物流传感器系统测试", self.toggle_module),
            ("送物模块硬件系统测试", self.toggle_module),
            ("机械臂调试工具", self.toggle_module),
            ("迎宾模块和机械臂USB摄像头测试", self.toggle_module)
        ]
        
        # 创建按钮并存储引用
        self.upper_buttons = []
        
        for text, command in modules:
            frame = ttk.Frame(scrollable_frame)
            frame.pack(fill=tk.X, padx=20, pady=5)
            
            btn = ttk.Button(frame, text=text, command=lambda t=text: command(t),
                            width=50)
            btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 10))
            
            status_label = ttk.Label(frame, text="● 未启动", foreground="red")
            status_label.pack(side=tk.RIGHT)
            
            self.upper_buttons.append((btn, status_label))
        
        # 添加说明
        note_frame = ttk.LabelFrame(scrollable_frame, text="使用说明", padding=10)
        note_frame.pack(fill=tk.X, padx=20, pady=20)
        
        note_text = """
        1. 每个模块启动时，其他模块将自动禁用
        2. 绿色按钮表示模块正在运行
        3. 再次点击运行中的模块可停止该模块
        4. 模块间切换时，请先停止当前运行模块
        """
        
        note_label = ttk.Label(note_frame, text=note_text, justify=tk.LEFT)
        note_label.pack()
    
    def create_status_bar(self):
        """创建状态栏"""
        status_bar = ttk.Frame(self.root, relief=tk.SUNKEN)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
        # 系统状态
        self.system_status = ttk.Label(status_bar, text="系统状态: 就绪")
        self.system_status.pack(side=tk.LEFT, padx=10)
        
        # CPU使用率
        self.cpu_usage = ttk.Label(status_bar, text="CPU: --%")
        self.cpu_usage.pack(side=tk.LEFT, padx=10)
        
        # 内存使用率
        self.memory_usage = ttk.Label(status_bar, text="内存: --%")
        self.memory_usage.pack(side=tk.LEFT, padx=10)
        
        # 网络状态
        self.network_status = ttk.Label(status_bar, text="网络: 已连接")
        self.network_status.pack(side=tk.LEFT, padx=10)
        
        # 时间显示
        self.time_label = ttk.Label(status_bar, text="")
        self.time_label.pack(side=tk.RIGHT, padx=10)
        self.update_time()
    
    def update_time(self):
        """更新时间显示"""
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.time_label.config(text=current_time)
        self.root.after(1000, self.update_time)
    
    # 以下是按钮回调函数（占位符）
    def connect_agv(self):
        """连接AGV"""
        ip = self.ip_entry.get()
        messagebox.showinfo("连接", f"尝试连接到AGV: {ip}")
        self.connection_status.config(text="● 已连接", fg="green")
    
    def control_agv(self, direction):
        """控制AGV移动"""
        directions_text = {
            '↑': "前进",
            '↓': "后退", 
            '←': "左转",
            '→': "右转",
            '停止': "停止"
        }
        self.direction_label.config(text=directions_text[direction])
        messagebox.showinfo("控制", f"AGV {directions_text[direction]}")
    
    def toggle_sensor(self):
        """切换传感器状态"""
        # 这里只是演示，实际需要实现具体逻辑
        pass
    
    def toggle_keyboard(self):
        """切换键盘控制"""
        messagebox.showinfo("键盘控制", "启动/停止键盘控制")
    
    def toggle_depth_camera(self):
        """切换深度相机"""
        messagebox.showinfo("深度相机", "打开/关闭深度相机")
    
    def toggle_lidar(self):
        """切换激光雷达"""
        messagebox.showinfo("激光雷达", "打开/关闭激光雷达")
    
    def toggle_knob_test(self):
        """旋钮测试"""
        messagebox.showinfo("旋钮测试", "开始/停止旋钮测试")
    
    def toggle_mag_sensor(self):
        """磁导航传感器测试"""
        messagebox.showinfo("磁导航", "开始/停止磁导航传感器测试")
    
    def toggle_microphone(self):
        """麦克风测试"""
        messagebox.showinfo("麦克风", "开始/停止麦克风测试")
    
    def modify_wifi(self):
        """修改WIFI名称"""
        new_wifi = simpledialog.askstring("WIFI修改", "请输入新的WIFI名称:")
        if new_wifi:
            messagebox.showinfo("WIFI修改", f"WIFI名称已修改为: {new_wifi}")
    
    def toggle_module(self, module_name):
        """切换模块状态"""
        messagebox.showinfo("模块控制", f"操作模块: {module_name}")
        
        # 这里可以实现禁用其他按钮的逻辑
        # for btn, label in self.upper_buttons:
        #     if btn['text'] != module_name:
        #         btn.state(['disabled'])

def main():
    root = tk.Tk()
    app = AGVControlGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()