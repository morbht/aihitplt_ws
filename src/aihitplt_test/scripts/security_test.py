# memory_monitor.py
import tracemalloc
import psutil
import os

class MemoryMonitor:
    def __init__(self):
        tracemalloc.start()
        self.process = psutil.Process(os.getpid())
    
    def log_memory(self):
        """记录当前内存使用"""
        current, peak = tracemalloc.get_traced_memory()
        mem_info = self.process.memory_info()
        print(f"内存使用 - 当前: {current/1024/1024:.2f}MB, "
              f"峰值: {peak/1024/1024:.2f}MB, "
              f"RSS: {mem_info.rss/1024/1024:.2f}MB")
        
        # 检查是否有内存泄漏趋势
        if current > 500 * 1024 * 1024:  # 500MB
            print("警告: 内存使用过高!")
            self.show_top_allocations()
    
    def show_top_allocations(self, limit=10):
        """显示内存分配最多的代码位置"""
        snapshot = tracemalloc.take_snapshot()
        top_stats = snapshot.statistics('lineno')
        print(f"内存分配Top {limit}:")
        for stat in top_stats[:limit]:
            print(stat)
