#!/usr/bin/env python3
import subprocess
import rospy
import os


class ProcessManager:
    """进程管理器"""

    def __init__(self):
        self._processes = {}  # {name: Popen}

    def start(self, name, script_path):
        """启动 bash 脚本（非阻塞）
        
        Args:
            name: 模块名，如 "jiasu"、"bazi"
            script_path: shell 脚本路径
            
        Returns:
            bool: 是否成功启动
        """
        if not os.path.exists(script_path):
            rospy.logerr(f"脚本不存在：{script_path}")
            return False

        if name in self._processes and self._processes[name].poll() is None:
            rospy.logwarn(f"{name} 已在运行，跳过")
            return False

        try:
            proc = subprocess.Popen(["bash", script_path])
            self._processes[name] = proc
            rospy.loginfo(f"已启动：{name} ({script_path})")
            return True
        except Exception as e:
            rospy.logerr(f"启动失败 {name}：{e}")
            return False

    def stop(self, name):
        """停止指定模块
        
        Args:
            name: 模块名
        """
        if name not in self._processes:
            return
        proc = self._processes[name]
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
        del self._processes[name]
        rospy.loginfo(f"已停止：{name}")

    def stop_all(self):
        """停止所有模块（在 COMPLETE/EMERGENCY 状态调用）"""
        for name in list(self._processes.keys()):
            self.stop(name)
        rospy.loginfo("所有模块已停止")

    def is_running(self, name):
        """检查模块是否在运行
        
        Returns:
            bool
        """
        return name in self._processes and self._processes[name].poll() is None

    def list_running(self):
        """列出所有运行中的模块
        
        Returns:
            list[str]
        """
        return [name for name in self._processes if self._processes[name].poll() is None]


# 使用示例（模拟 AMI 状态机的调用方式）
if __name__ == '__main__':
    rospy.init_node('process_manager_test')

    TASK_SCRIPTS = {
        "INSPECTION":       rospy.get_param("~INSPECTION", ""),
        "MANUAL_DRIVERING": rospy.get_param("~MANUAL_DRIVERING", ""),
        "EBS_TEST":         rospy.get_param("~EBS_TEST", ""),
        "ACCELERATION":     rospy.get_param("~ACCELERATION", ""),
        "SKIDPAD":          rospy.get_param("~SKIDPAD", ""),
        "TRACKDRIVER":      rospy.get_param("~TRACKDRIVER", ""),
    }

    manager = ProcessManager()

    # 模拟 AMI RUNNING 状态：启动任务脚本
    task = "ACCELERATION"
    script = TASK_SCRIPTS.get(task, "")
    if script:
        manager.start(task, script)
        rospy.loginfo(f"运行中模块：{manager.list_running()}")
    else:
        rospy.logwarn(f"任务 {task} 未配置脚本路径")

    rospy.sleep(2)

    # 模拟 COMPLETE/EMERGENCY 状态：停止所有模块
    manager.stop_all()
