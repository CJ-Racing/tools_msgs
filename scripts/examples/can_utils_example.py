#!/usr/bin/env python3
import can
import rospy


class CANUtils:
    """CAN 总线工具类"""

    def __init__(self, channel='can1', bitrate=500000):
        self.channel = channel
        self.bitrate = bitrate
        self.bus = None

    def init(self):
        """初始化 CAN 总线（参考 A23 ami.py 第 90 行）"""
        try:
            self.bus = can.interface.Bus(
                interface='socketcan',
                channel=self.channel,
                bitrate=self.bitrate,
                fd=True
            )
            rospy.loginfo(f"CAN 总线初始化成功：{self.channel}")
            return True
        except can.CanError as e:
            rospy.logerr(f"CAN 总线初始化失败：{e}")
            self.bus = None
            return False

    def send(self, arbitration_id, data, extended_id=False):
        """发送 CAN 帧
        Args:
            arbitration_id: CAN ID，如 0x02（传感器状态）、0x141（EBS）
            data: 8 字节列表
            extended_id: 是否扩展 ID
        """
        if not self.bus:
            rospy.logerr("CAN 未初始化")
            return False
        try:
            msg = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=extended_id)
            self.bus.send(msg)
            return True
        except can.CanError as e:
            rospy.logwarn(f"CAN 发送失败 ID=0x{arbitration_id:X}：{e}")
            return False

    def recv_once(self, timeout=0.1):
        """非阻塞接收一帧（参考 A23 ami.py bus.recv()）
        
        Returns:
            can.Message 或 None
        """
        if not self.bus:
            return None
        try:
            return self.bus.recv(timeout=timeout)
        except can.CanError as e:
            rospy.logwarn(f"CAN 接收异常：{e}")
            return None

    def parse_task_signal(self, msg):
        """解析任务选择信号（参考 A23 ami.py 第 98-113 行）
        
        ID 0x7B：data[0]=任务信号，data[1]=确认信号
        
        Returns:
            (task_signal, confirm_signal) 或 None
        """
        if msg is None or msg.arbitration_id != 0x7B:
            return None
        task_signal = int(msg.data[0])
        confirm_signal = int(msg.data[1])
        return task_signal, confirm_signal

    def send_sensor_status(self, nav_err, camera_err, lidar_err):
        """发送传感器状态到 ECU（参考 A23 error.py，ID 0x02）"""
        data = [
            0x01 if nav_err else 0x00,
            0x01 if camera_err else 0x00,
            0x01 if lidar_err else 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00
        ]
        return self.send(0x02, data)

    def send_speed_cmd(self, target_speed, acceleration, current_speed):
        """发送速度指令到 VCU
        
        Args:
            target_speed: 目标速度 (m/s)
            acceleration: 加速度 (m/s²)
            current_speed: 当前速度 (m/s)
        """
        data = [0x00] * 6
        data[0] = int(target_speed * 10) & 0xFF
        data[1] = int((acceleration + 1) * 20) & 0xFF
        data[2] = int(current_speed * 10) & 0xFF
        return self.send(0x01, data)

    def close(self):
        """关闭 CAN 总线"""
        if self.bus and self.bus.is_open:
            self.bus.close()
            rospy.loginfo("CAN 总线已关闭")


# 使用示例
if __name__ == '__main__':
    rospy.init_node('can_utils_test')

    can_utils = CANUtils(channel='can1')
    if not can_utils.init():
        exit(1)

    # 发送传感器正常状态
    can_utils.send_sensor_status(nav_err=False, camera_err=False, lidar_err=False)

    # 监听任务选择信号（类似 A23 ami.py 的主循环）
    count = 0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = can_utils.recv_once()
        if msg:
            count += 1
            if count > 10:
                result = can_utils.parse_task_signal(msg)
                if result:
                    task, confirm = result
                    rospy.loginfo(f"任务信号：{task}，确认：{confirm}")
        rate.sleep()

    can_utils.close()
