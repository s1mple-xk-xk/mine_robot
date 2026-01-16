import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math


class GetRobotPose(Node):
    """获取机器人在地图中的实时位姿"""

    def __init__(self):
        super().__init__('get_robot_pose')
        
        # 设置使用仿真时间
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        # TF 缓冲区，用于存储TF变换
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 坐标系名称
        self.target_frame = 'map'  # 目标坐标系（地图坐标系）
        self.source_frame = 'base_footprint'  # 源坐标系（机器人基坐标系）
        
        # 状态标志
        self.map_frame_available = False  # map 帧是否可用
        self.warning_count = 0  # 警告计数器，用于减少警告频率
        self.last_warning_time = None  # 上次警告时间
        
        # 创建定时器，定期获取位姿
        self.timer = self.create_timer(0.1, self.get_pose_callback)  # 10Hz
        
        self.get_logger().info('开始监听机器人位姿...（使用仿真时间）')
        self.get_logger().info(f'监听变换: {self.target_frame} -> {self.source_frame}')
        self.get_logger().info('等待 map 帧可用（需要 AMCL 初始化完成）...')

    def check_frame_available(self):
        """检查 map 帧是否可用"""
        try:
            # 尝试查找变换
            # 如果 map 帧不存在，会抛出异常
            self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time()
            )
            return True
        except TransformException:
            return False

    def get_pose_callback(self):
        """定时器回调函数，获取并打印机器人位姿"""
        # 首先检查 map 帧是否可用
        if not self.map_frame_available:
            if self.check_frame_available():
                self.map_frame_available = True
                self.get_logger().info('map 帧已可用，开始获取机器人位姿')
            else:
                # 减少警告频率：每5秒警告一次
                current_time = self.get_clock().now()
                if (self.last_warning_time is None or 
                    (current_time - self.last_warning_time).nanoseconds > 5e9):
                    self.get_logger().warn(
                        f'等待 map 帧可用（AMCL 可能尚未初始化完成）... '
                        f'请确保 Nav2 系统已启动且初始位姿已设置'
                    )
                    self.last_warning_time = current_time
                return
        
        try:
            # 查找从 map 到 base_footprint 的变换
            # 使用 rclpy.time.Time() 表示使用最新可用的变换，避免时间外推
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time()
            )
            
            # 提取位置信息
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            
            # 将四元数转换为欧拉角（偏航角）
            yaw = self.quaternion_to_yaw(rotation)
            
            # 打印位姿信息
            self.get_logger().info(
                f'机器人位姿 - X: {translation.x:.3f}, '
                f'Y: {translation.y:.3f}, '
                f'Yaw: {math.degrees(yaw):.2f}°'
            )
            
        except TransformException as ex:
            # 如果之前可用但现在不可用，可能是系统重启或AMCL停止
            if self.map_frame_available:
                self.map_frame_available = False
                self.get_logger().warn(
                    f'map 帧变为不可用: {ex}'
                )
            # 减少警告频率
            self.warning_count += 1
            if self.warning_count % 50 == 0:  # 每5秒警告一次（10Hz * 50 = 5秒）
                self.get_logger().warn(
                    f'无法获取变换 {self.target_frame} -> {self.source_frame}: {ex}'
                )

    def quaternion_to_yaw(self, quaternion):
        """将四元数转换为偏航角（绕Z轴旋转）"""
        # 四元数转欧拉角公式
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def get_current_pose(self):
        """获取当前机器人位姿，返回 PoseStamped 消息"""
        try:
            # 使用 rclpy.time.Time() 表示使用最新可用的变换，避免时间外推
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time()
            )
            
            # 创建 PoseStamped 消息
            pose = PoseStamped()
            pose.header.frame_id = self.target_frame
            pose.header.stamp = transform.header.stamp
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            
            return pose
            
        except TransformException as ex:
            self.get_logger().error(
                f'无法获取变换 {self.target_frame} -> {self.source_frame}: {ex}'
            )
            return None


def main(args=None):
    rclpy.init(args=args)
    
    node = GetRobotPose()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

