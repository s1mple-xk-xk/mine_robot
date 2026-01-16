import math
import os

import cv2
import rclpy
from autopatrol_interfaces.srv import SpeachText
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_from_euler


class PatrolNode(BasicNavigator):
    """
    简单的巡逻导航节点，基于 nav2_simple_commander。
    """

    def __init__(self, node_name: str = "patrol_node") -> None:
        # 确保 rclpy 已初始化
        if not rclpy.ok():
            rclpy.init()

        super().__init__(node_name)

        # TF 监听器用于获取当前位姿
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # 导航相关参数
        self.declare_parameter("initial_point", [-2.0, -0.5, 0.0])
        self.declare_parameter("target_points", [-1.0, 0.0, 0.0, 1.0, 1.0, 1.57])

        self.initial_point_ = self.get_parameter("initial_point").value
        self.target_points_ = self.get_parameter("target_points").value
        # 语音合成客户端
        self.speach_client_ = self.create_client(SpeachText, "speech_text")

        # 订阅与保存图像相关定义
        self.declare_parameter('image_save_path', '/home/xk/study/mine_slam/images/')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.image_save_path = self.get_parameter('image_save_path').value
        self.image_topic = self.get_parameter('image_topic').value
        self.bridge = CvBridge()
        self.latest_image = None
        self.subscription_image = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10)

    def get_pose_by_xyyaw(self, x: float, y: float, yaw: float, frame_id: str = "map") -> PoseStamped:
        """
        通过 x, y, yaw 生成 PoseStamped。
        """
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, float(yaw))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def init_robot_pose(self) -> None:
        """
        使用 initial_point 参数初始化机器人在 map 坐标系下的位姿。
        """
        if len(self.initial_point_) != 3:
            self.get_logger().warn("initial_point 参数长度不为 3，跳过初始位姿设置")
            return

        init_pose = self.get_pose_by_xyyaw(*self.initial_point_)
        self.setInitialPose(init_pose)
        # 等待 Nav2 激活
        self.waitUntilNav2Active()
        self.get_logger().info(
            f"初始位姿已设置: x={init_pose.pose.position.x:.2f}, "
            f"y={init_pose.pose.position.y:.2f}, "
            f"yaw={math.degrees(self.initial_point_[2]):.1f}°"
        )

    def get_target_points(self, frame_id: str = "map") -> list[PoseStamped]:
        """
        通过参数获取目标点集合，按 [x, y, yaw] 三元组解析。
        """
        values = list(self.target_points_)
        if len(values) % 3 != 0:
            self.get_logger().warn("target_points 长度不是 3 的倍数，尾部数据将被忽略")
        poses: list[PoseStamped] = []
        for i in range(0, len(values) - len(values) % 3, 3):
            x, y, yaw = values[i : i + 3]
            poses.append(self.get_pose_by_xyyaw(x, y, yaw, frame_id))
        return poses

    def nav_to_pose(self, target_pose: PoseStamped) -> TaskResult:
        """
        导航到指定的目标位姿，并返回任务结果。
        """
        self.speach_text("开始导航到目标点")
        self.goToPose(target_pose)

        while not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback:
                self.get_logger().info(
                    f"导航中... 剩余距离: {feedback.distance_remaining:.2f} m, "
                    f"耗时: {feedback.navigation_time.sec} s"
                )
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("导航成功完成")
            self.speach_text("导航成功")
        elif result == TaskResult.CANCELED:
            self.get_logger().warn("导航被取消")
        elif result == TaskResult.FAILED:
            self.get_logger().error("导航失败")
        else:
            self.get_logger().error(f"未知导航结果: {result}")
        return result

    def get_current_pose(self, source_frame: str = "base_link", target_frame: str = "map") -> Pose | None:
        """
        通过 TF 获取当前位姿(map -> base_link)。
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, Time(), timeout=Duration(seconds=1.0)
            )
        except Exception as exc:  # TF 可抛出多种异常，统一记录
            self.get_logger().warn(f"获取当前位姿失败: {exc}")
            return None

        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    def say(self, text: str, wait_sec: float = 2.0) -> bool:
        """
        调用语音服务播报文本。
        """
        if text is None:
            return False
        if not self.speach_client_.wait_for_service(timeout_sec=wait_sec):
            self.get_logger().warn("语音服务不可用")
            return False

        req = SpeachText.Request()
        req.text = str(text)
        future = self.speach_client_.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=wait_sec)
        if future.result() is None:
            self.get_logger().warn("语音服务调用超时或失败")
            return False
        return bool(future.result().result)

    def speach_text(self, text: str) -> None:
        """
        调用服务播放语音（按代码清单 7-45 风格）。
        """
        while not self.speach_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("语音合成服务未上线，等待中 ...")

        request = SpeachText.Request()
        request.text = text
        future = self.speach_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            result = future.result().result
            if result:
                self.get_logger().info(f"语音合成成功：{text}")
            else:
                self.get_logger().warn(f"语音合成失败：{text}")
        else:
            self.get_logger().warn("语音合成服务请求失败")

    def image_callback(self, msg):
        """
        将最新的消息放到 latest_image 中
        """
        self.latest_image = msg
        self.get_logger().debug(f"收到图像消息: {msg.width}x{msg.height}")

    def record_image(self):
        """
        记录图像
        """
        if self.latest_image is None:
            self.get_logger().warn("没有可用的图像数据，无法保存图片")
            return False

        pose = self.get_current_pose()
        if pose is None:
            self.get_logger().warn("无法获取当前位姿，无法保存图片")
            return False

        try:
            # 确保路径以 / 结尾
            save_path = self.image_save_path
            if save_path and not save_path.endswith('/'):
                save_path += '/'

            # 如果路径不为空，确保目录存在
            if save_path:
                os.makedirs(save_path, exist_ok=True)
                self.get_logger().info(f"图片保存路径: {save_path}")

            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')

            # 生成文件名
            filename = f'{save_path}image_{pose.position.x:3.2f}_{pose.position.y:3.2f}.png'
            self.get_logger().info(f"尝试保存图片到: {filename}")

            # 保存图像
            success = cv2.imwrite(filename, cv_image)
            if success:
                self.get_logger().info(f"图片保存成功: {filename}")
                return True
            else:
                self.get_logger().error(f"图片保存失败: {filename}")
                return False
        except Exception as e:
            self.get_logger().error(f"保存图片时发生错误: {e}")
            return False
def main(args=None) -> None:
    rclpy.init(args=args)
    patrol = PatrolNode()
    try:
        patrol.speach_text("正在初始化位置")
        patrol.init_robot_pose()
        patrol.speach_text("位置初始化完成")

        while rclpy.ok():
            targets = patrol.get_target_points()
            if not targets:
                patrol.get_logger().warn("未配置目标点，退出")
                break

            for pose in targets:
                x = pose.pose.position.x
                y = pose.pose.position.y
                patrol.speach_text(f"准备前往目标点 {x},{y}")
                result = patrol.nav_to_pose(pose)
                if result == TaskResult.SUCCEEDED:
                    # 记录图像
                    patrol.speach_text(f"已到达目标点 {x},{y}, 准备记录图像")
                    patrol.record_image()
                    patrol.speach_text(f"图像记录完成")
                else:
                    patrol.get_logger().warn("导航中断，停止后续目标")
                    break
    except KeyboardInterrupt:
        patrol.get_logger().info("巡逻被用户中断")
        patrol.cancelTask()
    finally:
        patrol.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()