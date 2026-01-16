from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.parameter import Parameter
import math


class NavToPose:
    """使用 BasicNavigator 的动作客户端实现单点导航操作"""

    def __init__(self):
        """初始化导航器"""
        rclpy.init()
        self.navigator = BasicNavigator()
        # 设置使用仿真时间
        self.navigator.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.get_logger().info('导航节点已初始化（使用仿真时间）')

    def get_logger(self):
        """获取日志记录器"""
        return self.navigator.get_logger()

    def wait_for_nav2(self):
        """等待 Nav2 系统激活"""
        self.get_logger().info('等待 Nav2 系统激活...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 系统已激活')

    def create_pose_stamped(self, x, y, yaw=0.0, frame_id='map'):
        """
        创建 PoseStamped 消息
        
        Args:
            x: 目标位置的 x 坐标
            y: 目标位置的 y 坐标
            yaw: 目标朝向角度（弧度），默认为 0.0
            frame_id: 坐标系名称，默认为 'map'
        
        Returns:
            PoseStamped: 位姿消息
        """
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        
        # 设置位置
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        
        # 将偏航角转换为四元数
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return pose

    def navigate_to_pose(self, x, y, yaw=0.0, frame_id='map'):
        """
        使用 BasicNavigator 的动作客户端导航到指定位姿
        
        Args:
            x: 目标位置的 x 坐标
            y: 目标位置的 y 坐标
            yaw: 目标朝向角度（弧度），默认为 0.0
            frame_id: 坐标系名称，默认为 'map'
        
        Returns:
            bool: 导航是否成功
        """
        # 创建目标位姿
        goal_pose = self.create_pose_stamped(x, y, yaw, frame_id)
        
        self.get_logger().info(
            f'开始导航到目标位姿: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.2f}°'
        )
        
        # 使用 BasicNavigator 的动作客户端发送导航目标
        # goToPose 内部使用了 NavigateToPose 动作客户端
        self.navigator.goToPose(goal_pose)
        
        # 等待导航完成，轮询任务状态
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            # 获取导航反馈信息
            feedback = self.navigator.getFeedback()
            
            # 定期打印导航进度
            if feedback and i % 10 == 0:
                self.get_logger().info(
                    f'导航中... 剩余距离: {feedback.distance_remaining:.2f} 米, '
                    f'已用时间: {feedback.navigation_time.sec} 秒'
                )
            
            # 处理 ROS2 回调
            rclpy.spin_once(self.navigator, timeout_sec=0.1)
        
        # 获取导航结果
        result = self.navigator.getResult()
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('导航成功完成！')
            return True
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('导航被取消')
            return False
        elif result == TaskResult.FAILED:
            self.get_logger().error('导航失败')
            return False
        else:
            self.get_logger().error(f'未知的导航结果: {result}')
            return False

    def cancel_navigation(self):
        """取消当前导航任务"""
        self.get_logger().info('取消导航任务...')
        self.navigator.cancelTask()

    def shutdown(self):
        self.get_logger().info('关闭导航脚本...')
        try:
            # 如果正在导航，先取消
            self.navigator.cancelTask()
        except Exception:
            pass
        rclpy.shutdown()




def main(args=None):
    """主函数 - 示例用法"""
    nav = NavToPose()
    
    try:
        # 等待 Nav2 系统激活
        nav.wait_for_nav2()
        
        # 示例：导航到目标位姿
        # 可以根据需要修改这些坐标
        target_x = -2.0
        target_y = 0.0
        target_yaw = math.pi / 4  # 45度
        
        # 执行导航
        success = nav.navigate_to_pose(target_x, target_y, target_yaw)
        
        if success:
            nav.get_logger().info('导航任务完成')
        else:
            nav.get_logger().error('导航任务失败')
        
    except KeyboardInterrupt:
        nav.get_logger().info('节点被用户中断')
        nav.cancel_navigation()
    except Exception as e:
        nav.get_logger().error(f'发生错误: {e}')
    finally:
        nav.shutdown()


if __name__ == '__main__':
    main()
