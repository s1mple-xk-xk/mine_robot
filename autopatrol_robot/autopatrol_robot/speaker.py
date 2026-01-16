import rclpy
from rclpy.node import Node

from autopatrol_interfaces.srv import SpeachText
import espeakng


class Speaker(Node):
    """
    简单的语音播报服务节点，使用 espeak-ng。
    """

    def __init__(self, node_name: str = "speaker") -> None:
        super().__init__(node_name)
        # 创建服务端
        self.speech_service = self.create_service(
            SpeachText, "speech_text", self.speak_text_callback
        )
        # 配置语音
        self.speaker = espeakng.Speaker()
        self.speaker.voice = "zh"

    def speak_text_callback(self, request: SpeachText.Request, response: SpeachText.Response):
        self.get_logger().info(f"正在朗读: {request.text}")
        try:
            self.speaker.say(request.text)
            self.speaker.wait()
            response.result = True
        except Exception as exc:
            self.get_logger().error(f"朗读失败: {exc}")
            response.result = False
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Speaker("speaker")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
