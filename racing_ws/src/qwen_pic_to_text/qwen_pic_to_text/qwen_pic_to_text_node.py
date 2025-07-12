#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, String  # 添加String消息类型
from cv_bridge import CvBridge
import cv2
import base64
import numpy as np
import os
from openai import OpenAI
import threading

class QwenPicToTextNode(Node):
    def __init__(self):
        super().__init__('qwen_pic_to_text_node')
        
        # 从参数或环境变量获取API密钥
        self.api_key = self.declare_parameter('api_key', "sk-283eeb7b6ee74671a6ebbfdcbf95303c").value
        
        # 添加信号订阅器
        self.signal_sub = self.create_subscription(
            Int32,
            '/sign4return',
            self.signal_callback,
            1
        )
        
        # 创建结果发布者 (新增部分)
        self.result_publisher = self.create_publisher(
            String,
            '/pic_info',
            1  # QoS深度设为1
        )
        
        # 初始化图像订阅器为None，稍后创建
        self.image_sub = None
        self.ready = False  # 准备状态标志
        
        self.bridge = CvBridge()
        self.image_received = False
        self.lock = threading.Lock()
        
        # 初始化OpenAI客户端
        self.client = OpenAI(
            api_key=self.api_key,
            base_url="https://dashscope.aliyuncs.com/compatible-mode/v1"
        )
        
        self.get_logger().info("图生文节点已启动，等待接收启动信号...")

    def signal_callback(self, msg):
        """处理启动信号的回调函数"""
        if msg.data == -1 and not self.ready:
            self.ready = True
            self.get_logger().info("接收到启动信号(-1)，开始订阅图像话题")
            
            # 创建图像订阅器
            self.image_sub = self.create_subscription(
                CompressedImage,
                '/image', 
                self.image_callback,
                1  # QoS深度设为1，只处理最新图像
            )

    def image_callback(self, msg):
        # 只有在准备状态下才处理图像
        if not self.ready:
            return
            
        with self.lock:
            if self.image_received:
                return  # 只处理第一张图片
            
            self.image_received = True
            self.get_logger().info("收到图像，开始处理...")
            
            try:
                np_arr = np.frombuffer(msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                # 将图像转为base64
                _, buffer = cv2.imencode('.jpg', cv_image)
                base64_image = base64.b64encode(buffer).decode('utf-8')
                
                # 调用通义千问API（非流式模式）
                description = self.describe_image(base64_image)
                
                # 输出完整回复
                self.get_logger().info("\n完整回复:\n" + description)
                
                # 发布结果到/pic_info话题 (新增部分)
                result_msg = String()
                result_msg.data = description
                self.result_publisher.publish(result_msg)
                self.get_logger().info("已发布结果到/pic_info话题")
                
            except Exception as e:
                self.get_logger().error(f"处理失败: {str(e)}")
            finally:
                # 处理完成后关闭节点
                self.get_logger().info("处理完成，关闭节点")
                self.timer = self.create_timer(2.0, self.shutdown_node)

    def describe_image(self, base64_image):
        try:
            # 使用非流式API获取完整结果（提高处理速度）
            completion = self.client.chat.completions.create(
                model="qwen-vl-plus",  # 使用您指定的模型
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "image_url",
                                "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"},
                            },
                            {"type": "text", "text": "描述书本内容"},
                        ],
                    }
                ],
                stream=False  # 禁用流式输出
            )
            
            # 直接获取完整回复
            return completion.choices[0].message.content
            
        except Exception as e:
            return f"API调用失败: {str(e)}"

    def shutdown_node(self):
        """关闭节点"""
        self.destroy_timer(self.timer)
        self.get_logger().info('节点即将关闭')
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = QwenPicToTextNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()