#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from cv_bridge import CvBridge
import cv2
import time
import sys
from pyzbar import pyzbar
from origincar_msg.msg import Sign
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

class QRDetectorNode(Node):
    def __init__(self):
        super().__init__('qr_detector_node')
        
        # 创建兼容的QoS配置
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # 订阅 /sign4return 话题 (使用兼容的QoS)
        self.sign_sub = self.create_subscription(
            Int32,
            '/sign4return',
            self.sign_callback,
            qos_profile=qos_profile)
        
        # 创建发布者
        self.sign_switch_pub = self.create_publisher(Sign, '/sign_switch', 10)
        self.Signmsg = Sign()
        
        # 创建二维码内容发布者
        self.publisher_ = self.create_publisher(String, 'qr_code', 10)
        
        # 创建 /sign4return 发布者 (使用兼容的QoS)
        self.sign4return_pub = self.create_publisher(
            Int32, 
            '/sign4return', 
            qos_profile=qos_profile
        )
        
        # 图像处理相关
        self.bridge = CvBridge()
        self.image_sub = None
        
        # 状态控制
        self.activated = False  # 默认不激活
        self.last_sign_time = 0
        self.last_process_time = 0
        
        # 参数配置
        self.process_interval = 0.2  # 处理间隔(秒)
        self.scale_factor = 1     # 图像缩放比例 (优化性能)
        
        self.get_logger().info('QR检测器节点已启动 (等待/sign4return信号)')

    def sign_callback(self, msg):
        """处理/sign4return信号的回调函数"""
        current_time = time.time()
        self.last_sign_time = current_time
        
        # 只处理激活信号
        if msg.data == 0:
            if not self.activated:
                self.get_logger().info('收到激活信号(0)，启动二维码检测')
                self.activated = True
                
                # 创建图像订阅者（如果尚未创建）
                if self.image_sub is None:
                    self.image_sub = self.create_subscription(
                        Image,
                        'image_raw',
                        self.image_callback,
                        10)
                    self.get_logger().info('已创建图像订阅')

    def image_callback(self, msg):
        """处理图像的回调函数（仅在激活状态下工作）"""
        if not self.activated:
            return
            
        current_time = time.time()
        
        # 按固定频率处理图像
        if current_time - self.last_process_time < self.process_interval:
            return
            
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 缩小图像尺寸 (优化性能)
            if self.scale_factor < 1.0:
                h, w = cv_image.shape[:2]
                cv_image = cv2.resize(
                    cv_image, 
                    (int(w * self.scale_factor), int(h * self.scale_factor)),
                    interpolation=cv2.INTER_AREA
                )
            
            # 检测二维码
            barcodes = pyzbar.decode(cv_image)
            
            # 处理检测结果
            if barcodes:
                for barcode in barcodes:
                    barcodeData = barcode.data.decode("utf-8")
                    
                    # 记录检测到的二维码
                    self.get_logger().info(f'检测到二维码: {barcodeData}')
                    
                    # 发布二维码内容
                    qr_msg = String()
                    qr_msg.data = barcodeData
                    self.publisher_.publish(qr_msg)
                    
                    # 处理特殊二维码指令
                    if barcodeData.isdigit():
                        qr_value = int(barcodeData)
                        self.Signmsg.sign_data = 3 if qr_value % 2 == 1 else 4
                        
                        # 发布转向指令
                        self.sign_switch_pub.publish(self.Signmsg)
                        self.get_logger().info(f'发布转向信号: {self.Signmsg.sign_data} (二维码: {barcodeData})')
                        
                        # 发送停止命令
                        self.send_stop_command()
                        
                        # 重置状态
                        self.activated = False
                        self.get_logger().info('二维码处理完成，等待下次激活')
                        return
                
            # 未检测到有效二维码时记录
            if current_time - self.last_sign_time > 5:  # 5秒内无新信号
                self.get_logger().debug('扫描中...', throttle_duration_sec=2)
                
        except Exception as e:
            self.get_logger().error(f'处理图像时出错: {str(e)}', throttle_duration_sec=5)
        finally:
            self.last_process_time = current_time
    
    def send_stop_command(self):
        """发送停止命令到机器人"""
        stop_msg = Int32()
        stop_msg.data = 5
        self.sign4return_pub.publish(stop_msg)
        self.get_logger().info('已发布停止命令(5)')

def main(args=None):
    rclpy.init(args=args)
    node = QRDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()