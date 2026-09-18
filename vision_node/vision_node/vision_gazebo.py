import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import sys
from importlib import import_module
from ultralytics import YOLO as YOLOModel  # type: ignore
from PIL import ImageFont, ImageDraw, Image as PILImage

from ros2_tools.srv import YOLO
from ros2_tools.srv import OCR

from . import config

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # 允许通过 ROS 参数覆盖资源路径
        self.declare_parameter('vision_node_src_dir', config.VISION_NODE_SRC_DIR)
        requested_src_dir = self.get_parameter('vision_node_src_dir').get_parameter_value().string_value
        try:
            resolved_src_dir = config.configure_paths(requested_src_dir)
            self.get_logger().info(f"使用 vision_node 资源目录: {resolved_src_dir}")
        except ValueError as exc:
            self.get_logger().warn(f"指定的 vision_node 目录无效，回退到默认值: {exc}")
            resolved_src_dir = config.VISION_NODE_SRC_DIR

        # 确保 PaddleOCR 依赖路径在 sys.path 中
        self._ensure_paddleocr_paths(resolved_src_dir)

        # 动态导入 PaddleOCR，确保在路径更新后加载
        try:
            paddleocr_module = import_module('paddleocr')
            self._paddleocr_ctor = paddleocr_module.PaddleOCR
        except Exception as exc:
            raise ImportError(f"无法导入 PaddleOCR，请检查目录 {resolved_src_dir}/ocr: {exc}") from exc
        
        # 初始化CvBridge用于ROS图像消息转换
        self.bridge = CvBridge()
        
        # 存储最新的图像数据
        self.camera1_image = None
        self.camera1_info = None
        self.camera2_image = None
        self.camera2_info = None
        
        # 创建订阅者
        self.camera1_image_sub = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.camera1_image_callback,
            10
        )
        self.camera1_info_sub = self.create_subscription(
            CameraInfo,
            '/camera1/camera_info',
            self.camera1_info_callback,
            10
        )
        self.camera2_image_sub = self.create_subscription(
            Image,
            '/camera2/image_raw',
            self.camera2_image_callback,
            10
        )
        self.camera2_info_sub = self.create_subscription(
            CameraInfo,
            '/camera2/camera_info',
            self.camera2_info_callback,
            10
        )
        
        # 创建服务
        self.srv_yolo = self.create_service(YOLO, 'yolo_trigger', self.yolo_callback)
        self.srv_ocr = self.create_service(OCR, 'ocr_trigger', self.ocr_callback)
        
        # 初始化YOLO模型 - 使用配置文件
        self.yolo_model_path = config.YOLO_MODEL_PATH
        self.yolo_font_path = config.YOLO_FONT_PATH
        self.yolo_save_dir = config.YOLO_SAVE_DIR
        os.makedirs(self.yolo_save_dir, exist_ok=True)
        
        try:
            self.yolo_model = YOLOModel(self.yolo_model_path)
            self.get_logger().info("YOLO模型加载成功")
        except Exception as e:
            self.get_logger().error(f"YOLO模型加载失败: {e}")
            self.yolo_model = None
        
        # 自定义YOLO标签 - 使用配置文件
        self.custom_labels = config.YOLO_LABELS
        
        # PaddleOCR 延迟初始化，避免启动时加载大模型
        self.ocr_engine = None
        self._ocr_init_attempted = False
        
        self.ocr_save_dir = config.OCR_SAVE_DIR
        os.makedirs(self.ocr_save_dir, exist_ok=True)
        
        self.get_logger().info("VisionNode初始化完成")

    def _ensure_paddleocr_paths(self, src_dir: str) -> None:
        """将 OCR 依赖目录注入 sys.path，便于按需覆盖。"""

        ocr_dir = os.path.join(src_dir, 'ocr')
        tools_dir = os.path.join(ocr_dir, 'tools')

        for path in (ocr_dir, tools_dir):
            if os.path.isdir(path) and path not in sys.path:
                sys.path.insert(0, path)

        if not os.path.isdir(ocr_dir):
            self.get_logger().warn(f"未找到本地 PaddleOCR 目录: {ocr_dir}，将使用已安装的 paddleocr 包")

    def _ensure_ocr_engine(self):
        """延迟初始化OCR引擎，仅在首次调用时加载"""
        if self.ocr_engine is not None:
            return True

        if self._ocr_init_attempted:
            return False

        self._ocr_init_attempted = True

        try:
            self.get_logger().info("正在初始化PaddleOCR引擎...")
            ocr_config = config.CURRENT_MODEL.copy()

            if "rec_char_dict_path" in ocr_config:
                dict_path = ocr_config["rec_char_dict_path"]
                if not os.path.exists(dict_path):
                    self.get_logger().warn("指定的字典文件不存在，将使用PaddleOCR默认字典")
                    del ocr_config["rec_char_dict_path"]

            self.ocr_engine = self._paddleocr_ctor(**ocr_config)
            self.get_logger().info("PaddleOCR引擎加载成功")
            return True
        except Exception as e:
            self.get_logger().error(f"PaddleOCR引擎加载失败: {e}")
            self.ocr_engine = None
            return False
    
    # Camera1 回调函数
    def camera1_image_callback(self, msg):
        try:
            self.camera1_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Camera1图像转换失败: {e}")
    
    def camera1_info_callback(self, msg):
        self.camera1_info = msg
    
    # Camera2 回调函数
    def camera2_image_callback(self, msg):
        try:
            self.camera2_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Camera2图像转换失败: {e}")
    
    def camera2_info_callback(self, msg):
        self.camera2_info = msg
    
    def yolo_callback(self, request, response):
        """YOLO检测服务回调函数 - 同时处理camera1和camera2"""
        self.get_logger().info("开始YOLO检测...")
        
        if self.yolo_model is None:
            response.success = False
            response.message = "YOLO模型未加载"
            return response
        
        # 检查两个相机的图像
        if self.camera1_image is None and self.camera2_image is None:
            response.success = False
            response.message = "Camera1和Camera2图像均未接收"
            return response
        
        try:
            import time
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            all_detection_results = []
            
            # 处理Camera1
            if self.camera1_image is not None:
                self.get_logger().info("处理Camera1图像...")
                camera1_results = self._process_yolo_image(
                    self.camera1_image.copy(), 
                    f"camera1_{timestamp}",
                    "Camera1"
                )
                all_detection_results.extend(camera1_results)
            else:
                self.get_logger().warn("Camera1图像未接收，跳过")
            
            # 处理Camera2
            if self.camera2_image is not None:
                self.get_logger().info("处理Camera2图像...")
                camera2_results = self._process_yolo_image(
                    self.camera2_image.copy(), 
                    f"camera2_{timestamp}",
                    "Camera2"
                )
                all_detection_results.extend(camera2_results)
            else:
                self.get_logger().warn("Camera2图像未接收，跳过")
            
            response.success = True
            if all_detection_results:
                response.message = f"YOLO检测完成，共检测到{len(all_detection_results)}个目标。结果: {'; '.join(all_detection_results)}"
            else:
                response.message = "YOLO检测完成，未检测到目标"
            
        except Exception as e:
            self.get_logger().error(f"YOLO检测失败: {e}")
            response.success = False
            response.message = f"YOLO检测失败: {str(e)}"
        
        return response
    
    def _process_yolo_image(self, frame, filename_prefix, camera_name):
        """处理单张图像的YOLO检测（参考test2_new.py）"""
        detection_results = []
        
        # 保存原始图像
        raw_path = os.path.join(self.yolo_save_dir, f"{filename_prefix}_raw.jpg")
        cv2.imwrite(raw_path, frame)
        self.get_logger().info(f"📸 已保存{camera_name}原始图像: {raw_path}")
        
        # YOLO检测
        results = self.yolo_model(frame) # type: ignore
        boxes = results[0].boxes
        
        # 转换为PIL图像以绘制中文（参考test2_new.py）
        img_pil = PILImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(img_pil)
        
        # 加载字体
        try:
            font = ImageFont.truetype(self.yolo_font_path, 28, encoding="utf-8")
        except Exception as e:
            self.get_logger().warn(f"无法加载字体 {self.yolo_font_path}: {e}，使用默认字体")
            font = ImageFont.load_default()
        
        # 处理每个检测框
        for box in boxes:
            cls_id = int(box.cls)
            conf = float(box.conf)
            label = self.custom_labels.get(cls_id, f"未知类别({cls_id})")
            
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            
            # 根据类别设置颜色（参考test2_new.py）
            # cls_id == 0: 社区内人员 -> 红色 (255, 0, 0)
            # cls_id == 1: 非社区人员 -> 绿色 (0, 255, 0)
            color = (255, 0, 0) if cls_id == 1 else (0, 255, 0)
            
            # 绘制矩形框
            draw.rectangle([x1, y1, x2, y2], outline=color, width=3)
            
            # 绘制中文标签
            text = f"{label} {conf:.2f}"
            draw.text((x1, y1 - 30), text, font=font, fill=color)
            
            # 记录结果
            result_str = f"[{camera_name}] {label} (置信度: {conf:.2f})"
            detection_results.append(result_str)
            self.get_logger().info(f"检测到: {result_str}")
        
        # 转回OpenCV格式
        frame = cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)
        
        # 保存检测结果图像
        result_path = os.path.join(self.yolo_save_dir, f"{filename_prefix}_result.jpg")
        cv2.imwrite(result_path, frame)
        self.get_logger().info(f"✅ {camera_name}检测结果已保存到: {result_path}")
        
        return detection_results
    
    def ocr_callback(self, request, response):
        """OCR识别服务回调函数 - 同时处理camera1和camera2"""
        self.get_logger().info("开始OCR识别...")
        
        if not self._ensure_ocr_engine():
            response.success = False
            response.message = "OCR引擎未加载"
            return response
        
        # 检查两个相机的图像
        if self.camera1_image is None and self.camera2_image is None:
            response.success = False
            response.message = "Camera1和Camera2图像均未接收"
            return response
        
        try:
            import time
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            all_ocr_results = []
            
            # 处理Camera1
            if self.camera1_image is not None:
                self.get_logger().info("处理Camera1图像OCR...")
                camera1_results = self._process_ocr_image(
                    self.camera1_image.copy(), 
                    f"camera1_{timestamp}",
                    "Camera1"
                )
                all_ocr_results.extend(camera1_results)
            else:
                self.get_logger().warn("Camera1图像未接收，跳过")
            
            # 处理Camera2
            if self.camera2_image is not None:
                self.get_logger().info("处理Camera2图像OCR...")
                camera2_results = self._process_ocr_image(
                    self.camera2_image.copy(), 
                    f"camera2_{timestamp}",
                    "Camera2"
                )
                all_ocr_results.extend(camera2_results)
            else:
                self.get_logger().warn("Camera2图像未接收，跳过")
            
            response.success = True
            if all_ocr_results:
                response.message = f"OCR识别完成，共识别到{len(all_ocr_results)}条文本。结果: {'; '.join(all_ocr_results)}"
            else:
                response.message = "OCR识别完成，未识别到文本"
            
        except Exception as e:
            self.get_logger().error(f"OCR识别失败: {e}")
            response.success = False
            response.message = f"OCR识别失败: {str(e)}"
        
        return response
    
    def _process_ocr_image(self, frame, filename_prefix, camera_name):
        """处理单张图像的OCR识别"""
        ocr_results = []
        
        # 保存原始图像
        raw_path = os.path.join(self.ocr_save_dir, f"{filename_prefix}_raw.jpg")
        cv2.imwrite(raw_path, frame)
        self.get_logger().info(f"📸 已保存{camera_name}原始图像: {raw_path}")
        
        # OCR识别
        result = self.ocr_engine.ocr(frame, cls=True)
        
        if result and result[0]:
            for line in result[0]:
                text = line[1][0]
                confidence = line[1][1]
                result_str = f"[{camera_name}] {text} (置信度: {confidence:.2f})"
                ocr_results.append(result_str)
                self.get_logger().info(f"识别到文本: {result_str}")
        
        # 保存结果图像
        result_path = os.path.join(self.ocr_save_dir, f"{filename_prefix}_result.jpg")
        cv2.imwrite(result_path, frame)
        self.get_logger().info(f"✅ {camera_name}OCR结果已保存到: {result_path}")
        
        return ocr_results

def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()
    rclpy.spin(vision_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
