import cv2
import numpy as np
import time
import os
import psutil
from rknnlite.api import RKNNLite
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import argparse

OBJ_THRESH = 0.55
NMS_THRESH = 0.2
IMG_SIZE = 640
CLASSES = ("fire")

def xywh2xyxy(x):
    # Convert [x, y, w, h] to [x1, y1, x2, y2]
    y = np.copy(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
    return y


def process(input, mask, anchors):

    anchors = [anchors[i] for i in mask]
    grid_h, grid_w = map(int, input.shape[0:2])

    box_confidence = input[..., 4]
    box_confidence = np.expand_dims(box_confidence, axis=-1)

    box_class_probs = input[..., 5:]

    box_xy = input[..., :2]*2 - 0.5

    col = np.tile(np.arange(0, grid_w), grid_w).reshape(-1, grid_w)
    row = np.tile(np.arange(0, grid_h).reshape(-1, 1), grid_h)
    col = col.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
    row = row.reshape(grid_h, grid_w, 1, 1).repeat(3, axis=-2)
    grid = np.concatenate((col, row), axis=-1)
    box_xy += grid
    box_xy *= int(IMG_SIZE/grid_h)

    box_wh = pow(input[..., 2:4]*2, 2)
    box_wh = box_wh * anchors

    box = np.concatenate((box_xy, box_wh), axis=-1)

    return box, box_confidence, box_class_probs


def filter_boxes(boxes, box_confidences, box_class_probs):
    """Filter boxes with box threshold. It's a bit different with origin yolov5 post process!

    # Arguments
        boxes: ndarray, boxes of objects.
        box_confidences: ndarray, confidences of objects.
        box_class_probs: ndarray, class_probs of objects.

    # Returns
        boxes: ndarray, filtered boxes.
        classes: ndarray, classes for boxes.
        scores: ndarray, scores for boxes.
    """
    boxes = boxes.reshape(-1, 4)
    box_confidences = box_confidences.reshape(-1)
    box_class_probs = box_class_probs.reshape(-1, box_class_probs.shape[-1])

    _box_pos = np.where(box_confidences >= OBJ_THRESH)
    boxes = boxes[_box_pos]
    box_confidences = box_confidences[_box_pos]
    box_class_probs = box_class_probs[_box_pos]

    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)
    _class_pos = np.where(class_max_score >= OBJ_THRESH)

    boxes = boxes[_class_pos]
    classes = classes[_class_pos]
    scores = (class_max_score* box_confidences)[_class_pos]

    return boxes, classes, scores


def nms_boxes(boxes, scores):
    """Suppress non-maximal boxes.

    # Arguments
        boxes: ndarray, boxes of objects.
        scores: ndarray, scores of objects.

    # Returns
        keep: ndarray, index of effective boxes.
    """
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]

    areas = w * h
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])

        w1 = np.maximum(0.0, xx2 - xx1 + 0.00001)
        h1 = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter = w1 * h1

        ovr = inter / (areas[i] + areas[order[1:]] - inter)
        inds = np.where(ovr <= NMS_THRESH)[0]
        order = order[inds + 1]
    keep = np.array(keep)
    return keep


def yolov5_post_process(input_data):
    masks = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
    anchors = [[10, 13], [16, 30], [33, 23], [30, 61], [62, 45],
               [59, 119], [116, 90], [156, 198], [373, 326]]

    boxes, classes, scores = [], [], []
    for input, mask in zip(input_data, masks):
        b, c, s = process(input, mask, anchors)
        b, c, s = filter_boxes(b, c, s)
        boxes.append(b)
        classes.append(c)
        scores.append(s)

    boxes = np.concatenate(boxes)
    boxes = xywh2xyxy(boxes)
    classes = np.concatenate(classes)
    scores = np.concatenate(scores)

    nboxes, nclasses, nscores = [], [], []
    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c = classes[inds]
        s = scores[inds]

        keep = nms_boxes(b, s)

        nboxes.append(b[keep])
        nclasses.append(c[keep])
        nscores.append(s[keep])

    if not nclasses and not nscores:
        return None, None, None

    boxes = np.concatenate(nboxes)
    classes = np.concatenate(nclasses)
    scores = np.concatenate(nscores)

    return boxes, classes, scores

def print_inference_result(boxes, scores, classes):
    """Draw the boxes on the image.

    # Argument:
        image: original image.
        boxes: ndarray, boxes of objects.
        classes: ndarray, classes of objects.
        scores: ndarray, scores of objects.
        all_classes: all classes name.
    """
    print("{:^12} {:^12}  {}".format('class', 'score', 'xmin, ymin, xmax, ymax'))
    print('-' * 50)
    for box, score, cl in zip(boxes, scores, classes):
        top, left, right, bottom = box
        top = int(top)
        left = int(left)
        right = int(right)
        bottom = int(bottom)

        print("{:^12} {:^12.3f} [{:>4}, {:>4}, {:>4}, {:>4}]".format(CLASSES[cl], score, top, left, right, bottom))

def draw(image, boxes, scores, classes):
    """Draw the boxes on the image.

    # Argument:
        image: original image.
        boxes: ndarray, boxes of objects.
        classes: ndarray, classes of objects.
        scores: ndarray, scores of objects.
        all_classes: all classes name.
    """
    print("{:^12} {:^12}  {}".format('class', 'score', 'xmin, ymin, xmax, ymax'))
    print('-' * 50)
    for box, score, cl in zip(boxes, scores, classes):
        top, left, right, bottom = box
        top = int(top)
        left = int(left)
        right = int(right)
        bottom = int(bottom)

        cv2.rectangle(image, (top, left), (right, bottom), (255, 0, 0), 2)
        cv2.putText(image, '{0} {1:.2f}'.format(CLASSES[cl], score),
                    (top, left - 6),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 255), 2)

        print("{:^12} {:^12.3f} [{:>4}, {:>4}, {:>4}, {:>4}]".format(CLASSES[cl], score, top, left, right, bottom))

def letterbox(im, new_shape=(640, 640), color=(0, 0, 0)):
    # Resize and pad image while meeting stride-multiple constraints
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return im, ratio, (dw, dh)



class FireDetectionNode(Node):
    def __init__(self):
        super().__init__('fire_detection_node')
        move_to_gps_target_dir = get_package_share_directory('move_to_gps_target')
        
        # 视频录制相关变量
        self.recording = False
        self.last_fire_time = None
        self.video_writer = None
        self.output_dir = os.path.expanduser('~/fire_detection')
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 帧率计算相关
        self.frame_rate = 30  # 默认值，后续动态计算
        self.last_frame_time = None
        self.frame_times = []

        # 初始化RKNN模型
        self.rknnModel = os.path.join(move_to_gps_target_dir, "rknnModel/fire.rknn")
        self.rknn_lite = RKNNLite()
        
        # 加载RKNN模型
        ret = self.rknn_lite.load_rknn(self.rknnModel)
        if ret != 0:
            self.get_logger().error("Load RKNN Model failed")
            exit(ret)
        
        # 初始化运行时环境
        ret = self.rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
        if ret != 0:
            self.get_logger().error("Init runtime environment failed")
            exit(ret)
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 创建图像订阅
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10  # QoS profile depth
        )
        
        # 初始化帧率控制变量
        self.last_process_time = time.time()
        self.process_interval = 0.1  # 10Hz (1/10)
        
        # 绑定CPU核心
        try:
            p = psutil.Process(os.getpid())
            p.cpu_affinity([4])
            self.get_logger().info("Process bound to core 4")
        except Exception as e:
            self.get_logger().warn(f"Failed to set CPU affinity: {str(e)}")


    def start_recording(self, frame):
        """根据实际帧率初始化视频写入器"""
        if self.video_writer is not None:
            self.video_writer.release()
        
        # 获取帧尺寸
        h, w = frame.shape[:2]
        
        # 创建视频编码器（根据动态计算的帧率）
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.output_dir, f"fire_{timestamp}.mp4")
        
        self.video_writer = cv2.VideoWriter(filename, fourcc, self.frame_rate, (w, h))
        if not self.video_writer.isOpened():
            self.get_logger().error("Failed to initialize video writer!")
            return
        
        self.recording = True
        self.get_logger().info(f"Started recording: {filename} (FPS: {self.frame_rate})")

        self.video_writer.write(frame)

    def stop_recording(self):
        """停止录制视频"""
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
        self.recording = False
        self.get_logger().info("Stopped recording")


    def update_frame_rate(self, current_time):
        """动态更新帧率计算"""
        if self.last_frame_time is not None:
            # 计算时间差并保存最近5次时间差
            self.frame_times.append(current_time - self.last_frame_time)
            if len(self.frame_times) > 5:
                self.frame_times.pop(0)
            
            # 计算平均帧率
            if len(self.frame_times) >= 2:
                avg_interval = sum(self.frame_times) / len(self.frame_times)
                self.frame_rate = round(1.0 / avg_interval)
        
        self.last_frame_time = current_time

    def image_callback(self, msg):
        # 转换ROS Image消息为OpenCV格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {str(e)}")
            return

        current_time = time.time()
        
        # 更新帧率计算
        self.update_frame_rate(current_time)
        

        
        # 检查处理间隔
        if (current_time - self.last_process_time) < self.process_interval:
            if self.recording:
                self.video_writer.write(cv_image)
                
                # 检查是否需要停止（5秒无火情）
                if (current_time - self.last_fire_time) >= 5:
                    self.stop_recording()
            return
        
        img_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        img_resized = cv2.resize(img_rgb, (IMG_SIZE, IMG_SIZE))
        
        # 推理
        img_input = np.expand_dims(img_resized, 0)
        outputs = self.rknn_lite.inference(inputs=[img_input], data_format=['nhwc'])
        
        # 后处理
        input0_data = outputs[0].reshape([3, -1] + list(outputs[0].shape[-2:]))
        input1_data = outputs[1].reshape([3, -1] + list(outputs[1].shape[-2:]))
        input2_data = outputs[2].reshape([3, -1] + list(outputs[2].shape[-2:]))
        
        input_data = [
            np.transpose(input0_data, (2, 3, 0, 1)),
            np.transpose(input1_data, (2, 3, 0, 1)),
            np.transpose(input2_data, (2, 3, 0, 1))
        ]
        
        boxes, classes, scores = yolov5_post_process(input_data)

        # 处理检测结果
        fire_detected = False
        if boxes is not None:
            self.draw_detections(cv_image, boxes, scores, classes)
            fire_detected = True

            # 更新火焰检测时间（无论是否在录制状态）
            self.last_fire_time = current_time
            
            # 首次检测到火焰时启动录制
            if not self.recording:
                self.start_recording(cv_image)
            else:
                self.video_writer.write(cv_image)

        # 显示处理结果
        # cv2.imshow('Fire Detection', cv_image)
        # cv2.waitKey(1)
        
        # 更新最后处理时间
        self.last_process_time = current_time


    def draw_detections(self, image, boxes, scores, classes):
        """在原始图像上绘制检测结果"""
        h, w = image.shape[:2]
        scale = min(IMG_SIZE / w, IMG_SIZE / h)
        
        for box, score, cl in zip(boxes, scores, classes):
            # 将坐标转换回原始图像尺寸
            top = int(box[0] / scale)
            left = int(box[1] / scale)
            right = int(box[2] / scale)
            bottom = int(box[3] / scale)
            
            # 确保坐标不越界
            top = max(0, min(top, w-1))
            left = max(0, min(left, h-1))
            right = max(0, min(right, w-1))
            bottom = max(0, min(bottom, h-1))
            
            # 绘制矩形和文字
            cv2.rectangle(image, (top, left), (right, bottom), (0, 0, 255), 2)
            cv2.putText(image, f'{CLASSES[cl]} {score:.2f}',
                        (top, left - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2)
            
            self.get_logger().info(
                f"Detected {CLASSES[cl]} with confidence {score:.2f} at [{top}, {left}, {right}, {bottom}]"
            )

    def __del__(self):
        # 清理资源
        if self.recording:
            self.stop_recording()
        self.rknn_lite.release()
        cv2.destroyAllWindows()
        self.get_logger().info("Fire detection node shutdown cleanly")

def bind_to_cpu(cpu_core):
    global cpu_core_msg,cpu_core_error_msg
    try:
        p = psutil.Process()  # 获取当前进程
        p.cpu_affinity([cpu_core])  # 设置进程的CPU核心亲和力
        cpu_core_msg=f'Process is bound to CPU core {cpu_core}'
        print(f'Process is bound to CPU core {cpu_core}')
    except Exception as e:
        cpu_core_error_msg=f'Failed to set CPU affinity: {e}'
        print(f'Failed to set CPU affinity: {e}')

def main(args=None):
    # 在rclpy初始化之前解析命令行参数
    parser = argparse.ArgumentParser(description="APM Controller Node")
    parser.add_argument('--cpu', type=int, default=None, help="CPU core to bind to")
    parsed_args, unknown_args = parser.parse_known_args(args)  # 解析CPU参数

    # 只处理CPU核心绑定参数，其他参数传递给rclpy
    global cpu_core_msg,cpu_core_error_msg
    if parsed_args.cpu is not None:
        bind_to_cpu(parsed_args.cpu)
    else:
        cpu_core_error_msg='parsed_args.cpu is None'

    # 初始化rclpy并传递未被argparse处理的参数
    rclpy.init(args=unknown_args)

    node = FireDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()