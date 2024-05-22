#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ultralytics import YOLO

class CameraSubscriber:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('camera_subscriber', anonymous=True)
        # 创建CV Bridge
        self.bridge = CvBridge()
        # 订阅相机主题
        self.image_sub = rospy.Subscriber("/wx250s/camera/image_raw", Image, self.callback)
        # 加载YOLOv8模型
        self.model = YOLO("yolov8n.pt")

    def callback(self, data):
        try:
            # 将ROS的图像消息转换为OpenCV的图像格式
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # 运行YOLOv8模型进行推理
        results = self.model(cv_image)

        # 可视化YOLOv8的推理结果
        annotated_frame = results[0].plot()

        # 显示带有YOLOv8推理结果的图像
        cv_image_with_detections = results[0].plot()
        cv2.imshow("YOLOv8 Inference", cv_image_with_detections)
        cv2.waitKey(3)

def main():
    cs = CameraSubscriber()
    try:
        # ROS spin用来阻止python退出直到ROS节点停止运行
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
