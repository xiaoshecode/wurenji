#!/usr/bin/python
#-*- encoding: utf8 -*-

#一个简单的穿环demo：利用hough圆检测识别图像中的圆，然后调整无人机姿态和位置，穿过圆心，降落；
#无人机移动逻辑是每前进1m，就调整一次姿态，然后再前进1m，当检测到圆的半径大于阈值时，认为到达环近点，向前飞行2m穿过环；
#运行roslaunch uav_sim demo2.launch后，再在另一个终端中运行rosrun uav_sim demo2.py 

import cv2
import rospy
import sys
# from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Bool
from collections import deque
import numpy as np
from ttauav_node.msg import uavdata


# navigation=np.array([0,0,0])#x,y,r 导航目地信息
# cv_img=None
# R_wu_ = R.from_quat([0, 0, 0, 1])#无人机位姿

# bridge=CvBridge()

# #获取图像回调函数：霍夫检测圆并更新navigation导航信息
# def imagecallback(data):
#     global cv_img
#     global navigation
#     try:
#         cv_img=bridge.imgmsg_to_cv2(data,'bgr8')#将图片转换为opencv格式
#         cv_img_cp=cv_img.copy()
#         img_gray = cv2.cvtColor(cv_img_cp, cv2.COLOR_BGR2GRAY)#转换为灰度图
#         img_g=cv2.GaussianBlur(img_gray, (3, 3), 0)#滤波
#         circles = cv2.HoughCircles(img_g, cv2.HOUGH_GRADIENT, 1, 100, param1=80, param2=40, minRadius=20, maxRadius=150)  #霍夫圆检测
#         #画圈
#         if circles is not None:
#             for i in circles[0,:]:
#                     cv2.circle(cv_img_cp, (i[0], i[1]), i[2], (255, 0, 0), 3)  # 画圆
#                     cv2.circle(cv_img_cp, (i[0], i[1]), 2, (255, 0, 0), 3)  # 画圆心
#                     #输出圆心的图像坐标和半径
#                     rospy.loginfo("( %d  ,  %d ),r=  %d ",i[0],i[1],i[2])
#                     #更新导航信息，此处是粗略估计图像和实际距离，其实可以用结合目标在图像中的位置和相机内外参数得到较准确的坐标
#                     navigation = np.array([(i[0] - 160) / i[2] * 70, (i[1] - 80) / i[2] * 70, i[2]])
#             image_result_pub.publish(bridge.cv2_to_imgmsg(cv_img_cp,encoding='bgr8'))    
#     except CvBridgeError as e:
#         print(e)
def uavdataCallback(msg):
    print(1)
    print(msg)
# if __name__ == '__main__':
#     rospy.init_node('demo2')
#     rospy.loginfo('demo2 node set up')

#     # image_sub=rospy.Subscriber('/iris/usb_cam/image_raw',Image,imagecallback)
#     # poseSub_ = rospy.Subscriber('/m3e/states', PoseStamped, poseCallback)
#     # image_result_pub=rospy.Publisher('/get_images/image_result_circle',Image,queue_size=10)
#     # rospy.sleep(2)
#     uavdata = rospy.Subscriber('/uavdata/get_logger_level',uavdata,uavdataCallback)
#     rospy.sleep(100)
def call_service():
    # rospy.init_node('node_name')  # 你的节点名
    # rospy.wait_for_service('/uavdata/get_loggers')  # 你的服务名
    # try:
    #     service = rospy.ServiceProxy('/uavdata/get_loggers', uavdata)
    #     your_request = uavdata.Request()  # 创建一个新的请求
    #     response = service(your_request)  # 你的请求
    #     print(response)
    # except rospy.ServiceException as e:
    #     print("Service call failed: %s"%e)

if __name__ == '__main__':
    call_service()
    # rospy.init_node('node_name')  # 你的节点名
    # topics = rospy.get_published_topics()
    # services = rospy.get_published_topics()

    # for service in services:
    #     print("Service: ", service)
    # for topic, type in topics:
    #     print("Topic: ", topic, " Type: ", type)

