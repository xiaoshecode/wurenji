#usr/bin/env python2.7
import rospy
from std_msgs.msg import String
import cv2
import csv
import pyzbar.pyzbar as pyzbar
import apriltag
import os

found = set()
qrcsv = '456.csv'

def process_frame(frame):
    QRCode_real_size = 60 # cm
    Focal_length = 1.887196348261401e+03 # pixel
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    distance = None

    at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))
    tags = at_detector.detect(gray)
    QRCode_pixel_size = None
    for tag in tags:
        QRCode_pixel_size = tag.corners[1].astype(int)[0]-tag.corners[0].astype(int)[0]

    if QRCode_pixel_size is not None:
        distance = QRCode_real_size * Focal_length / QRCode_pixel_size
        for i in range(20):
            rospy.logwarn("success to get frame, distance = %d cm" % distance)
    cv2.waitKey(5)


if __name__ == '__main__':
    rospy.init_node('qr_code_reader', anonymous=True)
    rtsp_url = "rtsp://127.0.0.1:8554/live"
    pub = rospy.Publisher('/qr_code_data', String, queue_size=10)

    try:
        while not rospy.is_shutdown():
            cap = cv2.VideoCapture(rtsp_url)
            ret, frame = cap.read()
            if cv2.waitKey(1) == ord('q'):
                break

            if ret:
                process_frame(frame)
            else:
                print("Failed to read frame")
                break
            
    except KeyboardInterrupt:
        print("Shutting down")

    cap.release()
    cv2.destroyAllWindows()

