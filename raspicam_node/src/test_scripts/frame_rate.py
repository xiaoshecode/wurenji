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

if __name__ == '__main__':
    rospy.init_node('qr_code_reader', anonymous=True)
    rtsp_url = "rtsp://127.0.0.1:8554/live"
    pub = rospy.Publisher('/qr_code_data', String, queue_size=10)
    i = 0
    start_time = rospy.get_time()
    try:
        while not rospy.is_shutdown():
            cap = cv2.VideoCapture(rtsp_url)
            ret, frame = cap.read()
            if cv2.waitKey(1) == ord('q'):
                break

            if ret:
                i += 1.0
                t = rospy.get_time() - start_time
                for aa in range(20):
                    rospy.logwarn("Frame Rate: %f" % (i/t))
            else:
                print("Failed to read frame")
                break
            
    except KeyboardInterrupt:
        print("Shutting down")

    cap.release()
    cv2.destroyAllWindows()

