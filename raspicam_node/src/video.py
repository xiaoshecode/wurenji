#usr/bin/env python2.7
import rospy
from std_msgs.msg import String
import cv2
import csv
import pyzbar.pyzbar as pyzbar

found = set()
qrcsv = '456.csv'

def process_frame(frame, i):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #gray = frame
    cv2.imwrite('chessboards/'+str(i)+'.png', gray)
    print("Saved "+str(i)+"-th fig")
    decoded_qr_codes = pyzbar.decode(gray)

    for qr_code in decoded_qr_codes:
       print("QR code data: ", qr_code.data)
       qr_code_data = qr_code.data
       if qr_code_data not in found:
        with open(qrcsv,'a+',) as f:
            csv_write = csv.writer(f)
            date = [qr_code_data]
            csv_write.writerow(date)
        found.add(qr_code_data)
        
        # Publish QR code data as a string
       pub.publish(qr_code.data)
       

#    cv2.imshow("QR Code Reader", gray)
    print("success to get frame")
    cv2.waitKey(1)


if __name__ == '__main__':
    i = 1
    is_capture = False
    rospy.init_node('qr_code_reader', anonymous=True)
    rtsp_url = "rtsp://127.0.0.1:8554/live"
    pub = rospy.Publisher('/qr_code_data', String, queue_size=10)

    #cap = cv2.VideoCapture(rtsp_url)

    try:
        while not rospy.is_shutdown():
            is_capture = input("Waiting for command\n")
            cap = cv2.VideoCapture(rtsp_url)
            ret, frame = cap.read()
            if cv2.waitKey(1) == ord('q'):
                break
            if is_capture:
                if ret:
                    process_frame(frame, i)
            else:
                print("Failed to read frame")
                break
            i += 1
            is_capture = 0
            
    except KeyboardInterrupt:
        print("Shutting down")

    cap.release()
    cv2.destroyAllWindows()

