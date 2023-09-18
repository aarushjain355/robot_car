#!/usr/bin/env python3

import rospy
import mediapipe as mp
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

cap = cv2.VideoCapture(0)
print(cap.isOpened())
bridge = CvBridge()

def talker():

    rospy.init_node("Camera Publisher", anonymous=True)
    pub = rospy.Publisher("images", Image, queue_size=10)
    rate = rospy.Rate(10)

    if not cap.isOpened():
        rospy.logerr("Cannot open the camera unfortunately")
        return
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break

        cv2.imshow("frame", frame)
        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        pub.publish(msg)

        if cv2.waitKey(1) == ord("q"):
            break
        
        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    