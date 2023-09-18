#!/usr/bin/env python3

import rospy
import mediapipe as mp
import numpy as np
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2

cap = cv2.VideoCapture(0)
print(cap.isOpened())
bridge = CvBridge()

FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
mp_drawings = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands
rospy.init_node("Hand_gesture_publisher", anonymous=True)


def get_coordinates(index, hand, results):
    
    output = None
    for index, classification in enumerate(results.multi_handedness):
        if classification.classification[0].index == index:

            #print(index)
            
            label = classification.classification[0].label
            sign = "nothing"
            wrist_coords = np.multiply(np.array((hand.landmark[mp_hands.HandLandmark.WRIST].x, hand.landmark[mp_hands.HandLandmark.WRIST].y)), [FRAME_WIDTH, FRAME_HEIGHT]).astype(int)
            thumb_cmc_coords = np.multiply(np.array((hand.landmark[mp_hands.HandLandmark.THUMB_CMC].x, hand.landmark[mp_hands.HandLandmark.THUMB_CMC].y)), [FRAME_WIDTH, FRAME_HEIGHT]).astype(int)
            thumb_mcp_coords = np.multiply(np.array((hand.landmark[mp_hands.HandLandmark.THUMB_MCP].x, hand.landmark[mp_hands.HandLandmark.THUMB_MCP].y)), [FRAME_WIDTH, FRAME_HEIGHT]).astype(int)
            thumb_ip_coords = np.multiply(np.array((hand.landmark[mp_hands.HandLandmark.THUMB_IP].x, hand.landmark[mp_hands.HandLandmark.THUMB_IP].y)), [FRAME_WIDTH, FRAME_HEIGHT]).astype(int)
            thumb_tip_coords = np.multiply(np.array((hand.landmark[mp_hands.HandLandmark.THUMB_TIP].x, hand.landmark[mp_hands.HandLandmark.THUMB_TIP].y)), [FRAME_WIDTH, FRAME_HEIGHT]).astype(int)
            index_finger_mcp_coords = np.multiply(np.array((hand.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].x, hand.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP].y)), [FRAME_WIDTH, FRAME_HEIGHT]).astype(int)
            index_finger_pip_coords = np.multiply(np.array((hand.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP].x, hand.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP].y)), [FRAME_WIDTH, FRAME_HEIGHT]).astype(int)
            index_finger_dip_coords = np.multiply(np.array((hand.landmark[mp_hands.HandLandmark.INDEX_FINGER_DIP].x, hand.landmark[mp_hands.HandLandmark.INDEX_FINGER_DIP].y)), [FRAME_WIDTH, FRAME_HEIGHT]).astype(int)
            index_finger_tip_coords = np.multiply(np.array((hand.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].x, hand.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y)), [FRAME_WIDTH, FRAME_HEIGHT]).astype(int)
            middle_finger_mcp_coords = np.multiply(np.array((hand.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].x, hand.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP].y)), [FRAME_WIDTH, FRAME_HEIGHT]).astype(int)
            middle_finger_pip_coords = np.multiply(np.array((hand.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP].x, hand.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP].y)), [FRAME_WIDTH, FRAME_HEIGHT]).astype(int)
            middle_finger_dip_coords = np.multiply(np.array((hand.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_DIP].x, hand.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_DIP].y)), [FRAME_WIDTH, FRAME_HEIGHT]).astype(int)
            middle_finger_tip_coords = np.multiply(np.array((hand.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].x, hand.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y)), [FRAME_WIDTH, FRAME_HEIGHT]).astype(int)
            ring_finger_mcp_coords = np.multiply(np.array((hand.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].x, hand.landmark[mp_hands.HandLandmark.RING_FINGER_MCP].y)), [FRAME_WIDTH, FRAME_HEIGHT]).astype(int)
            ring_finger_pip_coords = np.multiply(np.array((hand.landmark[mp_hands.HandLandmark.RING_FINGER_PIP].x, hand.landmark[mp_hands.HandLandmark.RING_FINGER_PIP].y)), [FRAME_WIDTH, FRAME_HEIGHT]).astype(int)
            ring_finger_dip_coords = np.multiply(np.array((hand.landmark[mp_hands.HandLandmark.RING_FINGER_DIP].x, hand.landmark[mp_hands.HandLandmark.RING_FINGER_DIP].y)), [FRAME_WIDTH, FRAME_HEIGHT]).astype(int)
            ring_finger_tip_coords = np.multiply(np.array((hand.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].x, hand.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].y)), [FRAME_WIDTH, FRAME_HEIGHT]).astype(int)
            pinky_mcp_coords = np.multiply(np.array((hand.landmark[mp_hands.HandLandmark.PINKY_MCP].x, hand.landmark[mp_hands.HandLandmark.PINKY_MCP].y)), [FRAME_WIDTH, FRAME_HEIGHT]).astype(int)
            pinky_dip_coords = np.multiply(np.array((hand.landmark[mp_hands.HandLandmark.PINKY_PIP].x, hand.landmark[mp_hands.HandLandmark.PINKY_PIP].y)), [FRAME_WIDTH, FRAME_HEIGHT]).astype(int)
            pinky_pip_coords = np.multiply(np.array((hand.landmark[mp_hands.HandLandmark.PINKY_DIP].x, hand.landmark[mp_hands.HandLandmark.PINKY_DIP].y)), [FRAME_WIDTH, FRAME_HEIGHT]).astype(int)
            pinky_tip_coords = np.multiply(np.array((hand.landmark[mp_hands.HandLandmark.PINKY_TIP].x, hand.landmark[mp_hands.HandLandmark.PINKY_TIP].y)), [FRAME_WIDTH, FRAME_HEIGHT]).astype(int)
            first_finger_vector1 = index_finger_tip_coords - index_finger_dip_coords
            first_finger_vector2 = index_finger_pip_coords - index_finger_dip_coords
            dot_product1 = np.dot(first_finger_vector1, first_finger_vector2)
            first_magnitude1 = np.linalg.norm(first_finger_vector1)
            first_magnitude2 = np.linalg.norm(first_finger_vector2)
            angle1 = np.degrees(np.arccos(dot_product1 / (first_magnitude1 * first_magnitude2)))
            second_finger_vector1 = middle_finger_tip_coords - middle_finger_dip_coords
            second_finger_vector2 = middle_finger_pip_coords - middle_finger_dip_coords
            dot_product2 = np.dot(second_finger_vector1, second_finger_vector2)
            second_magnitude1 = np.linalg.norm(second_finger_vector1)
            second_magnitude2 = np.linalg.norm(second_finger_vector2)
            angle2 = np.degrees(np.arccos(dot_product2 / (second_magnitude1 * second_magnitude2)))
            third_finger_vector1 = ring_finger_tip_coords - ring_finger_dip_coords
            third_finger_vector2 = ring_finger_pip_coords - ring_finger_dip_coords
            dot_product3 = np.dot(third_finger_vector1, third_finger_vector2)
            third_magnitude1 = np.linalg.norm(third_finger_vector1)
            third_magnitude2 = np.linalg.norm(third_finger_vector2)
            angle3 = np.degrees(np.arccos(dot_product3 / (third_magnitude1 * third_magnitude2)))
            fourth_finger_vector1 = pinky_tip_coords - pinky_dip_coords
            fourth_finger_vector2 = pinky_pip_coords - pinky_dip_coords
            dot_product4 = np.dot(fourth_finger_vector1, fourth_finger_vector2)
            fourth_magnitude1 = np.linalg.norm(fourth_finger_vector1)
            fourth_magnitude2 = np.linalg.norm(fourth_finger_vector2)
            angle4 = np.degrees(np.arccos(dot_product4 / (fourth_magnitude1 * fourth_magnitude2)))
            thumb_vector1 = thumb_tip_coords - thumb_ip_coords
            thumb_vector2 = thumb_mcp_coords - thumb_ip_coords
            dot_product5 = np.dot(thumb_vector1, thumb_vector2)
            thumb_magnitude1 = np.linalg.norm(thumb_vector1)
            thumb_magnitude2 = np.linalg.norm(thumb_vector2)
            angle5 = np.degrees(np.arccos(dot_product5 / (thumb_magnitude1 * thumb_magnitude2)))
            #angles = [angle1, angle2, angle3, angle4, angle5]
            #angles_array = np.array(angles)

            if angle1 > 170 and angle1 < 180 and angle2 > 170 and angle2 < 180 and angle3 > 170 and angle3 < 180 and angle4 > 170 and angle4 < 180 and angle5 > 170 and angle5 < 180:
                sign = "open_hand"
            else:
                sign = "closed_palm"
            
            return sign
            
        


def talker():

    publisher = rospy.Publisher("hand_gesture", String, queue_size=10)
    rate = rospy.Rate(10)
    
    if not cap.isOpened():
        rospy.logerr("Cannot open the camera unfortunately")
        return
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break

        with mp_hands.Hands(min_detection_confidence=0.3, min_tracking_confidence=0.4) as hands:
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = cv2.flip(image, 1)
            image.flags.writeable = False
            results = hands.process(image)
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            if results.multi_hand_landmarks:
                for num, hand in enumerate(results.multi_hand_landmarks):
                    mp_drawings.draw_landmarks(image, hand, mp_hands.HAND_CONNECTIONS,
                                               mp_drawings.DrawingSpec(color=(121, 21, 75), thickness = 3, circle_radius=4), 
                                               mp_drawings.DrawingSpec(color=(251, 43, 249), thickness = 3, circle_radius = 2),)


                    hand_gesture = get_coordinates(num, hand, results)
                    publisher.publish(hand_gesture)
            cv2.imshow("Frame", image)
        
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
        rate.sleep()


    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    