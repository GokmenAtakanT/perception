import cv2
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
import rclpy 
import numpy as np

def nothing(x):
    pass

cv2.namedWindow("Tracking")
cv2.createTrackbar("LH", "Tracking", 21, 255, nothing)
cv2.createTrackbar("LS", "Tracking", 69, 255, nothing)
cv2.createTrackbar("LV", "Tracking", 163, 255, nothing)
cv2.createTrackbar("UH", "Tracking", 255, 255, nothing)
cv2.createTrackbar("US", "Tracking", 255, 255, nothing)
cv2.createTrackbar("UV", "Tracking", 255, 255, nothing)

class Video_feed_in(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        self.subscriber = self.create_subscription(Image,'/carla/ego_vehicle/rgb_front/image',self.process_data,10)
        self.bridge   = CvBridge()  #converting ros images to opencv data

    def process_data(self, data): 
        frame = self.bridge.imgmsg_to_cv2(data,'bgr8')  #performing conversion
        #lower_bound = np.array(color, dtype=np.uint8)
        #upper_bound = np.array(color, dtype=np.uint8)

        l_h = cv2.getTrackbarPos("LH", "Tracking")
        l_s = cv2.getTrackbarPos("LS", "Tracking")
        l_v = cv2.getTrackbarPos("LV", "Tracking")

        u_h = cv2.getTrackbarPos("UH", "Tracking")
        u_s = cv2.getTrackbarPos("US", "Tracking")
        u_v = cv2.getTrackbarPos("UV", "Tracking")

        l_b = np.array([l_h, l_s, l_v], dtype=np.uint8)
        u_b = np.array([u_h, u_s, u_v], dtype=np.uint8)

        mask = cv2.inRange(frame, l_b, u_b)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        #cv2.rectangle(frame, (x, y), (x + w, y + h), color, 5)
        #cv2.putText(frame, class_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        cv2.imshow("Mask", mask)

        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = Video_feed_in()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()




