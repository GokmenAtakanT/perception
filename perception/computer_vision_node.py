
# import cv2
# from rclpy.node import Node 
# from cv_bridge import CvBridge 
# from sensor_msgs.msg import Image 
# import rclpy 


# class Video_feed_in(Node):
#     def __init__(self):

#         super().__init__('video_subscriber')
#         self.subscriber = self.create_subscription(Image,'/carla/ego_vehicle/rgb_front/image',self.process_data,10)
#         self.bridge   = CvBridge() # converting ros images to opencv data
 

        
#     def process_data(self, data): 
#         """Processes the data stream from the sensor (camera) and passes on to the 
#            Self Drive Algorithm which computes and executes the appropriate control
#            (Steering and speed) commands.

#         Args:
#             data (img_msg): image data from the camera received as a ros message
#         """
#         #self.Debug.setDebugParameters()

#         frame = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion

#         #Angle,Speed,img = self.Car.driveCar(frame)

#         #self.velocity.angular.z = Angle
#         #self.velocity.linear.x = Speed      

#         cv2.imshow("Frame",frame)
#         cv2.waitKey(1)
        
 
# def main(args=None):
#   rclpy.init(args=args)
#   image_subscriber = Video_feed_in()
#   rclpy.spin(image_subscriber)
#   rclpy.shutdown()

# if __name__ == '__main__':
# 	main()

import cv2
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
import rclpy 
import numpy as np

class Video_feed_in(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        self.subscriber = self.create_subscription(Image,'/carla/ego_vehicle/rgb_front/image',self.process_data,10)
        self.bridge   = CvBridge() # converting ros images to opencv data

    def process_data(self, data): 
        frame = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion

        # Define the color mappings
        classes = {
            #0: [0, 0, 0],         # None
            #1: [70, 70, 70],      # Buildings
            #2: [190, 153, 153],   # Fences
            #3: [72, 0, 90],       # Other
            #4: [220, 20, 60],     # Pedestrians
            #5: [153, 153, 153],   # Poles
            #6: [157, 234, 50],    # RoadLines
            #7: [128, 64, 128],    # Roads
            #8: [244, 35, 232],    # Sidewalks
            #9: [107, 142, 35],    # Vegetation
            #10: [0, 0, 255],      # Vehicles
            #11: [102, 102, 156],  # Walls
            12: [220, 220, 0]     # TrafficSigns
        }

        # Create masks for each class
        masks = {}
        for class_id, color in classes.items():
            lower_bound = np.array(color, dtype=np.uint8)
            upper_bound = np.array(color, dtype=np.uint8)
            mask = cv2.inRange(frame, lower_bound, upper_bound)
            masks[class_id] = mask

        # Find contours and draw bounding boxes for each class
        for class_id, mask in masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                class_name = str(class_id)
                color = classes[class_id]
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 5)
                cv2.putText(frame, class_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

def main(args=None):
  rclpy.init(args=args)
  image_subscriber = Video_feed_in()
  rclpy.spin(image_subscriber)
  rclpy.shutdown()

if __name__ == '__main__':
    main()
