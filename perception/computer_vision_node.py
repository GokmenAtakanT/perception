import cv2
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
import rclpy 


class Video_feed_in(Node):
    def __init__(self):

        super().__init__('video_subscriber')
        self.subscriber = self.create_subscription(Image,'/carla/ego_vehicle/rgb_front/image',self.process_data,10)
        self.bridge   = CvBridge() # converting ros images to opencv data
 

        
    def process_data(self, data): 
        """Processes the data stream from the sensor (camera) and passes on to the 
           Self Drive Algorithm which computes and executes the appropriate control
           (Steering and speed) commands.

        Args:
            data (img_msg): image data from the camera received as a ros message
        """
        #self.Debug.setDebugParameters()

        frame = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion

        #Angle,Speed,img = self.Car.driveCar(frame)

        #self.velocity.angular.z = Angle
        #self.velocity.linear.x = Speed      

        cv2.imshow("Frame",frame)
        cv2.waitKey(1)
        
 
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = Video_feed_in()
  rclpy.spin(image_subscriber)
  rclpy.shutdown()

if __name__ == '__main__':
	main()