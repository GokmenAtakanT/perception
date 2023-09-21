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

        font = cv2.FONT_HERSHEY_SIMPLEX
        img = frame
        cimg = img
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # color range
        lower_red1 = np.array([0,100,100])
        upper_red1 = np.array([10,255,255])
        lower_red2 = np.array([160,100,100])
        upper_red2 = np.array([180,255,255])
        lower_green = np.array([40,50,50])
        upper_green = np.array([90,255,255])
        lower_yellow = np.array([15,150,150])
        upper_yellow = np.array([35,255,255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        maskg = cv2.inRange(hsv, lower_green, upper_green)
        masky = cv2.inRange(hsv, lower_yellow, upper_yellow)
        maskr = cv2.add(mask1, mask2)

        size = img.shape

        r = 5
        bound = 4.0 / 10

        # hough circle detect
        r_circles = cv2.HoughCircles(maskr, cv2.HOUGH_GRADIENT, 1, 80,
                                    param1=50, param2=10, minRadius=0, maxRadius=30)

        g_circles = cv2.HoughCircles(maskg, cv2.HOUGH_GRADIENT, 1, 60,
                                    param1=50, param2=10, minRadius=0, maxRadius=30)

        y_circles = cv2.HoughCircles(masky, cv2.HOUGH_GRADIENT, 1, 30,
                                    param1=50, param2=5, minRadius=0, maxRadius=30)

        # traffic light detect
        if r_circles is not None:
            r_circles = np.uint16(np.around(r_circles))

            for i in r_circles[0, :]:
                if i[0] > size[1] or i[1] > size[0]or i[1] > size[0]*bound:
                    continue

                h, s = 0.0, 0.0
                for m in range(-r, r):
                    for n in range(-r, r):

                        if (i[1]+m) >= size[0] or (i[0]+n) >= size[1]:
                            continue
                        h += maskr[i[1]+m, i[0]+n]
                        s += 1
                if h / s > 50:
                    cv2.circle(cimg, (i[0], i[1]), i[2]+10, (0, 255, 0), 2)
                    cv2.circle(maskr, (i[0], i[1]), i[2]+30, (255, 255, 255), 2)
                    cv2.putText(cimg,'RED',(i[0], i[1]), font, 1,(255,0,0),2,cv2.LINE_AA)

        if g_circles is not None:
            g_circles = np.uint16(np.around(g_circles))

            for i in g_circles[0, :]:
                if i[0] > size[1] or i[1] > size[0] or i[1] > size[0]*bound:
                    continue

                h, s = 0.0, 0.0
                for m in range(-r, r):
                    for n in range(-r, r):

                        if (i[1]+m) >= size[0] or (i[0]+n) >= size[1]:
                            continue
                        h += maskg[i[1]+m, i[0]+n]
                        s += 1
                if h / s > 100:
                    cv2.circle(cimg, (i[0], i[1]), i[2]+10, (0, 255, 0), 2)
                    cv2.circle(maskg, (i[0], i[1]), i[2]+30, (255, 255, 255), 2)
                    cv2.putText(cimg,'GREEN',(i[0], i[1]), font, 1,(255,0,0),2,cv2.LINE_AA)

        if y_circles is not None:
            y_circles = np.uint16(np.around(y_circles))

            for i in y_circles[0, :]:
                if i[0] > size[1] or i[1] > size[0] or i[1] > size[0]*bound:
                    continue

                h, s = 0.0, 0.0
                for m in range(-r, r):
                    for n in range(-r, r):

                        if (i[1]+m) >= size[0] or (i[0]+n) >= size[1]:
                            continue
                        h += masky[i[1]+m, i[0]+n]
                        s += 1
                if h / s > 50:
                    cv2.circle(cimg, (i[0], i[1]), i[2]+10, (0, 255, 0), 2)
                    cv2.circle(masky, (i[0], i[1]), i[2]+30, (255, 255, 255), 2)
                    cv2.putText(cimg,'YELLOW',(i[0], i[1]), font, 1,(255,0,0),2,cv2.LINE_AA)

        cv2.imshow('detected results', cimg)
        cv2.waitKey(1)

def main(args=None):
  rclpy.init(args=args)
  image_subscriber = Video_feed_in()
  rclpy.spin(image_subscriber)
  rclpy.shutdown()

if __name__ == '__main__':
    main()

