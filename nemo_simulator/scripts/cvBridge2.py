#!/usr/bin/env python3
import cv2
import sys
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist # Alteração inteira
import matplotlib as plt





rospy.init_node("cv_bridge", anonymous=True)

class CameraHandler:
    def __init__(self):
       
        self.img = np.zeros((1000, 1000))
        # cria array shape 1000x1000 preenchido de zeros. Representa o array da imagem e resolução
       
        self.pub = rospy.Publisher('cmd_vel',Twist, queue_size=10) 

        self.bridge = CvBridge()
        rospy.Subscriber("camera/image_raw", Image, self.callback)
        rospy.wait_for_message("camera/image_raw", Image)
        

    def callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        except CvBridgeError:
            print("Frame Dropped: ")


    def fetchImg(self):
        return self.img


    def showImg(self):
        cv2.imshow("Frontal Camera", self.img)
    
    def mov(self,move_marlin = Twist()): 
       
        temp= rospy.Time.now().to_sec() 
       
    
        move_marlin.linear.x = temp*0.02
        move_marlin.linear.y = temp*0.2
        self.pub.publish(move_marlin) 
    

def mask(image_constrained, color):

    hsv_screenshot = cv2.cvtColor(image_constrained, cv2.COLOR_BGR2HSV)

    if color == "green":
        lower_green = np.array([36,25,25])
        upper_green = np.array([36,255,255])
        
        mask = cv2.inRange(hsv_screenshot, lower_green, upper_green)



    elif color == "gray" or "grey":
        lower_gray = np.array([0, 0, 128])
        upper_gray = np.array([180, 0, 130])

        mask = cv2.inRange(hsv_screenshot, lower_gray, upper_gray)

    result = cv2.bitwise_and(image_constrained, image_constrained, mask=mask)

    visualization_result = np.copy(result)

    return  mask

def get_move(mask_main_picture): 

    contours,hierarchy = cv2.findContours(mask_main_picture, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    cnt = contours[0]
    
    mom = cv2.moments(cnt)
    cx = mom['m10']/mom['m00']
    cy =  mom['m01']/mom['m00']


    return [cx,cy]

# ---------------------------

print("teste1")
handler = CameraHandler()
print("teste2")


rate = rospy.Rate(1) 
while not rospy.is_shutdown():

    print(handler.fetchImg().shape)
    feed = handler.fetchImg()
    desl= handler.mov() 
    cv2.imshow("feed", feed)
    cv2.waitKey()



    #considerando a imagem em pixel resolução 720x480, o eixo x correspondente ao lado esquerdo da
    #tela será 360.5 ~ 0 e o dereito será de 360.5 ~ 720.

    

    rate.sleep()
    print("teste")