#!/usr/bin/env python3





# Node erkennt rote pixel auf dem bild.
# wenn eine gewisse kurze zeit auf dem bild genug rote pixel gesehen wurde,
# published es eine msg, dass eine intersection da ist


import os
import rospy #type: ignore
import numpy as np
from duckietown.dtros import DTROS, NodeType #type: ignore

import cv2
from cv_bridge import CvBridge #type: ignore
from sensor_msgs.msg import CompressedImage #type: ignore
from std_msgs.msg import Bool #type: ignore




class DetectIntersectionNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(DetectIntersectionNode, self).__init__(node_name=node_name, node_type=NodeType.PLANNING)
        
        vehicle_name = os.environ['VEHICLE_NAME']
        #self._wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"


        camera_topic = f"/{vehicle_name}/camera_node/image/compressed"
        
        stop_line_detected_topic = f"/{vehicle_name}/detect_stop_line_node/stop_line_detected"

        #subscriber
        self.sub_image = rospy.Subscriber(camera_topic, CompressedImage, self.callback_image)

        #publisher
        self.pub = rospy.Publisher(stop_line_detected_topic, Bool, queue_size=1)


        self.image_msg = None
        self._bridge = CvBridge()


        #TODO Threshhold anpassen
        self.red_pixel_treshhold=500


        self.red_frames=0
        self.not_red_frames=0

        self.red_frames_threshhold=20
        self.not_red_frames_threshhold=20
    

        print("INIT: detect_intersection_node!")




    def callback_image(self, msg):
        
        image = self._bridge.compressed_imgmsg_to_cv2(msg)

        #cv2.imshow("image", image)

        höhe, breite, kanäle = image.shape

        o_l = (145,216)   
        o_r = (430,221)   
        u_l = (-160,400)   
        u_r = (672,640) 

        #cv2.circle(image, o_l, 5,(0,0, 255), -1)
        #cv2.circle(image, o_r, 5,(0,0, 255), -1)
        #cv2.circle(image, u_l, 5,(0,0, 255), -1)
        #cv2.circle(image, u_r, 5,(0,0, 255), -1)

                

        pts1=np.float32([o_l, u_l, o_r, u_r])
        pts2=np.float32([[0,0],[0,480],[640,0],[480,640]])

        
        matrix_trans = cv2.getPerspectiveTransform(pts1, pts2)
        img_trans = cv2.warpPerspective(image, matrix_trans, (breite,höhe))

        # Extrahiere die Rot-, Grün- und Blaukanäle
        B, G, R = cv2.split(img_trans)

        # Erstelle eine leere Maske für rote Bereiche
        red_mask_rgb = np.zeros_like(R)

        # Definiere Schwellenwerte für Rot. Beispiel: Rot muss größer als 150 sein und Grün/Blau müssen kleiner sein
        red_threshold = 140
        non_red_threshold = 120

        # Erstelle die Maske, die anzeigt, wo das Bild rot ist
        red_mask_rgb[(R > red_threshold) & (G < non_red_threshold) & (B < non_red_threshold)] = 255


        cv2.imshow("red rgb", red_mask_rgb)
        cv2.waitKey(0)


        num_red_pixels = np.sum(red_mask_rgb == 255)

        print(f"Anzahl der roten Pixel: {num_red_pixels}")

        

        #Wenn ununterbrochen auf einigen Bildern genug rote Pixel zu sehen sind, wird eine msg gepublished, 
        # dass eine stop line detectet wurde, andersum, wenn auf genug bildern nicht genug rote pixel erkannt wurden,
        # wird gepublished, dass keine rote linie detected wurde

        if(num_red_pixels>=self.red_pixel_treshhold):
            self.red_frames=self.red_frames+1
            self.not_red_frames=0
        
        else:
            self.not_red_frames=self.not_red_frames+1
            self.red_frames=0
            

        

        if(self.red_frames>=self.red_frames_threshhold):
            #msg publishen, dass stop line erkannt wurde
            msg1 = Bool()
            msg1.data = True
            print("Stop_line detected")
            self.pub.publish(msg1)

            #counter zurück auf 0 setzten
            self.red_frames=0

        if(self.not_red_frames>=self.not_red_frames_threshhold):
            #msg publishen, dass keine stop line erkannt wurde
            msg2 = Bool()
            msg2.data = False
            print("No stop_line detected")
            self.pub.publish(msg2)

            #counter zurück auf 0 setzten
            self.not_red_frames=0
