#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage


import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np


class LineDetectionSlwNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(LineDetectionSlwNode, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._line_images_topic = f"/{self._vehicle_name}/image_process_node/line_images"
        self._line_arrays_topic = f"/{self._vehicle_name}/line_detection_slw_node/line_arrays"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()

        # construct subscriber
        self.sub = rospy.Subscriber(self._line_images_topic, CompressedImage, self.callback)

        #publisher
        self.pub = rospy.Publisher(self._line_arrays_topic, Float32MultiArray, queue_size=1)


        self.height = 480
        self.width = 640
        self.num_elements = self.height * self.width
        self.last_time = rospy.get_time()


        self.last_yellow_startpoint=None
        self.last_white_startpoint=None
        self.last_yellow_startpoint_weight=0
        self.last_white_startpoint_weight=0

        self.yellow_threshold=30
        self.white_threshold=30




    def callback(self, msg):
        last_time1 = rospy.get_time()

        höhe = self.height
        breite = self.width

        """"
        dim = msg.layout.dim[0]


     
    
        # Größe der Linienmatrix
        size = dim.size


        data = np.array(msg.data)

        # Weise Linienmatrix rekonstruieren
        white_lines_flat = data[:size]
        white_lines = white_lines_flat.reshape((höhe, breite))

        # Gelbe Linienmatrix rekonstruieren
        yellow_lines_flat = data[size:2*size]
        yellow_lines = yellow_lines_flat.reshape((höhe, breite))



        #cv2.imshow("white lines", white_lines)
        #cv2.imshow("yellow lines", yellow_lines)

        #print("line images recived")

        """

        #Bild empfangen und zurück konvertieren aus Compressed Image

        try:
            #rospy.loginfo("Received message with format: %s", msg.format)
            
            # Überprüfen des Bildformats und der Kanalanzahl
            np_arr = np.frombuffer(msg.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            
            if image_np is None:
                raise CvBridgeError("Failed to decode image")

            #rospy.loginfo("Decoded image with shape: %s", image_np.shape)
            
            if len(image_np.shape) == 2:
                # Bild hat einen Kanal (Graustufen)
                uint8_matrix = image_np
            else:
                raise CvBridgeError("Unexpected number of channels: %s" % (image_np.shape,))

            #rospy.loginfo("Image successfully converted to uint8 matrix with shape: %s", uint8_matrix.shape)

            # Konvertieren zu Float-Werten (0 -> 0.0, 255 -> 1.0)
            matrix = (uint8_matrix / 255.0).astype(np.float32)

            #rospy.loginfo("Matrix converted to float with shape: %s", matrix.shape)
        except CvBridgeError as e:
            #rospy.logerr("CvBridge Error: %s", str(e))
            return  # Abbruch bei Fehler
        except Exception as e:
            rospy.logerr("Unexpected Error: %s", str(e))
            return  # Abbruch bei Fehler

            # Matrixen separieren
        try:
            yellow_lines = matrix[:, :self.width]
            white_lines = matrix[:, self.width:]
            #rospy.loginfo("Matrix successfully separated into yellow and white lines.")
        except Exception as e:
            rospy.logerr("Error separating matrix: %s", str(e))


        #Setzen der Startpunkte mithilfe der vorherigen Startpunkte

        if (self.last_yellow_startpoint != None) & (self.last_yellow_startpoint_weight > self.yellow_threshold):
            yellow_line_startpoint=self.last_yellow_startpoint

        else:
            yellow_his = np.sum(yellow_lines[höhe//2:,:],axis=0)

            #Summe für ausschlagkräftigkeit des startpunktes
            yellow_pixel_count= np.sum(yellow_his)

            try:
                yellow_line_startpoint = np.argmax(yellow_his)
                #print(f"yellow line startpoint {yellow_line_startpoint}")
            except:
                yellow_line_startpoint=143
                #print(f"Default Yellow Startpoint {yellow_line_startpoint}")




        if (self.last_white_startpoint != None) & (self.last_white_startpoint_weight > self.white_threshold):
            white_line_startpoint=self.last_white_startpoint
        
        else:
            white_his = np.sum(white_lines[höhe//2:,:],axis=0)

            try:
                white_line_startpoint = np.argmax(white_his)
                #print(f"white line startpoint  {white_line_startpoint}")
            except:
                white_line_startpoint=497
                #print(f"Default White Startpoint  {white_line_startpoint}")




        breite_slw=120
        breite_slw_init=100
        höhe_slw=32 #-->15 SLW auf 480 höhe gesamt

        yellow_lines_slw=np.copy(yellow_lines)
        white_lines_slw=np.copy(white_lines)
        gelb_x=[]
        weis_x=[]

        y=höhe-höhe_slw

        

        #GELB: initiales SLW und Histogram + Hochpunkt in jeweilige x-Arrays einfügen
        init_slw_gelb=yellow_lines[y:y+höhe_slw,yellow_line_startpoint-breite_slw_init//2:yellow_line_startpoint+breite_slw_init//2]

        init_slw_gelb_his = np.sum(init_slw_gelb,axis=0)

        #Summe für ausschlagkräftigkeit des startpunktes
        yellow_pixel_count = np.sum(init_slw_gelb_his)

        self.last_yellow_startpoint_weight = yellow_pixel_count



        try:
            yellow_pixel_count = np.max(init_slw_gelb_his)
        except:
            yellow_pixel_count = 0

        
        if yellow_pixel_count>10:
            
            init_slw_gelb_x =np.argmax(init_slw_gelb_his)
            gelb_x.append((init_slw_gelb_x+yellow_line_startpoint-breite_slw_init//2))
        
        else:
            gelb_x.append(yellow_line_startpoint)

        #print(f"init_slw_gelb_x+yellow_line_startpoint-breite_slw_init//2 {init_slw_gelb_x} + {yellow_line_startpoint} - {breite_slw_init//2}")

        cv2.rectangle(yellow_lines_slw,(yellow_line_startpoint-breite_slw_init//2,y+höhe_slw),(yellow_line_startpoint+breite_slw_init//2,y),(255,255,0),2)

        #cv2.imshow("yellow + init Window", yellow_lines_slw)

        
        #WEIS: initiales SLW und Histogram + Hochpunkt in jeweilige x-Arrays einfügen
        init_slw_weis=white_lines[y:y+höhe_slw,white_line_startpoint-breite_slw_init//2:white_line_startpoint+breite_slw_init//2]

        init_slw_weis_his = np.sum(init_slw_weis,axis=0)

        #Summe für ausschlagkräftigkeit des startpunktes
        white_pixel_count = np.sum(init_slw_weis_his)

        self.last_white_startpoint_weight=white_pixel_count



        try:
            white_pixel_count = np.max(init_slw_weis_his)
        except:
            white_pixel_count = 0

        

        if white_pixel_count>10:
            
            init_slw_weis_x =np.argmax(init_slw_weis_his)
            weis_x.append((init_slw_weis_x+white_line_startpoint-breite_slw_init//2))
        
        else:
            weis_x.append(white_line_startpoint)

        cv2.rectangle(white_lines_slw,(white_line_startpoint-breite_slw_init//2,y+höhe_slw),(white_line_startpoint+breite_slw_init//2,y),(255,255,0),2)

        #cv2.imshow("white + init Window", white_lines_slw)
        
        


        #print(f"gelb_x_array {gelb_x[0]}")
        #print(f"gelbe max. pixelanzahl von init window {yellow_pixel_count}")
        #print(f"weis_x_array {weis_x[0]}")
        #print(f"weise max. pixelanzahl von init window {np.max(init_slw_weis_his)}")


        #restliche linepoints finden
        #yellow_weight und white_weight sind summen der gefundenen Pixel, um dies bei der Berechnung des mittelpunktes zu gewichten

        yellow_weight=0
        white_weight=0

        i=0

        while y>0:

            y= y-höhe_slw

            #GELB:
            slw_gelb=yellow_lines[y:y+höhe_slw, gelb_x[i]-breite_slw//2:gelb_x[i]+breite_slw//2]

            slw_gelb_his = np.sum(slw_gelb,axis=0)

            try:
                yellow_slw_pixel_count = np.argmax(slw_gelb_his)
            except:
                yellow_slw_pixel_count = 0

            yellow_weight=yellow_weight+yellow_slw_pixel_count

            if yellow_slw_pixel_count>1:
            
                slw_gelb_x = np.argmax(slw_gelb_his)
                gelb_x.append((slw_gelb_x+gelb_x[i]-breite_slw//2))
        
            else:
                gelb_x.append(gelb_x[i])
                
            cv2.rectangle(yellow_lines_slw,(gelb_x[i+1]-breite_slw//2,y+höhe_slw),(gelb_x[i+1]+breite_slw//2,y),(255,255,0),2)
            
            
            #WEIS:
            slw_weis=white_lines[y:y+höhe_slw, weis_x[i]-breite_slw//2:weis_x[i]+breite_slw//2]

            slw_weis_his = np.sum(slw_weis,axis=0)

            try:
                white_slw_pixel_count = np.argmax(slw_weis_his)
            except:
                white_slw_pixel_count = 0

            white_weight=white_weight+white_slw_pixel_count

            if white_slw_pixel_count>1:
            
                slw_weis_x =np.argmax(slw_weis_his)
                weis_x.append((slw_weis_x+weis_x[i]-breite_slw//2))
            
            else:
                weis_x.append(weis_x[i])
                
            cv2.rectangle(white_lines_slw,(weis_x[i+1]-breite_slw//2,y+höhe_slw),(weis_x[i+1]+breite_slw//2,y),(255,255,0),2)

            
            i= i+1


        #Startpunkte für nächsten aufruf aktualisieren
        self.last_yellow_startpoint=gelb_x[0]
        self.last_white_startpoint=weis_x[0]




        #Publishen

        msg = Float32MultiArray()
        msg.data=[white_weight]+weis_x+[yellow_weight]+gelb_x
        self.pub.publish(msg)


        cv2.imshow("yellow + slws", yellow_lines_slw)

        #print("gelb_x: ", gelb_x)

        cv2.imshow("white + slws", white_lines_slw)

        cv2.waitKey(1)


        time = rospy.get_time()
        dt = time - last_time1
        #self.last_time = time
        #print("ldsn time: ", dt)


  

if __name__ == '__main__':
    # create the node
    node = LineDetectionSlwNode(node_name='line_detection_slw_node')
    # keep spinning
    rospy.spin()