#!/usr/bin/env python3

import os
import rospy
import numpy as np
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool


import cv2
from cv_bridge import CvBridge
#from augmented_reality_basics import Augmenter

#from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

class MyImageProcessNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyImageProcessNode, self).__init__(node_name=node_name, node_type=NodeType.VISUALIZATION)

        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"

        self._line_following_topic = f"/{self._vehicle_name}/pid_intersection_controll_node/line_following"
    

        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
    

        # construct subscriber
        self.sub_image = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback_image)
        
        self.sub_msg = rospy.Subscriber(self._line_following_topic, Bool, self.callback_msg)

        #Test
        #aug = Augmenter()

        self.last_time = rospy.get_time()


        #von line detection node
        #self._line_images_topic = f"/{self._vehicle_name}/image_process_node/line_images"
        self._line_arrays_topic = f"/{self._vehicle_name}/line_detection_slw_node/line_arrays"
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()

        # construct subscriber
        #self.sub = rospy.Subscriber(self._line_images_topic, CompressedImage, self.callback)

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

        #Speicher des Bildes

        self.image_msg = None

        #Line following
        self.line_following=True

        print("INIT: image_process_node")


    def callback_image(self, msg):

        #print("image_process_node: Bild erhalten!")
        
        self.image_msg = msg


    def callback_msg(self, msg):

        print("image_process_node: lf_msg erhalten!")

        #while schleife soll auch nur laufen, wenn ein neues bild zur Verfügung steht und Line following noch aktiv ist

        #Kann noch gemacht werden, aber while schleife braucht normal sowieso länger, als die neuen Bilder der Kamera (30FPS)
        
        while msg.data:
            
            # convert JPEG bytes to CV image
            last_time1 = rospy.get_time()
            
            image = self._bridge.compressed_imgmsg_to_cv2(self.image_msg)

            #cv2.imshow("image", image)

            höhe, breite, kanäle = image.shape

            #Ground_Projection maybe mit BirdEye probieren
            #o_l = (65,240)
            #o_r = (465,245)
            #u_l = (-300,400)
            #u_r = (535,400)

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

            #Auflösung niedriger machen oder Blur
            sigma = 6 
            img_trans_blur = cv2.GaussianBlur(img_trans,(0,0), sigma)

            #Gradienten/Kantenerkennung
            imggray = cv2.cvtColor(img_trans_blur, cv2.COLOR_BGR2GRAY)

            sobelx = cv2.Sobel(imggray,cv2.CV_64F,1,0)
            sobely = cv2.Sobel(imggray,cv2.CV_64F,0,1)

            Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)
            Gdir = cv2.phase(np.array(sobelx, np.float32), np.array(sobely, dtype=np.float32), angleInDegrees=True)

            threshold = 10
            mask_mag = (Gmag > threshold)
            #cv2.imshow("mag",Gmag*mask_mag)

            #Umwanden in HSV für Farbfiler
            image_hsv = cv2.cvtColor(img_trans, cv2.COLOR_BGR2HSV)

            #Farbfilterung
            white_lower_hsv = np.array([0, 0, 142])
            white_upper_hsv = np.array([179, 73, 255])
            yellow_lower_hsv = np.array([22, 75, 111])
            yellow_upper_hsv = np.array([29, 255, 244])

            mask_white = cv2.inRange(image_hsv, white_lower_hsv, white_upper_hsv)
            mask_yellow = cv2.inRange(image_hsv, yellow_lower_hsv, yellow_upper_hsv)


            #Linke und rechte Maske

            mask_left=np.ones(sobelx.shape)
            #mask_left[:,(breite//2):]=0

            mask_right=np.ones(sobelx.shape)
            #mask_right[:,:(breite//2)]=0

            #cv2.imshow("yellow+right", mask_yellow*mask_right)
            #cv2.imshow("white+left", mask_white*mask_left)

            mask_sobelx_pos = (sobelx > 0)
            mask_sobelx_neg = (sobelx < 0)
            mask_sobely_pos = (sobely > 0)
            mask_sobely_neg = (sobely < 0)


            #mask_sobely macht oft löcher
            #cv2.imshow("mask_sobely_neg", mask_sobelx_neg*mask_right)
            

            mask_left_edge = mask_left * mask_mag  * mask_sobelx_neg #* mask_sobely_pos
            mask_right_edge = mask_right * mask_mag * mask_sobelx_pos #* mask_sobely_neg 

            #cv2.imshow("mask_left_edges", mask_left_edge)
            #cv2.imshow("mask_right_edges", mask_right_edge)

            #cv2.imshow("mask_left_edge", mask_left_edge)
            #cv2.imshow("mask_right_edge", mask_right_edge)
            
            white_lines = mask_white * mask_right_edge
            yellow_lines = mask_yellow * mask_left_edge


            #cv2.imshow("yellow_lines", yellow_lines)
            #cv2.waitKey(1)

            #cv2.imshow("white_lines", white_lines)
            #cv2.imshow("yellow_lines", yellow_lines)
            cv2.imshow("Transformed Image", img_trans)
            #cv2.imshow("weiss", mask_white)

            #time_bevore = rospy.get_time()


            """
            #Alles fürs Publishen der line_images_msg
            
            white_lines_flat = white_lines.flatten()
            yellow_lines_flat = yellow_lines.flatten()

            
            msg = Float32MultiArray()

            
            dim_white = MultiArrayDimension()
            dim_white.label = "white_lines"
            dim_white.size = white_lines.size
            dim_white.stride = white_lines.shape[0] * white_lines.shape[1]

            dim_yellow = MultiArrayDimension()
            dim_yellow.label = "yellow_lines"
            dim_yellow.size = yellow_lines.size
            dim_yellow.stride = yellow_lines.shape[0] * yellow_lines.shape[1]

            # Füge die Dimensionen zum Layout hinzu
            msg.layout = MultiArrayLayout()
            msg.layout.dim = [dim_white, dim_yellow]
            
            # Füge die abgeflachten Daten zur Nachricht hinzu
            data_white=white_lines_flat.tolist()

            

            data_yellow=yellow_lines_flat.tolist()

            
            msg.data = data_white + data_yellow
            
            # Publish die Nachricht
            self.pub.publish(msg)
            

            #Publishen mit sensor_msg/CompressedImage
            #rospy.loginfo("Yellow lines min: %s, max: %s", np.min(yellow_lines), np.max(yellow_lines))
            #rospy.loginfo("White lines min: %s, max: %s", np.min(white_lines), np.max(white_lines))


            #zu einer matrix zusammenfügen
            gelbUndWeis= np.hstack((yellow_lines, white_lines))
            rospy.loginfo("Combined image min: %s, max: %s", np.min(gelbUndWeis), np.max(gelbUndWeis))

            cv2.imshow("gelbundweis", gelbUndWeis)
            #float werte in uint8 umwandeln
            uint8_matrix = (gelbUndWeis).astype(np.uint8)
            cv2.imshow("Published Image", uint8_matrix)

            compressed_image = self._bridge.cv2_to_compressed_imgmsg(uint8_matrix, dst_format='png')

            self.pub.publish(compressed_image)

            dt = rospy.get_time()-time_bevore
            #print("Publish Delay: ", dt)
            #rate.sleep()

            #print("line images published")
            #time = rospy.get_time()
            #dt = time - last_time1
            #self.last_time = time
            #print("ipn time: ", dt)
            """





            #von line detection node
            #Setzen der Startpunkte mithilfe der vorherigen Startpunkte, wenn sinnvoll, sonst neu finden

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

            #anzahl der pixel in den einzelnen SLW
            gelb_x_weight=[]        
            weis_x_weight=[]

            #anzahl der pixel in alles slws zusammen
            yellow_weight=0
            white_weight=0



            y=höhe-höhe_slw

            

            #GELB: initiales SLW und Histogram + Hochpunkt in jeweilige x-Arrays einfügen
            init_slw_gelb=yellow_lines[y:y+höhe_slw,yellow_line_startpoint-breite_slw_init//2:yellow_line_startpoint+breite_slw_init//2]

            init_slw_gelb_his = np.sum(init_slw_gelb,axis=0)

            #Summe für ausschlagkräftigkeit des startpunktes
            yellow_pixel_count = np.sum(init_slw_gelb_his)

            self.last_yellow_startpoint_weight = yellow_pixel_count



            try:
                yellow_pixel_count = np.sum(init_slw_gelb_his)
            except:
                yellow_pixel_count = 0

            
            if yellow_pixel_count>10:
                
                init_slw_gelb_x =np.argmax(init_slw_gelb_his)
                gelb_x.append((init_slw_gelb_x+yellow_line_startpoint-breite_slw_init//2))
                gelb_x_weight.append(yellow_pixel_count)
                yellow_weight=yellow_weight+yellow_pixel_count
            
            else:
                gelb_x.append(yellow_line_startpoint)
                gelb_x_weight.append(0.0)

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
                white_pixel_count = np.sum(init_slw_weis_his)
            except:
                white_pixel_count = 0

            

            if white_pixel_count>10:
                
                init_slw_weis_x =np.argmax(init_slw_weis_his)
                weis_x.append((init_slw_weis_x+white_line_startpoint-breite_slw_init//2))
                weis_x_weight.append(white_pixel_count)
                white_weight=white_weight+white_pixel_count
            
            else:
                weis_x.append(white_line_startpoint)
                weis_x_weight.append(0.0)

            cv2.rectangle(white_lines_slw,(white_line_startpoint-breite_slw_init//2,y+höhe_slw),(white_line_startpoint+breite_slw_init//2,y),(255,255,0),2)

            #cv2.imshow("white + init Window", white_lines_slw)
            
            


            #print(f"gelb_x_array {gelb_x[0]}")
            #print(f"gelbe max. pixelanzahl von init window {yellow_pixel_count}")
            #print(f"weis_x_array {weis_x[0]}")
            #print(f"weise max. pixelanzahl von init window {np.max(init_slw_weis_his)}")


            #restliche linepoints finden
            #yellow_weight und white_weight sind summen der gefundenen Pixel, um dies bei der Berechnung des Straßen-Mittelpunktes zu gewichten



            i=0

            while y>0:

                y= y-höhe_slw

                #GELB:
                slw_gelb=yellow_lines[y:y+höhe_slw, gelb_x[i]-breite_slw//2:gelb_x[i]+breite_slw//2]

                slw_gelb_his = np.sum(slw_gelb,axis=0)

                try:
                    yellow_slw_pixel_count = np.sum(slw_gelb_his) #Hier hab ich ohne zu testen np.argmax zu np.sum gemacht, müsste aber stimmen
                except:
                    yellow_slw_pixel_count = 0

                

                if yellow_slw_pixel_count>1:  #TODO Hier noch anderen threshold auswählen
                
                    slw_gelb_x = np.argmax(slw_gelb_his)
                    gelb_x.append((slw_gelb_x+gelb_x[i]-breite_slw//2))
                    gelb_x_weight.append(yellow_slw_pixel_count)
                    yellow_weight=yellow_weight+yellow_slw_pixel_count
            
                else:
                    gelb_x.append(gelb_x[i])
                    gelb_x_weight.append(0.0)
                    
                cv2.rectangle(yellow_lines_slw,(gelb_x[i+1]-breite_slw//2,y+höhe_slw),(gelb_x[i+1]+breite_slw//2,y),(255,255,0),2)
                
                
                #WEIS:
                slw_weis=white_lines[y:y+höhe_slw, weis_x[i]-breite_slw//2:weis_x[i]+breite_slw//2]

                slw_weis_his = np.sum(slw_weis,axis=0)

                try:
                    white_slw_pixel_count = np.sum(slw_weis_his)#Hier hab ich ohne zu testen np.max zu np.sum gemacht, müsste aber stimmen
                except:
                    white_slw_pixel_count = 0

                

                if white_slw_pixel_count>1: #TODO Hier noch anderen threshold auswählen
                
                    slw_weis_x =np.argmax(slw_weis_his)
                    weis_x.append((slw_weis_x+weis_x[i]-breite_slw//2))
                    weis_x_weight.append(white_slw_pixel_count)
                    white_weight=white_weight+white_slw_pixel_count
                
                else:
                    weis_x.append(weis_x[i])
                    weis_x_weight.append(0.0)
                    
                cv2.rectangle(white_lines_slw,(weis_x[i+1]-breite_slw//2,y+höhe_slw),(weis_x[i+1]+breite_slw//2,y),(255,255,0),2)

                
                i= i+1


            #Startpunkte für nächsten aufruf aktualisieren
            self.last_yellow_startpoint=gelb_x[0]
            self.last_white_startpoint=weis_x[0]

            #Hier jetzt noch die x² funktion duchlegen und diese + weights publishen


            #für gelb

            y_werte_gelb = np.arange(höhe - höhe_slw / 2, höhe_slw / 2 - höhe_slw, -höhe_slw) #Erstellt array das von 480 bis 0 in gleichen abständen punkte defineirt

            indices_to_remove_gelb = np.where(gelb_x_weight == 0.0)[0]

            # Entferne die Werte an diesen Indizes in red_y und x_array
            gelb_x = np.delete(gelb_x, indices_to_remove_gelb)
            y_werte_gelb = np.delete(y_werte_gelb, indices_to_remove_gelb)

            if gelb_x.size()>=3:
            #Achtung x und y vertauscht, damit man einen y wert hineingeben kann und einen x wert raus bekommt
                a_gelb, b_gelb, c_gelb = np.polyfit(y_werte_gelb, gelb_x, 2) # a*x² + b*x + c

            else:
                a_gelb=None
                b_gelb=None
                c_gelb=None




            #für weis

            y_werte_weis = np.arange(höhe - höhe_slw / 2, höhe_slw / 2 - höhe_slw, -höhe_slw) #Erstellt array das von 480 bis 0 in gleichen abständen punkte defineirt

            indices_to_remove_weis = np.where(weis_x_weight == 0.0)[0]

            # Entferne die Werte an diesen Indizes in red_y und x_array
            weis_x = np.delete(weis_x, indices_to_remove_weis)
            y_werte_weis = np.delete(y_werte_weis, indices_to_remove_weis)

            if weis_x.size()>=3:
            #Achtung x und y vertauscht, damit man einen y wert hineingeben kann und einen x wert raus bekommt
                a_weis, b_weis, c_weis = np.polyfit(y_werte_weis, weis_x, 2) # a*x² + b*x + c

            else:
                a_weis=None
                b_weis=None
                c_weis=None

            #Publishen

            msg = Float32MultiArray()
            msg.data=[yellow_weight, a_gelb, b_gelb, c_gelb, white_weight, a_weis, b_weis, c_weis]
            self.pub.publish(msg)
            print("image_process_node published!")

            cv2.imshow("yellow + slws", yellow_lines_slw)

            #print("gelb_x: ", gelb_x)

            cv2.imshow("white + slws", white_lines_slw)


            #time = rospy.get_time()
            #dt = time - last_time1
            #self.last_time = time
            #print("ldsn time: ", dt)
    

            cv2.waitKey(1)





if __name__ == '__main__':
    # create the node
    node = MyImageProcessNode(node_name='my_image_process_node')
    # keep spinning
    rospy.spin()

    