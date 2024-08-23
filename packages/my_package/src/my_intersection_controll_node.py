#!/usr/bin/env python3

import os
import rospy
import numpy as np
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped
from duckietown_msgs.msg import Twist2DStamped

from duckietown_msgs.msg import WheelEncoderStamped

from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool



class IntersectionControllNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(IntersectionControllNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        
        vehicle_name = os.environ['VEHICLE_NAME']
        #self._wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"

        my_wheel_cmd_topic=f"/{vehicle_name}/low_level_pid_controller/my_wheel_cmd"

        self._left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

        self._intersection_direction_topic = f"/{vehicle_name}/msg_node/direction"


        self._camera_topic = f"/{vehicle_name}/camera_node/image/compressed"


        self._line_following_topic = f"/{vehicle_name}/pid_intersection_controll_node/line_following"
        

        # temporary data storage
        self._ticks_left = None
        self._ticks_right = None



        #subscriber
        self.sub_image = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback_image)

        self._sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self._sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)

        self.sub = rospy.Subscriber(self._intersection_direction_topic, String, self.callback)



        #publisher
        self.pub = rospy.Publisher(my_wheel_cmd_topic, WheelsCmdStamped, queue_size=1)


        #Zeit für Differentation

        self.last_time=rospy.get_time()
        self.phi_last=0

        #Konstanten für Intersection
        self.pre_intersection_ready=False

        self.kp=0.1
        self.ki=0.1
        self.kd=0.01

        self.omega_i_err=0
        self.omega_err_last=0


        #Die Konstanten sollten stimmen

        self.umfang_links=2*0.033*np.pi    #in m
        self.umfang_rechts=2*0.033*np.pi   #in m

        self.aufloesung=135 #Ticks pro Umdrehung

        self.abstand_raeder=0.1 #in m

        #TODO Werte herausfinden

        self.v_gerade=0.7
        self.v_rechts=0.5       
        self.v_links=0.5


        #Werte überprüft
        self.radius_kurve_rechts=0.105   #in m
        self.radius_kurve_links=0.34     #in m

            #in m
        self.strecke_soll_zurueckgelegt_kurve_links_aussen=0.61261
        self.strecke_soll_zurueckgelegt_kurve_links_innen=0.45553
        self.strecke_soll_zurueckgelegt_kurve_rechts_aussen=0.24347
        self.strecke_soll_zurueckgelegt_kurve_rechts_innen=0.8639
        self.strecke_soll_zurueckgelegt_gerade=0.445




        #für pre_intersection

        self.image_msg = None
        self._bridge = CvBridge()
        self.y_start_last = None

        self.b_soll=50 #Abstand in Pixeln von Auto zur roten Linie
        self.phi_soll=0 #Winkel zur roten Linie, also eigenlich 0

            #Noch justierbar
        self.init_red_pixel_threshold = 50
        self.weight_threshhold= 20

        #für beide pre intersection pid regler
        self.v_kp=5
        self.v_ki=2
        self.v_kd=0.5

        self.omega_kp=5
        self.omega_ki=2
        self.omega_kd=0.5



        self.b_i_err=0
        self.b_err_last=0

        self.phi_i_err=0
        self.phi_err_last=0

        print("INIT: my_intersection_control_node!")





    def callback_left(self, msg):

        self._ticks_left=msg.data

    def callback_right(self, msg):

        self._ticks_right=msg.data

    def callback_image(self, msg):
        
        self.image_msg = msg

    def pre_intersection(self):

        while True:

            image = self._bridge.compressed_imgmsg_to_cv2(self.image_msg)

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


            #TODO Werte wählen
            red_lower_hsv = np.array([0, 0, 0])
            red_upper_hsv = np.array([179, 255, 255])


            mask_red = cv2.inRange(image_hsv, red_lower_hsv, red_upper_hsv)


            mask_sobelx_pos = (sobelx > 0)
            mask_sobelx_neg = (sobelx < 0)
            mask_sobely_pos = (sobely > 0)
            mask_sobely_neg = (sobely < 0)



            #TODO Hier noch schauen, welche sobel masks korrekt sind
            mask_edge = mask_mag  * mask_sobelx_neg #* mask_sobely_pos
            

            #cv2.imshow("mask_edge", mask_edge)

            
            red_line = mask_red * mask_edge
    
            #cv2.imshow("red_line", red_line)
            #cv2.waitKey(1)

            cv2.imshow("Transformed Image", img_trans)



            #Hier kommt dann das Sliding Window für die rote Linie

            #Erst einmal Startpunkt auswählen

            red_his = np.sum(red_line[:,:],axis=1)
            

            pixel_count = np.sum(red_his)
            

            y_start=None

            try:
                y_start = np.argmax(red_his)
                
            except:
                y_start=None

            #Schauen ob neuer Y-Startpunkt weiter am auto liegt, als der letze und genug rote pixel erkannt wurden, sonst alten y wert nehmen 
            if (self.y_start_last != None)&(y_start>self.y_start_last)&(pixel_count>self.init_red_pixel_threshold):
                self.y_start_last=y_start

            else:
                y_start=self.y_start_last



            höhe_slw=60
            höhe_slw_init=100
            breite_slw=40 #-->16 SLW auf 640 breite gesamt

            red_line_slw=np.copy(red_line)
        
            red_y=[]
            red_y_weight=[]
        
            x=breite-breite_slw
            

            #ROT: initiales SLW und Histogram + Hochpunkt in jeweilige x-Arrays einfügen
            init_slw=red_line[y_start-höhe_slw_init//2:y_start+höhe_slw_init,x:x+breite_slw]

            init_slw_his = np.sum(init_slw,axis=1)

            #Summe für ausschlagkräftigkeit des punktes
            weight = np.sum(init_slw_his)

            
            if weight>self.weight_threshhold:
                
                init_slw_y =np.argmax(init_slw_his)
                red_y.append((init_slw_y+y_start-höhe_slw_init//2))
                red_y_weight.append(weight)
            
            else:
                red_y.append(y_start)
                red_y_weight(0.0)

            
            cv2.rectangle(red_line_slw,(x,y_start-höhe_slw_init),(x+breite_slw,y_start+höhe_slw_init),(255,255,0),2)

            #cv2.imshow("red + init Window", red_line_slw)

            #Hier jetzt alle anderen SLW platzieren

            i=0

            while x>0:

                x= x - breite_slw

                slw_red=red_line[red_y[i]-höhe_slw:red_y[i]+höhe_slw, x:x+breite_slw]

                slw_red_his = np.sum(slw_red,axis=1)

                try:
                    red_slw_pixel_count = np.sum(slw_red_his)
                except:
                    red_slw_pixel_count = 0


                if red_slw_pixel_count>self.weight_threshhold: #DONE, aber Konstante kann auch verschieden sein, zu self.weight_threshold (Hier noch threshold einfügen maybe self.weight_threshold)
                
                    slw_red_y = np.argmax(slw_red_his)
                    red_y.append((slw_red_y+red_y[i]-höhe_slw//2))
                    red_y_weight.append(red_slw_pixel_count)
            
                else:
                    red_y.append(red_y[i])
                    red_y_weight.append(0.0)
                    
                cv2.rectangle(red_line_slw,(x,y_start-höhe_slw),(x+breite_slw,y_start+höhe_slw),(255,255,0),2)
                    



            #Dann eine Gerade durchlegen mit punkten bei denen weight > threshhold ist

            x_array = np.linspace(breite-breite_slw//2, breite_slw, num=breite//breite_slw, endpoint=True, retstep=False, dtype=None, axis=0)

            #Das ganze Koordinatensystem um 640/2 nach links verschieben, damit der Mittelpunkt der Gerade in der Mitte des Bilds ist.
            #Somit beeinflusst omega nicht b, sondern nur a und es kann v und omega seperat geregelt werden

            x_array = x_array - breite//2

            indices_to_remove = np.where(red_y_weight == 0.0)[0]

            # Entferne die Werte an diesen Indizes in red_y und x_array
            red_y = np.delete(red_y, indices_to_remove)
            x_array = np.delete(x_array, indices_to_remove)

            a_ist, b_ist = np.polyfit(x_array, red_y, 1) #Polynom 1. Grades wird duch die Punkte gelegt --> y=ax+b

            phi_ist = np.arctan(a_ist)



            #Hier wird überprüft, ob Positionierung passt. Wenn ja, dann fertig und aus der while schleife raus 
            # --> pre_intersection methode wird beendet

            if(phi_ist == self.phi_soll)&(b_ist==self.b_soll):
                message = Twist2DStamped(v=0, omega=0)
                self.pub.publish(message)
                self.pre_intersection_ready=True
                break

            #Dann was überlegen, dass ich parallel zur Roten Linie mit einem bestimmten Abstand stehe

            #a muss gegen Null gehen und b auf einen bestimmten bestimmten Wert, da es der Abstand von Auto zur roten Linie ist.
            #Zwei seperate PID-Regler dafür machen

            time_now = rospy.get_time()

            dt = time_now-self.last_time

            self.last_time=time_now

            #v(b) regeln

            b_err = self.b_soll - b_ist
            self.b_i_err = self.b_i_err + dt*b_err
            b_d_err = (b_err - self.b_err_last)/dt

            v_stell = b_err * self.v_kp + self.b_i_err * self.v_ki + b_d_err * self.v_kd

            self.b_err_last = b_err

            #omega(a) regeln (Ob a oder phi benutzen noch schauen, aber da linearer Zusammenhang wahrsacheinlich egal)

            phi_err = self.phi_soll - phi_ist
            self.phi_i_err = self.phi_i_err + dt*phi_err
            phi_d_err = (phi_err - self.phi_err_last)

            omega_stell = phi_err * self.omega_kp + self.phi_i_err * self.omega_ki + phi_d_err * self.omega_kd

            self.phi_err_last = phi_err

            #Message publishen
            message = Twist2DStamped(v=v_stell, omega=omega_stell)
        
            self.pub.publish(message)





    def callback(self, msg):
        """
        #Hier wird für Line Following eine msg gepublished, damit es Line Following stoppt
        msg_lf = Bool()
        msg_lf.data = False
        self.pub_msg.publish(msg_lf)
        print("Line Following wird beendet!")
        """

        #Hier in dieser methode wird alles für die Positionerung gemacht
        #self.pre_intersection()


        if msg.data == "g":

            v_l=self.v_gerade
            v_r=self.v_gerade

            strecke_soll_zurueckgelegt_links = self.strecke_soll_zurueckgelegt_gerade
            strecke_soll_zurueckgelegt_rechts = self.strecke_soll_zurueckgelegt_gerade


        elif msg.data == "r":

            v_l = self.v_rechts * (self.radius_kurve_rechts - self.abstand_raeder / 2) / self.radius_kurve_rechts #0.26190m/s
            v_r = self.v_rechts * (self.radius_kurve_rechts + self.abstand_raeder / 2) / self.radius_kurve_rechts #0.73810m/s

            strecke_soll_zurueckgelegt_links = self.strecke_soll_zurueckgelegt_kurve_rechts_aussen
            strecke_soll_zurueckgelegt_rechts = self.strecke_soll_zurueckgelegt_kurve_rechts_innen

        elif msg.data == "l":

            v_l = self.v_links * (self.radius_kurve_links - self.abstand_raeder / 2) / self.radius_kurve_links  #0.42647m/s
            v_r = self.v_links * (self.radius_kurve_links + self.abstand_raeder / 2) / self.radius_kurve_links  #0.57353m/s

            strecke_soll_zurueckgelegt_links = self.strecke_soll_zurueckgelegt_kurve_links_innen
            strecke_soll_zurueckgelegt_rechts = self.strecke_soll_zurueckgelegt_kurve_links_aussen
            

        else:
            print("Error --> Keine Richtung angegeben")

        start_links = self._ticks_left/self.aufloesung*self.umfang_links
        start_rechts = self._ticks_right/self.aufloesung*self.umfang_rechts


        #Werte zurücksetzen
        self.phi_last=0
        self.omega_i_err=0
        self.omega_err_last=0

        #print("Start links, Start_rechts: ", start_links, start_rechts)
        #passt
        #print("v, omega, links, rechts: ", v_stell, omega_soll, strecke_soll_zurueckgelegt_links, strecke_soll_zurueckgelegt_rechts)

        #print("Vor While in pid_intersection_controll_node")

        while True: #self.pre_intersection_ready:


            
            strecke_links=((self._ticks_left/self.aufloesung)*self.umfang_links)-start_links
            strecke_rechts=((self._ticks_right/self.aufloesung)*self.umfang_rechts)-start_rechts

            #print("Strecke rechts: ", strecke_rechts, strecke_soll_zurueckgelegt_rechts)
            #print("Strecke links:  ", strecke_links, strecke_soll_zurueckgelegt_links)
            """
            delta_strecke=strecke_rechts-strecke_links

            print("Strecke delta:  ", delta_strecke)

            time=rospy.get_time()

            dt=time-self.last_time

            print("dt: ", dt)
            self.last_time=time

            phi=delta_strecke/(self.abstand_raeder/2) #winkel des autos in rad

            phi_grad = np.rad2deg(phi)
            print("phi_grad des autos", phi_grad)

            omega_ist = (phi-self.phi_last)/dt
            
            self.phi_last=phi

            omega_err=omega_soll-omega_ist

            #I-Anteil
            self.omega_i_err=self.omega_i_err+omega_err*dt

            #D-Anteil
            omega_d_err=(omega_err-self.omega_err_last)/dt

            self.omega_err_last=omega_err

            omega_stell = omega_soll + omega_err*self.kp #+ self.omega_i_err*self.ki + omega_d_err*self.kd
            print(" ")
            print("omega_ist:   ", omega_ist)
            print(" ")
            print("omega_soll:  ", omega_soll)
            print(" ")
            print("omega_p:     ", omega_err*self.kp)
            print(" ")
            print("omega_i:     ", self.omega_i_err*self.ki)
            print(" ")
            print("omega_d:     ", omega_d_err*self.kd)
            print(" ")
            print("omega_stell: ", omega_stell)
            print(" ")
            """
            #Publishen
            rate = rospy.Rate(10)
            message = WheelsCmdStamped(vel_left=v_l, vel_right=v_r)
            
            self.pub.publish(message)

            rate.sleep()


            #Hier kann auch 'oder' hin, muss noch getestet werden, was sinnvoller ist
            #Außerdem kann hier auch noch statt >= ein gewisser bereich angegeben werden, sodass, wenn es überschwingt, wieder zurück geregelt wird
            if (strecke_links >= strecke_soll_zurueckgelegt_links)|(strecke_rechts >= strecke_soll_zurueckgelegt_rechts):
                print("Intersection erfolgreich überwunden!!")
                message = WheelsCmdStamped(vel_left=0.0, vel_right=0.0)
                self.pub.publish(message)

                #message_stop = WheelsCmdStamped(vel_left=0.0, vel_right=0.0)
                #self.pub_stop.publish(message_stop)
                #self.pre_intersection_ready=False
                break

        #Hier ausserhalb der while schleife kann nach dem erfolgreichen intersection controll eine msg gepublished werden, damit wieder line following erfolgen kann

        #msg_lf1 = Bool()
        #msg_lf1.data = False
        #self.pub_msg.publish(msg_lf1)
        #print("Nach While in pid_intersection_controll_node")


if __name__ == '__main__':
    # create the node
    node = IntersectionControllNode(node_name='intersection_controll_node')
    # keep spinning
    rospy.spin()
       
