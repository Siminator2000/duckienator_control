#!/usr/bin/env python3

import os
import rospy #type: ignore
import numpy as np
from duckietown.dtros import DTROS, NodeType #type: ignore
from duckietown_msgs.msg import WheelsCmdStamped #type: ignore
from duckietown_msgs.msg import Twist2DStamped #type: ignore

from duckietown_msgs.msg import WheelEncoderStamped #type: ignore

from std_msgs.msg import String #type: ignore

import cv2
from cv_bridge import CvBridge #type: ignore
from sensor_msgs.msg import CompressedImage #type: ignore
from std_msgs.msg import Bool #type: ignore



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

        #Werte zum publishen mit 10Hz zwischenspeichern
        self.new_pub=False
        self.v_stell_links=0.0
        self.v_stell_rechts=0.0



        #Die Konstanten sollten stimmen

        self.umfang_links=2*0.033*np.pi    #in m
        self.umfang_rechts=2*0.033*np.pi   #in m

        self.aufloesung=135 #Ticks pro Umdrehung

        self.abstand_raeder=0.1 #in m

        #TODO Werte herausfinden

        self.v_gerade=0.5
        self.v_rechts=0.4
        self.v_links=0.7


        #Werte überprüft
        self.radius_kurve_rechts=0.105   #in m
        self.radius_kurve_links=0.34     #in m

            #in m
        self.strecke_soll_zurueckgelegt_kurve_links_aussen=0.61261
        self.strecke_soll_zurueckgelegt_kurve_links_innen=0.45553
        self.strecke_soll_zurueckgelegt_kurve_rechts_aussen=0.24347
        self.strecke_soll_zurueckgelegt_kurve_rechts_innen=0.08639
        self.strecke_soll_zurueckgelegt_gerade=0.445




        #für pre_intersection

        self.image_msg = None
        self._bridge = CvBridge()
        self.y_start_last = None

        self.b_soll=430 #Abstand in Pixeln von Auto zur roten Linie
        self.phi_soll=0 #Winkel zur roten Linie, also eigenlich 0

            #Noch justierbar
        #TODO
        self.init_red_pixel_threshold = 1
        self.weight_threshhold= 1

        #für beide pre intersection pid regler
        self.v_kp=0.002
        self.v_ki=0.001
        self.v_kd=0.00005

        self.omega_kp=5
        self.omega_ki=2
        self.omega_kd=0.5



        self.b_i_err=0
        self.b_err_last=0

        self.phi_i_err=0
        self.phi_err_last=0

        self.v_max_pre=1
        self.v_min_pre=-0.2


        self.phi_tol=0.01
        self.b_tol=10

        print("INIT: my_intersection_control_node!")





    def callback_left(self, msg):

        self._ticks_left=msg.data

    def callback_right(self, msg):

        self._ticks_right=msg.data

    def callback_image(self, msg):
        
        self.image_msg = msg

    def pre_intersection(self):

        self.new_pub=True

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
            """
            #Kontrast erhöhen
            r_image, g_image, b_image = cv2.split(img_trans)

            r_image_eq = cv2.equalizeHist(r_image)
            g_image_eq = cv2.equalizeHist(g_image)
            b_image_eq = cv2.equalizeHist(b_image)

            img_trans = cv2.merge((r_image_eq, g_image_eq, b_image_eq))
            """

            #Auflösung niedriger machen oder Blur

            
            #Gradienten/Kantenerkennung

            imggray = cv2.cvtColor(img_trans, cv2.COLOR_BGR2GRAY)
            sigma = 3 
            img_gray_blur = cv2.GaussianBlur(imggray,(0,0), sigma)
            img_blur = cv2.GaussianBlur(img_trans,(0,0), sigma)
            cv2.imshow("gray", img_gray_blur)

            """
            #Kontrast des grauen bildes erhöhen
            imggray_kon = cv2.equalizeHist(imggray)
            img_gray_blur_kon = cv2.GaussianBlur(imggray_kon,(0,0), sigma)
            cv2.imshow("gray kon", img_gray_blur_kon)
            """
            sobelx = cv2.Sobel(img_gray_blur,cv2.CV_64F,1,0)
            sobely = cv2.Sobel(img_gray_blur,cv2.CV_64F,0,1)

            
            Gdir = cv2.phase(np.array(sobelx, np.float32), np.array(sobely, dtype=np.float32), angleInDegrees=True)

            Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)
            tolerance_angle = 45
            mask_phase = np.where(
                (sobely > 0) & # positive Richtung (dunkel -> hell)
                ((Gdir >= 90 - tolerance_angle) & (Gdir <= 90 + tolerance_angle)), 
                1, 0).astype(np.uint8)
            
            #erkannte kanten verstärken
            kernel = np.ones((3,3),np.uint8)
            mask_phase = cv2.dilate(mask_phase, kernel, iterations = 1)

            threshold = 1
            mask_mag = (Gmag > threshold)
            #cv2.imshow("mag",Gmag*mask_mag)



            """
            #Neue kantenerkennung
            r_channel, g_channel, b_channel = cv2.split(img_trans)

            # Kantenerkennung auf dem Rot-Kanal
            edges_r = cv2.Canny(r_channel, 50, 150)

            cv2.imshow('Kanten', edges_r)
            """










            #Umwanden in HSV für Farbfiler
            image_hsv = cv2.cvtColor(img_trans, cv2.COLOR_BGR2HSV)

            #Farbfilterung

            
            #TODO Werte wählen
            #Schwarz wird rausgefiltert. Nur rot ist schlecht erkennbar
            red_lower_hsv = np.array([0, 0, 188])
            red_upper_hsv = np.array([179, 172, 255])


            mask_red = cv2.inRange(image_hsv, red_lower_hsv, red_upper_hsv)
            

            # Extrahiere die Rot-, Grün- und Blaukanäle
            B, G, R = cv2.split(img_trans)

            # Erstelle eine leere Maske für rote Bereiche
            red_mask_rgb = np.zeros_like(R)

            # Definiere Schwellenwerte für Rot. Beispiel: Rot muss größer als 150 sein und Grün/Blau müssen kleiner sein
            red_threshold = 140
            non_red_threshold = 120

            # Erstelle die Maske, die anzeigt, wo das Bild rot ist
            red_mask_rgb[(R > red_threshold) & (G < non_red_threshold) & (B < non_red_threshold)] = 255


            # Definiere die Anzahl der oberen Pixel, die du behalten möchtest
            n_top_pixels = 20

            # Erstelle einen Kernel für die morphologische Operation
            kernel = np.ones((n_top_pixels, red_mask_rgb.shape[1]), dtype=np.uint8)

            # Wende die morphologische Operation an, um nur die oberen Teile der Maske zu behalten
            red_mask_rgb = cv2.morphologyEx(red_mask_rgb, cv2.MORPH_ERODE, kernel)


            cv2.imshow("red rgb", red_mask_rgb)












            #probieren andere kantenerkennung
            trysobelx = cv2.Sobel(red_mask_rgb,cv2.CV_64F,1,0)
            trysobely = cv2.Sobel(red_mask_rgb,cv2.CV_64F,0,1)
            tryGdir = cv2.phase(np.array(trysobelx, np.float32), np.array(trysobely, dtype=np.float32), angleInDegrees=True)
            tryGmag = np.sqrt(trysobelx*trysobelx + trysobely*trysobely)
            tolerance_angle = 45
            trymask_phase = np.where(
                (trysobely > 0) & # positive Richtung (dunkel -> hell)
                ((tryGdir >= 90 - tolerance_angle) & (tryGdir <= 90 + tolerance_angle)), 
                1, 0).astype(np.uint8)
            threshold = 10
            trymask_mag = (tryGmag > threshold)
            #cv2.imshow("mag",Gmag*mask_mag)


            tryred_lines = trymask_phase*red_mask_rgb


            # Definiere einen vertikalen Kernel
            kernel = np.ones((20, 1), np.uint8)  # 20x1 Kernel (20 Pixel in y-Richtung und 1 Pixel in x-Richtung)

            # Wende Dilation mit dem definierten Kernel an
            tryred_lines = cv2.dilate(tryred_lines, kernel, iterations=1)

    
            cv2.imshow("tryLines", tryred_lines)
            
            


            """
            #Rotes bild
            red = image_hsv.copy()
            
            red[:, :, 0] = 0
            red[:, :, 1] = 0

            

            cv2.imshow("red",red)
            """

            mask_sobelx_pos = (sobelx > 0)
            mask_sobelx_neg = (sobelx < 0)
            mask_sobely_pos = (sobely > 0)
            mask_sobely_neg = (sobely < 0)



            #TODO Hier noch schauen, welche sobel masks korrekt sind
            #mask_edge = mask_mag *mask_phase

          

            cv2.waitKey(1)

            

            #cv2.imshow("mask_edge", mask_edge)

            
            #red_line = mask_edge * mask_red
    
            #cv2.imshow("red_line", red_line)
            #cv2.waitKey(1)
            #cv2.imshow("Lines", mask_edge)
            cv2.imshow("Transformed Image", img_trans)


            red_line=red_mask_rgb




            #Hier kommt dann das Sliding Window für die rote Linie

            #Erst einmal Startpunkt auswählen


            red_his = np.sum(red_line[:,:],axis=1)
            

            pixel_count = np.sum(red_his)
            

            y_start=None

            try:
                y_start = np.argmax(red_his)
                
            except:
                y_start=None
                print("Except y_start!")

            
            #Schauen ob neuer Y-Startpunkt weiter am auto liegt, als der letze und genug rote pixel erkannt wurden, sonst alten y wert nehmen 

            if (y_start is None) or ((self.y_start_last is not None) and (y_start < self.y_start_last) and (pixel_count < self.init_red_pixel_threshold)):
                y_start = self.y_start_last
            


            #print("Startpunkt: ", y_start)

            

            höhe_slw=80
            höhe_slw_init=80
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
                red_y_weight.append(0.0)

            
            cv2.rectangle(red_line_slw,(x,y_start-höhe_slw_init),(x+breite_slw,y_start+höhe_slw_init),(255,255,0),2)

            #cv2.imshow("red + init Window", red_line_slw)
            
            #Hier jetzt alle anderen SLW platzieren

            i=0

            while x>0:

                x= x - breite_slw

                slw_red=red_line[red_y[i]-höhe_slw//2:red_y[i]+höhe_slw//2, x:x+breite_slw]

                slw_red_his = np.sum(slw_red,axis=1)

                try:
                    red_slw_pixel_count = np.sum(slw_red_his)
                except:
                    red_slw_pixel_count = 0


                if red_slw_pixel_count>self.weight_threshhold: #DONE, aber Konstante kann auch verschieden sein, zu self.weight_threshold (Hier noch threshold einfügen maybe self.weight_threshold)
                    #print("neuen y wert")
                    slw_red_y = np.argmax(slw_red_his)
                    red_y.append((slw_red_y+red_y[i]-höhe_slw//2))
                    red_y_weight.append(red_slw_pixel_count)
            
                else:
                    #print("alten y-wert")
                    red_y.append(red_y[i])
                    red_y_weight.append(0.0)
                #print(red_y[i])
                i=i+1 
                cv2.rectangle(red_line_slw, (x, red_y[i] - höhe_slw//2), (x + breite_slw, red_y[i] + höhe_slw//2), (255, 255, 0), 2)

                
                    

            cv2.imshow("red + slws", red_line_slw)
            
            #Dann eine Gerade durchlegen mit punkten bei denen weight > threshhold ist

            x_array = np.arange(0, breite, breite_slw)

            x_array = x_array+breite_slw//2

            #Das ganze Koordinatensystem um 640/2 nach links verschieben, damit der Mittelpunkt der Gerade in der Mitte des Bilds ist.
            #Somit beeinflusst omega nicht b, sondern nur a und es kann v und omega seperat geregelt werden

            x_array = x_array - breite//2
            #print(x_array)
            red_y_weight = np.array(red_y_weight)
            red_y = np.array(red_y)

            indices_to_remove = np.where(red_y_weight == 0.0)[0]

            # Entferne die Werte an diesen Indizes in red_y und x_array
            red_y = np.delete(red_y, indices_to_remove)
            x_array = np.delete(x_array, indices_to_remove)


            #print("Länge: ", len(red_y))
            
            if len(red_y)>=2:
                a_ist, b_ist = np.polyfit(x_array, red_y, 1) #Polynom 1. Grades wird duch die Punkte gelegt --> y=ax+b
            else:
                a_ist = 0.0
                b_ist = 0.0
            print("koeff: ", a_ist, b_ist)


            #Linie plotten
            left_point = breite_slw//2, a_ist*-300+b_ist
            right_point = breite- breite_slw//2, a_ist*300+b_ist

            cv2.line(red_line_slw, left_point, right_point, (0, 0, 255), 3)
            cv2.line(red_line_slw, (breite_slw//2, self.b_soll), (breite-breite_slw//2, self.b_soll), (255, 0, 0), 2)
            
            cv2.imshow("Stop Line", red_line_slw)

            phi_ist = np.arctan(a_ist)



            #Hier wird überprüft, ob Positionierung passt. Wenn ja, dann fertig und aus der while schleife raus 
            # --> pre_intersection methode wird beendet

            if (abs(phi_ist - self.phi_soll) <= self.phi_tol) and (abs(b_ist - self.b_soll) <= self.b_tol):

                message = WheelsCmdStamped(vel_left=0.0, vel_right=0.0)
                self.pub.publish(message)

                self.pre_intersection_ready=True
                self.new_pub=False
                print("Pre-Intersection Ready!!!")
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

            v_stell = b_err * self.v_kp #+ self.b_i_err * self.v_ki + b_d_err * self.v_kd

            #Werte begrenzen
            if v_stell < self.v_min_pre:
                v_stell = self.v_min_pre
            elif v_stell > self.v_max_pre:
                v_stell = self.v_max_pre
            

            self.b_err_last = b_err

            #omega(a) regeln (Ob a oder phi benutzen noch schauen, aber da linearer Zusammenhang wahrsacheinlich egal)

            phi_err = phi_ist - self.phi_soll
            self.phi_i_err = self.phi_i_err + dt*phi_err
            phi_d_err = (phi_err - self.phi_err_last)/dt

            omega_stell = phi_err * self.omega_kp #+ self.phi_i_err * self.omega_ki + phi_d_err * self.omega_kd

            self.phi_err_last = phi_err

            #Message publishen
            
            v_left = v_stell - (omega_stell * self.abstand_raeder) / 2
            v_right = v_stell + (omega_stell * self.abstand_raeder) / 2

            print("v_left", v_left)
            print("v_right", v_right)

            self.v_stell_links=v_left
            self.v_stell_rechts=v_right

            """
            #Das opublishen wird in einem seperaten teil gemacht, damit die bilderkennung mit höherer frequenz laufen kann 
            # und trotzdem nur mit 10 hz die geschwindligkeit gepublished wird
            message = WheelsCmdStamped(vel_left=v_left, vel_right=v_right)
            self.pub.publish(message)
            rate = rospy.Rate(10)
            rate.sleep()
            """


            




    def callback(self, msg):
        """
        #Hier wird für Line Following eine msg gepublished, damit es Line Following stoppt
        msg_lf = Bool()
        msg_lf.data = False
        self.pub_msg.publish(msg_lf)
        print("Line Following wird beendet!")
        """

        #Hier in dieser methode wird alles für die Positionerung gemacht
        self.pre_intersection()

        #self.new_pub = True
        if msg.data == "g":

            v_l_soll=self.v_gerade
            v_r_soll=self.v_gerade

            strecke_soll_zurueckgelegt_links = self.strecke_soll_zurueckgelegt_gerade
            strecke_soll_zurueckgelegt_rechts = self.strecke_soll_zurueckgelegt_gerade


        elif msg.data == "r":

            v_l_soll = self.v_rechts * (self.radius_kurve_rechts + self.abstand_raeder / 2) / self.radius_kurve_rechts #0.73810m/s
            v_r_soll = self.v_rechts * (self.radius_kurve_rechts - self.abstand_raeder / 2) / self.radius_kurve_rechts #0.26190m/s

            strecke_soll_zurueckgelegt_links = self.strecke_soll_zurueckgelegt_kurve_rechts_aussen
            strecke_soll_zurueckgelegt_rechts = self.strecke_soll_zurueckgelegt_kurve_rechts_innen

        elif msg.data == "l":

            v_l_soll = self.v_links * (self.radius_kurve_links - self.abstand_raeder / 2) / self.radius_kurve_links  #0.42647m/s
            v_r_soll = self.v_links * (self.radius_kurve_links + self.abstand_raeder / 2) / self.radius_kurve_links  #0.57353m/s

            strecke_soll_zurueckgelegt_links = self.strecke_soll_zurueckgelegt_kurve_links_innen
            strecke_soll_zurueckgelegt_rechts = self.strecke_soll_zurueckgelegt_kurve_links_aussen
            

        else:
            print("Error --> Keine Richtung angegeben")

        
        

        start_links = self._ticks_left/self.aufloesung*self.umfang_links
        start_rechts = self._ticks_right/self.aufloesung*self.umfang_rechts



        #print("Start links, Start_rechts: ", start_links, start_rechts)
        #passt
        #print("v, omega, links, rechts: ", v_stell, omega_soll, strecke_soll_zurueckgelegt_links, strecke_soll_zurueckgelegt_rechts)

        #print("Vor While in pid_intersection_controll_node")



        rate = rospy.Rate(10)

        while self.pre_intersection_ready:



            strecke_links=((self._ticks_left/self.aufloesung)*self.umfang_links)-start_links
            strecke_rechts=((self._ticks_right/self.aufloesung)*self.umfang_rechts)-start_rechts

            print("Strecke rechts: ", strecke_rechts, strecke_soll_zurueckgelegt_rechts)
            print("Strecke links:  ", strecke_links, strecke_soll_zurueckgelegt_links)

     
            if (strecke_links < strecke_soll_zurueckgelegt_links) and (strecke_rechts < strecke_soll_zurueckgelegt_rechts):
                
                message = WheelsCmdStamped(vel_left=v_l_soll, vel_right=v_r_soll)
                
                self.pub.publish(message)

                

            #Hier kann auch 'oder' hin, muss noch getestet werden, was sinnvoller ist
            #Außerdem kann hier auch noch statt >= ein gewisser bereich angegeben werden, sodass, wenn es überschwingt, wieder zurück geregelt wird
            else:
                print("Intersection erfolgreich überwunden!!")
                message1 = WheelsCmdStamped(vel_left=0.0, vel_right=0.0)
                self.pub.publish(message1)
                #self.new_pub = False
                break
            
            rate.sleep()

        #Hier ausserhalb der while schleife kann nach dem erfolgreichen intersection controll eine msg gepublished werden, damit wieder line following erfolgen kann
   

        #msg_lf1 = Bool()
        #msg_lf1.data = False
        #self.pub_msg.publish(msg_lf1)
        #print("Nach While in pid_intersection_controll_node")


    
    def run(self):
        rate = rospy.Rate(10)  # Setzt die Rate auf 10 Hz
        stop_published = False  # Variable, um zu verfolgen, ob 0-Geschwindigkeiten bereits veröffentlicht wurden
        
        while True:
            while self.new_pub:
                # Publiziert die aktuellen Geschwindigkeiten
                message = WheelsCmdStamped(vel_left=self.v_stell_links, vel_right=self.v_stell_rechts)
                self.pub.publish(message)
                stop_published = False  # Setzt zurück, da wir in der Schleife aktiv publizieren
                
                # Hält die Rate der inneren Schleife bei 10 Hz
                rate.sleep()
            
            # Nur einmal veröffentlichen, wenn die innere Schleife verlassen wird
            if not stop_published:
                stop_message = WheelsCmdStamped(vel_left=0, vel_right=0)
                self.pub.publish(stop_message)
                stop_published = True  # Setzt die Variable, um erneutes Publizieren zu verhindern
            
            # Hält die Rate auch im äußeren Loop bei 10 Hz
            rate.sleep()
            


if __name__ == '__main__':
    # create the node
    node = IntersectionControllNode(node_name='intersection_controll_node')
    node.run()
    # keep spinning
    rospy.spin()
       
