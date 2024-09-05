#!/usr/bin/env python3

import os
import rospy #type: ignore
import numpy as np
from duckietown.dtros import DTROS, NodeType #type: ignore
from std_msgs.msg import Float32MultiArray #type: ignore
from duckietown_msgs.msg import WheelsCmdStamped #type: ignore
import cv2
import math



class MyLaneFollowingNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyLaneFollowingNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']


        line_arrays_topic = f"/{vehicle_name}/line_detection_slw_node/line_arrays"

        wheel_topic=f"/{vehicle_name}/wheels_driver_node/wheels_cmd"

        # construct subscriber
        self.sub = rospy.Subscriber(line_arrays_topic, Float32MultiArray, self.callback)

        #publisher
        self.pub = rospy.Publisher(wheel_topic, WheelsCmdStamped, queue_size=1)
        

        #Konstanten
        #21cm entsprechen 354 Pixel -->
        self.factor=0.21/354 #=0.0006   Faktor um pixel in meter zu rechnen --> wird benutzt, um den radius zu berechnen

        self.strassenbreite=354  #in Pixel
        self.strassenmitte=320   #in Pixel

        self.wheel_base=0.1     #in m






        self.v_max = 0.7
        self.v_min = 0.2

        #Unteres/oberes omega
        #TODO werte anpassen
        self.omega_min=0.3
        self.omega_max=2


        #PID Regler für Straßenmittenregelung

        #TODO PID Werte anpassen
        self.kp_x=1
        self.ki_x=0.5
        self.kd_x=0.1



        self.time_last = rospy.gettime()
        self.x_err_last=0
        self.x_i_err=0

        print("INIT: my_lane_following_node")
        



    def callback(self, msg):

        #print("my_lane_following_node: main msg erhalten")

        data = np.array(msg.data)
        #print("alles", data)
        yellow_weight = data[0]
        a_gelb = data[1]
        b_gelb = data[2]
        c_gelb = data[3]
        white_weight = data[4]
        a_weis = data[5]
        b_weis = data[6]
        c_weis = data[7]

        y_0=140

        
        def f_gelb(y):
            return a_gelb*y**2 + b_gelb*y + c_gelb
        
        def f_abl_gelb(y):
            return 2*a_gelb*y + b_gelb
        
        def f_weis(y):
            return a_weis*y**2 + b_weis*y + c_weis
        
        def f_abl_weis(y):
            return 2*a_weis*y + b_weis
        
        if(a_gelb!=0.0)&(b_gelb!=0.0)&(c_gelb!=0.0):
            steigung_gelb = f_abl_gelb(y_0)
            steigung_normale_gelb=-1/steigung_gelb
            x_0_gelb=f_gelb(y_0)
        else:
            steigung_gelb = None
            steigung_normale_gelb= None
            x_0_gelb = None

        if(a_weis!=0.0)&(b_weis!=0.0)&(c_weis!=0.0):
            steigung_weis = f_abl_weis(y_0)
            steigung_normale_weis=-1/steigung_weis
            x_0_weis = f_weis(y_0)
        else:
            steigung_weis=None
            steigung_normale_weis=None
            x_0_weis = None




        #Hier den Anfahrtspunkt berechnen

        #TODO Hier auch noch einen größeren Wert wählen, damit der Mittelpunkt auch wirklich der Mittelpunkt ist, also die Drehachse
        y_0=480

        
        if(a_gelb!=0.0)&(b_gelb!=0.0)&(c_gelb!=0.0):
            steigung_gelb = f_abl_gelb(y_0)
            steigung_normale_gelb=-1/steigung_gelb
            x_0_gelb=f_gelb(y_0)
        else:
            steigung_gelb = None
            steigung_normale_gelb = None
            x_0_gelb = None

        if(a_weis!=0.0)&(b_weis!=0.0)&(c_weis!=0.0):
            steigung_weis = f_abl_weis(y_0)
            steigung_normale_weis=-1/steigung_weis
            x_0_weis = f_weis(y_0)
        else:
            steigung_weis=None
            steigung_normale_weis=None
            x_0_weis = None

        #Falls die Steigung zu groß ist, kann angenommen werden, dass sich die Mitte rechts horizontal von der gelben Linie befindet
        def mitte_gelb():
            if abs(steigung_normale_gelb) > 999:
                x_mitte_gelb=x_0_gelb+(self.strassenbreite/2)
                
            else:

                alpha_gelb=np.arctan(steigung_normale_gelb)
                if alpha_gelb<0:
                    phi_gelb=-np.pi/2-alpha_gelb
                else:
                    phi_gelb=np.pi/2-alpha_gelb

                

                delta_x_gelb=(self.strassenbreite/2)/np.cos(phi_gelb)
                #print("gelb delta x pixel: ", delta_x_gelb)

                x_mitte_gelb=x_0_gelb+abs(delta_x_gelb)

                


            return x_mitte_gelb
        
        #Falls die Steigung zu groß ist, kann angenommen werden, dass sich die Mitte links horizontal von der weisen Linie befindet
        def mitte_weis():
            if abs(steigung_normale_weis) > 999:
                x_mitte_weis=x_0_weis-(self.strassenbreite/2)
                
            else:

                alpha_weis=np.arctan(steigung_normale_weis)
                if(alpha_weis<0):

                    phi_weis=-np.pi/2-alpha_weis
                else:
                    phi_weis=np.pi/2-alpha_weis

                delta_x_weis=(self.strassenbreite/2)/np.cos(phi_weis)
                #print("weis delta x pixel: ", delta_x_weis)

                x_mitte_weis=x_0_weis-abs(delta_x_weis)



            return x_mitte_weis


#Hier auch analog zum Radius, abschätzung über den wirklichen mittelpunkt machen mit der gewichtungen
        if(steigung_weis is None)&(steigung_gelb is not None):
            x_mitte_gelb = mitte_gelb()

            x_mitte = x_mitte_gelb

            
        elif(steigung_weis is not None)&(steigung_gelb is None):
            x_mitte_weis = mitte_weis()

            x_mitte = x_mitte_weis


        elif(steigung_weis is not None)&(steigung_gelb is not None):
            weis_fac=white_weight/(white_weight+yellow_weight)
            gelb_fac=yellow_weight/(white_weight+yellow_weight)

            x_mitte_gelb = mitte_gelb()

            x_mitte_weis, phi_weis = mitte_weis()

    
            # Berechnung des gewichteten Mittelpunkts
            x_mitte = weis_fac * x_mitte_weis + gelb_fac * x_mitte_gelb



        else: #(steigung_weis is None)&(steigung_gelb is None):
              
            x_mitte = self.strassenmitte




        
        #Hier Punkte der Polynome plotten
        höhe=480
        breite=640
        #dem bild wird ein bereich am unteren ende eingefügt das mit den berechneten werten genau so befüllt wird
        #TODO schauen, welcher zusätzliche bereich sinnvoll ist
        calculated=200
        höhe_slw=32

        img_points = np.zeros((höhe+calculated, breite, 3), dtype=np.uint8)
        #den zusätzlichen bereich mit grauen pixeln markieren
        img_points[höhe:, :, :]=[50, 50, 50]
        num_points = 42


        #GELB

        #überprüfen, ob polynom existiert
        if(steigung_gelb!=None):
            
            #Polynom zeichnen

            y_values_gelb = np.linspace(0, höhe+calculated, num_points)
            x_values_gelb = np.polyval([a_gelb, b_gelb, c_gelb], y_values_gelb)

            for x, y in zip(x_values_gelb, y_values_gelb):
                if 0 <= x < breite and 0 <= y < höhe+calculated:
                    cv2.circle(img_points, (int(x), int(y)), 5, (0, 255, 255), -1)  #dunkles Gelb, Radius 5

            #Anfahrtspunkt gelb zeichnen
            cv2.circle(img_points, (x_mitte_gelb, y_0), radius=5, color=(0, 170, 170), thickness=-1)




        #Wenn Polynom nicht existiert, dann Rotes Rechteck zeichnen
        else:
            # Farben definieren (BGR: Rot)
            abstand = 10

            # Rechteck-Eigenschaften
            rect_width = 100  # Breite des Rechtecks
            rect_height = 50  # Höhe des Rechtecks

            # Berechne die Position des Rechtecks
            rect_x = abstand  # Abstand vom linken Rand
            rect_y = höhe - rect_height - abstand  # Abstand vom unteren Rand

            # Rechteck zeichnen
            top_left = (rect_x, rect_y)
            bottom_right = (rect_x + rect_width, rect_y + rect_height)
            red_color = (0, 0, 255)
            cv2.rectangle(img_points, top_left, bottom_right, red_color, -1)



        #WEIS 

        #überprüfen, ob polynom existiert
        if(steigung_weis!=None):


            #polynom zeichnen
            y_values_weis = np.linspace(0, höhe+calculated, num_points)
            x_values_weis = np.polyval([a_weis, b_weis, c_weis], y_values_weis)

            for x, y in zip(x_values_weis, y_values_weis):
                if 0 <= x < breite and 0 <= y < höhe+calculated:
                    cv2.circle(img_points, (int(x), int(y)), 5, (255, 255, 255), -1)  #Weis, Radius 5


        #weisen anfahrtspunkt zeichnen
            cv2.circle(img_points, (x_mitte_weis, y_0), radius=5, color=(170, 170, 170), thickness=-1)



        #Wenn Polynom nicht existiert, dann Rotes Rechteck zeichnen
        else:
            red_color = (0, 0, 255)

            # Rechteck-Eigenschaften
            rect_width = 100  # Breite des Rechtecks
            rect_height = 50  # Höhe des Rechtecks

            # Rechteck-Position (unten rechts)
            rect_x = breite - rect_width - 10  # 10 Pixel Abstand vom rechten Rand
            rect_y = höhe - rect_height - 10  # 10 Pixel Abstand vom unteren Rand

            # Rechteck zeichnen
            top_left = (rect_x, rect_y)
            bottom_right = (rect_x + rect_width, rect_y + rect_height)
            cv2.rectangle(img_points, top_left, bottom_right, red_color, -1)
        



        #Gesamten Anfahrtspunkt plotten
        cv2.circle(img_points, (x_mitte, y_0), radius=5, color=(255, 0, 0), thickness=-1)


        cv2.imshow('Polynom und Anfahrtspunkt', img_points)
        cv2.waitKey(1)




#Hier kommt die regelung


        time= rospy.gettime()
        
        dt=time-self.time_last

        

        #um in den selben größenordnungen zu bleiben, auch in m rechnen
        x_err = (self.strassenmitte - x_mitte)*self.factor

        
        x_d_err = (x_err-self.x_err_last)/dt

        self.x_i_err = self.x_i_err + x_err*dt

        
        #Auch hier ist, wenn der Fehler positiv ist, muss das auto weiter links fahren, wenn der Fehler negativ ist, weiter rechts
        omega = self.kp_x * x_err + self.ki_x * self.x_i_err + self.kd_x * x_d_err


       

        #Werte aktualisieren
        self.x_err_last = x_err
        self.time_last = time








        #wenn keine linien erkannt wurden, dann lieber langsam fahren
        if steigung_gelb is None and steigung_weis is None:
            print("Kein Mittelpunkt der Straße ermittelt!!")
            v=self.v_min

        if omega <= self.omega_min:
            v = self.v_min
        elif omega >= self.omega_max:
            v = self.v_max
        else:
            # Lineare Interpolation der Geschwindigkeit v basierend auf omega
            v = self.v_min + (omega - self.omega_min) * (self.v_max - self.v_min) / (self.omega_max - self.omega_min)

        # Berechne die Geschwindigkeiten der Räder basierend auf der Gesamtgeschwindigkeit und der Winkelgeschwindigkeit
        v_left = v - (omega * self.wheel_base) / 2
        v_right = v + (omega * self.wheel_base) / 2

        print("v_left, v_right", v_left, v_right)
        message = WheelsCmdStamped(vel_left=v_left, vel_right=v_right)

        #print("my_lane_following_node: published!")
        self.pub.publish(message)
            
        

if __name__ == '__main__':
    # create the node
    node = MyLaneFollowingNode(node_name='my_lane_following_node')
    # keep spinning
    rospy.spin()