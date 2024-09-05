#!/usr/bin/env python3

import os
import rospy #type: ignore
import numpy as np
from duckietown.dtros import DTROS, NodeType #type: ignore
from std_msgs.msg import Float32MultiArray, Bool #type: ignore
import cv2
import time
import csv
import datetime
from duckietown_msgs.msg import WheelsCmdStamped #type: ignore


#Hier wird der image process node subscribt und dann alles genau gleich gemacht, 
# wie bei line following, um die position des autos herauszufinden.
#  diese werte werden dann mit timestamps in eine csv datei geschrieben




class CarPositionNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(CarPositionNode, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']


        line_arrays_topic = f"/{vehicle_name}/line_detection_slw_node/line_arrays"

        line_following_topic = f"/{vehicle_name}/pid_intersection_controll_node/line_following"

        #my_wheel_cmd_topic=f"/{vehicle_name}/low_level_pid_controller/my_wheel_cmd"



        # construct subscriber
        #Subscriber für das gelbe und weise polynom
        self.sub = rospy.Subscriber(line_arrays_topic, Float32MultiArray, self.callback)

        #subscriber für die geschwindigkeiten
        #self.sub_v = rospy.Subscriber(my_wheel_cmd_topic, WheelsCmdStamped, self.callback_v)
        

        #publisher, damit image_processing_node arbeitet,muss einmal eine msg mit True gepublished werden
           
        self.pub_line_following = rospy.Publisher(line_following_topic, Bool, queue_size=1)
    

        #Timer
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback) #ruft Funktion alle 0.1 Sekunden auf

        #CSV Datei
        # Zeitstempel erstellen
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        # CSV-Datei mit Zeitstempel erstellen
        directory = '/data/validation'
        if not os.path.exists(directory):
            os.makedirs(directory)
            
        self.csv_file = open(f'{directory}/middle_{timestamp}.csv', mode='w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time', 'x_soll', 'phi_soll', 'x_ist', 'phi_ist', 'v_links', 'v_rechts'])



        self.strassenbreite=354
        self.strassenmitte=320


        #Für geschwindigkeiten der räder
        self.time_last=rospy.get_time()
        
        umfang_links=2*3.3*np.pi * 0.01   #m
        umfang_rechts=2*3.3*np.pi * 0.01  #m

        res = 135 #auflösung räder bzw. ticks pro ganze umdrehung

        self.strecke_pro_tick_links = (np.pi * umfang_links)/res
        self.strecke_pro_tick_rechts = (np.pi * umfang_rechts)/res

        self.time_last_l=None
        self.time_last_r=None
        self.ticks_left_last=None
        self.ticks_right_last=None

        self.ticks_left=None
        self.ticks_right=None

        #zwischenspeichern der Werte
        self.v_left=0.0
        self.v_right=0.0
        self.x_car=None
        self.phi_car_soll=0.0
        self.x_car_soll=None
        self.phi_car=None

        print("INIT: car_position_node")
        


    #def callback_v(self, msg):
    #    self.v_left = msg.vel_left
    #    self.v_rig = msg.vel_right


    def callback_left(self, data):
        self.ticks_left = data.data


    def callback_right(self, data):
        self.ticks_right = data.data


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
        
        

        
        def f_gelb(y):
            return a_gelb*y**2 + b_gelb*y + c_gelb
        
        def f_abl_gelb(y):
            return 2*a_gelb*y + b_gelb
        
        def f_weis(y):
            return a_weis*y**2 + b_weis*y + c_weis
        
        def f_abl_weis(y):
            return 2*a_weis*y + b_weis
        


        #Hier Punkte der Polynome plotten
        höhe=480
        breite=640
        höhe_slw=32

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
        if(a_gelb != 0.0)and(b_gelb != 0.0)and(c_gelb != 0.0):
            
            #Polynom zeichnen

            y_values_gelb = np.linspace(0, höhe+calculated, num_points)
            x_values_gelb = np.polyval([a_gelb, b_gelb, c_gelb], y_values_gelb)

            for x, y in zip(x_values_gelb, y_values_gelb):
                if 0 <= x < breite and 0 <= y < höhe+calculated:
                    cv2.circle(img_points, (int(x), int(y)), 5, (0, 255, 255), -1)  #dunkles Gelb, Radius 5


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
        if(a_weis != 0.0)and(b_weis != 0.0)and(c_weis != 0.0):


            #polynom zeichnen
            y_values_weis = np.linspace(0, höhe+calculated, num_points)
            x_values_weis = np.polyval([a_weis, b_weis, c_weis], y_values_weis)

            for x, y in zip(x_values_weis, y_values_weis):
                if 0 <= x < breite and 0 <= y < höhe+calculated:
                    cv2.circle(img_points, (int(x), int(y)), 5, (255, 255, 255), -1)  #Weis, Radius 5


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
        









        

#Hier alles um rauszufinden, wo das Auto im Moment auf der Straße positioniert ist

        #TODO Hier auch noch einen größeren Wert wählen, damit der Mittelpunkt auch wirklich der Mittelpunkt ist, also die Drehachse
        y_0_car=480

        
        if(a_gelb!=0.0)&(b_gelb!=0.0)&(c_gelb!=0.0):
            steigung_gelb_car = f_abl_gelb(y_0_car)
            steigung_normale_gelb_car=-1/steigung_gelb_car
            x_0_gelb_car=f_gelb(y_0_car)
        else:
            steigung_gelb_car = None
            steigung_normale_gelb_car= None
            x_0_gelb_car = None

        if(a_weis!=0.0)&(b_weis!=0.0)&(c_weis!=0.0):
            steigung_weis_car = f_abl_weis(y_0_car)
            steigung_normale_weis_car=-1/steigung_weis_car
            x_0_weis_car = f_weis(y_0_car)
        else:
            steigung_weis_car=None
            steigung_normale_weis_car=None
            x_0_weis_car = None

        #Falls die Steigung zu groß ist, kann angenommen werden, dass sich die Mitte rechts horizontal von der gelben Linie befindet
        def mitte_gelb_car():
            if abs(steigung_normale_gelb_car) > 999:
                x_mitte_gelb_car=x_0_gelb_car+(self.strassenbreite/2)
                phi_gelb=0
                
            else:

                alpha_gelb=np.arctan(steigung_normale_gelb_car)
                if alpha_gelb<0:
                    phi_gelb=-np.pi/2-alpha_gelb
                else:
                    phi_gelb=np.pi/2-alpha_gelb

                

                delta_x_gelb=(self.strassenbreite/2)/np.cos(phi_gelb)
                #print("gelb delta x pixel: ", delta_x_gelb)

                x_mitte_gelb_car=x_0_gelb_car+abs(delta_x_gelb)

                
                
                print("gelb winkel", np.rad2deg(phi_gelb))


            return x_mitte_gelb_car, phi_gelb
        
        #Falls die Steigung zu groß ist, kann angenommen werden, dass sich die Mitte links horizontal von der weisen Linie befindet
        def mitte_weis_car():
            if abs(steigung_normale_weis_car) > 999:
                x_mitte_weis_car=x_0_weis_car-(self.strassenbreite/2)
                phi_weis=0
                
            else:

                alpha_weis=np.arctan(steigung_normale_weis_car)
                if(alpha_weis<0):

                    phi_weis=-np.pi/2-alpha_weis
                else:
                    phi_weis=np.pi/2-alpha_weis

                delta_x_weis=(self.strassenbreite/2)/np.cos(phi_weis)
                #print("weis delta x pixel: ", delta_x_weis)

                x_mitte_weis_car=x_0_weis_car-abs(delta_x_weis)


                print("weis winkel", np.rad2deg(phi_weis))


            return x_mitte_weis_car, phi_weis


#Hier auch analog zum Radius, abschätzung über den wirklichen mittelpunkt machen mit der gewichtungen
        if(steigung_weis_car is None)&(steigung_gelb_car is not None):
            x_mitte_gelb_car, phi_gelb_car = mitte_gelb_car()

            x_mitte_car = x_mitte_gelb_car


            phi_car = phi_gelb_car
            
        elif(steigung_weis_car is not None)&(steigung_gelb_car is None):
            x_mitte_weis_car, phi_weis_car = mitte_weis_car()

            x_mitte_car = x_mitte_weis_car

            
            phi_car = phi_weis_car

        elif(steigung_weis_car is not None)&(steigung_gelb_car is not None):
            weis_fac=white_weight/(white_weight+yellow_weight)
            gelb_fac=yellow_weight/(white_weight+yellow_weight)

            x_mitte_gelb_car, phi_gelb_car = mitte_gelb_car()

            x_mitte_weis_car, phi_weis_car = mitte_weis_car()

    
            # Berechnung des gewichteten Mittelpunkts
            x_mitte_car = weis_fac * x_mitte_weis_car + gelb_fac * x_mitte_gelb_car

            phi_car = phi_weis_car*weis_fac + phi_gelb_car*gelb_fac

            #Berechnung des Winkels zur x-achse


        else: #(steigung_weis is None)&(steigung_gelb is None):
              
            x_mitte_car = self.strassenmitte
            phi_car=0



#Hier alles Für das Plotten des Auto Mittelpunktes
        """
        #gelbe linie zum auto, wenn gelbes Polynom existiert
        if steigung_gelb_car is not None:
            cv2.line(img_points, (int(x_0_gelb_car), int(y_0_car)), (int(x_mitte_gelb_car), int(y_0_car)), (0, 177, 177), 3)

        #weise linie zum auto, wenn weises Polynom existiert
        if steigung_weis_car is not None:
            cv2.line(img_points, (int(x_0_weis_car), int(y_0_car)), (int(x_mitte_weis_car), int(y_0_car)), (177, 177, 177), 3)

        """

        #Wenn Position existiert, Auto plotten
        if(x_mitte_car is not None):
            
            phi_car_in_grad=np.rad2deg(phi_car)
            print("Winkel des Autos zur Straße: ", phi_car_in_grad)
            """
            # Größe des Rechtecks
            rect_width = 100
            rect_height = 50

            # Farbe des Rechtecks (Blau)
            color = (255, 0, 0)

            # Berechnung der Eckpunkte des Rechtecks vor der Rotation
            rect_pts = np.array([
                [-rect_width // 2, -rect_height // 2],  # obere linke Ecke
                [rect_width // 2, -rect_height // 2],   # obere rechte Ecke
                [rect_width // 2, rect_height // 2],    # untere rechte Ecke
                [-rect_width // 2, rect_height // 2]    # untere linke Ecke
            ])

            # Rotationstransformationsmatrix erstellen
            rotation_matrix = cv2.getRotationMatrix2D((0, 0), phi_car_in_grad, 1.0)

            # Rotierte Punkte berechnen
            rotated_pts = np.dot(rect_pts, rotation_matrix[:, :2].T)

            # Verschieben der Punkte zu x_mitte_car, y_0_car
            rotated_pts[:, 0] += x_mitte_car
            rotated_pts[:, 1] += y_0_car

            # Umwandlung der Punkte in Integer-Werte für OpenCV
            rotated_pts = rotated_pts.astype(np.int32)

            # Zeichnen des Rechtecks mit den rotieren Punkten
            cv2.fillPoly(img_points, [rotated_pts], color=(255, 0, 0))
            """
            
            # Länge der Linie
            line_length = 75

            # Berechne die Endpunkte der Linie mit dem gleichen Winkel

            # Endpunkt 1 berechnen (in eine Richtung entlang des Winkels)
            x1 = int(x_mitte_car + abs(line_length * np.cos(phi_car)))
            y1 = int(y_0_car + abs(line_length * np.sin(phi_car)))

            # Endpunkt 2 berechnen (in die andere Richtung entlang des Winkels)
            x2 = int(x_mitte_car - abs(line_length * np.cos(phi_car)))
            y2 = int(y_0_car - abs(line_length * np.sin(phi_car)))

            # Zeichnen der roten Linie 
            cv2.line(img_points, (x1, y1), (x2, y2), [0, 0, 255], thickness=2)
           

            #Gelben punkt im mittelpunkt zeichnen
            cv2.circle(img_points, (int(x_mitte_car), int(y_0_car)), radius=5, color=(0, 255, 255), thickness=-1)









        cv2.imshow('Car Position', img_points)
        cv2.waitKey(1)




        #Werte übergeben für csv datei
        self.x_car=int(x_mitte_car)
        self.phi_car=int(phi_car)
        self.x_car_soll=int(self.strassenbreite/2)
        








    def timer_callback(self, event):

        #Berechnungen der Geschwindigkeiten     
        time = rospy.get_time()

        if (self.ticks_left_last is None)|(self.time_last_l is None)|(self.ticks_left is None):
            self.v_l_ist = 0.0
        else:
            dt = time-self.time_last_l
            self.v_l_ist = (self.ticks_left - self.ticks_left_last)*self.strecke_pro_tick_links/dt
        #print("Geschwindigkeit links: ", self.v_l_ist)
        self.time_last_l = time
        self.ticks_left_last = self.ticks_left


        time = rospy.get_time()

        if (self.ticks_right_last is None)|(self.time_last_r is None)|(self.ticks_right is None):
            self.v_r_ist = 0.0
        else:
            dt = time-self.time_last_r
            self.v_r_ist = (self.ticks_right - self.ticks_right_last)*self.strecke_pro_tick_rechts/dt
        #print("Geschwindigkeit rechts: ", self.v_r_ist)
        self.time_last_r = time
        self.ticks_right_last = self.time_last




        if (self.v_left is not None and 
            self.v_right is not None and 
            self.x_car is not None and 
            self.y_car is not None and 
            self.x_car_soll is not None and 
            self.y_car_soll is not None):
            # Werte in die CSV-Datei schreiben
            self.csv_writer.writerow([time, self.v_left, self.v_right, self.x_car, self.y_car, self.x_car_soll, self.y_car_soll])

        self.time_last=time

    def on_shutdown(self):
        self.csv_file.close()



    def publish_message(self):
       #published nach 1 sekunde die msg, dass die image process node anfangen soll, das bild zu verarbeiten. 
       #diese zeit wird benötigt, sodass die image process node fertig initialisiert wurde

        msg = Bool()
        msg.data = True
        time.sleep(3)
        print("Starten des image process prozess!")
        self.pub_line_following.publish(msg)
        
       




if __name__ == '__main__':
    # create the node
    node = CarPositionNode(node_name='car_position_node')
    # keep spinning
    node.publish_message()
    
    rospy.spin()
