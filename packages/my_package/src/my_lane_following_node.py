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

        my_wheel_cmd_topic=f"/{vehicle_name}/low_level_pid_controller/my_wheel_cmd"

        # construct subscriber
        self.sub = rospy.Subscriber(line_arrays_topic, Float32MultiArray, self.callback)

        #publisher
        self.pub = rospy.Publisher(my_wheel_cmd_topic, WheelsCmdStamped, queue_size=1)
        

        #Konstanten
        #21cm entsprechen 354 Pixel -->
        self.factor=0.21/354 #=0.0006   Faktor um pixel in meter zu rechnen --> wird benutzt, um den radius zu berechnen

        self.strassenbreite=354  #in Pixel
        self.strassenmitte=320   #in Pixel

        self.wheel_base=0.1     #in m

        self.curve_threshhold=0.4   #in m

        self.r_max = 0.34 #in m
        self.r_min = 0.105 #in m
        self.v_max = 0.7
        self.v_min = 0.2


        #PID Regler für Straßenmittenregelung

        #TODO PID Werte anpassen
        self.kp_x=1
        self.ki_x=0.5
        self.kd_x=0.1

        self.kp_phi=1
        self.ki_phi=0.5
        self.kd_phi=0.1


        self.time_last = rospy.gettime()
        self.x_err_last=0
        self.x_i_err=0

        self.phi_err_last=0
        self.phi_i_err=0

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

        #Falls die Steigung zu gering ist, kann angenommen werden, dass sich die Mitte rechts horizontal von der gelben Linie befindet
        def mitte_gelb():
            if abs(steigung_gelb) < 0.1:
                x_mitte_gelb=x_0_gelb+self.strassenbreite/2
                y_mitte_gelb=y_0
            else:
                delta_y_gelb=(self.strassenbreite/2)/np.sqrt(1+steigung_normale_gelb*steigung_normale_gelb)
                delta_x_gelb=steigung_normale_gelb*delta_y_gelb

                #unterscheiden, damit in die korrekte richtung die straßenbreite/2 dazugezählt wird

                if steigung_gelb>0:
                    y_mitte_gelb=y_0-delta_y_gelb
                    x_mitte_gelb=x_0_gelb-delta_x_gelb
                else:
                    y_mitte_gelb=y_0+delta_y_gelb
                    x_mitte_gelb=x_0_gelb+delta_x_gelb


            return x_mitte_gelb, y_mitte_gelb
        
        #Falls die Steigung zu gering ist, kann angenommen werden, dass sich die Mitte links horizontal von der weisen Linie befindet
        def mitte_weis():
            if abs(steigung_weis) < 0.1:
                x_mitte_weis=x_0_weis-self.strassenbreite/2
                y_mitte_weis=y_0
            else:
                delta_y_weis=(self.strassenbreite/2)/np.sqrt(1+steigung_normale_weis*steigung_normale_weis)
                delta_x_weis=steigung_normale_weis*delta_y_weis


                #unterscheiden, damit in die korrekte richtung die straßenbreite/2 dazugezählt wird

                if steigung_weis>0: 
                    y_mitte_weis=y_0+delta_y_weis
                    x_mitte_weis=x_0_weis+delta_x_weis
                else:
                    y_mitte_weis=y_0-delta_y_weis
                    x_mitte_weis=x_0_weis-delta_x_weis


            return x_mitte_weis, y_mitte_weis
        






        #TODO Hier noch etwas zur y-koordinate dazurechnen, da die lenkachse nicht direkt am unteren ende der kamera ist, sondern noch etwas weiter unter --> nachmessen!!!


        def radius_soll(x,y):
            #Berechnet anhand des gegebenen anfahrtspunkt einen radius, den das auto fahren soll, um den punkt zu erreichen
            #Wichtig:    wenn der radius positiv ist, handelt es sich um eine Linkskurve
            #            wenn der Radius negativ ist, um eine Rechtskurve

            #Koordinatentransformation, damit es aus sicht des autos ist
            x_auto = 320-x
            y_auto = 480-y

            x_in_m=x_auto#*self.factor
            y_in_m=y_auto#*self.factor

            radius = ((y_in_m*y_in_m)+(x_in_m*x_in_m))/(2*x_in_m)

            return radius


        #Gewichtung der werte nach erkannten Kanten, viele weise kanten --> mitte der strase orientiert sich an weiser linie
        #viele gelbe kanten --> mitte der strase orientiert sich an gelber linie




        if(steigung_weis==None)&(steigung_gelb!=None):
            x_mitte_gelb, y_mitte_gelb = mitte_gelb()
            radius = radius_soll(x_mitte_gelb, y_mitte_gelb)

            
        elif(steigung_weis!=None)&(steigung_gelb==None):
            x_mitte_weis, y_mitte_weis = mitte_weis()
            radius = radius_soll(x_mitte_weis, y_mitte_weis)


        elif(steigung_weis!=None)&(steigung_gelb!=None):
            weis_fac=white_weight/(white_weight+yellow_weight)
            gelb_fac=yellow_weight/(white_weight+yellow_weight)

            x_mitte_gelb, y_mitte_gelb = mitte_gelb()
            radius_gelb = radius_soll(x_mitte_gelb, y_mitte_gelb)

            x_mitte_weis, y_mitte_weis = mitte_weis()
            radius_weis = radius_soll(x_mitte_weis, y_mitte_weis)

            radius = radius_weis*weis_fac + radius_gelb*gelb_fac

        else: #(steigung_weis==None)&(steigung_gelb==None):
              
            radius = None







        
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




            #Normale von gelb Zeichnen

            cv2.line(img_points, (int(x_0_gelb), int(y_0)), (int(x_mitte_gelb), int(y_mitte_gelb)), (0, 200, 200), 2)


            """
            #Kreis zeichen 
            def draw_partial_circle(center_x, center_y, radius, start_angle, end_y):
    
                
                # Startwinkel in Radianten umrechnen
                start_angle_rad = math.radians(start_angle)
                current_angle = start_angle_rad
                
                while True:
                    # Berechne x und y Koordinaten
                    x = int(center_x + radius * math.cos(current_angle))
                    y = int(center_y + radius * math.sin(current_angle))
                    
                    # Zeichne einen Punkt
                    cv2.circle(img_points, (x, y), 1, (0, 90, 90), -1)  # gelber Punkt
                    
                    # Überprüfe, ob der y-Wert unterschritten wurde
                    if y < end_y:
                        break
                    
                    # Erhöhe den Winkel
                    current_angle += 0.01  # Kleinere Schritte für einen glatteren Kreis
                

            draw_partial_circle(320-radius_gelb, 480, radius_gelb, 0, y_mitte_gelb)
            """


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




            #Normale von weis Zeichnen


            cv2.line(img_points, (int(x_0_weis), int(y_0)), (int(x_mitte_weis), int(y_mitte_weis)), (128, 128, 128), 2)


            """
            #Kreis zeichen 
            def draw_partial_circle(center_x, center_y, radius, start_angle, end_y):
    
                
                # Startwinkel in Radianten umrechnen
                start_angle_rad = math.radians(start_angle)
                current_angle = start_angle_rad
                
                while True:
                    # Berechne x und y Koordinaten
                    x = int(center_x + radius * math.cos(current_angle))
                    y = int(center_y + radius * math.sin(current_angle))
                    
                    # Zeichne einen Punkt
                    cv2.circle(img_points, (x, y), 1, (90, 90, 90), -1)  # grauer Punkt
                    
                    # Überprüfe, ob der y-Wert unterschritten wurde
                    if y < end_y:
                        break
                    
                    # Erhöhe den Winkel
                    current_angle += 0.01  # Kleinere Schritte für einen glatteren Kreis
                

            draw_partial_circle(320-radius_weis, 480, radius_weis, 0, y_mitte_weis)
            """




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
        


        #Route Zeichen
        if(radius==None):
            rot = (0, 0, 255) 

            # Dicke der Linie
            dicke = 4

            # Zeichne eine vertikale Linie in der Mitte des Bildes
            cv2.line(img_points, (int(breite / 2), 0), (int(breite / 2), höhe), rot, dicke)


        elif(radius > 0) & (radius < 10000):
            cv2.circle(img_points, (int(320-radius),480), int(radius), (255, 0, 0), 2)

        elif(radius < 0) & (radius > -10000):
            cv2.circle(img_points, (int(320-radius),480), int(-radius), (255, 0, 0), 2)
  
        else:
            gruen = (0, 255, 0) 

            # Dicke der Linie
            dicke = 4

            # Zeichne eine vertikale Linie in der Mitte des Bildes
            cv2.line(img_points, (int(breite / 2), 0), (int(breite / 2), höhe), gruen, dicke)



        

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


        cv2.imshow('Regelung', img_points)
        cv2.waitKey(1)


        




        #Radius umrechnen, von pixel in m
        if radius!=None:
            radius = radius*self.factor
    




        #Da ich jetzt den Radius habe und auch den Mittelpunkt des autos, kann darauf geregelt werden, dass sich das auto in der Mitte der straße befinden soll.
        # Das heißt, wenn das auto weiter links des mittelpunktes ist, und der anfahrtspunkt auch weiter links ist, muss der radius größer gewählt werden,
        # wenn das auto weiter links des mittelpunktes ist, und der anfahrtspunkt werter rechts ist, muss der radius kleiner gewählt werden
        # Andersherum ist das simultan


        time= rospy.gettime()
        
        dt=time-self.time_last

        

        #um in den selben größenordnungen zu bleiben, auch in m rechnen
        x_err = (self.strassenmitte - x_mitte_car)*self.factor

        
        x_d_err = (x_err-self.x_err_last)/dt

        self.x_i_err = self.x_i_err + x_err*dt

        
        #Auch hier ist, wenn der Fehler positiv ist, muss das auto weiter links fahren, wenn der Fehler negativ ist, weiter rechts
        delta_radius_x = self.kp_x * x_err + self.ki_x * self.x_i_err + self.kd_x * x_d_err


        phi_err = phi_car

        phi_d_err = (phi_err-self.phi_err_last)/dt

        self.phi_i_err = self.x_i_err + phi_err*dt

        delta_radius_phi = self.kp_phi * phi_err + self.ki_phi * self.phi_i_err + self.kd_phi * phi_d_err


        #Werte aktualisieren
        self.x_err_last = x_err
        self.time_last = time
        self.phi_err_last=phi_err







        #Dann einfach Radius mit delta Radius addieren, um eine geregelte Winkelgeschwindigkeit zu erreichen
        radius = radius + delta_radius_x + delta_radius_phi






        #wenn keine linien erkannt wurden(dann ist radius = None), dann lieber langsam fahren
        if radius==None:
            print("Kein Mittelpunkt der Straße ermittelt!!")
            v_r=self.v_min
            v_l=self.v_min

        #Gerade fahren, wenn der radius größer als 6m ist
        elif(abs(radius) > 6):
            print("Radius>6m --> gerade fahren")
            v_l=self.v_max
            v_r=self.v_max

        
        # Wenn der Radius größer als r_max ist, setzen wir die Geschwindigkeit auf v_max
        elif (abs(radius) > self.r_max):
            print("Radius > r_max")
            v_soll = self.v_max
            if(radius>0):
                v_l = v_soll * (abs(radius) - self.wheel_base / 2) / abs(radius)
                v_r = v_soll * (abs(radius) + self.wheel_base / 2) / abs(radius)
            else:
                v_l = v_soll * (abs(radius) + self.wheel_base / 2) / abs(radius)
                v_r = v_soll * (abs(radius) - self.wheel_base / 2) / abs(radius)                
            
        # Wenn der Radius kleiner als r_min ist, setzen wir die Geschwindigkeit auf v_min
        elif abs(radius) < self.r_min:
            print("Radius < r_min")
            v_soll = self.v_min
            if(radius>0):
                v_l = v_soll * (abs(radius) - self.wheel_base / 2) / abs(radius)
                v_r = v_soll * (abs(radius) + self.wheel_base / 2) / abs(radius)
            else:
                v_l = v_soll * (abs(radius) + self.wheel_base / 2) / abs(radius)
                v_r = v_soll * (abs(radius) - self.wheel_base / 2) / abs(radius)  
        else:
            print("Lineare Interpolation")
            # Linearer Interpolation zwischen r_min und r_max, um die Geschwindigkeit zu berechnen
            v_soll = self.v_min + (self.v_max - self.v_min) * (abs(radius) - self.r_min) / (self.r_max - self.r_min)
            if(radius>0):
                v_l = v_soll * (abs(radius) - self.wheel_base / 2) / abs(radius)
                v_r = v_soll * (abs(radius) + self.wheel_base / 2) / abs(radius)
            else:
                v_l = v_soll * (abs(radius) + self.wheel_base / 2) / abs(radius)
                v_r = v_soll * (abs(radius) - self.wheel_base / 2) / abs(radius)  
        



        print("Radius: ", radius)
        print("v_l, v_r", v_l, v_r)
        message = WheelsCmdStamped(vel_left=v_l, vel_right=v_r)

        #print("my_lane_following_node: published!")
        self.pub.publish(message)
            
        

if __name__ == '__main__':
    # create the node
    node = MyLaneFollowingNode(node_name='my_lane_following_node')
    # keep spinning
    rospy.spin()