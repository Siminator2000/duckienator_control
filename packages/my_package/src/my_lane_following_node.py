#!/usr/bin/env python3

import os
import rospy
import numpy as np
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import Float32MultiArray
from duckietown_msgs.msg import WheelsCmdStamped



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

        #TODO
        self.factor=1 #Faktor um pixel in meter zu rechnen --> wird benutzt, um den radius zu berechnen

        self.strassenbreite=354  #in Pixel
        self.strassenmitte=320   #in Pixel

        self.wheel_base=0.1     #in m

        self.curve_threshhold=0.4   #in m

        self.r_max = 0.5 #in m
        self.r_min = 0.1 #in m
        self.v_max = 1 
        self.v_min = 0.4

        print("INIT: my_lane_following_node")
        



    def callback(self, msg):

        print("my_lane_following_node: main msg erhalten")

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

        y_0=70

        
        def f_gelb(y):
            return a_gelb*y**2 + b_gelb*y + c_gelb
        
        def f_abl_gelb(y):
            return 2*a_gelb*y + b_gelb
        
        def f_weis(y):
            return a_weis*y**2 + b_weis*y + c_weis
        
        def f_abl_weis(y):
            return 2*a_weis*y + b_weis
        
        if(a_gelb!=None)&(b_gelb!=None)&(c_gelb!=None):
            steigung_gelb = f_abl_gelb(70)
            steigung_normale_gelb=-1/steigung_gelb
            x_0_gelb=f_gelb(y_0)
        else:
            steigung_gelb = None
            steigung_normale_gelb= None
            x_0_gelb = None

        if(a_weis!=None)&(b_weis!=None)&(c_weis!=None):
            steigung_weis = f_abl_weis(70)
            steigung_normale_weis=-1/steigung_weis
            x_0_weis = f_weis(y_0)
        else:
            steigung_weis=None
            steigung_normale_weis=None
            x_0_weis = None

        def mitte_gelb():
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
        
        def mitte_weis():
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
        
        def radius_soll(x,y):
            #Berechnet anhand des gegebenen anfahrtspunkt einen radius, den das auto fahren soll, um den punkt zu erreichen

            #Koordinatentransformation, damit es aus sicht des autos ist
            x_auto = x
            y_auto = 480-y

            x_in_m=x_auto*self.factor
            y_in_m=y_auto*self.factor

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

        #wenn keine linien erkannt wurden(dann ist radius = None), dann lieber langsam fahren
        if radius==None:
            v_r=self.v_min
            v_l=self.v_min

        
        # Wenn der Radius kleiner als r_min ist, setzen wir die Geschwindigkeit auf v_min
        elif (radius >= self.r_max):
            v_soll = self.v_max
            v_l = v_soll * (radius - self.wheel_base / 2) / radius
            v_r = v_soll * (radius + self.wheel_base / 2) / radius
            
        # Wenn der Radius größer als r_max ist, setzen wir die Geschwindigkeit auf v_max
        elif radius <= self.r_min:
            v_soll = self.v_min
            v_l = v_soll * (radius - self.wheel_base / 2) / radius
            v_r = v_soll * (radius + self.wheel_base / 2) / radius
        else:
            # Linearer Interpolation zwischen r_min und r_max, um die Geschwindigkeit zu berechnen
            v_soll = self.v_min + (self.v_max - self.v_min) * (radius - self.r_min) / (self.r_max - self.r_min)
            v_l = v_soll * (radius - self.wheel_base / 2) / radius
            v_r = v_soll * (radius + self.wheel_base / 2) / radius
        

        #werte skalieren
        #A=abstand auto->anfahrpunkt in cm
        #G=fehler in cm
        #mid=mitte der fahrbahn, normal 640/2

        #DONE Nochmal überlegen, ob das auch wirklich die mitte ist, auch bei Kurven,
        #Lösung maybe durch np.polyfit und dann Normale bestimmen
        #Hierzu muss aber auch noch bei der image process node eine gewichtung veröffentlicht werden,
        #sodass jeder punkt eine gewichtung hat und nicht nur allgemein weis und gelb



        message = WheelsCmdStamped(vel_left=v_l, vel_right=v_r)

        print("my_lane_following_node: published!")
        self.pub.publish(message)
            
        

if __name__ == '__main__':
    # create the node
    node = MyLaneFollowingNode(node_name='my_lane_following_node')
    # keep spinning
    rospy.spin()