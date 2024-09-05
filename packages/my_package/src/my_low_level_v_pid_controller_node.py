#!/usr/bin/env python3

import os
import rospy #type: ignore
from duckietown.dtros import DTROS, NodeType #type: ignore
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped #type: ignore

import numpy as np


class MyLowLevelVPIDControllNode(DTROS):

    def __init__(self, node_name):
        #print("mylowlevelvpidnode anfang initilize....")
        

        # initialize the DTROS parent class
        super(MyLowLevelVPIDControllNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        #rospy.loginfo("Entered __init__ method")

        vehicle_name = os.environ['VEHICLE_NAME']

        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"

        my_wheel_cmd_topic=f"/{vehicle_name}/low_level_pid_controller/my_wheel_cmd"

        left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"

        # publisher
        self.pub = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        # subscriber
        self.sub = rospy.Subscriber(my_wheel_cmd_topic, WheelsCmdStamped, self.callback)

        self.sub_left = rospy.Subscriber(left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(right_encoder_topic, WheelEncoderStamped, self.callback_right)


        #Konstanten
        umfang_links=2*3.3*np.pi * 0.01   #m
        umfang_rechts=2*3.3*np.pi * 0.01  #m

        res = 135 #auflösung räder bzw. ticks pro ganze umdrehung

        self.strecke_pro_tick_links = (np.pi * umfang_links)/res
        self.strecke_pro_tick_rechts = (np.pi * umfang_rechts)/res



        #Variablen

        self.v_l_soll=0
        self.v_r_soll=0
        self.v_l_soll_last = 0
        self.v_r_soll_last = 0
        self.v_l_aktuell = 0
        self.v_r_aktuell = 0

        #callback
        self.time_last = rospy.get_time()
        self.v_l_i_err = 0
        self.v_r_i_err = 0
        self.v_l_err_last = 0
        self.v_r_err_last = 0

        self.v_i_max_windup = 1 #max 0.2 m/s TODO noch anpassen maybe

        self.min_wheel_speed=0.2

        #TODO Werte mit modellierung herausfinden
        self.kp_l = 0.4   #0.85125553370002
        self.ki_l = 2.0     #8.92833051544263
        self.kd_l = 0

        self.kp_r = 0.4     #0.880181306230901
        self.ki_r = 2.0     #6.19587472008344
        self.kd_r = 0

        #Werte zwischenspeichern, damit sie mit einer gewissen frequenz gepublished werden können
        self.v_l_stell=0
        self.v_r_stell=0


        #callback_left
        self.ticks_left = None
        self.ticks_left_last = None
        self.time_last_l = None
        self.v_l_ist = None

        #callback_right
        self.ticks_right = None
        self.ticks_right_last = None
        self.time_last_r = None
        self.v_r_ist = None

        print("MY_LOW_LEVEL_V_PID_NODE initilized!!!!")
        #rospy.loginfo("MY_LOW_LEVEL_V_PID_NODE initialized!!!!")

    # die berechnungen der geschwindigkeiten maybe in eine andere schleife (run()), damit es mit 10Hz aktualisiert wird
    def callback_left(self, data):
        self.ticks_left = data.data


    def callback_right(self, data):
        self.ticks_right = data.data

    """
    def callback(self, msg):
        
        self.v_l_soll = msg.vel_left
        self.v_r_soll = msg.vel_right

        if((self.v_l_soll_last==0) and (self.v_r_soll_last==0)):

            max_steps = 30  # Anzahl der Schritte, bis die Zielgeschwindigkeit erreicht wird
            rate = rospy.Rate(30)  # 10 Hz Aktualisierungsrate

            # Berechne die Schrittweite für die Geschwindigkeitssteigerung
            step_size_l = self.v_l_soll / max_steps
            step_size_r = self.v_r_soll / max_steps

            for step in range(1, max_steps + 1):
                # Erhöhe die Geschwindigkeiten schrittweise
                self.v_l_aktuell = step_size_l * step
                self.v_r_aktuell = step_size_r * step


                # Warte auf die nächste Iteration
                rate.sleep()
                print("Anfahrvorgang!")

        # Aktualisiere die gespeicherten letzten Zielgeschwindigkeiten
        if(((abs(self.v_l_aktuell - self.v_l_soll) < 0.01) and (abs(self.v_r_aktuell - self.v_r_soll) < 0.01))
           or ((self.v_l_soll==0.0)and(self.v_r_soll==0.0))
           or(self.v_l_soll_last!=0.0 and self.v_r_soll_last != 0.0)):
            
            self.v_l_soll_last = self.v_l_soll
            self.v_r_soll_last = self.v_r_soll

            self.v_l_aktuell=self.v_l_soll
            self.v_r_aktuell=self.v_r_soll
            print("Kein Anfahrvorgang mehr!")

        #Zum Testen
        #self.v_l_aktuell=msg.vel_left
        #self.v_r_aktuell=msg.vel_right
        """
    def callback(self, msg):
        self.v_l_soll = msg.vel_left
        self.v_r_soll = msg.vel_right
        
        # Sofortiges Abbremsen
        if self.v_l_soll == 0 and self.v_r_soll == 0:
            self.v_l_aktuell = 0
            self.v_r_aktuell = 0
            self.v_l_soll_last = 0
            self.v_r_soll_last = 0
            print("Sofortiges Abbremsen!")
            return

        # Sanftes Anfahren
        if self.v_l_soll_last == 0 and self.v_r_soll_last == 0:
            max_steps = 30  # Anzahl der Schritte, bis die Zielgeschwindigkeit erreicht wird
            rate = rospy.Rate(30)  # 30 Hz Aktualisierungsrate
            
            # Berechne die Schrittweite für die Geschwindigkeitssteigerung
            step_size_l = self.v_l_soll / max_steps
            step_size_r = self.v_r_soll / max_steps
            
            for step in range(1, max_steps + 1):
                # Erhöhe die Geschwindigkeiten schrittweise
                self.v_l_aktuell = step_size_l * step
                self.v_r_aktuell = step_size_r * step
                rate.sleep()
            
            print("Anfahrvorgang abgeschlossen!")
        
        # Aktualisiere die Geschwindigkeiten und die gespeicherten letzten Zielgeschwindigkeiten
        self.v_l_aktuell = self.v_l_soll
        self.v_r_aktuell = self.v_r_soll
        self.v_l_soll_last = self.v_l_soll
        self.v_r_soll_last = self.v_r_soll




    def run(self):
        # published mit 10Hz
        rate = rospy.Rate(10)

        
        while not rospy.is_shutdown():

            #Berechnungen der Geschwindigkeiten     
            time = rospy.get_time()

            if (self.ticks_left_last is None)|(self.time_last_l is None):
                self.v_l_ist = 0.0
            else:
                dt = time-self.time_last_l
                self.v_l_ist = (self.ticks_left - self.ticks_left_last)*self.strecke_pro_tick_links/dt
            #print("Geschwindigkeit links: ", self.v_l_ist)
            self.time_last_l = time
            self.ticks_left_last = self.ticks_left


            time = rospy.get_time()

            if (self.ticks_right_last is None)|(self.time_last_r is None):
                self.v_r_ist = 0.0
            else:
                dt = time-self.time_last_r
                self.v_r_ist = (self.ticks_right - self.ticks_right_last)*self.strecke_pro_tick_rechts/dt
            #print("Geschwindigkeit rechts: ", self.v_r_ist)
            self.time_last_r = time
            self.ticks_right_last = self.ticks_right

            #Werte berechnen
            #schauen, ob das auto anhalten soll, wenn ja, dann geschwindigkeit auf null setzen
            if (self.v_l_soll == 0.0)&(self.v_r_soll == 0.0):
                self.v_l_stell=0.0
                self.v_r_stell=0.0

                #Werte auch zurücksetzten
                self.v_l_i_err = 0
                self.v_r_i_err = 0
                self.v_l_err_last = 0
                self.v_r_err_last = 0

            else:
                time = rospy.get_time()

                dt = time - self.time_last
                

                #P
                v_l_err = self.v_l_aktuell - self.v_l_ist
                v_r_err = self.v_r_aktuell - self.v_r_ist

                #I mit Anti Windup
                if (self.v_l_i_err<self.v_i_max_windup) & (self.v_l_i_err>-self.v_i_max_windup):
                    self.v_l_i_err = self.v_l_i_err  + dt*v_l_err
                else:
                    self.v_l_i_err  = np.sign(self.v_l_i_err )*self.v_i_max_windup
                    print("V_i links maximal/minimal/nan!!!!!", self.v_l_i_err)

                if (self.v_r_i_err<self.v_i_max_windup) & (self.v_r_i_err>-self.v_i_max_windup):
                    self.v_r_i_err = self.v_r_i_err  + dt*v_r_err
                else:
                    self.v_r_i_err  = np.sign(self.v_r_i_err )*self.v_i_max_windup
                    print("V_i rechts maximal/minimal/nan!!!!!", self.v_r_i_err)

                #D
                v_l_d_err = (self.v_l_err_last-v_l_err)/dt
                v_r_d_err = (self.v_r_err_last-v_r_err)/dt

                #Werte übergeben
                self.v_l_err_last=v_l_err
                self.v_r_err_last=v_r_err
                self.time_last = time

                #v stell berechnen
                self.v_l_stell = self.kp_l * v_l_err + self.ki_l * self.v_l_i_err + self.kd_l * v_l_d_err 
                self.v_r_stell = self.kp_r * v_r_err + self.ki_r * self.v_r_i_err + self.kd_r * v_r_d_err
                
                """
                #TODO vielleicht auskommentieren, muss noch schauen, ob das wirklich sinnvoll ist
                # Bestimme das langsame Rad und stelle sicher, dass es nicht unter die Mindestgeschwindigkeit fällt
                if (self.v_l_stell < (self.min_wheel_speed)) | ((self.v_r_stell < self.min_wheel_speed)):
                    # Berechne den erforderlichen Anstieg, um das langsamste Rad auf die Mindestgeschwindigkeit zu setzen
                    delta_v = self.min_wheel_speed - min(self.v_l_stell, self.v_r_stell)
                    
                    # Beide Radgeschwindigkeiten um delta_v erhöhen, um die Differenz beizubehalten
                    self.v_l_stell += delta_v
                    self.v_r_stell += delta_v
                """


            

            #Publishen der Stell Geschwindigkeiten
            message = WheelsCmdStamped(vel_left=self.v_l_stell, vel_right=self.v_r_stell)
            message1 = WheelsCmdStamped(vel_left=0, vel_right=0) #Zum in der Bib testen, ohne, dass das auto fährt
            self.pub.publish(message)
            #print("v_l_soll_last", self.v_l_soll_last)
            rate.sleep()
            #rospy.loginfo(f"Publishing wheel command: left={self.v_l_stell}, right={self.v_r_stell}")





    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self.pub.publish(stop)

if __name__ == '__main__':
    # create the node
    node = MyLowLevelVPIDControllNode(node_name='my_low_level_v_pid_controller_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()