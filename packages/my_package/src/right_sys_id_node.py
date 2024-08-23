#!/usr/bin/env python3

import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
import numpy as np
import csv
from datetime import datetime



class SysIdNode(DTROS):

    def __init__(self, node_name):
  
        super(SysIdNode, self).__init__(node_name=node_name, node_type=NodeType.DIAGNOSTICS)

        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"    

        self.umfang_links=2*3.3*np.pi * 0.01   #m
        self.umfang_rechts=2*3.3*np.pi * 0.01  #m

        self.res = 135 #auflösung räder bzw. ticks pro ganze umdrehung
      
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)


        #Wheel encoder
        self._left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"
        
        self._ticks_left = None
        self._ticks_right = None
        
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)

        #Timer
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback) #ruft Funktion alle 0.1 Sekunden auf

        self.time_last_l = None
        self.time_last_r = None

        self._ticks_left_last = None
        self._ticks_right_last = None

        self.strecke_pro_tick_links = (np.pi * self.umfang_links)/self.res
        self.strecke_pro_tick_rechts = (np.pi * self.umfang_rechts)/self.res

        self.v_r_soll = None
        self.v_l_soll = None
        self.v_r_ist = None
        self.v_l_ist = None

        
        self.csv_file = open('/data/sys_id/sys_id.csv', mode='w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time', 'v_r_soll', 'v_r_ist'])



    def callback_left(self, data):
        self._ticks_left = data.data
        rospy.loginfo_once(f"Ticks_left_init: {self._ticks_left}")




    def callback_right(self, data):
        self._ticks_right = data.data



    def timer_callback(self, event):

        time_l = rospy.get_time()

        if (self._ticks_left_last == None)|(self.time_last_l == None):
            self.v_l_ist = None
        else:
            dt = time_l-self.time_last_l
            self.v_l_ist = (self._ticks_left - self._ticks_left_last)*self.strecke_pro_tick_links/dt

        self.time_last_l = time_l
        self._ticks_left_last = self._ticks_left


        time_r = rospy.get_time()

        if (self._ticks_right_last == None)|(self.time_last_r == None):
            self.v_r_ist = None
        else:
            dt = time_r-self.time_last_r
            self.v_r_ist = (self._ticks_right - self._ticks_right_last)*self.strecke_pro_tick_rechts/dt

        self.time_last_r = time_r
        self._ticks_right_last = self._ticks_right



        if (self.v_r_soll != None) & (self.v_l_soll != None) & (self.v_r_ist != None) & (self.v_l_ist != None):
            # Daten zusammenfügen

            time=rospy.get_time()

            datetime_obj = datetime.fromtimestamp(time)

            # Zeit im gewünschten Format für MATLAB darstellen
            matlab_time_str = datetime_obj.strftime('%Y-%m-%d %H:%M:%S.%f')

            # Das Format '%f' gibt Mikrosekunden zurück, aber MATLAB braucht Millisekunden.
            # Daher kürzen wir die Mikrosekunden auf Millisekunden (erste 3 Stellen).
            matlab_time_str = matlab_time_str[:-3]

            values = [matlab_time_str, self.v_r_soll, self.v_r_ist]
            
            # Schreiben der Daten in eine Zeile
            self.csv_writer.writerow(values)


    def run(self):

        dauer=2 #2 sekunden je wert
        """
        self.v_l_soll=0.0
        self.v_r_soll=0.0
        message1 = WheelsCmdStamped(vel_left=self.v_l_soll, vel_right=self.v_r_soll)
        rospy.loginfo("Publiziere 1.")
        self._publisher.publish(message1)
        rospy.sleep(dauer/2)  

        self.v_l_soll=0.0
        self.v_r_soll=0.3
        message3 = WheelsCmdStamped(vel_left=self.v_l_soll, vel_right=self.v_r_soll)
        rospy.loginfo("Publiziere 3.")
        self._publisher.publish(message3)
        rospy.sleep(dauer)

        self.v_l_soll=0.0
        self.v_r_soll=0.0
        message4 = WheelsCmdStamped(vel_left=self.v_l_soll, vel_right=self.v_r_soll)
        rospy.loginfo("Publiziere 4.")
        self._publisher.publish(message4)
        rospy.sleep(dauer/2)

        self.v_l_soll=0.0
        self.v_r_soll=0.4
        message4 = WheelsCmdStamped(vel_left=self.v_l_soll, vel_right=self.v_r_soll)
        rospy.loginfo("Publiziere 5.")
        self._publisher.publish(message4)
        rospy.sleep(dauer)

        self.v_l_soll=0.0
        self.v_r_soll=0.0
        message6 = WheelsCmdStamped(vel_left=self.v_l_soll, vel_right=self.v_r_soll)
        rospy.loginfo("Publiziere 6.")
        self._publisher.publish(message6)
        rospy.sleep(dauer/2)

        self.v_l_soll=0.0
        self.v_r_soll=0.5
        message7 = WheelsCmdStamped(vel_left=self.v_l_soll, vel_right=self.v_r_soll)
        rospy.loginfo("Publiziere 7.")
        self._publisher.publish(message7)
        rospy.sleep(dauer)

        self.v_l_soll=0.0
        self.v_r_soll=0.0
        message8 = WheelsCmdStamped(vel_left=self.v_l_soll, vel_right=self.v_r_soll)
        rospy.loginfo("Publiziere 8.")
        self._publisher.publish(message8)
        rospy.sleep(dauer/2)
        
        self.v_l_soll=0.0
        self.v_r_soll=0.6
        message9 = WheelsCmdStamped(vel_left=self.v_l_soll, vel_right=self.v_r_soll)
        rospy.loginfo("Publiziere 9.")
        self._publisher.publish(message9)
        rospy.sleep(dauer)
   
        self.v_l_soll=0.0
        self.v_r_soll=0.0
        message11 = WheelsCmdStamped(vel_left=self.v_l_soll, vel_right=self.v_r_soll)
        rospy.loginfo("Publiziere 11.")
        self._publisher.publish(message11)
        rospy.sleep(dauer/2)



        self.csv_file.close()

        rospy.signal_shutdown("Alle Werte gepublished und gespeichert. Knoten wird heruntergefahren!")

        """
        #Werte für Validierung
        self.v_l_soll=0.0
        self.v_r_soll=0.0
        message1 = WheelsCmdStamped(vel_left=self.v_l_soll, vel_right=self.v_r_soll)
        rospy.loginfo("Publiziere 1.")
        self._publisher.publish(message1)
        rospy.sleep(dauer/2)  

        self.v_l_soll=0.0
        self.v_r_soll=0.28
        message3 = WheelsCmdStamped(vel_left=self.v_l_soll, vel_right=self.v_r_soll)
        rospy.loginfo("Publiziere 3.")
        self._publisher.publish(message3)
        rospy.sleep(dauer)

        self.v_l_soll=0.0
        self.v_r_soll=0.0
        message4 = WheelsCmdStamped(vel_left=self.v_l_soll, vel_right=self.v_r_soll)
        rospy.loginfo("Publiziere 4.")
        self._publisher.publish(message4)
        rospy.sleep(dauer/2)

        self.v_l_soll=0.0
        self.v_r_soll=0.69
        message4 = WheelsCmdStamped(vel_left=self.v_l_soll, vel_right=self.v_r_soll)
        rospy.loginfo("Publiziere 5.")
        self._publisher.publish(message4)
        rospy.sleep(dauer)

        self.v_l_soll=0.0
        self.v_r_soll=0.0
        message6 = WheelsCmdStamped(vel_left=self.v_l_soll, vel_right=self.v_r_soll)
        rospy.loginfo("Publiziere 6.")
        self._publisher.publish(message6)
        rospy.sleep(dauer/2)

        self.v_l_soll=0.0
        self.v_r_soll=0.44
        message7 = WheelsCmdStamped(vel_left=self.v_l_soll, vel_right=self.v_r_soll)
        rospy.loginfo("Publiziere 7.")
        self._publisher.publish(message7)
        rospy.sleep(dauer)

        self.v_l_soll=0.0
        self.v_r_soll=0.0
        message8 = WheelsCmdStamped(vel_left=self.v_l_soll, vel_right=self.v_r_soll)
        rospy.loginfo("Publiziere 8.")
        self._publisher.publish(message8)
        rospy.sleep(dauer/2)
 
        self.v_l_soll=0.0
        self.v_r_soll=0.89
        message9 = WheelsCmdStamped(vel_left=self.v_l_soll, vel_right=self.v_r_soll)
        rospy.loginfo("Publiziere 9.")
        self._publisher.publish(message9)
        rospy.sleep(dauer)

        self.v_l_soll=0.0
        self.v_r_soll=0.0
        message11 = WheelsCmdStamped(vel_left=self.v_l_soll, vel_right=self.v_r_soll)
        rospy.loginfo("Publiziere 11.")
        self._publisher.publish(message11)
        rospy.sleep(dauer/2)



        self.csv_file.close()

        rospy.signal_shutdown("Alle Werte gepublished und gespeichert. Knoten wird heruntergefahren!")
        

    def on_shutdown(self):
        
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    # create the node
    node = SysIdNode(node_name='sys_id_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()