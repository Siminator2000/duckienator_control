#!/usr/bin/env python3



import os
import math
import rospy #type: ignore
from duckietown.dtros import DTROS, NodeType #type: ignore
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped #type: ignore
import numpy as np
from datetime import datetime
import csv
import random



class SysCalNode(DTROS):

    def __init__(self, node_name):
  
        super(SysCalNode, self).__init__(node_name=node_name, node_type=NodeType.DIAGNOSTICS)

        vehicle_name = os.environ['VEHICLE_NAME']
        my_wheels_topic = f"/{vehicle_name}/low_level_pid_controller/my_wheel_cmd"

        self.umfang_links=2*3.3*np.pi * 0.01   #m
        self.umfang_rechts=2*3.3*np.pi * 0.01  #m

        self.res = 135 #auflösung räder bzw. ticks pro ganze umdrehung
      
        self._publisher = rospy.Publisher(my_wheels_topic, WheelsCmdStamped, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback) #ruft Funktion alle 0.1 Sekunden auf

        self.csv_file = open('/data/sys_id/sys_id.csv', mode='w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time','v_l_soll', 'v_r_soll', 'v_l_ist', 'v_r_ist'])

        #Wheel encoder
        self._left_encoder_topic = f"/{vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{vehicle_name}/right_wheel_encoder_node/tick"
        
        self.ticks_left = None
        self.ticks_right = None
        self.ticks_right_last=None
        self.ticks_left_last=None
        
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)


        self.strecke_pro_tick_links = (np.pi * self.umfang_links)/self.res
        self.strecke_pro_tick_rechts = (np.pi * self.umfang_rechts)/self.res


        self.v_r_soll = None
        self.v_l_soll = None
        self.v_r_ist = None
        self.v_l_ist = None
        self.time_last_r = None
        self.time_last_l = None



    def callback_left(self, data):
        self.ticks_left = data.data

    def callback_right(self, data):
        self.ticks_right = data.data

    def timer_callback(self, event):

        time_l = rospy.get_time()

        if (self.ticks_left_last == None)|(self.time_last_l == None):
            self.v_l_ist = None
        else:
            dt = time_l-self.time_last_l
            self.v_l_ist = (self.ticks_left - self.ticks_left_last)*self.strecke_pro_tick_links/dt

        self.time_last_l = time_l
        self.ticks_left_last = self.ticks_left


        time_r = rospy.get_time()

        if (self.ticks_right_last == None)|(self.time_last_r == None):
            self.v_r_ist = None
        else:
            dt = time_r-self.time_last_r
            self.v_r_ist = (self.ticks_right - self.ticks_right_last)*self.strecke_pro_tick_rechts/dt

        self.time_last_r = time_r
        self.ticks_right_last = self.ticks_right




        if (self.v_r_soll != None) & (self.v_l_soll != None) & (self.v_r_ist != None) & (self.v_l_ist != None):

            # Daten zusammenfügen

            time=rospy.get_time()

            datetime_obj = datetime.fromtimestamp(time)

            # Zeit im gewünschten Format für MATLAB darstellen
            matlab_time_str = datetime_obj.strftime('%Y-%m-%d %H:%M:%S.%f')

            # Das Format '%f' gibt Mikrosekunden zurück, aber MATLAB braucht Millisekunden.
            # Daher kürzen wir die Mikrosekunden auf Millisekunden (erste 3 Stellen).
            matlab_time_str = matlab_time_str[:-3]

            values = [matlab_time_str, self.v_l_soll, self.v_r_soll, self.v_l_ist, self.v_r_ist]
            
            # Schreiben der Daten in eine Zeile
            self.csv_writer.writerow(values)






    def run(self):
        dauer = 2  # 2 Sekunden je Wert

        # Rechte Radgeschwindigkeit in 0,1-Schritten von 0 bis 2
        for i in range(21):  # 21 Schritte von 0 bis 2 (0.0, 0.1, ..., 2.0)
            self.v_r_soll = i * 0.1
            self.v_l_soll = 0.0
            message = WheelsCmdStamped(vel_left=self.v_l_soll, vel_right=self.v_r_soll)
            rospy.loginfo(f"Publiziere {i+1}. Rechte Geschwindigkeit: {self.v_r_soll}, Linke Geschwindigkeit: {self.v_l_soll}")
            self._publisher.publish(message)
            rospy.sleep(dauer)


        # Linke Radgeschwindigkeit in 0,1-Schritten von 0 bis 2
        for i in range(21):  # 21 Schritte von 0 bis 2 (0.0, 0.1, ..., 2.0)
            self.v_l_soll = i * 0.1
            self.v_r_soll = 0.0
            message = WheelsCmdStamped(vel_left=self.v_l_soll, vel_right=self.v_r_soll)
            rospy.loginfo(f"Publiziere {i+22}. Linke Geschwindigkeit: {self.v_l_soll}, Rechte Geschwindigkeit: {self.v_r_soll}")
            self._publisher.publish(message)
            rospy.sleep(dauer)
        
        # Linke Radgeschwindigkeit auf 0 setzen
        self.v_l_soll = 0.0
        message = WheelsCmdStamped(vel_left=self.v_l_soll, vel_right=self.v_r_soll)
        rospy.loginfo(f"Publiziere 43. Linke Geschwindigkeit: {self.v_l_soll}, Rechte Geschwindigkeit: {self.v_r_soll}")
        self._publisher.publish(message)
        rospy.sleep(dauer)
        """
        # 10 zufällige Kombinationen von Geschwindigkeiten bis 2
        for i in range(10):
            self.v_l_soll = round(random.uniform(0, 2.0), 1)
            self.v_r_soll = round(random.uniform(0, 2.0), 1)
            message = WheelsCmdStamped(vel_left=self.v_l_soll, vel_right=self.v_r_soll)
            rospy.loginfo(f"Publiziere {i+44}. Linke Geschwindigkeit: {self.v_l_soll}, Rechte Geschwindigkeit: {self.v_r_soll}")
            self._publisher.publish(message)
            rospy.sleep(dauer)


            self.csv_file.close()
            rospy.signal_shutdown("Alle Werte gepublished und gespeichert. Knoten wird heruntergefahren!")
        """



    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)


if __name__ == '__main__':
    # create the node
    node = SysCalNode(node_name='sys_cal_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()