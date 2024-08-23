#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import os

from std_msgs.msg import Bool

def msg_node():
   
    rospy.init_node('msg_node', anonymous=True)
    
    
    

    vehicle_name = os.environ['VEHICLE_NAME']

    intersection_direction_topic = f"/{vehicle_name}/msg_node/direction"
    pub_intersection = rospy.Publisher(intersection_direction_topic, String, queue_size=1)

    line_following_topic = f"/{vehicle_name}/pid_intersection_controll_node/line_following"
    pub_line_following = rospy.Publisher(line_following_topic, Bool, queue_size=1)
    

    
    #node läuft bis user interruption
    while not rospy.is_shutdown():
        #print("in while von msg_node")
        msg_lf1 = Bool()
        msg_lf1.data = False
        print("msg node: lf=False")
        pub_line_following.publish(msg_lf1)


        try:
            #Usereingabe
            #print("in try von msg_node")
            user_input = input("Gib \n'l' um links abzubiegen\n'r' um rechts abzubiegen\n'g' um gerade zu fahren \n'lf' um Line Following zu starten\n'e' um die Node zu beenden\n ein: ")

            
            
            if user_input.lower() == 'e':
                rospy.loginfo("Beende msg-node.")
                break
            
            elif (user_input == 'l')|(user_input == 'r')|(user_input == 'g'):
           
                print("Intersection-Controll: ", user_input)

                """
                print("Line Following deaktiviert!")
                msg = Bool()
                msg.data = False
                pub_line_following.publish(msg)
                """
                msg_string = String()
                msg_string.data = user_input
                pub_intersection.publish(msg_string)

            elif user_input == 'lf':
                msg_lf2 = Bool()
                msg_lf2.data = True
                pub_line_following.publish(msg_lf2)    

            else:
                print("Vertippt? Nochmal eingeben!")
        
        except EOFError:
           
            rospy.loginfo("EOF erreicht. Beende msg-node.")
            break

if __name__ == '__main__':
    try:
        msg_node()
    except rospy.ROSInterruptException:
        pass
