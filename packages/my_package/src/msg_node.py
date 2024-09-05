#!/usr/bin/env python3

import rospy #type: ignore
from std_msgs.msg import String #type: ignore
import os
import time

from std_msgs.msg import Bool #type: ignore

def msg_node():
   
    rospy.init_node('msg_node', anonymous=True)
    
    
    

    vehicle_name = os.environ['VEHICLE_NAME']

    intersection_direction_topic = f"/{vehicle_name}/msg_node/direction"
    pub_intersection = rospy.Publisher(intersection_direction_topic, String, queue_size=1)

    line_following_topic = f"/{vehicle_name}/pid_intersection_controll_node/line_following"
    pub_line_following = rospy.Publisher(line_following_topic, Bool, queue_size=1)
    
    msg_lf1 = Bool()
    msg_lf1.data = False
    print("msg node: lf=False")
    pub_line_following.publish(msg_lf1)
    print("INIT: msg_node")
    

    
    #node läuft bis user interruption
    while not rospy.is_shutdown():
        #print("in while von msg_node")



        try:
            #Usereingabe
            #print("in try von msg_node")
            user_input = input("\n\n\nGib \n'l' um links abzubiegen\n'r' um rechts abzubiegen\n'g' um gerade zu fahren \n'lf' um Line Following zu starten\n'!lf' um Line Following zu beenden\n'e' um die Node zu beenden\n ein: \n\n\n")

            
            
            if user_input.lower() == 'e':
                rospy.loginfo("\nBeende msg-node.\n")
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
                time.sleep(5)
                msg_lf3 = Bool()
                msg_lf3.data = False
                pub_line_following.publish(msg_lf3)  
 

            else:
                print("\nVertippt? Nochmal eingeben!\n")
        
        except EOFError:
           
            rospy.loginfo("\nEOF erreicht. Beende msg-node.\n")
            break

if __name__ == '__main__':
    try:
        msg_node()
    except rospy.ROSInterruptException:
        pass
