#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

import pyxhook

running = True

# rc key define  w a s d i j k l key1 key2 key3 
RC_LENGTH = 11 
rc_keys = ['0'] * RC_LENGTH

#This function is called every time a key is presssed
def key_down_event( event ):
    #print key info
    print event
    
    if event.Ascii == 119:  # w 
        rc_keys[0] = '1'

    if event.Ascii == 97:  # a 
        rc_keys[1] = '1'

    if event.Ascii == 115:  # s 
        rc_keys[2] = '1'

    if event.Ascii == 100:  # d 
        rc_keys[3] = '1'
    
    if event.Ascii == 105:  # i 
        rc_keys[4] = '1'

    if event.Ascii == 106:  # j
        rc_keys[5] = '1'

    if event.Ascii == 107:  # k 
        rc_keys[6] = '1'

    if event.Ascii == 108:  # l 
        rc_keys[7] = '1'
    #If the ascii value matches spacebar
    if event.Ascii == 32:
        global running 
        running = False
        pass

def key_up_event( event ):
    #print key info
    print event
    
    if event.Ascii == 119:  # w 
        rc_keys[0] = '0'

    if event.Ascii == 97:  # a 
        rc_keys[1] = '0'

    if event.Ascii == 115:  # s 
        rc_keys[2] = '0'

    if event.Ascii == 100:  # d 
        rc_keys[3] = '0'
    
    if event.Ascii == 105:  # i 
        rc_keys[4] = '0'

    if event.Ascii == 106:  # j
        rc_keys[5] = '0'

    if event.Ascii == 107:  # k 
        rc_keys[6] = '0'

    if event.Ascii == 108:  # l 
        rc_keys[7] = '0'

def rc_talker():
    '''init pyxhook'''
    #Create hookmanager
    hookman = pyxhook.HookManager()
    #Define our callback to fire when a key is pressed down
    hookman.KeyDown = key_down_event
    hookman.KeyUp   = key_up_event
    #Hook the keyboard
    hookman.HookKeyboard()
    #Start our listener
    hookman.start()

    pub = rospy.Publisher('rc_sender', String, queue_size=10)
    # 2016-10-26 test urdf joint control
    # controls a file in ~/toy_code/toy_ros_space/src/rc_sim_oub/test.launch
    #roslaunch test.launch model:='$(find urdf_tutorial)/urdf/07-physics.urdf' gui:=True
    pub_2 = rospy.Publisher('test_joint', JointState, queue_size=10)
    rospy.init_node('rc_talker')
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        pub.publish(''.join(rc_keys))

        # 2016-10-26 test urdf joint control
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ['gripper_extension']
        msg.position = [-0.12*int(rc_keys[0])]
        pub_2.publish(msg)
        #print rc_keys
        rate.sleep()
        if running == False:
            #Close the listener when we are done
            hookman.cancel()
            break


if __name__ == '__main__':
    try:
        rc_talker()
    except rospy.ROSInterruptException:
        pass
