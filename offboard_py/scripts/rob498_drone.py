#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from std_srvs.srv import Empty, EmptyResponse 

current_state = State()
service_mode = "INIT"

def state_cb(msg):
    global current_state
    current_state = msg
   
# Callback handlers
def handle_launch():
    global service_mode 
    service_mode = "LAUNCH"
    print('Launch Requested. Your drone should take off.')
    return

def handle_test():
    global service_mode 
    service_mode = "TEST"
    print('Test Requested. Your drone should perform the required tasks. Recording starts now.')

def handle_land():
    # If there is still stuff to execute in launch, wait for it to finish, then land 
    global service_mode 
    service_mode = "LAND"
    print('Land Requested. Your drone should land.')
    return 

def handle_abort():
    # Land even if stuff left to do in launch 
    global service_mode 
    service_mode = "ABORT" 
    print('Abort Requested. Your drone should land immediately due to safety considerations')
    return 

# Service callbacks
def callback_launch(request):
    handle_launch()
    return EmptyResponse()

def callback_test(request):
    handle_test()
    return EmptyResponse()

def callback_land(request):
    handle_land()
    return EmptyResponse()

def callback_abort(request):
    handle_abort()
    return EmptyResponse()

def callback_vicon(vicon_msg):
    print("Received vicon message")
    print(vicon_msg)

def vicon_running(topic_name='vicon/ROB498_Drone/ROB498_Drone'):
    # Get a list of tuples containing the names and data types of all the topics that are currently published
    published_topics = rospy.get_published_topics()

    # Check if the topic exists by searching for its name in the list of published topics
    if any(topic_name in topic for topic in published_topics):
        print("using vicon.")
        return True
    else:
        print("not using vicon.")
        return False

if __name__ == "__main__":
    rospy.init_node("rob498_drone")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    if vicon_running():
        # Subscribe to the vicon topic /vicon/ROB498_Drone/ROB498_Drone
        rospy.Subscriber('vicon/ROB498_Drone/ROB498_Drone', PoseStamped, callback_vicon)
    else:
        # Use data from the RealSense camera
        pass

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    
    srv_launch = rospy.Service('/rob498_drone_05/comm/launch', Empty, callback_launch)
    srv_test = rospy.Service('/rob498_drone_05/comm/test', Empty, callback_test)
    srv_land = rospy.Service('/rob498_drone_05/comm/land', Empty, callback_land)
    srv_abort = rospy.Service('/rob498_drone_05/comm/abort', Empty, callback_abort)

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if service_mode == 'INIT':
            pose.pose.position.z = 0
        elif service_mode == 'LAUNCH':
            pose.pose.position.z = 1.15
        elif service_mode == 'LAND': 
            pose.pose.position.z = 0
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()

        local_pos_pub.publish(pose)

        rate.sleep()
