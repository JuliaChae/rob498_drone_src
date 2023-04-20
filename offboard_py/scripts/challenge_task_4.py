#! /usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, EmptyResponse
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from tf.transformations import quaternion_matrix
import tf
import pdb

class ChallengeTask3:

    def __init__(self):
        self.STATE = 'INIT'
        self.num_waypoints = 6 # Expected number of waypoints
        self.WAYPOINTS = None
        # self.WAYPOINTS = np.array([[0, 5, 1],[5, 5, 1],[5, 0, 1]])
        # self.WAYPOINTS = np.array([[-2, 2, 1],[5, 5, 1],[5, 0, 1]])
        # self.WAYPOINTS = np.array([[3,3,1]])
        self.WAYPOINTS_AVOID = np.zeros((0,3))
        self.WAYPOINTS_ORIG = None
        self.WAYPOINT_FLAG = np.full((self.num_waypoints, ), False)
        self.WAYPOINTS_RECEIVED = False
        self.halfface = 0.15
        self.PERTURB_OFFSET = np.asarray([[-self.halfface, -self.halfface, -self.halfface]])
        self.PERTURB_FLAG = np.full((self.PERTURB_OFFSET.shape[0], ), False)
        self.name = 'rob498_drone_05'  # Change 00 to your team ID
        self.pose = PoseStamped() # pose to set local_position to 
        self.current_state = State() 
        self.waypoint_cnt = -1 # Current idx of the waypoint
        self.current_waypoint = np.zeros((0,3)) # Waypoint position we are going 
        self.T_odom_vicon = None # Transform between odom and vicon frames
        self.pose_vicon_drone = np.eye(4) # Transform from drone to vicon frame (current step)
        self.T_odom_drone = np.eye(4) # Transform from the drone to the odom frame
        self.current_pose = np.zeros((0,3)) # Current pose of the drone from /mavros/odometry/out
        self.debug = True
        self.use_vicon = False
        self.yaw_angle = 0
        pos_angles = np.linspace(0, 1, 20)*np.pi 
        neg_angles = np.linspace(-0.9, 0, 19)*np.pi 
        self.spin_angles = np.hstack([pos_angles, neg_angles])
        # self.spin_angles = pos_angles
        #self.spin_angles = [np.pi/8, np.pi/4, 3*np.pi/8, np.pi/2,  5*np.pi/8, 3*np.pi/4, -3*np.pi/4, -5*np.pi/8, -np.pi/2, -3*np.pi/8, -np.pi/4, -np.pi/8, 0]
        self.spin_angle_count = 0
        self.stopping_angles = [np.pi/4, 3*np.pi/4, -3*np.pi/4, -np.pi/4]
        # self.stopping_angles = [np.pi/4]
        self.spin_done = False
        self.obstacles_received = False    
        # self.obstacle_map = np.asarray([[-1, 1], [0, 3.5]]) 
        self.obstacle_map = None
        # self.obstacle_type = ["R", "L"]
        self.obstacle_type = ["L"]
        self.planning_done = False
        self.avoid_distance = 1.5
        self.path_distance = 1.25

    def state_cb(self, msg):
        self.current_state = msg

    # Callback handlers
    def handle_launch(self):
        self.STATE = 'LAUNCH'
        print('Launch Requested.')

    def handle_test(self):
        self.STATE = 'TEST'
        print('Test Requested.')

    def handle_land(self):
        self.STATE = 'LAND'
        print('Land Requested.')

    def handle_abort(self):
        self.STATE = 'ABORT'
        print('Abort Requested.')
    
    def handle_spin(self):
        self.STATE = 'SPIN'
        print('spin')

    # Service callbacks
    def callback_spin(self, request):
        self.handle_spin()
        return EmptyResponse()

    def callback_launch(self, request):
        self.handle_launch()
        return EmptyResponse()

    def callback_test(self, request):
        self.handle_test()
        return EmptyResponse()

    def callback_land(self, request):
        self.handle_land()
        return EmptyResponse()

    def callback_abort(self, request):
        self.handle_abort()
        return EmptyResponse()

    def callback_waypoints(self, msg):
        if self.WAYPOINTS_RECEIVED:
            return
        if self.T_odom_vicon is None:
            return 
        print('Waypoints Received')
        self.WAYPOINTS_RECEIVED = True
        self.WAYPOINTS = np.empty((0,3))
        self.WAYPOINT_ORIG = np.empty((0,3))
        for i, pose in enumerate(msg.poses):
            y_offset = 0
            """
            if i < 3:
                y_offset = -0.5 
            elif i == 3:
                y_offset = 0
            else: 
                y_offset = 0.2
            pos_h = np.array([pose.position.x - 0.3, pose.position.y + 0.3 + y_offset, pose.position.z, 1])
            pos_transformed = np.matmul(self.T_odom_vicon, pos_h.T) # Transform from vicon frame to (pixhawk)odom frame
            pos = pos_transformed[:-1].T
            print("Subwaypoint added: \n")
            print("In Vicon frame: ", pos_h[:-1])
            print("In odom frame: ", pos)
            self.WAYPOINTS = np.vstack((self.WAYPOINTS, pos))
            self.WAYPOINTS_ORIG = np.vstack((self.WAYPOINT_ORIG, pos_h[:-1]))
            """
            pos_h = np.array([pose.position.x, pose.position.y + y_offset, pose.position.z, 1])
            pos_transformed = np.matmul(self.T_odom_vicon, pos_h.T) # Transform from vicon frame to (pixhawk)odom frame
            pos = pos_transformed[:-1].T
            print("Waypoint added: \n")
            print("In Vicon frame: ", pos_h[:-1])
            print("In odom frame: ", pos)
            self.WAYPOINTS = np.vstack((self.WAYPOINTS, pos))
            self.WAYPOINTS_ORIG = np.vstack((self.WAYPOINT_ORIG, pos_h[:-1]))
        # self.WAYPOINTS[:,2] = 0.4
        #print('Waypoints:\n', self.WAYPOINTS)
        if self.waypoint_cnt == -1:
            self.waypoint_cnt = 0
            if self.use_vicon:
                self.update_waypoint(self.WAYPOINTS_ORIG[self.waypoint_cnt,:])
            else:
                self.current_waypoint = self.WAYPOINTS[self.waypoint_cnt,:]
                print("FIRST WAYPOINT SET")
            print(self.WAYPOINTS)
            print(self.current_waypoint)

    def callback_vicon(self, vicon_msg):
        if self.STATE == 'INIT':
            # Construct rotation matrix between vicon and drone frames
            R = quaternion_matrix(np.array([vicon_msg.transform.rotation.x, 
                                            vicon_msg.transform.rotation.y, 
                                            vicon_msg.transform.rotation.z,
                                            vicon_msg.transform.rotation.w 
                                            ]))
            t = vicon_msg.transform.translation
            self.T_vicon_odom = np.eye(4)
            self.T_vicon_odom[:3,:3] = R[:3,:3]
            self.T_vicon_odom[:3, 3] = np.array([t.x, t.y, t.z]).T
            self.T_odom_vicon = np.linalg.inv(self.T_vicon_odom)
        elif self.STATE != 'INIT' and self.use_vicon:
            # Construct rotation matrix between vicon and odom frames
            R = quaternion_matrix(np.array([vicon_msg.transform.rotation.x, 
                                            vicon_msg.transform.rotation.y, 
                                            vicon_msg.transform.rotation.z,
                                            vicon_msg.transform.rotation.w 
                                            ]))
            t = vicon_msg.transform.translation
            self.pose_vicon_drone = np.eye(4)
            self.pose_vicon_drone[:3,:3] = R[:3,:3]
            self.pose_vicon_drone[:3, 3] = np.array([t.x, t.y, t.z]).T
        R = quaternion_matrix(np.array([vicon_msg.transform.rotation.x,
                                        vicon_msg.transform.rotation.y,
                                        vicon_msg.transform.rotation.z,
                                        vicon_msg.transform.rotation.w
                                        ]))
        t = vicon_msg.transform.translation
        T_vicon_drone = np.eye(4)
        T_vicon_drone[:3,:3] = R[:3,:3]
        T_vicon_drone[:3, 3] = np.array([t.x, t.y, t.z]).T # translation of drone in vicon frame
        T_drone_odom = np.matmul(np.linalg.inv(T_vicon_drone), np.linalg.inv(self.T_odom_vicon))
        self.vicon_trans = np.linalg.inv(T_drone_odom)[:3,3]

    
    def callback_odom(self, odom_msg):
        self.current_pose = np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z])
        R = quaternion_matrix(np.array([odom_msg.pose.pose.orientation.x,
                                        odom_msg.pose.pose.orientation.y,
                                        odom_msg.pose.pose.orientation.z,
                                        odom_msg.pose.pose.orientation.w]))
        #print(R)
        t = odom_msg.pose.pose.position
        self.T_odom_drone = np.eye(4)
        self.T_odom_drone[:3,:3] = R[:3, :3]
        self.T_odom_drone[:3, 3] = np.array([t.x, t.y, t.z]).T
        q = np.array([odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w])

        euler_angles =  tf.transformations.euler_from_quaternion(q)
        self.yaw_angle = euler_angles[2]
    
    def callback_obstacles(self, obstacle_msg):
        if self.obstacles_received or not self.spin_done:
            pass
        else:
            self.obstacle_map = np.empty((0,2))
            self.obstacle_type = []
            self.obstacles_received = True

            print("Obstacles received")
            for i, pose in enumerate(obstacle_msg.poses):
                obs = np.array([pose.position.x, pose.position.y])

                # Obstacle Map in the Odom Frame
                self.obstacle_map = np.vstack((self.obstacle_map, obs))
                if (obs[0]*obs[1] < 0):
                    self.obstacle_type.append("R")
                else:
                    self.obstacle_type.append("L")

    def vicon_running(self, topic_name='/vicon/ROB498_Drone/ROB498_Drone'):
        # Get a list of tuples containing the names and data types of all the topics that are currently published
        published_topics = rospy.get_published_topics()
        # Check if the topic exists by searching for its name in the list of published topics
        if any(topic_name in topic for topic in published_topics):
            print("vicon available.")
            return True
        else:
            print("vicon not available.")
            return False

    def start_up(self):
        # Do not change the node name and service topics!
        rospy.init_node(self.name)

        self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)
        self.odom_sub = rospy.Subscriber("mavros/odometry/out", Odometry, callback=self.callback_odom)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
        if self.vicon_running():
            # Subscribe to the vicon topic /vicon/ROB498_Drone/ROB498_Drone
            self.vicon_sub = rospy.Subscriber('vicon/ROB498_Drone/ROB498_Drone', TransformStamped, self.callback_vicon)
        else:
            # Use data from the RealSense camera
            pass

        rospy.wait_for_service("/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode) 
        self.srv_launch = rospy.Service(self.name+'/comm/launch', Empty, self.callback_launch)
        self.srv_test = rospy.Service(self.name+'/comm/test', Empty, self.callback_test)
        self.srv_land = rospy.Service(self.name+'/comm/land', Empty, self.callback_land)
        self.srv_abort = rospy.Service(self.name+'/comm/abort', Empty, self.callback_abort)
        self.srv_spin = rospy.Service(self.name+'/comm/spin', Empty, self.callback_spin)

        self.sub_waypoints = rospy.Subscriber(self.name+'/comm/waypoints', PoseArray, self.callback_waypoints)
        self.sub_obstacles = rospy.Subscriber("/obstacles_map", PoseArray, self.callback_obstacles)

        return 

    def update_waypoint(self, waypoint):

        curr_waypoint_h = np.hstack((waypoint, 1))
        self.T_odom_vicon = np.matmul(self.T_odom_drone, self.linalg.inv(self.pose_vicon_drone))
        curr_waypoint_h = np.matmul(self.T_odom_vicon, curr_waypoint_h)
        self.current_waypoint = curr_waypoint_h[:-1]

    def perturb_from_waypoint(self):
        print("PERTURBING FROM WAYPOINT!!", )
        perturb_waypoints = self.current_waypoint + self.PERTURB_OFFSET
        perturb_ind = 0   
        rate = rospy.Rate(20)     
        start = rospy.Time.now() 
        while not rospy.is_shutdown() or (rospy.Time.now() - start) > rospy.Duration(5.0):
            self.pose.pose.position.x = perturb_waypoints[perturb_ind,0]
            self.pose.pose.position.y = perturb_waypoints[perturb_ind,1]
            self.pose.pose.position.z = perturb_waypoints[perturb_ind,2]
            print(np.linalg.norm(perturb_waypoints[perturb_ind, :] - self.current_pose))
            print(perturb_ind)
            if np.linalg.norm(perturb_waypoints[perturb_ind, :] - self.current_pose) < 0.1 and not self.PERTURB_FLAG[perturb_ind]:
                self.PERTURB_FLAG[perturb_ind] = True
                perturb_ind += 1
            
            if perturb_ind >= self.PERTURB_OFFSET.shape[0]:
                print("Reached perturb indices")
                break
            
            if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - self.last_req) > rospy.Duration(5.0)):
                if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")
                
                self.last_req = rospy.Time.now()
            else:
                if(not self.current_state.armed and (rospy.Time.now() - self.last_req) > rospy.Duration(5.0)):
                    if(self.arming_client.call(self.arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")
            
                    self.last_req = rospy.Time.now()
            #print("Comm node: Publishing pose...")
            #print(self.pose)
            self.local_pos_pub.publish(self.pose)
            rate.sleep()
        print("Done Perturbing")
        self.PERTURB_FLAG = np.full((self.PERTURB_OFFSET.shape[0], ), False)
        return 

    def run_planning(self):
        print(self.obstacles_received, self.WAYPOINTS_RECEIVED)
        if not self.obstacles_received or not self.WAYPOINTS_RECEIVED: 
            return False 
        
        curr_pos = self.current_pose
        # print(self.obstacle_map)
        # print(self.WAYPOINTS)

        for i, way_pt in enumerate(self.WAYPOINTS):

            # Get the waypoint relative to the "current" position
            way_pt = way_pt[:2] - curr_pos[:2]

            # Get unit vector for the waypoint
            way_pt_mag = np.linalg.norm(way_pt)
            way_pt_norm = way_pt/way_pt_mag
            # print("way_pt_norm: ", way_pt_norm)
            # Get the obstacle distances to the path
            obs_map_rel = self.obstacle_map - curr_pos[:2] 
            # print("obs rel: ", obs_map_rel)
            obs_projs = np.expand_dims((np.sum(obs_map_rel*np.vstack([way_pt]*obs_map_rel.shape[0]), axis=1)/(way_pt_mag**2)), axis=-1)*way_pt
            # print("projs: ", obs_projs)
            obs_projs_mags = np.linalg.norm(obs_projs, axis=1) 
            obs_dist_vecs = obs_map_rel - obs_projs
            obs_dists = np.linalg.norm(obs_dist_vecs, axis=1)
            # print("dist: ", obs_dists)

            # Get obstacles that are along the path
            obs_mask = np.logical_and(np.squeeze(obs_projs_mags > 0), np.squeeze(obs_projs_mags < way_pt_mag))
            obs_mask = np.logical_and(obs_mask, np.squeeze(obs_dists < self.path_distance))

            for j, obs in enumerate(self.obstacle_map):
                on_right = None
                if obs_mask[j] == True: 
                    curr_obs_dist_vec = obs_dist_vecs[j]
                    if obs_dists[j] == 0:
                        # I dont know if you want to have different code for the if and else
                        # right now they are the same. You swap the x and y components of the
                        # vector, negate the second component, and then normalize it. This 
                        # should give you a vector that is perpendicular to the way_pt.
                        if way_pt[0] > way_pt[1]:
                            # curr_obs_dist_unit = np.asarray([0, 1])
                            curr_obs_dist_unit = np.array([-way_pt[1], way_pt[0]])
                            curr_obs_dist_unit = curr_obs_dist_unit / np.linalg.norm(curr_obs_dist_unit)  
                        else:
                            # curr_obs_dist_unit = np.asarray([1, 0])
                            curr_obs_dist_unit = np.array([-way_pt[1], way_pt[0]])
                            curr_obs_dist_unit = curr_obs_dist_unit / np.linalg.norm(curr_obs_dist_unit) 
                    else:
                        curr_obs_dist_unit = curr_obs_dist_vec/obs_dists[j]
                    if np.cross(np.hstack((way_pt, np.asarray([1]))), np.hstack((obs_map_rel[j], np.asarray([1]))))[2] > 0:
                        # print("to the right")
                        on_right = True
                    else: 
                        # print("to the left")
                        on_right = True
                    AVOID_WAYPOINTS = np.empty((0,3))
                    if (self.obstacle_type[j] == "R" and on_right) or (self.obstacle_type[j] == "L" and not on_right): 
                        dir_sign = -1 
                    else: 
                        dir_sign = 1
                    
                    # compute additional waypoints
                    # account for the distance from the waypoint line to the obstacle
                    # print("testing:", curr_pos[:2], dir_sign, curr_obs_dist_unit, self.avoid_distance)
                    wp_1 = curr_pos[:2] + dir_sign*curr_obs_dist_unit*(self.avoid_distance + dir_sign*obs_dists[j])   
                    wp_1_full = np.hstack((wp_1, np.asarray([self.WAYPOINTS[i, 2]])))
                    wp_2 = wp_1 + way_pt
                    wp_2_full = np.hstack((wp_2, np.asarray([self.WAYPOINTS[i, 2]])))
                    AVOID_WAYPOINTS = np.vstack((wp_1_full, wp_2_full))
                    # print("Adding waypoints to avoid obstacle!")
                    # print("Obstacle: ", obs)
                    # print("Added Waypoints: ", np.asarray([[wp_1], [wp_2]]))                   
                    # add additional waypoints into self.WAYPOINTS
                    # print(self.WAYPOINTS.shape)
                    # print(AVOID_WAYPOINTS)
                    if self.WAYPOINTS_AVOID is None:
                        self.WAYPOINTS_AVOID = AVOID_WAYPOINTS
                    else:
                        self.WAYPOINTS_AVOID = np.vstack([self.WAYPOINTS_AVOID, AVOID_WAYPOINTS])
            waypoint_dip = self.WAYPOINTS[i]
            waypoint_dip[2] = self.WAYPOINTS[i, 2] + 0.2
            if self.WAYPOINTS_AVOID is None:
                self.WAYPOINTS_AVOID = np.vstack([self.WAYPOINTS[i], waypoint_dip]) 
            else:
                self.WAYPOINTS_AVOID = np.vstack([self.WAYPOINTS_AVOID, self.WAYPOINTS[i], waypoint_dip])
            curr_pos = self.WAYPOINTS[i]

        print("DONE PLANNING")
        self.planning_done = True 
        self.WAYPOINTS = self.WAYPOINTS_AVOID
        self.num_waypoints = self.WAYPOINTS.shape[0]
        print("All waypoints: ", self.WAYPOINTS)

    # Main node
    def comm_node(self):
        self.start_up()

        # Setpoint publishing MUST be fasterTrue than 2Hz
        rate = rospy.Rate(20)

        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.current_state.connected):
            rate.sleep()

        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 0

        # Send a few setpoints before starting
        for i in range(100):   
            if(rospy.is_shutdown()):
                break

            self.local_pos_pub.publish(self.pose)
            rate.sleep()

        self.offb_set_mode = SetModeRequest()
        self.offb_set_mode.custom_mode = 'OFFBOARD'

        self.arm_cmd = CommandBoolRequest()
        self.arm_cmd.value = True

        self.last_req = rospy.Time.now()

        print('This is the ROB498 Challenge 3 drone waypoint following node')
        while not rospy.is_shutdown():
            # Your code goes here
            if self.STATE == 'INIT':
                #print('Comm node: Initializing...')
                self.pose.pose.position.z = 0
            if self.STATE == 'LAUNCH':
                #print('Comm node: Launching...')
                self.pose.pose.position.z = 0.5
                self.pose.pose.position.x = 0.0
                self.pose.pose.position.y = 0.0
                #self.current_waypoint = np.asarray([0,0,0.4])
                #self.perturb_from_waypoint()
            if self.STATE == 'SPIN':

                if not self.spin_done:

                    q = tf.transformations.quaternion_from_euler(0, 0, self.spin_angles[self.spin_angle_count])
                    print("Current spin angle: ", self.spin_angles[self.spin_angle_count])
                    print("At this angle: ", self.yaw_angle)


                    if np.abs(self.spin_angles[self.spin_angle_count] - self.yaw_angle) < np.pi/20:
                        print("next small increment")
                        # Publish the pose for 5 seconds
                        if len(self.stopping_angles) > 0:
                            if np.abs(self.yaw_angle - self.stopping_angles[0]) < np.pi/20:
                                print("Reaching recording point {0}".format(4 - len(self.stopping_angles)))
                                start = rospy.Time.now()
                                while not rospy.is_shutdown() and (rospy.Time.now() - start) < rospy.Duration(8.0):
                                    self.local_pos_pub.publish(self.pose)
                                    rate.sleep()
                                print("Leaving recording point")
                                self.stopping_angles = self.stopping_angles[1:]
                        
                        # Increment the small spin angle count
                        self.spin_angle_count += 1
                        if self.spin_angle_count >= len(self.spin_angles):
                            self.spin_done = True
                            print("SPIN DONE")

                    self.pose.pose.position.z = 0.5
                    self.pose.pose.position.x = 0.0
                    self.pose.pose.position.y = 0.0
                    self.pose.pose.orientation.x = q[0]
                    self.pose.pose.orientation.y = q[1]
                    self.pose.pose.orientation.z = q[2]
                    self.pose.pose.orientation.w = q[3]

                if self.spin_done: 
                    # Run planning given the static map 
                    if not self.planning_done: 
                        print("RUN PLANNING")
                        self.run_planning()
                        
                    else:
                        print("Spin done, waiting for test...")
                        self.local_pos_pub.publish(self.pose)

            elif (self.STATE == 'TEST' or (self.STATE == 'LAND' and self.waypoint_cnt < self.num_waypoints-1)):
                #print('Comm node: Testing...')
                if self.use_vicon:
                    self.update_waypoint(self.WAYPOINTS_ORIG[self.waypoint_cnt,:])
                
                if self.waypoint_cnt >= self.num_waypoints:
                    self.STATE = 'LAND'
                    break

                # Set pose to current goal waypoint
                if self.current_waypoint.shape[0] != 0: 
                    #print(self.WAYPOINTS, self.current_waypoint)
                    self.pose.pose.position.x = self.current_waypoint[0]
                    self.pose.pose.position.y = self.current_waypoint[1]
                    self.pose.pose.position.z = self.current_waypoint[2]

                    # Check if we are within the 15 cm sphere of waypoint 
                    #print(self.waypoint_cnt, self.current_waypsoint, self.current_pose)
                    #print("Current norm:", np.linalg.norm(self.current_waypoint - self.current_pose))

                    if np.linalg.norm(self.current_waypoint - self.current_pose) < 0.1 and not self.WAYPOINT_FLAG[self.waypoint_cnt]:
                        print("ARRIVED AT WAYPOINT")
                        print("Waypoint count, current waypoint, current pose:", self.waypoint_cnt, self.current_waypoint, self.current_pose)
                        print("ARRIVED WITHIN NORM:", np.linalg.norm(self.current_waypoint - self.current_pose))
                        print("Current pose and vicon_trans", self.current_pose, self.vicon_trans)
                        print("Error btwn vicon and odom: ", np.linalg.norm(self.current_pose - self.vicon_trans))
                        # start_time = rospy.Time.now()

                        #if self.waypoint_cnt %2 == 1 and self.waypoint_cnt > 2:
                            #self.perturb_from_waypoint()
                        
                        self.WAYPOINT_FLAG[self.waypoint_cnt] = True 
                            
                        print("WAYPOINT REACHED, MOVING TO NEXT WAYPOINT")
                        # Increment waypoint counter and waypoint                         
                        self.waypoint_cnt += 1
                        if self.use_vicon:
                            self.current_waypoint = self.WAYPOINTS_ORIG[self.waypoint_cnt, :]
                        else:
                            self.current_waypoint = self.WAYPOINTS[self.waypoint_cnt, :]
            elif self.STATE == 'LAND' or self.waypoint_cnt > self.num_waypoints-1:
                print('Comm node: Landing...')
                self.pose.pose.position.z = 0
            
            elif self.STATE == 'ABORT':
                print('Comm node: Aborting...')
                self.pose.pose.position.z = 0
            
            if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - self.last_req) > rospy.Duration(5.0)):
                if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")
                
                self.last_req = rospy.Time.now()
            else:
                if(not self.current_state.armed and (rospy.Time.now() - self.last_req) > rospy.Duration(5.0)):
                    if(self.arming_client.call(self.arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")
            
                    self.last_req = rospy.Time.now()
            #print("Comm node: Publishing pose...")
            #print(self.pose)
            self.local_pos_pub.publish(self.pose)
            rate.sleep()

if __name__ == "__main__":
    challenge = ChallengeTask3()
    challenge.comm_node()
