#!/usr/bin/env python3      
import rospy
import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
import time

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped, Pose
from cv_bridge import CvBridge
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from ar_track_alvar_msgs.msg import AlvarMarkers
from visualization_msgs.msg import Marker
from geometry_msgs.msg  import Vector3
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import tf.transformations as tf

from robot import *


# init_time = time.time() 
def callBack(image_data, marker_data, info_data):
    #########################################
    global loop_index, w, THETA, rList, arlist, init_time, m, v
    # print("Time:", time.time() - init_time)
    loop_index = loop_index + 1
    t = rospy.get_time()
    # print(t)
    #########################################
    # start_time = time.time()
    ######################################### Read the imagefrom ROS topic and convert it to opencv using a bridge  
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_data, desired_encoding = 'passthrough')
    ######################################### Initializing a binary image using the centers
    imsize = cv_image.shape
    # print(imsize)
    # binary_image_to_use = np.zeros((imsize[0], imsize[1]), np.float64)
    ######################################### Extract the cam_intrinsics from the ros message
    #cam_intrinsics (a 1D tuple)
    intrinsics = info_data.K 
    fx = intrinsics[0]
    fy = intrinsics[4]
    px = intrinsics[2]
    py = intrinsics[5]
    ########################################## Extract the marker ID from /ar_pose_marker topic
    position = {}
    pixel_coordinates_for_cv = {}
    ########################################## Circles Specifications
    # radius = 5 # Radius of the circle
    # color = {1: (0, 255, 0), 2: (255, 0, 0), 3: (0, 0, 255), 4: (0, 127, 127)} # Color in BGR format
    # thickness = 2  # Thickness of the circle outline
    ########################################## Extract the marker id and center postion
    Norm_Cords_Mat = np.zeros((4,2))
    # binary_image = np.zeros((imsize[0], imsize[1], 3), np.uint8)
    Lp = np.zeros((8,6))
    e_feat = np.zeros((8,1))
    # desired normalized u and v arrange as 8 by 1 matrix (u1 v1 u2 v2 u3 v3 u4 v4)
    des_feat = np.array([[0.1],
                         [0.1],
                         [-0.1],
                         [0.1],
                         [-0.1],
                         [0.1],
                         [-0.1], 
                         [-0.1]])
    id = 1000
    for i in range(0,4):
        try:
            marker_info = marker_data.markers[i].pose
            id = marker_data.markers[i].id
        except:
            print(f'The features with ID = {id} is lost.')
            
        # The poistition of the markers in the camera frame
        xi = marker_info.pose.position.x
        yi = marker_info.pose.position.y
        zi = marker_info.pose.position.z
    
        
        xnorm = (xi/zi) 
        ynorm = (yi/zi) 
        norm_cor = np.array([[xnorm],[ynorm]])
    
        # Norm_Cords_Mat[id-1,:] = np.squeeze(norm_cor, axis=1)
        # print(xnorm, ynorm)
        
        if i == 0:
            Lp[0:2,:] = np.array([[-1/zi,  0, xnorm/zi, xnorm*ynorm, -(1+xnorm**2), ynorm],
                                  [0, -1/zi, ynorm/zi, (1+ynorm**2), -xnorm*ynorm, -xnorm]])
            e_feat[0:2,:] = des_feat[0:2,:] - norm_cor
            
        if i == 1:
            Lp[2:4,:] = np.array([[-1/zi,  0, xnorm/zi, xnorm*ynorm, -(1+xnorm**2), ynorm],
                                  [0, -1/zi, ynorm/zi, (1+ynorm**2), -xnorm*ynorm, -xnorm]])
            e_feat[2:4,:] =   des_feat[2:4,:] - norm_cor
        
        if i == 2:
            Lp[4:6,:] = np.array([[-1/zi,  0, xnorm/zi, xnorm*ynorm, -(1+xnorm**2), ynorm],
                                  [0, -1/zi, ynorm/zi, (1+ynorm**2), -xnorm*ynorm, -xnorm]])
            e_feat[4:6,:] = des_feat[4:6,:] - norm_cor
            
        if i == 3:
            Lp[6:8,:] = np.array([[-1/zi,  0, xnorm/zi, xnorm*ynorm, -(1+xnorm**2), ynorm],
                                  [0, -1/zi, ynorm/zi, (1+ynorm**2), -xnorm*ynorm, -xnorm]])
            e_feat[6:8,:] =  des_feat[6:8,:] - norm_cor
      
    print(f'{loop_index} Running ...')    

    ###################################################
    global feature_error_publisher
    ferror = Vector3()
    ferror.x = e_feat[0,0]
    ferror.y = e_feat[1,0]
    ferror.z = 0
    
    print("error vector:", ({e_feat[0,0]},{e_feat[1,0]}))
    feature_error_publisher.publish(ferror)
    
    Error_List.append(e_feat)
    t = rospy.get_time()
    Time_List.append(t)
    dt = Time_List[loop_index] - Time_List[loop_index-1]
    error_integral = integrator(Error_List, dt)
    
    
    # theta, flag = connecting_line(Norm_Cords_Mat[0:2,:])

    # if flag == True:
    #     theta = THETA[loop_index-10]
        
    # THETA.append(theta)   
     
    print(f'{loop_index} Running ...')    
    
    
    # qa = theta 
    # qa_des = 0
    # error_qa = qa_des - qa
    # ua = yaw_controller(error_qa)
    
    
    
    lam = 0.5
    u_f = lam * Lp.transpose() @ e_feat
    # u_f = -lam * np.linalg.pinv(Lp) @ e_feat
    # compen = Lp[:,3] * ua
    # compen = np.expand_dims(compen, axis=1)
    # u_f = lam * Lp.transpose()  @ (e_feat - compen)
    # u_f = lam * Lp.transpose() @ e_feat + 0.01 * Lp.transpose() @ error_integral
    # u_f = lam * Lp.transpose() @ e_feat + 0.1 * Lp.transpose() @ (np.tanh(10*e_feat/np.abs(e_feat)))
    
    # this is the twist vector expressed in the camera frame but we need to transform it to the uav frame
    # it is also possible to transform it to the global frame.
    # uf is expressed in the camera frame
    # AdT --> the adjoint transformation from the cam to uav frame.
    
    # print("Relative Pose:",{rel_pose_data})
    RRR = np.array([[ 0, 0, 1],
                    [-1,  0, 0],
                    [ 0,  -1, 0]])
    
    ttt = np.array([[0],[0],[-0.1]])    
    AdT = adjoint_matrix(RRR,ttt)
    

    # AdT = np.array([[0, -1,  0,  0,  0,  0],
    #                 [-1,  0,  0,  0,  0,  0],
    
    #                 [ 0,  0, -1,  0,  0,  0],
    #                 [ 0,  0,  0,  0, -1,  0],
    #                 [ 0,  0,  0, -1,  0,  0],
    #                 [ 0,  0,  0,  0,  0, -1]])
    # u_F = np.array([u_f[0,0],u_f[1,0],u_f[2,0],0,0,u_f[3,0]])  # in camera coordinates [vx vy vz 0 0 wz]
    
    u_f = AdT @ u_f# twist expressed in the uav(body) frame
    

    
    # ---------------------------------------------------------------------
    
    # yaw_feature_pub.publish(error_qa)
    
    
    # print(u_f)
    ######################################### Publish the velocity command to the UAV
    global pub
    twist = Twist()
    twist.linear.x  = u_f[0]
    twist.linear.y  = u_f[1]
    twist.linear.z  = u_f[2]
    twist.angular.z = u_f[5]

    # twist.angular.z = ua
    pub.publish(twist)
    print(twist)

    ########################################### end of the main callback











########################################### Perception Blocks 
########################################### End of Perception Blocks 
########################################### End of Perception Blocks 
########################################### End of Perception Blocks 


########################################### Control Block
########################################### Control Block 
########################################### Control Block 



def skew(x):
    sk = np.array([[0, -x[2,0], x[1,0]],[x[2,0], 0, -x[0,0]], [-x[1,0], x[0,0], 0]])
    return sk



##################################################################################### End of Neural Network Blocks


import math
# def quat2eulers(q0:float, q1:float, q2:float, q3:float) -> tuple:
#     """
#     Compute yaw-pitch-roll Euler angles from a quaternion.
    
#     Args
#     ----
#         q0: Scalar component of quaternion.
#         q1, q2, q3: Vector components of quaternion.
    
#     Returns
#     -------
#         (roll, pitch, yaw) (tuple): 321 Euler angles in radians
#     """
#     roll = math.atan2(
#         2 * ((q2 * q3) + (q0 * q1)),
#         q0**2 - q1**2 - q2**2 + q3**2
#     )  # radians
#     pitch = math.asin(2 * ((q1 * q3) - (q0 * q2)))
#     yaw = math.atan2(
#         2 * ((q1 * q2) + (q0 * q3)),
#         q0**2 + q1**2 - q2**2 - q3**2
#     )
#     return (roll, pitch, yaw)

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return (roll_x, pitch_y, yaw_z) # in radians
########################################### Control Block
########################################### Control Block 
########################################### Control Block 
def adjoint_matrix(R,t):
    
    # Extract rotation matrix (3x3) and translation vector
    tcross = skew(t)
    # Create the first row block
    AdT_r1 = np.hstack((R, np.zeros((3, 3))))
    # Create the second row block
    AdT_r2 = np.hstack((tcross @ R, R))
    # Stack the blocks vertically to form a 6x6 matrix
    AdT = np.vstack((AdT_r1, AdT_r2))
    
    return AdT

def integrator(xhist, dt):
    xhist = np.array(xhist)
    
    xint = np.sum(xhist, axis = 0) * dt
    return xint

def connecting_line(normalized_cord):
    flag = False
    try:
        x1, y1 = normalized_cord[0,:]
        x2, y2 = normalized_cord[1,:]
        theta = np.arctan2((y2 - y1), (x2 - x1))
        theta = theta % (2*np.pi)
    except:
        theta = 0
        flag = True
    return theta, flag  

def yaw_controller(e):
    eta = 0.05
    # kd = 0#control gain
    # s =  e + kd * edot 
    # u = eta * np.sqrt(np.abs(s)) * np.sign(s)
    u = eta*e
    return u

###################################################################################### ROS Network
if __name__ == '__main__':
    
    global loop_index, Error_List, Contrl_Input_List, Time_List, Yaw_Error_List, w, THETA, rList, yaw_control_input_list, position_mat, orientaion_mat
    global pose_time, arlist
    global m, v
    ##############################################
    loop_index = 0
    Error_List = [np.zeros((8,1))]
    Time_List = [0]
    THETA = []


    rospy.init_node('sepahvand_node', anonymous=True)
    point_publisher = rospy.Publisher("image_point", Marker, queue_size=1)
    
    image_sub = message_filters.Subscriber('bebop/image_raw', Image)
    marker_sub= message_filters.Subscriber('ar_pose_marker', AlvarMarkers)
    camera_info_sub = message_filters.Subscriber('/bebop/camera_info', CameraInfo) 
    
    rel_pose_sub = message_filters.Subscriber('/cam_pose_in_uav_frame', PoseStamped) 
    
    
    ts = message_filters.TimeSynchronizer([image_sub, marker_sub, camera_info_sub], 1, 1)
    ts.registerCallback(callBack)
    
    # uav_pose_info = rospy.Subscriber('/bebop/odom', Odometry, callback = pose_callback) 
    
    
    feature_error_publisher = rospy.Publisher("feature_error", Vector3, queue_size=1)
    # yaw_feature_pub = rospy.Publisher("qa", Float32, queue_size=1)
    # norm_one_ferror = rospy.Publisher("norm", Float32, queue_size=1)
    pub = rospy.Publisher("bebop/cmd_vel", Twist, queue_size = 1)
    
    rospy.spin()
######################################################################################End of ROS Network