#!/usr/bin/env python3      
import rospy
import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
import time

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from ar_track_alvar_msgs.msg import AlvarMarkers
from visualization_msgs.msg import Marker
from geometry_msgs.msg  import Vector3
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

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
    marker_pos_in_img = np.zeros((4,2))
    marker_pos_for_moment = np.zeros((4,2))
    # binary_image = np.zeros((imsize[0], imsize[1], 3), np.uint8)
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
        
        
        # (451, 329)

        position[id] = (xi, yi, zi)
        xc = (fx*xi/zi) + px
        yc = (fy*yi/zi) + py
        pixel_coordinates_for_cv[id] = (int(xc), int(yc))
        ########################################
        marker_pos_in_img[i,:] = np.array([xc, yc])  # the xc and yc are reported in the frame with z downward
        # however, our coordinate frame's z axis in upward
        marker_pos_for_moment[i,:] = np.array([yc, xc]) 
        arlist.append(marker_pos_in_img)
        ########################################
        # image_with_marker = cv.circle(cv_image, pixel_coordinates_for_cv[id], radius, color[id], thickness)
        # color2 = (255,255,255); thickness2 = 0; radius2 = 0
        # binary_image = cv.circle(binary_image, pixel_coordinates_for_cv[id], radius2, color2, thickness2)
        # gray = cv.cvtColor(binary_image, cv.COLOR_BGR2GRAY)

        # # apply thresholding to convert grayscale to binary image
        # ret,thresh = cv.threshold(gray,128,128,128)
        

     
    ########################################### Display the markers along with the image
    # Find the indices of non-zero elements
    
    # nonzero_indices = np.nonzero(thresh)
    # # Print the indices
    # print("Indices of non-zero elements:")
    # for index in zip(nonzero_indices[0], nonzero_indices[1]):
    #     print(index)
        
    # combined_image = cv.hconcat([image_with_marker, binary_image])
    # # Display the combined image
    # # cv.imshow('Two Images', combined_image)
    # cv.imshow('Two Images', thresh)
    # cv.waitKey(0) 
    # cv.destroyAllWindows()
    ########################################### Create the secondary image
    half_of_img_hight = 240
    half_of_img_width = 428
    m00 = moments_c(marker_pos_for_moment, 0, 0, half_of_img_hight, half_of_img_width)
    m01 = moments_c(marker_pos_for_moment, 0, 1, half_of_img_hight, half_of_img_width)
    m10 = moments_c(marker_pos_for_moment, 1, 0, half_of_img_hight, half_of_img_width)
    m00 = moments_c(marker_pos_for_moment, 0, 0, half_of_img_hight, half_of_img_width)
    m01 = moments_c(marker_pos_for_moment, 0, 1, half_of_img_hight, half_of_img_width)
    m10 = moments_c(marker_pos_for_moment, 1, 0, half_of_img_hight, half_of_img_width)
    pg = m01/m00
    qg = m10/m00
    mu20 = moments_c2(marker_pos_for_moment, 2, 0, pg, qg, half_of_img_hight, half_of_img_width) 
    mu02 = moments_c2(marker_pos_for_moment, 0, 2, pg, qg, half_of_img_hight, half_of_img_width) 
    ###################################################################
    alpha = mu20 + mu02
    alphad = 30000
    qz = np.sqrt (alphad/(alpha))
    qx = (pg*qz)/fx
    qy = (qg*qz)/fy
    
    print(f'states:,{qx,qy,qz}')
    #euler_angles = quat2eulers(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    #yaw = euler_angles[0]
    states = np.array([qx, qy, qz])
    des_state = np.array([0, 0, 1.5])
    error = des_state - states
    theta, flag = connecting_line(pixel_coordinates_for_cv)
    
    if flag == True:
        theta = THETA[loop_index-10]
        
    print(f'{loop_index} Running ...')    
    THETA.append(theta)
    
    qa = theta
    qa_des = np.pi
    error_qa = qa_des - qa
    Yaw_Error_List.append(error_qa)
    Error_List.append(error)

    Time_List.append(t)
    dt = Time_List[loop_index] - Time_List[loop_index-1]
    error_dot = derivative(Error_List[loop_index], Error_List[loop_index-1], dt)
    error_qa_dot = derivative(Yaw_Error_List[loop_index], Yaw_Error_List[loop_index-1], dt)

    error_integral = integrator(Error_List, dt)
    
    r = r_calculator(error, error_dot)
    rList.append(r)
    ri = integrator(rList, dt)
    
    ################################################### Neural Network Predition
    r = r_calculator(error, error_dot)    
    nn_output, w, m, v = neural_net(error, w, m, v, dt)     
    ###################################################
    ua = yaw_controller(error_qa, error_qa_dot)
    u = controller(error, error_dot, error_integral, ua, nn_output)
    Contrl_Input_List.append(u) 
    yaw_control_input_list.append(ua)
    ###################################################
    global feature_error_publisher
    ferror = Vector3()
    ferror.x = error[0]
    ferror.y = error[1]
    ferror.z = error[2]
    feature_error_publisher.publish(ferror)
    yaw_feature_pub.publish(error_qa)
    ######################################### Publish the fist norm of the feature error
    ferror_all = np.append(error, error_qa)
    first_norm = np.sum(np.abs(ferror_all))
    norm_one_ferror.publish(first_norm)
    ######################################### Publish the velocity command to the UAV
    global pub
    twist = Twist()
    twist.linear.x  = u[0]
    twist.linear.y  = u[1]
    twist.linear.z  = u[2]
    twist.angular.z = ua
    pub.publish(twist)
    ########################################### Save the Data for plotting
    # Error_List_np = np.array(Error_List)
    # Time_List_np = np.array(Time_List)
    # Yaw_List_np = np.array(Yaw_Error_List)
    # Contrl_Input_np = np.array(Contrl_Input_List)
    # yaw_control_input_np = np.array(yaw_control_input_list)
    # arlistmat = np.stack(arrays = arlist, axis = 2)
    
    
    # path = '/home/shayan/shayan_ros_/src/be bop_simulator/bebop_gazebo/src/'
    
    # np.save(file = path + 'ferror', arr = Error_List_np)
    # np.save(file = path + 'time', arr = Time_List_np)
    # np.save(file = path + 'yawerror', arr = Yaw_List_np)
    # np.save(file = path + 'controlinput', arr = Contrl_Input_np)
    # np.save(file = path + 'yawcontrolinput', arr = yaw_control_input_np)
    # np.save(file = path + 'arlistmat', arr = arlistmat)
    
    
    
    
    
    
    
    
    
    ########################################### end of the main callback





    ########################################### beginning of the pose callback
def pose_callback(data):
    # global uav_pose_info, position_mat, pose_time, orientaion_mat
    # uav_pose_info = data
    # xb = uav_pose_info.pose.pose.position.x
    # yb = uav_pose_info.pose.pose.position.y
    # zb = uav_pose_info.pose.pose.position.z
    
    # q1 = uav_pose_info.pose.pose.orientation.x
    # q2 = uav_pose_info.pose.pose.orientation.y
    # q3 = uav_pose_info.pose.pose.orientation.z
    # q4 = uav_pose_info.pose.pose.orientation.w
    
    # tb = rospy.get_time()
    # pose_time.append(tb)
    # #print(tb)
    # path = '/home/shayan/shayan_ros_/src/bebop_simulator/bebop_gazebo/src/'
    
    # uav_postition = np.array([[xb, yb, zb]])
    
    
    # [roll, pitch, yaw] = euler_from_quaternion(q1, q2, q3, q4)
    
    # uav_orientation = np.array([[roll, pitch, yaw]])
    
    # position_mat = np.vstack(tup = (position_mat, uav_postition))
    # orientaion_mat = np.vstack(tup = (orientaion_mat, uav_orientation))
    
    
    # np.save(file = path + 'position', arr = position_mat)
    # np.save(file = path + 'orientaion', arr = orientaion_mat)
    # np.save(file = path + 'pose_time', arr = np.array(pose_time))
    
    # listener = tf.TransformListener()
    # listener.waitForTransform('/ar_marker_1', '/odom', rospy.Time(), rospy.Duration(4.0))
    # try:
    #     # Get the transform
    #     (trans, rot) = listener.lookupTransform('/ar_marker_1', '/odom', rospy.Time(0))

    #     # Print the translation and rotation
    #     print("Translation:", trans)
    #     #print("Rotation:", rot)
    
    # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #     print("Error occurred while looking up the transform.")
    return 0
    ########################################### end of the pose callback










########################################### Perception Blocks 
def moments_c(marker_center_mat, p, q, half_image_hight, half_image_width):
    mpq = 0
    image_center = np.array([half_image_hight, half_image_width])
    marker_center_mat = marker_center_mat - np.tile(image_center, reps=(4,1))
    for i in range(0,4):
        temp = (marker_center_mat[i,0]**q) * (marker_center_mat[i,1]**p)
        mpq = mpq + temp
    return mpq    

def moments_c2(marker_center_mat, p, q, pg, qg, half_image_hight, half_image_width):
    mpq = 0
    center_of_gravity = np.array([pg, qg])
    # first, transform center of gravity to the pixel coordinates
    center_of_gravity = center_of_gravity + np.array([half_image_hight, half_image_width])
    marker_center_mat = marker_center_mat - np.tile(center_of_gravity, reps=(4,1))
    for i in range(0,4):
        temp = (marker_center_mat[i,0]**q) * (marker_center_mat[i,1]**p)
        mpq = mpq + temp
    return mpq   

def connecting_line(pixel_coordinates):
    flag = False
    try:
        x1, y1 = pixel_coordinates[1]
        x2, y2 = pixel_coordinates[2]
        theta = np.arctan2((y2 - y1), (x2 - x1))
        theta = theta % (2*np.pi)
    except:
        theta = 0
        flag = True
    return theta, flag   

def derivative(xnew, xold, dt):
    xdot = (xnew - xold)/dt
    return xdot

def integrator(xhist, dt):
    xhist = np.array(xhist)
    
    xint = np.sum(xhist, axis = 0) * dt
    return xint
########################################### End of Perception Blocks 
########################################### End of Perception Blocks 
########################################### End of Perception Blocks 


########################################### Control Block
########################################### Control Block 
########################################### Control Block 


    
def controller(e, edot, ri, yawdot, NN_term):
    Kr = -np.diag([0.9, 0.9, 1]) #control gain
    Ki = -np.diag([1e-1, 1e-1, 1e-3]) # should be very small
    gamma =  np.diag([0.5, 0.5, 0.5])
    r = gamma @ e + np.diag([0.01, 0.01, 0.01]) @ edot
    e3 = np.array([[0],[0],[1]])
    mu1 = skew(yawdot * e3)
    mu2 = np.zeros((3,3))
    NN_terms = NN_term[:,0]
    NN_terms[2] = 0
    u = -(gamma - 2*mu1) @ (r - gamma @ e) + (mu2 @ e) - (Kr @ r) - (Ki @ ri) + 0*NN_terms
    return u


# def controller(e, edot, ei, yawdot, NN_term):
#     s = np.diag([0.5, 0.5, 0.5]) @ e + np.diag([0.08, 0.08, 0.08]) @ edot +  np.diag([0.1, 0.1, 0.1]) @ ei
#     u =  abs(s) * np.tanh(s)
#     return u


def yaw_controller(e, edot):
    eta = 0.5
    # kd = 0#control gain
    # s =  e + kd * edot 
    # u = eta * np.sqrt(np.abs(s)) * np.sign(s)
    u = eta*e
    return u



def skew(x):
    sk = np.array([[0, -x[2,0], x[1,0]],[x[2,0], 0, -x[0,0]], [-x[1,0], x[0,0], 0]])
    return sk


def r_calculator(e , edot):
    gamma =  np.diag([0.5, 0.5, 0.5])
    r = gamma @ e + edot
    return r



def NormalizeData(data):
    a =  (data - np.min(data)) / (np.max(data) - np.min(data))
    return a
##################################################################################### Neural Network Blocks

def neural_net(r, w_old, m_old, v_old, dt):
    r = (r - np.min(r)) / (np.max(r) - np.min(r))
    #r = (2*r) - 1
    m = m_old
    v = np.array([0.5,  0.5, 0.5])
    # k = 0.01
    # lam = 1e-2
    k = 5#0.3
    lam = 1e-5#1e-2
    h = np.array([ 
                  [expon_mf(r[0], m[0], v[0])], 
                  [expon_mf(r[1], m[1], v[1])],
                  [expon_mf(r[2], m[2], v[2])]
                  ])
    
    hm = np.diag([ 
                  [wavelet_der(r[0], m[0], v[0]), 0, 0], 
                  [0, wavelet_der(r[1], m[1], v[1]),0],
                  [0, 0, wavelet_der(r[2], m[2], v[2])],
                  ])
    
    hv = np.diag([ 
                [wavelet_der_v(r[0], m[0], v[0]), 0, 0], 
                [0, wavelet_der_v(r[1], m[1], v[1]),0],
                [0, 0, wavelet_der_v(r[2], m[2], v[2])],
                ])
        
    mdot = lam *(r.T @ w_old @ hm)
    vdot = lam *(r.T @ w_old @ hv)
    wdot = lam * (h * r.T - k * np.linalg.norm(r) * w_old)
    
    w_new = w_old + wdot * dt
    m_new = m_old + mdot * dt
    v_new = v_old + vdot * dt
    nn_output = w_new.T @ h
    return nn_output, w_new, m_new, v_new
   
    ##
    
    

       
    
def expon_mf(x, m, v):
    y = np.exp(-0.5*(x-m)**2/v**2)
    return y


def wavelet_der(x, m, v):
    y = -((x-m)/v) * np.exp(-0.5*(x-m)**2/v**2)
    return y

def wavelet_der_v(x, m, v):
    y = -((x-m)**2/v**3) * np.exp(-0.5*(x-m)**2/v**2)
    return y
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


###################################################################################### ROS Network
if __name__ == '__main__':
    
    global loop_index, Error_List, Contrl_Input_List, Time_List, Yaw_Error_List, w, THETA, rList, yaw_control_input_list, position_mat, orientaion_mat
    global pose_time, arlist
    global m, v
    ##############################################
    loop_index = 0
    Error_List = [np.zeros((3))]
    Time_List = [0]
    Contrl_Input_List = [np.zeros((3))]
    Yaw_Error_List = [0]
    yaw_control_input_list = [0]
    rList = [np.zeros((3))]
    w = np.zeros(shape = (3,3))
    m = np.array([0.25,  0.5, 0.75])
    v = np.array([0.3,  0.3, 0.3])
    THETA = []
    position_mat = np.zeros(shape = (1,3))
    orientaion_mat = np.zeros(shape = (1,3))
    pose_time = []
    arlist = []
    ###################################################
        
    
    
    rospy.init_node('sepahvand_node', anonymous=True)
    
    #define a subscriber node that listens to the ar_pose_marker ros topic
    # sub_marker_pose = rospy.Subscriber('ar_pose_marker', AlvarMarkers, markers_finder)  
    #define a subscriber node that listens to the bebop/image_raw ros topic
    # sub_img = rospy.Subscriber('bebop/image_raw', Image, camera_image) 
    #define a subscriber to extract the camera intrinsics
    # sub_cam_info = rospy.Subscriber('/bebop/camera_info', CameraInfo, cam_info) 
    point_publisher = rospy.Publisher("image_point", Marker, queue_size=1)
    
    image_sub = message_filters.Subscriber('bebop/image_raw', Image)
    marker_sub= message_filters.Subscriber('ar_pose_marker', AlvarMarkers)
    camera_info_sub = message_filters.Subscriber('/bebop/camera_info', CameraInfo) 
    
    ts = message_filters.TimeSynchronizer([image_sub, marker_sub, camera_info_sub], 1, 1)
    ts.registerCallback(callBack)
    
    uav_pose_info = rospy.Subscriber('/bebop/odom', Odometry, callback = pose_callback) 
    
    
    feature_error_publisher = rospy.Publisher("feature_error", Vector3, queue_size=1)
    yaw_feature_pub = rospy.Publisher("qa", Float32, queue_size=1)
    norm_one_ferror = rospy.Publisher("norm", Float32, queue_size=1)
    pub = rospy.Publisher("bebop/cmd_vel", Twist, queue_size = 1)
    
    rospy.spin()
######################################################################################End of ROS Network