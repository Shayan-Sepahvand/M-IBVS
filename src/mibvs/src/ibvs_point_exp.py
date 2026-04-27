#!/usr/bin/env python3      
import rospy
import numpy as np
import time
from geometry_msgs.msg import Twist, PoseArray
import tf.transformations as tf
import time

def callBack(pose_data):

    global loop_index, timer, erros_array
    loop_index = loop_index + 1
    t = rospy.get_time()
   
   
   
   #-------------------------------------------
   # regulate the yaw angle to +90 degrees    
   # here we require the camera (child) and world (parent)
   
   # read the yaw angle wrt the global frame
    euler = tf.euler_from_quaternion([pose_data.poses[0].orientation.x, pose_data.poses[0].orientation.y, pose_data.poses[0].orientation.z, pose_data.poses[0].orientation.w])
   
   
   # checked euler = (roll, pitch , yaw)
    # print ("Euler: ", euler)
    yaw_uav = euler[2]
    yaw_setpoint = 1.57
    
    yaw_e = yaw_setpoint - yaw_uav
   
    k_psi = 0.2
    yaw_control_input = k_psi * yaw_e
   
    global pub
    twist = Twist()
    twist.angular.z = yaw_control_input
    pub.publish(twist)
   
    if(np.abs(yaw_e)<0.1):
   
   
   
   
   
   
        des_feat = np.array([[+0.048],
                            [-0.048],
                            [0.048], 
                            [0.048],
                            [-0.048],
                            [-0.048],
                            [-0.048],
                            [0.048]])


        Lp, e_feat = interaction_matrix(pose_data, des_feat)   
        
        lam = 0.1
        # u_f = lam * Lp.transpose() @ e_feat
        u_f = lam * np.linalg.pinv(Lp) @ e_feat

        print(f'{loop_index} Running The IBVS') 


        cam_rot_in_uav = tf.quaternion_matrix([pose_data.poses[-1].orientation.x, pose_data.poses[-1].orientation.y, pose_data.poses[-1].orientation.z, pose_data.poses[-1].orientation.w])[:3, :3]

        cam_origin_in_uav = tf.quaternion_matrix([pose_data.poses[-1].orientation.x, pose_data.poses[-1].orientation.y, pose_data.poses[-1].orientation.z, pose_data.poses[-1].orientation.w])[3, :3]  

        AdT = adjoint_matrix(cam_rot_in_uav, cam_origin_in_uav)
        
        # u_F = np.array([u_f[0,0],u_f[1,0],u_f[2,0],0,0,u_f[3,0]])  # in camera coordinates [vx vy vz 0 0 wz]
        
        u_f = AdT @ u_f # twist expressed in the uav(body) frame
        # erros_array.append(e_feat)
        # global pub
        # twist = Twist()
        # control_bound = 10
        # current_time = time.time()
        # if abs(u_F[0])<control_bound and abs(u_F[1])<control_bound:
        #     if (current_time - timer) > 1: 
        twist.linear.x  = u_f[0]
        twist.linear.y  = u_f[1]
        twist.linear.z  = u_f[2]
        twist.angular.z = u_f[-1]
            
        #     print ("controls: ")
        #     print (u_F)
        # else:
        #     print("Control values were more than ", control_bound)
        # twist.angular.z = -kp * np.tanh(np.abs(yaw_error)/yaw_error)
        pub.publish(twist)
        print("control", u_f)
    
    # np.save("erros.npy", erros_array)

# --------------------------------------------------------------------------------
# --------------------------------------------------------------------------------
# --------------------------------------------------------------------------------

def skew(x):
    sk = np.array([[0, -x[2], x[1]],[x[2], 0, -x[0]], [-x[1], x[0], 0]])
    return sk


def adjoint_matrix(R,t):
    tcross = skew(t)
    AdT_r1 = np.hstack((R, np.zeros((3, 3))))
    AdT_r2 = np.hstack((tcross @ R, R))
    AdT = np.vstack((AdT_r1, AdT_r2))
    return AdT

def interaction_matrix(pose_data, des_feat):
    
    Lp = np.zeros((8,6))
    e_feat = np.zeros((8,1))
    
    for i in range(0,4):
 
        xi = pose_data.poses[i+1].position.x
        yi = pose_data.poses[i+1].position.y
        zi = pose_data.poses[i+1].position.z
    
        xnorm = (xi/zi) 
        ynorm = (yi/zi) 
        norm_cor = np.array([[xnorm],[ynorm]])
            
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
            
    return Lp, e_feat      
    
# def connecting_line(normalized_cord):
#     flag = False
#     try:
#         x1, y1 = normalized_cord[0,:]
#         x2, y2 = normalized_cord[1,:]
#         theta = np.arctan2((y2 - y1), (x2 - x1))
#         theta = theta % (2*np.pi)
#     except:
#         theta = 0
#         flag = True
#     return theta, flag  


if __name__ == '__main__':
    
    global loop_index
    timer = time.time()
    loop_index = 0
    Error_List = [np.zeros((8,1))]
    Time_List = [0]
    erros_array = []

    rospy.init_node('sepahvand_node', anonymous=True)
    sub_cam_info = rospy.Subscriber('/features_poses_in_camera_frame', PoseArray, callBack) 
    # pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size = 1)
    pub = rospy.Publisher("bebop/cmd_vel", Twist, queue_size = 1)
    rospy.spin()

