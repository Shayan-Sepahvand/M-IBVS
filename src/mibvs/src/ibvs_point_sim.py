#!/usr/bin/env python3  
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers

class Controller():
    def __init__(self, KP):
        self.kp = KP
        # Camera to UAV Rotation
        self.Rs = np.array([[ 1,  0,  0],
                            [ 0,  0,  1],
                            [ 0, -1,  0]])
        self.camera_offset = 0.1
        # Camera offset translation (converted to 3x1 for skew math)
        self.t = np.array([[0.0], [self.camera_offset], [0.0]])    

    def skew(self, x):
        # x comes in as (3,1), we need to extract scalars
        return np.array([[0, -x[2,0], x[1,0]],
                         [x[2,0], 0, -x[0,0]], 
                         [-x[1,0], x[0,0], 0]])

    def adjoint_matrix(self):
        tcross = self.skew(self.t)
        AdT_r1 = np.hstack((self.Rs, np.zeros((3, 3))))
        AdT_r2 = np.hstack((tcross @ self.Rs, self.Rs))
        AdT = np.vstack((AdT_r1, AdT_r2))
        return AdT

    def im_jacob(self, marker_data, des_feat):
        Lp = np.zeros((8,6))
        e_feat = np.zeros((8,1)) 
        
        # Guard: Check if we actually have enough markers
        num_markers = len(marker_data.markers)
        if num_markers < 4:
            rospy.logwarn_once("Waiting for 4 markers...")
            return None, None

        for i in range(0, 4):
            marker_info = marker_data.markers[i].pose.pose
            xi = marker_info.position.x
            yi = marker_info.position.y
            zi = marker_info.position.z
        
            xnorm = xi/zi
            ynorm = yi/zi
            norm_cor = np.array([[xnorm],[ynorm]])
            
            # Fill Jacobian and Error rows
            row = i * 2
            Lp[row:row+2,:] = np.array([[-1/zi,  0, xnorm/zi, xnorm*ynorm, -(1+xnorm**2), ynorm],
                                        [0, -1/zi, ynorm/zi, (1+ynorm**2), -xnorm*ynorm, -xnorm]])
            e_feat[row:row+2,:] = des_feat[row:row+2,:] - norm_cor
                    
        return Lp, e_feat
    
    def ibvs_crtl_law(self, Lp, es):   
        # Simple proportional control with pseudo-inverse
        return self.kp * np.linalg.pinv(Lp) @ es

# Main Callback -------------------------------------------------------------------------

def callBack(marker_data, args):
    ctrl, pub = args
    
    # Desired features (u1, v1... u4, v4)
    des_feat = np.array([[0.114], [0.114], [-0.122], [0.113], 
                         [0.114], [-0.120], [-0.122], [-0.120]])
    
    Lp, es = ctrl.im_jacob(marker_data, des_feat)
    
    if Lp is not None:
        uf_cam = ctrl.ibvs_crtl_law(Lp, es)  
        AdT = ctrl.adjoint_matrix()
        uf_body = AdT @ uf_cam # Transform twist to UAV body frame
        
        twist = Twist()
        twist.linear.x  = uf_body[0]
        twist.linear.y  = uf_body[1]
        twist.linear.z  = uf_body[2]
        twist.angular.z = uf_body[5]
        pub.publish(twist)

# Main ----------------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        rospy.init_node('bebop_ibvs_controller')
        
        ctrl = Controller(KP=0.08)
        pub = rospy.Publisher("bebop/cmd_vel", Twist, queue_size=1)
        
        # Passing ctrl and pub into the callback safely
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, callBack, callback_args=(ctrl, pub))
        
        rospy.loginfo("IBVS Node Started")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
