#!/usr/bin/env python3   
import rospy
import tf
from geometry_msgs.msg  import Pose, PoseArray

def main():
    rospy.init_node('frame_transformation')
    rospy.sleep(0.5)  # Wait for the listener to start
    message_frequency = 100
    rate = rospy.Rate(message_frequency)
    listener = tf.TransformListener()
    

    
    


    pose_publisher = rospy.Publisher('/features_poses_in_camera_frame', PoseArray, queue_size = 1)

    while not rospy.is_shutdown():
 
        pose_array = PoseArray()
        
        parent_frame = '/world'
        child_frame = '/base_link'
        # world and UAV
        listener.waitForTransform(parent_frame, child_frame, rospy.Time(), rospy.Duration(1.0))  # parent  and then child
        (translation, quaternion)  = listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
        
        pose = Pose()
 
        pose.position.x = translation[0]
        pose.position.y = translation[1]
        pose.position.z = translation[2]
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = parent_frame
        
        pose_array.poses.append(pose) 
        
        try:
            # marker 1 and camera optical
            parent_frame = '/camera_optical'
            # parent_frame = '/base_link'
            child_frame = '/marker1'
            
            listener.waitForTransform(parent_frame, child_frame, rospy.Time(), rospy.Duration(1.0))  # parent  and then child
            (translation, quaternion)  = listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
            
            
            pose = Pose()

            pose.position.x = translation[0]
            pose.position.y = translation[1]
            pose.position.z = translation[2]
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = parent_frame
            
            pose_array.poses.append(pose)        
        except:
            pose = Pose()

            pose.position.x = 2.4
            pose.position.y = 1
            pose.position.z = 2
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1
            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = parent_frame
            
            pose_array.poses.append(pose)  
            
            
                
        # marker 1 and camera optical
        parent_frame = '/camera_optical'
        # parent_frame = '/disk'
        child_frame = '/marker2'
        
        try:
            listener.waitForTransform(parent_frame, child_frame, rospy.Time(), rospy.Duration(1.0))  # parent  and then child
            (translation, quaternion)  = listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
        
        
            pose = Pose()

            pose.position.x = translation[0]
            pose.position.y = translation[1]
            pose.position.z = translation[2]
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = parent_frame
            
            pose_array.poses.append(pose) 
        except:
            pose = Pose()

            pose.position.x = 2.4
            pose.position.y = 1
            pose.position.z = 2
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1
            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = parent_frame
            
            pose_array.poses.append(pose)  
        # marker 1 and camera optical
        parent_frame = '/camera_optical'
        # parent_frame = '/disk'
        child_frame = '/marker3'
    

        
        try:
            listener.waitForTransform(parent_frame, child_frame, rospy.Time(), rospy.Duration(1.0))  # parent  and then child      
            (translation, quaternion)  = listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
            pose = Pose()

            pose.position.x = translation[0]
            pose.position.y = translation[1]
            pose.position.z = translation[2]
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = parent_frame
            
            pose_array.poses.append(pose) 
        
        except:
            pose = Pose()

            pose.position.x = 2.4
            pose.position.y = 1
            pose.position.z = 2
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1
            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = parent_frame
            
            pose_array.poses.append(pose)  
        # marker 1 and camera optical
        parent_frame = '/camera_optical'
        # parent_frame = '/disk'
        child_frame = '/marker4'
        

        
        try:
            listener.waitForTransform(parent_frame, child_frame, rospy.Time(), rospy.Duration(1.0))  # parent  and then child        
            (translation, quaternion)  = listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))            
            pose = Pose()

            pose.position.x = translation[0]
            pose.position.y = translation[1]
            pose.position.z = translation[2]
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = parent_frame
            pose_array.poses.append(pose) 
        
        except:
            pose = Pose()

            pose.position.x = 2.4
            pose.position.y = 1
            pose.position.z = 2
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1
            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = parent_frame
            
            pose_array.poses.append(pose)      
            
            
        try:
            # tip and uav
            # parent_frame = '/base_link'
            parent_frame = '/base_link'
            child_frame = '/camera_optical'
            
            listener.waitForTransform(parent_frame, child_frame, rospy.Time(), rospy.Duration(1.0))  # parent  and then child
            (translation, quaternion)  = listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))
            
            
            pose = Pose()

            pose.position.x = translation[0]
            pose.position.y = translation[1]
            pose.position.z = translation[2]
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = parent_frame
            
            pose_array.poses.append(pose)        
        except:
            pose = Pose()

            pose.position.x = 0.6
            pose.position.y = 0
            pose.position.z = 0
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1
            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = parent_frame
            
            pose_array.poses.append(pose)                  
        

        
        
        
        
        
        
        
        
        
       

        pose_publisher.publish(pose_array)
        
        rate.sleep()
    
    
    
if __name__ == '__main__':
    main()