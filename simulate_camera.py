#!/usr/bin/env python3
import rospy
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox

def start_fake_camera():
    rospy.init_node('fake_yolo_node')
    # Publish to the exact topic your main script is listening to
    pub = rospy.Publisher('/darknet_ros/bounding_boxes', BoundingBoxes, queue_size=10)
    rate = rospy.Rate(10) # Publish at 10 Hz

    rospy.loginfo("Simulate camera is running! Broadcasting dummy cube...")

    while not rospy.is_shutdown():
        msg = BoundingBoxes()
        
        # Create a fake cube based on your real data
        fake_cube = BoundingBox()
        fake_cube.Class = "cube"
        fake_cube.xmin = 335
        fake_cube.xmax = 483
        fake_cube.ymin = 36
        fake_cube.ymax = 269
        fake_cube.z = 118.0 
        
        # Add it to the message array and publish
        msg.bounding_boxes.append(fake_cube)
        pub.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        start_fake_camera()
    except rospy.ROSInterruptException:
        pass