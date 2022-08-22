import rospy

from std_msgs.msg import Float64MultiArray,MultiArrayDimension,Float64


if __name__=="__main__":
    rospy.init_node('gripper_publisher')

    pub=rospy.Publisher('/panda_gripper_controller/command',
                        Float64MultiArray,queue_size=10)

    
    msg=Float64MultiArray()
    msg.layout.dim=[MultiArrayDimension('',2,1)]
    msg.data=[5,6]

    rate=rospy.Rate(10)
    print("Moving...")
    pub.publish(msg)