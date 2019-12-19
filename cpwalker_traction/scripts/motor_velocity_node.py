#!/usr/bin/python
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float64
import time

"""Global variables"""
joints = ["right_wheel", "left_wheel"]

class MotorVel(object):
    def __init__(self, joint_name):
        self.joint_name = joint_name
        self.verbose = rospy.get_param("traction_hw/verbose", False)
        self.enc_status = 0
        self.ppr = 512.0*4      # Encoder resolution. 1 CPR = 4 PPR
        self.ratio = 53.3         # Motor transmission
        self.meas_time = time.time()     # Current measurement time
        """ROS initialization"""
        #rospy.init_node('{}_vel_node'.format(node_name), anonymous=True)
        self.init_pubs_()
        self.init_subs_()

    def init_pubs_(self):
        self.vel_pub = rospy.Publisher('{}_vel'.format(self.joint_name), Float64, queue_size=100)

    def init_subs_(self):
        rospy.Subscriber("{}_encoder".format(self.joint_name), Int32, self.enc_callback_)

    def enc_callback_(self, msg):
        '''Publish motor velocity after converting encoder reading'''
        self.vel_pub.publish(self.enc2vel(msg.data))

    """ Converts from encoder reading to motor velocity.
        The characteristic funtion depends on the encoder resolution and the motor transmission."""
    def enc2vel(self, enc_count):
        delta_time = time.time() - self.meas_time
        delta_count = enc_count - self.enc_status
        # print delta_count
        wheel_vel = ((delta_count / self.ppr) * 1.0/self.ratio) / (delta_time / 60.0)
        if self.verbose: print "Wheel velocity [rpm]: {}".format(wheel_vel)
        self.enc_status = enc_count
        self.meas_time = time.time()
        return wheel_vel

def main():
    """Module initialization"""
    rospy.init_node('wheels_vel_node', anonymous=True)       
    for joint_name in joints:
        if rospy.has_param("traction_hw/joints/{}".format(joint_name)):
            joint_hw = rospy.get_param("traction_hw/joints/{}".format(joint_name))
            exec("{} = MotorVel('{}')".format(joint_name, joint_name))

    # rate = rospy.Rate(100)      # Hz
    rospy.logwarn("Processing sensor data...")
    while not rospy.is_shutdown():
        rospy.spin()
        # rate.sleep()

    """Clean up ROS parameter server"""
    try:
        rospy.delete_param("traction_hw")
        rospy.delete_param("spi_comm")
    except KeyError:
        pass

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt as e:
        print("Program finished\n")
        sys.stdout.close()
        os.system('clear')
        raise
