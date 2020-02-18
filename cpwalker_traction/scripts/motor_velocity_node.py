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
        self.ppr = 500.0*4.0                           # Encoder resolution. 1 CPR = 4 PPR
        self.ratio = 53.3                              # Motor transmission # Check!
        self.window_size = 5                           # Running mean filter - window size
        self.vel_array = [0] * int(self.window_size)   # Running mean filter - past velocities array (sizeof(self.window_size))
        self.last_measured_time = time.time()          # Current measurement time
        """ROS initialization"""
        self.init_pubs_()
        self.init_subs_()

    def init_pubs_(self):
        self.vel_pub = rospy.Publisher('{}_vel'.format(self.joint_name), Float64, queue_size=100)

    def init_subs_(self):
        rospy.Subscriber("{}_encoder".format(self.joint_name), Int32, self.enc_callback_)

    def enc_callback_(self, msg):
        current_vel = self.enc2vel(msg.data)
        filtered_vel = self.running_mean(current_vel)
        '''Publish motor velocity after converting encoder reading'''
        self.vel_pub.publish(filtered_vel)

    """ Converts from encoder reading to motor velocity.
        The characteristic funtion depends on the encoder resolution and the motor transmission."""
    def enc2vel(self, enc_count):
        current_time = time.time()
        delta_time = current_time - self.last_measured_time
        delta_count = enc_count - self.enc_status
        
        shaft_turn = delta_count / self.ppr 
        shaft_vel = shaft_turn / (delta_time / 60.0) # in rpm
        wheel_vel = shaft_vel / self.ratio           # in rpm
        #if self.verbose: print "{} Wheel velocity [rpm]: {}".format(self.joint_name,wheel_vel)
        self.enc_status = enc_count
        self.last_measured_time = current_time
        return wheel_vel

    ''' Running mean filter to smooth velocity readings '''
    def running_mean(self,current_vel):
        self.vel_array.pop()
        self.vel_array.insert(0,current_vel)
        return sum(self.vel_array) / self.window_size

def main():
    """Module initialization"""
    rospy.init_node('wheels_vel_node', anonymous=False)       
    for joint_name in joints:
        if rospy.has_param("traction_hw/joints/{}".format(joint_name)):
            joint_hw = rospy.get_param("traction_hw/joints/{}".format(joint_name))
            exec("{} = MotorVel('{}')".format(joint_name, joint_name))

    rospy.logwarn("Processing sensor data...")
    while not rospy.is_shutdown():
        rospy.spin()

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
