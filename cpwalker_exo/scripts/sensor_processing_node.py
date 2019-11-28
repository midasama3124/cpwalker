#!/usr/bin/python
import rospy
from cpwalker_util.can_rpi import CANbus
from std_msgs.msg import UInt16
from std_msgs.msg import Float64

"""Global variables"""
joints = ["right_knee", "left_knee", "right_hip", "left_hip"]
hw_names = ["Potentiometer", "Gauge", "FSR1", "FSR2"]

class SensorProcess(object):
    def __init__(self, node_name, has_hw):
        self.node_name = node_name
        self.verbose = rospy.get_param("exo_hw/verbose", False)
        self.has_pot = has_hw[0]
        self.has_gauge = has_hw[1]
        self.has_fsr1 = has_hw[2]
        self.has_fsr2 = has_hw[3]
        self.filter_states = [0.0]*2
        """ROS initialization"""
        rospy.init_node('{}_proc_node'.format(node_name), anonymous=True)
        self.init_pubs_()
        self.init_subs_()

    def init_pubs_(self):
        if self.has_pot:
            self.angle_pub = rospy.Publisher('{}_angle'.format(self.node_name), Float64, queue_size=100)
        if self.has_gauge:
            self.gauge_pub = rospy.Publisher('{}_gauge'.format(self.node_name), Float64, queue_size=100)
        if self.has_fsr1:
            self.fsr1_pub = rospy.Publisher('{}_fsr1'.format(self.node_name), Float64, queue_size=100)
        if self.has_fsr2:
            self.fsr2_pub = rospy.Publisher('{}_fsr2'.format(self.node_name), Float64, queue_size=100)

    def init_subs_(self):
        if self.has_pot:
            rospy.Subscriber("{}_pot".format(self.node_name), UInt16, self.pot_callback_)
        if self.has_gauge:
            rospy.Subscriber("{}_raw_gauge".format(self.node_name), UInt16, self.gauge_callback_)
        if self.has_fsr1:
            rospy.Subscriber("{}_raw_fsr1".format(self.node_name), UInt16, self.fsr1_callback_)
        if self.has_fsr2:
            rospy.Subscriber("{}_raw_fsr2".format(self.node_name), UInt16, self.fsr2_callback_)

    def pot_callback_(self, msg):
        '''Publish angle value after filtering potentiometer reading'''
        # filt_pot = self.
        # rospy.loginfo(msg.data)
        self.angle_pub.publish(self.pot2angle(self.biquad_filter(msg.data)))

    def gauge_callback_(self):
        # TODO: Include strain gauge processing
        pass

    def fsr1_callback_(self):
        # TODO: Include fsr processing
        pass

    def fsr2_callback_(self):
        # TODO: Include fsr processing
        pass

    """Biquad filter for potentiometer readings (Exported from Simulink)"""
    def biquad_filter(self, pot):
        '''Filter parameters'''
        a0 = 0.000956278151630223
        a1 = -1.9106427906788
        a2 = 0.914467903285316
        b0 = 2.0
        den_accum = (a0*pot - a1*self.filter_states[0]) - a2*self.filter_states[1];
        filt_pot = (b0*self.filter_states[0] + den_accum) + self.filter_states[1];
        '''Updating filter states'''
        self.filter_states[1] = self.filter_states[0];
        self.filter_states[0] = den_accum;
        return filt_pot

    """ Converts from potentiometer reading to angle value.
        The transference funtion depends on the joint being used."""
    def pot2angle(self, pot):
        if self.node_name == 'left_knee':
            angle = -0.105*pot + 98.401
        if self.node_name == 'right_knee':
            angle = 0.1087*pot - 10.109
        if self.node_name == 'left_hip':
            angle = -0.0983*pot + 67.598
        if self.node_name == 'right_hip':
            angle = -0.1061*pot + 56.675
        return angle

def main():
    """Module initialization"""
    for joint_name in joints:
        if rospy.has_param("exo_hw/joints/{}".format(joint_name)):
            joint_hw = rospy.get_param("exo_hw/joints/{}".format(joint_name))
            has_hw = []
            for hw_component in hw_names:
                has_hw.append(joint_hw[hw_component])
            if True in has_hw:
                exec("{} = SensorProcess('{}', has_hw)".format(joint_name, joint_name))

    # rate = rospy.Rate(100)      # Hz
    rospy.logwarn("Processing sensor data...")
    while not rospy.is_shutdown():
        rospy.spin()
        # rate.sleep()

    """Clean up ROS parameter server"""
    try:
        rospy.delete_param("exo_hw")
        rospy.delete_param("can_comm")
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
