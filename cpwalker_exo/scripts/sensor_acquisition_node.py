#!/usr/bin/python
import rospy
import struct

from cpwalker_util.can_rpi import CANbus
from std_msgs.msg import UInt16

"""Global variables"""
joints = ["right_knee", "left_knee", "right_hip", "left_hip"]
hw_names = ["Potentiometer", "Gauge", "FSR1", "FSR2"]

class SensorAcq(object):
    def __init__(self, joint_name, has_hw):
        self.joint_name = joint_name
        self.verbose = rospy.get_param("exo_hw/verbose", False)
        self.has_pot = has_hw[0]
        self.has_gauge = has_hw[1]
        self.has_fsr1 = has_hw[2]
        self.has_fsr2 = has_hw[3]
        """ROS initialization"""
        self.init_pubs_()

    def init_pubs_(self):
        if self.has_pot:
            self.pot_pub = rospy.Publisher('{}_pot'.format(self.joint_name), UInt16, queue_size=100)
        if self.has_gauge:
            self.gauge_pub = rospy.Publisher('{}_raw_gauge'.format(self.joint_name), UInt16, queue_size=100)
        if self.has_fsr1:
            self.fsr1_pub = rospy.Publisher('{}_raw_fsr1'.format(self.joint_name), UInt16, queue_size=100)
        if self.has_fsr2:
            self.fsr2_pub = rospy.Publisher('{}_raw_fsr2'.format(self.joint_name), UInt16, queue_size=100)

    def get_sensor_data(self,msg):
        ''' Bytearray decoding returns tuple. Meaning of arguments:
        >: big endian, <: little endian, H: unsigned short (2 bytes), B: unsigned char'''
        if self.has_pot:
            pot_msg = struct.unpack('<H', msg.data[:2])[0]    # 2 most significant bytes correspond to potentiometer reading
            self.pot_pub.publish(pot_msg)
        if self.has_gauge:
            sensor_msg = struct.unpack('<H', msg.data[2:4])[0]   # 3-4 bytes correspond to strain_gauge reading
            self.gauge_pub.publish(sensor_msg)
        if self.has_fsr1:
            sensor_msg = struct.unpack('<H', msg.data[4:6])[0]   # 5-6 bytes correspond to potentiometer reading
            self.fsr1_pub.publish(sensor_msg)
        if self.has_fsr2:
            sensor_msg = struct.unpack('<H', msg.data[6:])[0]    # 7-8 bytes correspond to potentiometer reading
            self.fsr2_pub.publish(sensor_msg)

def main():
    rospy.init_node('exo_acq_node', anonymous=True)
    """Module initialization"""
    has_joint = []
    joint_can_id = {}
    for joint_name in joints:
        if rospy.has_param("exo_hw/joints/{}".format(joint_name)):
            has_joint.append(joint_name)
            joint_hw = rospy.get_param("exo_hw/joints/{}".format(joint_name))

            can_id = rospy.get_param("exo_hw/joints/{}/can_id".format(joint_name))
            joint_can_id.update( {joint_name : can_id} ) 
            
            has_hw = []
            for hw_component in hw_names:
                has_hw.append(joint_hw[hw_component])
            if True in has_hw:
                exec("{} = SensorAcq('{}', has_hw)".format(joint_name, joint_name))
    number_of_joints = len(has_joint)
    inv_joint_can_id = {v: k for k, v in joint_can_id.iteritems()} # inverting key and value 
    
    # CAN
    can_channel = rospy.get_param("can_comm/exo_port", "can0")
    can_bus = CANbus(channel=can_channel)

    rate_param = rospy.get_param("exo_hw/sampling_frequency", 1)
    rate = rospy.Rate(rate_param) 
    '''
    Honestly, this is not the best way to do this. What we're doing here is
    to loop over sending one command and trying to read as fast as possible
    the exact number of messages that should be arriving in response. 
    '''
    rospy.loginfo("[Exo] Reading sensor data...")
    while not rospy.is_shutdown():
        can_bus.send_command()
        for i in range(number_of_joints):
            msg = can_bus.receive_data()
            if msg is not None:
                try:
                    joint = inv_joint_can_id[msg.arbitration_id] # get joint name from joint can id
                    exec("{}.get_sensor_data(msg)".format(joint))
                except:
                    rospy.logerr("[Exo] Error! Unknown id {}!".format(msg.arbitration_id))
    	rate.sleep()

    for joint_name in has_joint:
        exec("del {}".format(joint_name))
    
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
