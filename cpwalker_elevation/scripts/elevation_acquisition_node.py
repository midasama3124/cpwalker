#!/usr/bin/python
import rospy
import struct
from scipy import signal

from cpwalker_util.can_rpi import CANbus
from std_msgs.msg import Float64

'''
This code is directly adapted from the exo code, as the communication scheme should be the same.
'''
"""Global variables"""
hw_names = ["Potentiometer", "Gauge"]
'''
Also, I'm assuming:
    - all of the sensors are connected to a single CAN module (i.e., a single DAQ);
    - this CAN bus is used exclusively for the elevation acquisition
'''
class SensorAcq(object):
    def __init__(self, joint_name, has_hw):
        self.joint_name = joint_name
        self.verbose = rospy.get_param("elevation_hw/verbose", False)
        self.has_pot = has_hw[0]
        self.has_gauge = has_hw[1]
        """ROS initialization"""
        self.init_params()
        self.init_pubs_()

    def init_params(self):
        self.pot_msg = Float64(0.0)
        self.gauge_msg = Float64(0.0)

        # FIR Filter
        # From CPWalker Simulink Model
        self.filter_den = 1.0
        self.filter_num = [0.00024042439949025545, 0.00049836390813927627, 0.00097417229437224417, 0.0017000443550953693, 
            0.0027373631842420856, 0.0041421027122268989, 0.0059580488476079062, 0.0082097899804267037, 0.010896842583805376,
            0.013988542477833011, 0.017421366592009706, 0.021099314003761898, 0.024896741653447137, 0.028664469167552876, 
            0.032238878049440665, 0.035452434826228466, 0.038145665650585241, 0.040179625609237338, 0.04144581448006561, 
            0.04187577805775819, 0.04144581448006561, 0.040179625609237338, 0.038145665650585241, 0.035452434826228466,  
            0.032238878049440665, 0.028664469167552876, 0.024896741653447137, 0.021099314003761898, 0.017421366592009706,  
            0.013988542477833011, 0.010896842583805376, 0.0082097899804267037, 0.0059580488476079062, 0.0041421027122268989,
            0.0027373631842420856, 0.0017000443550953693, 0.00097417229437224417, 0.00049836390813927627, 0.00024042439949025545]
        # Data array for potentiometer filtering
        self.pot_data_array = [0] * int(len(self.filter_num))   # past data array initialization

    def init_pubs_(self):
        if self.has_pot:
            self.pot_pub = rospy.Publisher('{}_pot'.format(self.joint_name), Float64, queue_size=100)
        if self.has_gauge:
            self.gauge_raw_pub = rospy.Publisher('{}_raw_gauge'.format(self.joint_name), Float64, queue_size=100)
            self.gauge_force_pub = rospy.Publisher('{}_gauge_force'.format(self.joint_name), Float64, queue_size=100)

    def get_sensor_data(self,msg):
        ''' Bytearray decoding returns tuple. Meaning of arguments:
        >: big endian, <: little endian, H: unsigned short (2 bytes), B: unsigned char'''
        if self.has_pot:
            pot_data = struct.unpack('<H', msg.data[:2])[0]    # 2 most significant bytes correspond to potentiometer reading
            self.pot_msg.data = self.pot_processing(pot_data)
            self.pot_pub.publish(self.pot_msg)
            # To debug: print side by side raw and filtered values
            rospy.loginfo_throttle(1,"[Elev Acq] Pot:    {}  {}".format(pot_data,self.pot_msg.data))
        if self.has_gauge:
            gauge_data = struct.unpack('<H', msg.data[2:4])[0]   # 3-4 bytes correspond to strain_gauge reading
            self.gauge_msg.data = self.gauge_processing(gauge_data)
            self.gauge_force_pub.publish(self.gauge_msg)
            self.gauge_raw_pub.publish(gauge_data)
            # To debug: print side by side raw and filtered values
            rospy.loginfo_throttle(1,"[Elev Acq] Gauge:  {}  {}".format(gauge_data,self.gauge_msg.data))

    def pot_processing(self,pot_data):
        pot_processed = self.pot_fir_filter(pot_data)
        return pot_processed

    # TO DO: Simulink model from CPWalker wasn't considering the elevation strain gauge
    def gauge_processing(self,gauge_data):
        gauge_filtered = self.gauge_fir_filter(gauge_data)
        gauge_processed = self.gauge_to_force(gauge_data)
        return gauge_processed
    
    def pot_fir_filter(self,data):
        # Rolling window
        self.pot_data_array.pop()
        self.pot_data_array.insert(0,data)
        # Filter
        #data_filtered = signal.lfilter(self.filter_num, self.filter_den, pot_data_array)
        data_filtered = data
        return data_filtered
    
    def gauge_fir_filter(self,data):
        gauge_filtered = data
        return gauge_filtered

    def gauge_to_force(self,data):
        gauge_force = data
        return gauge_force

def main():
    rospy.init_node('elevation_acq_node', anonymous=True)
    """Module initialization"""
    if rospy.has_param("elevation_hw/joints/elevation"):
        joint_hw = rospy.get_param("elevation_hw/joints/elevation")
        can_id = rospy.get_param("elevation_hw/joints/elevation/can_id")
        has_hw = []
        for hw_component in hw_names:
            has_hw.append(joint_hw[hw_component])
        if True in has_hw:
            elevation = SensorAcq('elevation', has_hw)
    
    # CAN
    can_channel = rospy.get_param("can_comm/elevation_port", "can1")
    can_bus = CANbus(channel=can_channel)

    rate_param = rospy.get_param("elevation_hw/sampling_frequency", 100)
    rate = rospy.Rate(rate_param) 
    rospy.loginfo("[Elevation Acquisition] Reading sensor data...")
    while not rospy.is_shutdown():
        can_bus.send_command()
        msg = can_bus.receive_data()
        if msg is not None:
            elevation.get_sensor_data(msg)
    	rate.sleep()

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
        print("[Elevation] Node closed\n")
        sys.stdout.close()
        os.system('clear')
        raise
