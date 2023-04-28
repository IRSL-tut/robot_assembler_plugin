#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64
import numpy as np

class TOFConverter:
    def __init__(self):
        rospy.init_node('tof_converter', anonymous=True)
        rospy.Subscriber("/AssembleRobot/TOF_Sensor0/point_cloud", PointCloud2, self.callback)
        self.pub = rospy.Publisher('/AssembleRobot/TOF_Sensor0/value', Float64, queue_size=1)
    def callback(self, data):
        width = data.width
        height = data.height
        pcdata = np.frombuffer(data.data, dtype=np.float32).reshape((height,width,4))
        z_tmp = np.copy(pcdata[:,:,2])
        z_tmp[z_tmp==np.inf] = np.nan
        z_tmp[z_tmp==-np.inf] = np.nan
        z_value = np.nanmin(z_tmp)
        send_data = Float64(z_value)
        self.pub.publish(send_data)
 
def listener():
    tc = TOFConverter()
    rospy.spin()

if __name__ == '__main__':
    listener()
