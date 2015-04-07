#!/usr/bin/env python

# Reference
#  https://git.lcsr.jhu.edu/wvu-jhu/experiment-slave/mtsUserStudaySlave.cpp
#  https://github.com/zchen24/tutorial/blob/master/kdl/mainWrench.cpp

import rospy
import tf
import copy
from geometry_msgs.msg import WrenchStamped

from comedi import *
import math
import PyKDL


class JR3FTSensor:
    """
    JR3 Force Torque Sensor, use after ros has been initialized
    """
    def __init__(self, devname='/dev/comedi0'):
        """
        Open and initialize JR3 force sensor
        
        Args:
          devname (str): JR3 device name e.g. /dev/comedi0
        """

        # open jr3
        self.jr3 = comedi_open(devname)
        if self.jr3 is None:
            errno = comedi_errno()
            print "Error (%d) %s" % (errno, comedi_strerror(errno))
            return

        # read maxdata and rangeinfo
        self.maxdata = []
        self.rangeinfo = []        
        jr3subdev = 0
        jr3filterid = 1
        jr3range = 0
        jr3numchan = comedi_get_n_channels(self.jr3, jr3subdev)
        counter = 0
        print 'numchan = ', jr3numchan    
        for i in range(jr3numchan):
            self.rangeinfo.append(comedi_get_range(self.jr3, jr3subdev, i, jr3range))
            self.maxdata.append(comedi_get_maxdata(self.jr3, jr3subdev, i))

        # electronics bias
        self.bias = PyKDL.Wrench()  # init to all 0

        # ros
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        pass

    def __del__(self):
        """
        Closes JR3 sensor 
        """
        if self.jr3 is not None:
            comedi_close(self.jr3)
            print 'deleted jr3'

    def __read_chan(self, chan):
        """
        Return physical value read from chan

        Args:
          chan(int): channel number

        Returns:
          phydata: physical value 
        """
        if (chan < 0 or chan > 55):
            rospy.logerr('jr3 read channel out of range (0-55)')
            return 0
            
        [ret, data] = comedi_data_read(self.jr3, 0, chan, 0, AREF_GROUND)
        if ret < 0:
            rospy.logerr('failed to read chan')
            return 0
        else:
            phydata = comedi_to_phys(data, self.rangeinfo[chan], self.maxdata[chan])
            return phydata

    def read_raw(self, filter_id=0):
        """
        Return sensor raw reading with filter id (filter_id)

        Args:
          filter_id(int): sensor filter id

        Returns:
          ft(PyKDL.Wrench): raw force sensor reading
        """
        if (filter_id < 0 or filter_id > 6):
            rospy.logerr('filter id out of range (0-6)')
            return PyKDL.Wrench()
        
        ft = PyKDL.Wrench()
        ft.force = PyKDL.Vector(self.__read_chan(0),
                                self.__read_chan(1),
                                self.__read_chan(2))
        ft.torque = PyKDL.Vector(self.__read_chan(3),
                                 self.__read_chan(4),
                                 self.__read_chan(5))
        
        # reverse kernel hack & English to SI
        ft[0] = ft[0] * 1000.0 * 4.44822162
        ft[1] = ft[1] * 1000.0 * 4.44822162
        ft[2] = ft[2] * 1000.0 * 4.44822162
        ft[3] = ft[3] * 10000.0 * 0.112984829
        ft[4] = ft[4] * 10000.0 * 0.112984829
        ft[5] = ft[5] * 10000.0 * 0.112984829

        # jr3 is left-handed, rotate about X by pi
        ft[1] = -ft[1]
        ft[4] = -ft[4]

        # print 'b:  ' + str(self.bias)
        return (ft - self.bias)

    # compensate tool mass, return gravity related torque
    def compensate_tool_mass(self):
        """
        Return tool mass induced wrench
        """
        
        # input Rotation of tool
        try:
            t = rospy.Time(0)
            self.listener.waitForTransform('/wam/cutter_jr3_link',
                                           '/world', t, rospy.Duration(1.0))
            # NTOE: result is /world w.r.t. /wam/cutter_jr3_link
            (trans,rot) = self.listener.lookupTransform('/wam/cutter_jr3_link', '/world', t)

            # convert to KDL Frame
            # print 'trans: ' + str(trans)
            # print 'rot:   ' + str(rot)
            
            com = PyKDL.Vector(0.0, 0.0, 0.03)  # com w.r.t. ft 

            # ft w.r.t. com 
            frame_com_rot = PyKDL.Rotation().Quaternion(rot[0], rot[1], rot[2], rot[3])
            frame_com_offset = com
            frame_com = PyKDL.Frame(frame_com_rot, frame_com_offset)

            # print ''
            # print frame_com
            
            # force from gravity
            mass = 0.135  # kg
            g = PyKDL.Wrench()
            g.force = PyKDL.Vector(0.0, 0.0, -9.81 * mass)
            g.torque = PyKDL.Vector(0.0, 0.0, 0.0)
            
            # transform gravity
            g = frame_com * g
            return g

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print 'error'
            return PyKDL.Wrench()
        
        # rospy.loginfo('compensate')

    def zeros(self):
        """
        Compute bias value
        """
        # set bias to 0
        self.bias = PyKDL.Wrench()

        # read sensor
        ft = self.read_raw(0)

        # compensate tool
        g = self.compensate_tool_mass()

        # compute bias
        self.bias = ft - g
        # self.bias = ft

        print 'new bias value = '
        print self.bias



def main():

    # init ROS
    rospy.init_node('jr3_ft_sensor')
    rospy.loginfo('JR3 FT Sensor Node Started')

    # publisher
    # pub_ft = rospy.Publisher('jr3/wrench', WrenchStamped)
    pub_ft_tool = rospy.Publisher('jr3/wrench', WrenchStamped)

    # JR3 Sensor
    jr3 = JR3FTSensor('/dev/comedi0')
    is_zeroed = False
    counter = 0

    # loop untill ctrl-c
    rate = rospy.Rate(500)
    while not rospy.is_shutdown():

        if not is_zeroed:
            jr3.zeros()
            is_zeroed = True

        # print jr3.__read_chan(0)
        ft = jr3.read_raw(0)
        g = jr3.compensate_tool_mass()
        ft = ft - g

        # transform to tip
        # frame_tool: tool tip w.r.t. jr3 
        frame_tool_rot = PyKDL.Rotation(0.9511, 0.0000, -0.3090,
                                        0.0000, 1.0000, 0.0000,
                                        0.3090, 0.0000, 0.9511)
        frame_tool_offset = PyKDL.Vector(0.0, 0.0, 0.09)
        frame_tool = PyKDL.Frame(frame_tool_rot, frame_tool_offset)
        # wa = F_a_b * wb
        ft_tool = frame_tool.Inverse() * ft

        ft_tool_msg = WrenchStamped()
        ft_tool_msg.header.frame_id = 'wam/cutter_tip_link'
        ft_tool_msg.header.stamp = rospy.Time.now()
        
        ft_tool_msg.wrench.force.x = ft_tool[0]
        ft_tool_msg.wrench.force.y = ft_tool[1]
        ft_tool_msg.wrench.force.z = ft_tool[2]
        ft_tool_msg.wrench.torque.x = ft_tool[3]
        ft_tool_msg.wrench.torque.y = ft_tool[4]
        ft_tool_msg.wrench.torque.z = ft_tool[5]
        
        pub_ft_tool.publish(ft_tool_msg)

        if (counter%500 == 0):
            # print '-------***'
            # print 'g:  ' + str(g)
            # print 'ft: ' + str(ft)
            print 'tl: ' + str(ft_tool)

        # sleep
        counter += 1
        rate.sleep()        

if __name__ == '__main__':
    main()
