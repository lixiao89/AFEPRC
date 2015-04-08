#!/usr/bin/env python

"""
Script to define a line virtual fixture:
 - start hybrid, put wam into manual mode
 - press key 1 to register point 1
 - press key 2 to register point 2
 - repeat if necessary
 - press enter to finish process, publish msg or srv to teleop
 
"""

import rospy
import tf
import copy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String
import tf_conversions.posemath as pm
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import math
import PyKDL

import readline
import cmd
import threading 


class HelloWorld(cmd.Cmd):
    """Simple command processor example"""

    def __init__(self):
        cmd.Cmd.__init__(self)

        self.prompt = 'prompt: '
        self.intro = "Simple Interface for Defining Line VF"

        # help doc
        self.ruler = '-'

        # ros
        self.pub_hybrid_key = rospy.Publisher('/hybrid/key', String)
        self.pub_vf_marker = rospy.Publisher('/linevf/marker', Marker)
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        # states
        self.tip_pose = PyKDL.Frame()
        self.vf_pose_1 = None      # two pose defined by key
        self.vf_pose_2 = None
        self.line_vf_pose = None

        self.t = threading.Thread(target=self.thread_update_pose)
        self.t.start()

    def thread_update_pose(self):
        # update robot pose
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            try:
                t = rospy.Time(0)
                self.listener.waitForTransform('/wam/base_link',
                                               '/wam/cutter_tip_link', t, rospy.Duration(1.0))
                # NTOE: result is /world w.r.t. /wam/cutter_jr3_link
                (trans,rot) = self.listener.lookupTransform('/wam/base_link', '/wam/cutter_tip_link', t)
                self.tip_pose.p = PyKDL.Vector(trans[0], trans[1], trans[2])
                self.tip_pose.M = PyKDL.Rotation.Quaternion(rot[0], rot[1], rot[2], rot[3])
                # print 'trans: ' + str(trans)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                self.tip_pose = PyKDL.Frame()
                print 'error'

            if self.vf_pose_1 and self.vf_pose_2:
                 msg = Marker()
                 msg.header.frame_id = "/wam/base_link"
                 msg.header.stamp = rospy.Time.now()

                 msg.ns = "linevf"
                 msg.type = Marker.LINE_STRIP
                 msg.action = Marker.ADD
                 msg.scale.x = 0.002

                 # color
                 msg.color.r = 0.0
                 msg.color.g = 0.0
                 msg.color.b = 1.0
                 msg.color.a = 1.0

                 # pose
                 msg.pose.position.x = 0.0
                 msg.pose.position.y = 0.0
                 msg.pose.position.z = 0.0
                 msg.pose.orientation.x = 0.0
                 msg.pose.orientation.y = 0.0
                 msg.pose.orientation.z = 0.0
                 msg.pose.orientation.w = 1.0

                 # points
                 p1 = Point()
                 p1.x = self.vf_pose_1.p.x()
                 p1.y = self.vf_pose_1.p.y()
                 p1.z = self.vf_pose_1.p.z()
                 msg.points.append(p1)

                 p2 = Point()
                 p2.x = self.vf_pose_2.p.x()
                 p2.y = self.vf_pose_2.p.y()
                 p2.z = self.vf_pose_2.p.z()
                 msg.points.append(p2)

                 msg.lifetime = rospy.Duration()
                 # self.pub_vf_marker.publish('hello')
                 self.pub_vf_marker.publish(msg)

            if self.line_vf_pose:
                # publish
                line_vf_tf = pm.toTf(self.line_vf_pose)
                # print line_vf_tf
                self.br.sendTransform(line_vf_tf[0],
                                      line_vf_tf[1],
                                      rospy.Time.now(),
                                      "/linevf",
                                      "/wam/base_link")                
                 
            rate.sleep()
            pass

        return 

    def cmdloop(self, intro=None):
        print 'cmdloop(%s)' % intro                
        return cmd.Cmd.cmdloop(self, intro)

    def postcmd(self, stop, line):
        if rospy.is_shutdown():
            print 'ros is shut_down()'
            return cmd.Cmd.postcmd(self, True, line)
        else:
            print 'postcmd(%s, %s)' % (stop, line)
        return cmd.Cmd.postcmd(self, stop, line)

    def default(self, line):
        print 'default(%s)' % line
        return cmd.Cmd.default(self, line)

    def emptyline(self):
        print 'emptyline() called'
        return cmd.Cmd.emptyline(self)

    def do_r(self, line):
        """
        Put robot into ready state
        """
        print 'ready key is pressed'
        self.pub_hybrid_key.publish('r')

    def do_m(self, line):
        """
        Put robot into manual state, gravity compensation
        """
        print 'manual key is pressed'
        self.pub_hybrid_key.publish('m')

    def do_h(self, line):
        """
        Put robot into hybrid Force/Postion mode
        """
        print 'hybrid key is pressed'
        self.pub_hybrid_key.publish('h')

    def do_c(self, line):
        """
        Put robot into hybrid Force/Postion mode
        """
        print 'correction key is pressed'
        self.pub_hybrid_key.publish('c')

        
    def do_i(self, line):
        """
        Put robot into hybrid Force/Postion mode
        """
        print 'forward key is pressed'
        self.pub_hybrid_key.publish('i')
        
    def do_k(self, line):
        """
        Put robot into hybrid Force/Postion mode
        """
        print 'backward key is pressed'
        self.pub_hybrid_key.publish('k')

        
    def do_greet(self, person):
        """
        greet [person]
        Greet the named person
        """
        if person:
            print "hi, ", person
        else:
            print "hi"

    def do_1(self, line):
        self.vf_pose_1 = copy.deepcopy(self.tip_pose)
        print self.vf_pose_1
        print '1 is pressed'

    def do_2(self, line):
        self.vf_pose_2 = copy.deepcopy(self.tip_pose)
        print self.vf_pose_2
        print '2 is pressed'

    def emptyline(self):
        print 'Computing Line Virtual Fixture'
        if self.vf_pose_1 is None:
            print 'Invalid Pose 1, Please Sample Pose 1'
            return 

        if self.vf_pose_2 is None:
            print 'Invalid Pose 2, Please Sample Pose 2'
            return

        # compute
        vec0 = self.vf_pose_2.p - self.vf_pose_1.p
        print vec0
        print vec0.Normalize(0.01)
        if (vec0.Normalize(0.01) == 0):
            print 'Error: pose 1 and pose 2 are too close'
            return

        axisy = PyKDL.Vector(0.0, 1.0, 0.0);
        axisz = PyKDL.Vector(0.0, 0.0, 1.0);

        # use y axis to find 1st orthogonal
        vec1 = vec0 * axisy
        # check to it's not too close to vec0
        if (PyKDL.dot(vec0, vec1) >= 0.98):
            vec1 = vec0 * axisz
        if (vec1[0] == 0 and vec1[1] == 0 and vec1[2] == 0):
            vec1 = vec0 * axisz

        # find 2nd orthogonal vector
        vec2 = vec0 * vec1
        vec1.Normalize()
        vec2.Normalize()

        # construct Rotation
        rot = PyKDL.Rotation()
        rot.UnitX(vec0)
        rot.UnitY(vec1)
        rot.UnitZ(vec2)

        # vf
        self.line_vf_pose = PyKDL.Frame(rot, self.vf_pose_1.p)
        print self.line_vf_pose
        

    def do_EOF(self, line):
        print "Exit"
        if not rospy.is_shutdown():
            rospy.signal_shutdown('Quit from CMD');
        return True

def main():

    # init ROS
    rospy.init_node('define_vf')
    rospy.loginfo('Started Define VF')

    HelloWorld().cmdloop()

if __name__ == '__main__':
    main()
