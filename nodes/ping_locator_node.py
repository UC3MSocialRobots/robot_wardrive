#!/usr/bin/env python

# :license       LASR_UC3M v1.0, ver LICENCIA.txt

# Este programa es software libre: puede redistribuirlo y/o modificarlo
# bajo los terminos de la Licencia Academica Social Robotics Lab - UC3M
# publicada por la Universidad Carlos III de Madrid, tanto en su version 1.0
# como en una version posterior.

# Este programa se distribuye con la intencion de que sea util,
# pero SIN NINGUNA GARANTIA. Para mas detalles, consulte la
# Licencia Academica Social Robotics Lab - UC3M version 1.0 o posterior.

# Usted ha recibido una copia de la Licencia Academica Social
# Robotics Lab - UC3M en el fichero LICENCIA.txt, que tambien se encuentra
# disponible en <URL a la LASR_UC3Mv1.0>.

"""
Relates ping delay to a location of the robot.

:author: Victor Gonzalez Pacheco
:maintainer: Victor Gonzalez Pacheco
"""

import rospy
import sys
from sh import ping
from threading import Thread
from rosh import Bagy

from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_wardrive.msg import PingLocation


class Pinger(object):

    """
    Pings a server at the desired ratio.

    Subscribes: 'amcl_pose' (geometry_msgs/PoseWithCovarianceStamped)
    Publishes 'ping' (robot_wardrive/PingLocation)

    """

    def __init__(self, bagy_name, host="8.8.8.8", interval=0.5):
        """Init.

        Args:
        -----
        host (str): host to ping. Default: '8.8.8.8'
        interval (float): interval between pings in seconds. Default: 0.5
        """
        super(Pinger, self).__init__()
        rospy.on_shutdown(self.shutdown)
        self.pose = None
        self.ping_logger = Bagy(bagy_name, 'w', PingLocation)
        self._pinger = Thread(target=self.do_ping,
                              kwargs={'host': host, 'interval': interval})

        rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.pose_cb)
        self.publisher = rospy.Publisher('ping', PingLocation)

    def do_ping(self, host="8.8.8.8", interval=0.5):
        """Get ping information and publish it."""
        for data in ping(host, "-i", str(interval), _iter=True,
                         _err=self.publish_ping_err):
            delay = data.split()[-2].split("=")[-1]
            self.publish_delay(delay)

    def log_ping_location(self, msg):
        """Log a PingLocation msg to the node Bagy."""
        self.ping_logger.write(msg)

    def publish_delay(self, delay):
        """Publish a PingLocation message from a delay."""
        try:
            ping_msg = PingLocation(delay=float(delay), pose=self.pose)
        except ValueError:
            pass
        else:
            ping_msg.header.stamp = rospy.get_rostime()
            self.log_ping_location(ping_msg)
            self.publisher.publish(ping_msg)

    def publish_ping_err(self, error_):
        """
        Callback triggered when ping returns an error (e.g. no connection).

        It publishes a PingLocation message with delay field set to -1.0
        """
        rospy.logerr("Ping returned error: {}".format(str(error_)))
        self.publish_delay(-1.0)

    def pose_cb(self, amcl_msg):
        """Get pose of the robot."""
        self.pose = amcl_msg.pose.pose

    def shutdown(self):
        """
        Hook to be executed when rospy.shutdown is called.

        Closes the Bagy when the node shutdowns.
        """
        self.ping_logger.close()

    def run(self):
        """Run the node."""
        self._pinger.start()
        rospy.spin()


def _init_node(node_name):
    """Common routines for start a node."""
    rospy.init_node(node_name)
    rospy.loginfo("Initializing {} Node".format(rospy.get_name()))

_DEFAULT_NAME = 'ping_locator_node'

if __name__ == '__main__':
    bagy_file = rospy.myargv(argv=sys.argv)[1]
    rospy.loginfo("Bagy name is: {}".format(bagy_file))
    try:
        _init_node(_DEFAULT_NAME)
        pinger = Pinger(bagy_name=bagy_file)
        pinger.run()
    except rospy.ROSInterruptException:
        pass
