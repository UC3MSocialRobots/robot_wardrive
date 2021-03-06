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
Matches Wifi Signal Strenght with AMCL PoseWithCovarianceStamped messages.

:author: Victor Gonzalez Pacheco, Jose Carlos Castillo Montoya
:maintainer: Victor Gonzalez Pacheco
"""

from functools import partial
import sys
from sh import cat

import rospy
from rospy_utils import coroutines as co
from rosh import Bagy

from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_wardrive.msg import SignalLocation


def get_signal():
    """Get WiFi signal data."""
    return cat("/proc/net/wireless")


def parse_signal(raw_signal):
    """Return signal from data.

    Args:
        raw_signal: signal raw data.
    Return:
        (float, float, float): signal's link, level and noise.
        (0.0, 0.0, 0.0): In case wifi is not detected.

    Example:
        >>> from sh import cat
        >>> signal = cat("/proc/net/wireless")
        >>> link, level, noise = parse_signal(signal)
    """
    signal = raw_signal.split('\n')[2].split()[2:5]
    return map(float, signal) or (0.0, 0.0, 0.0)


def make_signal_location_msg(amcl_pose):
    """Merge a current position with WiFi signal level."""
    link, level, noise = parse_signal(get_signal())
    return SignalLocation(header=amcl_pose.header,
                          pose=amcl_pose.pose.pose,
                          link=link, level=level, noise=noise)


def write_to_bagy(msg, bagy):
    """
    Write msg to a Bagy.

    Log in case of error.
    """
    try:
        bagy.write(msg)
    except ValueError:
        bname = bagy.name
        rospy.logdebug("Trying to write on closed bagy: {}".format(bname))


def _init_node(node_name):
    """Common routines for start a node."""
    rospy.init_node(node_name)
    rospy.loginfo("Initializing {} Node".format(rospy.get_name()))

_DEFAULT_NAME = 'robot_wardrive'

if __name__ == '__main__':
    bagy_name = rospy.myargv(argv=sys.argv)[1]
    rospy.loginfo("Bagy name is: {}".format(bagy_name))
    bagy = Bagy(bagy_name, 'w', SignalLocation)
    bagy_logger = partial(write_to_bagy, bagy=bagy)
    try:
        _init_node(_DEFAULT_NAME)
        pipe = co.pipe([co.mapper(make_signal_location_msg),
                        co.do(bagy_logger),
                        co.publisher('signal_location', SignalLocation)])
        co.PipedSubscriber('amcl_pose', PoseWithCovarianceStamped, pipe)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        bagy.close()  # Close Bagy no mather how the program ends
