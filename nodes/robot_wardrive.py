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

import rospy
from rospy_utils import coroutines as co
from sh import cat

from geometry_msgs.msg import PoseWithCovarianceStamped
from wifi_monitor.msg import SignalLocation


def get_signal():
    """Get WiFi signal data."""
    return cat("/proc/net/wireless")


def parse_signal(raw_signal):
    """Return signal from data.

    Args:
        raw_signal: signal raw data.
    Return:
        (float, float, float): signal's link, level and noise.

    Example:
        >>> from sh import cat
        >>> signal = cat("/proc/net/wireless")
        >>> link, level, noise = parse_signal(signal)
    """
    return map(float, raw_signal.split('\n')[2].split()[2:5])


def make_signal_location_msg(amcl_pose):
    """Merge a current position with WiFi signal level."""
    link, level, noise = parse_signal(get_signal())
    return SignalLocation(pose=amcl_pose.pose.pose,
                          link=link, level=level, noise=noise)


# def monitor_signal(signal, logger=rospy.loginfo):
#     """Output signal to logger once a second."""
#     while(1):
#         rospy.sleep(1)
#         signal = cat("/proc/net/wireless")
#         print parse_signal(signal)


def _init_node(node_name):
    """Common routines for start a node."""
    rospy.init_node(node_name)
    rospy.loginfo("Initializing {} Node".format(rospy.get_name()))

_DEFAULT_NAME = 'wifi_monitor'

if __name__ == '__main__':

    try:
        _init_node(_DEFAULT_NAME)
        pipe = co.pipe([co.do(make_signal_location_msg),
                        co.publisher('signal_location', SignalLocation)])
        co.PipedSubscriber('amcl_pose', PoseWithCovarianceStamped, pipe)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
