#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# import os
# this_file = os.path.abspath(__file__)
# pkg_path = os.path.dirname(this_file)

# print '################################################################'
# print "This file is {}".format(this_file)
# print '################################################################'

# import rospkg
# THIS_PKG = 'rospy_utils'
# rospack = rospkg.RosPack()
# pkg_path = rospack.get_path(THIS_PKG)

d = generate_distutils_setup(
    # #  don't do this unless you want a globally visible script
    # scripts=['bin/myscript'],
    packages=['robot_wardrive'],
    package_dir={'': 'src'}  # ,
#   package_xml_path=pkg_path
)

setup(**d)
