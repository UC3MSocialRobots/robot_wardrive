'''
Functions to parse YAML locations.

:author: Victor Gonzalez-Pacheco
:date: 2015-05

Example of how to use these functions:
    >>> import yaml
    >>> # Load:
    >>> locations = None
    >>> with open('locations_roboticslab.yaml', 'rw') as locs:
    >>>     locations = [loc for loc in yaml.load_all(locs)]

    >>> locations = locations_to_yaml(locations)

    # Save:
    >>> with open('roboticslab_locations_parsed.yaml', 'w') as locs:
    >>>     locs.write(yaml.dump(parsed_locations))
'''

import tf
from geometry_msgs.msg import (Point, Quaternion)


def from_quaternion(q):
    return tf.transformations.euler_from_quaternion([q.w, q.x, q.y, q.z])


def parse_location(location):
    p = Point(**location['position'])
    q = from_quaternion(Quaternion(**location['orientation']))
    return p, q, location['id'], location['location_name']


def pose_to_dict(p, orient, id_, name):
    return {'id': id_, 'name': name,
            'x': p.x, 'y': p.y, 'z': p.z, 'theta': orient[0]}


def locations_to_yaml(locations):
    return {loc['id']: pose_to_dict(*parse_location(loc)) for loc in locations}
