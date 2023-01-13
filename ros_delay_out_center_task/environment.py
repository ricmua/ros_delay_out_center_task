""" Provides a virtual environment interface compatible with the 
    `delay_out_center_task`.

Examples
--------

These examples are intended to match those included with the 
`delay_out_center_task` package.

First, initialize a ROS2 interface.

>>> import rclpy
>>> rclpy.init()

Create a node to pass to the environment.

>>> import rclpy.node
>>> node = rclpy.node.Node('environment')

Initialize the environment.

>>> environment = Environment(node=node)

Upon initialization, the environment should contain a cursor sphere with 
default attribute values.

>>> environment.get_radius('cursor')
1.0
>>> environment.get_position('cursor')
(0.0, 0.0, 0.0)
>>> environment.get_color('cursor')
(0.0, 0.0, 0.0, 1.0)

Appropriate topics should also be initialized.

>>> import pprint
>>> topic_map = dict(node.get_topic_names_and_types())
>>> pprint.pp(list(topic_map))
['/cursor/color',
 '/cursor/position',
 '/cursor/radius',
 '/destroy',
 '/initialize',
 '/parameter_events',
 '/rosout']

No other objects should be present in the environment, at this time, but 
spheres can be initialized and destroyed when required. Verify this mechanism.

>>> len(environment.objects) == 1
True
>>> environment.initialize_sphere(key='test')
>>> len(environment.objects) == 2
True
>>> environment.get_radius('test')
1.0
>>> environment.get_position('test')
(0.0, 0.0, 0.0)
>>> environment.destroy_sphere(key='test')
>>> len(environment.objects) == 1
True
>>> try: environment.get_position('test')
... except KeyError: print('Object does not exist')
Object does not exist

Clean up.

>>> node.destroy_node()
>>> rclpy.shutdown()

"""

# Copyright 2022-2023 Carnegie Mellon University Neuromechatronics Lab (a.whit)
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# Contact: a.whit (nml@whit.contact)


# Import threading.
import threading

# Import ros_spheres_environment.
from ros_spheres_environment import Client

# Import ros_threading_timer.
import ros_threading_timer


# Virtual environment class.
class Environment(Client):
    """ Wrapper around a `spheres_environment` client that provides the 
        interface expected by the `delay_center_out_task` package.
        
    The `delay_center_out_task` package expects property accessor functions for 
    objects in the virtual environment. The `spheres_environment` does not 
    implement these, so this wrapper provides them.
    
    More generally, the `spheres_environment` interface does not quite match 
    what is expected for the `delay_center_out_task`. Harmonization of the two 
    interfaces is planned (obviating the need for this class / module).
    """
    def __init__(self, *args, #Timer=threading.Timer, 
                       Timer=None,
                       **kwargs):
        #self.timer = Timer
        super().__init__(*args, **kwargs)
        ros_timer = self.node.create_timer(timer_period_sec=1, 
                                           callback=lambda: None)
        ros_timer.cancel()
        Timer = Timer if Timer else \
                ros_threading_timer.TimerWrapper(ros_timer, node=self.node)
        self.timer = Timer
        self.objects = self
        self.initialize_object('cursor')
        
    def initialize_sphere(self, *args, **kwargs):
        self.initialize_object(*args, **kwargs)
        
    def destroy_sphere(self, *args, **kwargs):
        self.destroy_object(*args, **kwargs)
        
    def get_position(self, key):
        position = self[key].position
        return (position['x'], position['y'], position['z'])
    
    def set_position(self, x=0.0, y=0.0, z=0.0, key='cursor'):
        self[key].position = dict(x=x, y=y, z=z)
        
    def get_radius(self, key): return self[key].radius
    
    def set_radius(self, value, key='cursor'): self[key].radius = value
    
    def get_color(self, key):
        color = self[key].color
        return (color['r'], color['g'], color['b'], color['a'])
    
    def set_color(self, r=0.0, g=0.0, b=0.0, a=1.0, key='cursor'):
        self[key].color = dict(r=r, g=g, b=b, a=a)
    
  

