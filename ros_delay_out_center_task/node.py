""" A ROS2 node that starts a state machine and model for a cursor-based 
    center-out, out-center behavioral task, with a delay period after the 
    initial hold. 

Examples
--------

>>> import spheres_environment
>>> environment = spheres_environment.Environment()

>>> import rclpy
>>> rclpy.init()

>>> import rclpy.node

>>> from ros_spheres_environment import Server
>>> environment_server_node = rclpy.node.Node('environment_server', 
...                                           namespace='machine')
>>> environment_server = Server(node=environment_server_node, 
...                             environment=environment)

>>> from ros_transitions import Client
>>> task_client_node = rclpy.node.Node('task_client', namespace='machine')
>>> task_client = Client(node=task_client_node)

>>> node = Node()
>>> machine = node.machine
>>> model = node.model

>>> transitioned = model.start_block()
Event: start_block
State: intertrial
>>> model.state == 'intertrial'
True

>>> node.destroy_node()
>>> server_node.destroy_node()
>>> del server
>>> rclpy.shutdown()

"""

# Copyright 2022-2023 Carnegie Mellon University Neuromechatronics Lab (a.whit)
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# Contact: a.whit (nml@whit.contact)


# ros_transitions imports.
import ros_transitions

# delay_out_center_task imports.
from delay_out_center_task import Model
from delay_out_center_task import Machine

# ros_delay_out_center_task imports
from ros_delay_out_center_task.environment import Environment

## Import ros_spheres_environment.
#from ros_spheres_environment import Client

# Import ros_parameters_collections.
import ros_parameter_collections


# Node class.
class Node(ros_transitions.Node):
    """ A ROS2 node that starts a state machine and model for a cursor-based 
        center-out, out-center behavioral task, with a delay period after the 
        initial hold.
    """
    def __init__(self, *args, node_name='delay_out_center_task', **kwargs):
        
        # Invoke the superclass constructor.
        super().__init__(*args, node_name=node_name, **kwargs)
        
        # Initialize a parameters interface.
        parameters = ros_parameter_collections.MutableMapping()
        parameters.node = self
        
        # Initialize the pytransitions state machine.
        self.environment = Environment(node=self) #Environment()
        self.model = Model(environment=self.environment, parameters=parameters)
        self.machine = Machine(model=self.model)
        
        ## Create a cursor.
        #self.environment.initialize_object('cursor')
        
    
  

