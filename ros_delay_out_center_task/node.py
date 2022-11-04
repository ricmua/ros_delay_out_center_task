""" A ROS2 node that starts a state machine and model for a cursor-based 
    center-out, out-center behavioral task, with a delay period after the 
    initial hold. 

Examples
--------

>>> 

"""

# Copyright 2022 Carnegie Mellon University Neuromechatronics Lab (a.whit)
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# Contact: a.whit (nml@whit.contact)


# delay_out_center_task imports.
from delay_out_center_task import Environment
from delay_out_center_task import Model
from delay_out_center_task import Machine

# ros_transitions imports.
import ros_transitions


# Node class.
class Node(ros_transitions.Node):
    """ A ROS2 node that starts a state machine and model for a cursor-based 
        center-out, out-center behavioral task, with a delay period after the 
        initial hold.
    """
    def __init__(self, *args, node_name='delay_out_center_task', **kwargs):
        
        # Initialize the pytransitions state machine.
        self.environment = Environment()
        self.model = Model(environment=self.environment)
        self.machine = Machine(model=self.model)
        
        # Invoke the superclass constructor.
        super().__init__(*args, node_name=node_name, **kwargs)
    
  

