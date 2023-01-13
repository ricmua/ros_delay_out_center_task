""" Test [fixtures] for using the [pytest] framework to test the integration of 
    the [ros_delay_out_center_task] package with the [ros_transitions] package.

[ros_transitions]: https://github.com/ricmua/ros_transitions
[delay_out_center_task]: https://github.com/ricmua/delay_out_center_task
[fixtures]: https://docs.pytest.org/en/6.2.x/fixture.html
[pytest]: https://docs.pytest.org

Examples
--------

>>> 

"""

# Copyright 2022-2023 Carnegie Mellon University Neuromechatronics Lab (a.whit)
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# Contact: a.whit (nml@whit.contact)


# Import pytest.
import pytest

# Import ROS.
import rclpy
import rclpy.node

# Import spheres_environment.
import spheres_environment

# Import ros_transitions
import ros_transitions

# Import ros_spheres_environment.
import ros_spheres_environment

# Import delay_out_center_task.
import delay_out_center_task

# Import ros_delay_out_center_task.
import ros_delay_out_center_task

# Import ros_parameter_collections.
import ros_parameter_collections

# Import ros_threading_timer.
import ros_threading_timer


# Initialize a ROS2 interface.
@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield rclpy
    rclpy.shutdown()
    
  

# Initialize a function for spinning ROS2 nodes once, without blocking.
SPIN_TIMEOUT_SECONDS = 0.002
@pytest.fixture
def spin(ros):
    def spin(node): rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT_SECONDS)
    yield spin
    
  

# Initialize a ros_transitions node.
@pytest.fixture
def node(ros):
    
    # Initialize a ros_transitions node.
    node = ros_transitions.Node()
    
    # Yield the product.
    yield node
    
    # Cleanup.
    node.destroy_node()
    
  

# Initialize a ros_transitions client node.
@pytest.fixture
def client_node(ros):
    
    # Initialize a ros_transitions node.
    node = ros_transitions.Client()
    
    # Yield the product.
    yield node
    
    # Cleanup.
    node.destroy_node()
    
  

# Initialize a timer generator.
@pytest.fixture
def Timer(node):
    
    # Re-define the TimerWrapper class to add a future.
    class TimerWrapper(ros_threading_timer.TimerWrapper):
        def __call__(self, *args, **kwargs):
            self._future = rclpy.task.Future()
            self._time_0 = self._node.get_clock().now()
            return super().__call__(*args, **kwargs)
        def _callback_wrapper(self, *args, **kwargs):
            tT = self._time_T = self._node.get_clock().now()
            self._elapsed_ns = (self._time_T - self._time_0).nanoseconds
            self._future.set_result(self._elapsed_ns)
            return super()._callback_wrapper(*args, **kwargs)
    
    # Initialize a stopped ROS2 timer.
    ros_timer = node.create_timer(timer_period_sec=1, callback=lambda: None)
    ros_timer.cancel()
    
    # Wrap the ROS2 timer and node.
    # If the node is not specified, the `join` method may fail.
    Timer = TimerWrapper(ros_timer, node=node)
    #Timer = ros_threading_timer.one_shot.TimerWrapper(ros_timer, node=node)
    
    # Yield the product.
    yield Timer
    
    # Cleanup.
    del Timer
    
  

# Initialize a virtual environment interface.
@pytest.fixture
def environment(node, Timer):
    environment = ros_delay_out_center_task.Environment(node=node, Timer=Timer)
    yield environment
    del environment
    
  

# Initialize a parameters interface.
@pytest.fixture
def parameters(node):
    
    # Initialize an empty parameters interface.
    parameters = ros_parameter_collections.MutableMapping()
    
    # Associate the parameter mapping with a ROS2 node.
    parameters.node = node
    
    # Yield the product.
    yield parameters
    
    # Cleanup.
    del parameters
    
  

# Initialize a sample pytransitions model.
@pytest.fixture
def model(environment, parameters):
    
    # Initialize the model using the provided environment and parameters 
    # interfaces.
    model = delay_out_center_task.Model(environment=environment, 
                                        parameters=parameters)
    
    # Update the timeout parameter for the first active state (intertrial).
    # This allows time for processing and cancellation of the intertrial 
    # timeout timer, in order to facilitate simple isolated tests.
    model.parameters['timeout_s.intertrial'] = 0.500
    
    # Yield the product.
    yield model
    
    # Cleanup.
    del model
    
  

# Initialize a sample pytransitions state machine.
@pytest.fixture
def machine(model):
    
    # Initialize the state machine, using the provided model.
    machine = delay_out_center_task.Machine(model=model)
    
    # Yield the product.
    yield machine
    
    # Cleanup.
    del machine
    
  

# Set up a client ROS2 node for interacting with the ros_transitions node.
@pytest.fixture
def client(ros):
    
    # Initialize a ROS2 client node.
    client_node = ros_transitions.Client()
    
    # Yield the product.
    yield client_node
    
    # Clean up.
    client_node.destroy_node()
    
  

# Set up a client ros_spheres_environment server, to receive messages sent from 
# the task environment interface.
@pytest.fixture
def environment_server(ros):
    
    # Initialize a spheres_environment.Environment.
    environment = spheres_environment.Environment()
    
    # Initialize a ROS2 server node.
    # Use the task namespace.
    server_node = rclpy.node.Node('environment', namespace='machine')
    
    # Initialize a ros_spheres_environment server.
    Server = ros_spheres_environment.Server
    server = Server(node=server_node, environment=environment)
    
    # Yield the product.
    yield server
    
    # Clean up.
    del server
    server_node.destroy_node()
    
  

# Effect a state transition by triggering an event.
@pytest.fixture
def trigger(node, client, spin):
    
    # Define an event trigger function.
    def trigger(key):
      
      # Publish a trigger request to the ROS2 graph.
      client.trigger(key)
      spin(client)
      
      # Allow the ros_transitions node to process the request.
      spin(node)
      state = node.machine.model.state
      
      # Allow the ros_transitions node to publish the results.
      spin(node)
      spin(client)
      spin(node)
      spin(client)
      client_event = client.events[-1]
      client_state = client.states[-1]
      
      # Return the result record.
      return dict(state=state, 
                  client_state=client_state, 
                  client_event=client_event)
    
    yield trigger
    
  

# __main__
if __name__ == '__main__':
    import doctest
    doctest.testmod()
    
  

