""" Tests conforming with [pytest] framework requirements, for testing the 
    integration of the [ros_delay_out_center_task] package with the 
    [ros_transitions] package.

[pytest]: https://docs.pytest.org
[ros_transitions]: https://github.com/ricmua/ros_transitions
[delay_out_center_task]: https://github.com/ricmua/delay_out_center_task

Usage examples: 

`python3 -m pytest path/to/test`

`pytest test_package::test_module`

`pytest -k test_pattern path/to/test/root`

"""

# Copyright 2022-2023 Carnegie Mellon University Neuromechatronics Lab (a.whit)
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# Contact: a.whit (nml@whit.contact)


# Import ROS2.
import rclpy.task

# Import pytest.
import pytest

# Import ros_transitions.
import ros_transitions

# Import delay_out_center_task.
import delay_out_center_task

# Import fixtures.
from .fixtures import ros
from .fixtures import node
from .fixtures import client_node
#from .fixtures import environment_node
#from .fixtures import environment_client_node
from .fixtures import environment
from .fixtures import parameters
from .fixtures import Timer
from .fixtures import spin
from .fixtures import model
from .fixtures import machine
from .fixtures import trigger
from .fixtures import environment_server
from .fixtures import SPIN_TIMEOUT_SECONDS


# Test baseline conditions for the task node.
def test_node_baseline(node):
    
    # Prepare a topic list.
    topic_map = dict(node.get_topic_names_and_types())
    
    # Assert the baseline state.
    assert node.machine == None
    assert '/machine/trigger' in topic_map
    assert '/machine/event' in topic_map
    assert '/machine/state' in topic_map
  

# Test baseline conditions for the environment interface.
def test_environment_baseline(environment):
    
    # Assert the baseline state.
    assert list(environment) == ['cursor']
    
  

# Test baseline conditions for the state model.
def test_model_baseline(model):
    
    # Assert the baseline state.
    assert model.target_index == None
    assert isinstance(model.targets, list)
    assert len(model.targets) > 0
    
  

# Test baseline conditions for the state machine.
def test_machine_baseline(machine):
    
    # Assert the baseline state.
    assert machine.model.state == 'inactive'
    
  

# Test baseline conditions for the client node.
def test_client_baseline(client_node):
    
    # Assert the baseline state.
    assert client_node.states == []
    assert client_node.events == []
    
  

# Test a single state transition.
def test_node_machine_integration(node, machine):
    
    # Add the machine to the node.
    node.machine = machine
    
  

# Test a ROS2 parameters initialization.
def test_model_parameters_integration(node, parameters):
    
    # Initialize a basic, non-ROS-enabled environment.
    environment = delay_out_center_task.Environment()
    
    # Initialize a basic, non-ROS-enabled model.
    model_a = delay_out_center_task.Model(environment=environment)
    
    # Initialize a basic, non-ROS-enabled model, with a ROS2 parameters 
    # interface.
    model_b = delay_out_center_task.Model(environment=environment, 
                                          parameters=parameters)
    
    # Verify that the parameters match.
    for key in model_a.parameters:
        assert model_a.parameters[key] == model_b.parameters[key]
        assert model_a.parameters[key] == node.get_parameter(key).value
    
    # Verify that a parameter can be changed.
    model_b.parameters['timeout_s.intertrial'] = 1
    assert node.get_parameter('timeout_s.intertrial').value == 1
    
  

# Test ROS2 timer initialization and use.
def test_model_timer_integration(Timer):
    
    # Define a timeout callback to count timeouts.
    #timeout_count = 0
    #def callback(*args, **kwargs): timeout_count += 1
    class Counter:
        def __init__(self): self.count = 0
        def callback(self, *args, **kwargs): self.count += 1
    counter = Counter()
    
    # Initialize a basic environment interface, but with a ROS2 timer.
    environment = delay_out_center_task.Environment(timer=Timer)
    
    # Initialize a basic model using the modified environment.
    model = delay_out_center_task.Model(environment=environment)
    
    # Cause a timeout to occur.
    model.set_timeout(timeout_s=0.100, callback=counter.callback)
    model.timeout_timer.join(timeout=0.100+0.001)
    assert counter.count == 1
    
  

# Test integration with a (remote) spheres environment.
def test_environment_integration(model, environment_server, spin):
    
    # Initialize shorthand.
    environment = environment_server.environment
    
    # Initialize a target sphere.
    model.environment.initialize_object('target')
    
    # Transmit the object intitialization request to the remote environment.
    spin(model.environment.node)
    spin(environment_server.node)
    
    # Verify that the object has been created.
    assert 'target' in environment
    
    # Set a non-home position.
    model.environment.set_position(x=1.0, y=1.0, z=1.0, key='target')
    
    # Transmit the position update request to the remote environment.
    spin(model.environment.node)
    spin(environment_server.node)
    
    # Verify that the position has been updated.
    assert environment['target'].position == dict(x=1.0, y=1.0, z=1.0)
    
    # Via the model, set the home target position.
    model.set_home_target()
    
    # Transmit the position update request to the remote environment.
    spin(model.environment.node)
    spin(environment_server.node)
    
    # Verify that the position has been updated.
    assert environment['target'].position == dict(x=0.0, y=0.0, z=0.0)
    
  

# Test a single state transition.
def test_transition_trigger(node, machine, client_node, spin):
    
    # Add the machine to the node.
    node.machine = machine
    
    # Trigger a state transition (remotely).
    client_node.trigger('start_block')
    
    # Permit the trigger request to be transmitted.
    spin(client_node)
    spin(node)
    
    # Clean up by canceling the intertrial timeout timer.
    node.machine.model.on_exit_intertrial()
    
    # Verify that the trigger was transmitted to the ros_transitions node.
    assert node.machine.model.state == 'intertrial'
    
  

# Test state transition reporting.
def test_transition_reporting(node, machine, client_node, spin):
    
    # Add the machine to the node.
    node.machine = machine
    
    # Trigger a state transition (directly).
    assert node.machine.model.trigger('start_block')
    
    # Permit the state and event to be reported.
    spin(node)
    spin(client_node)
    spin(node)
    spin(client_node)
    
    # Clean up by canceling the intertrial timeout timer.
    node.machine.model.on_exit_intertrial()
    
    # Verify that the transition information was transmitted to the client.
    assert client_node.states == ['intertrial']
    assert client_node.events == ['start_block']
    
  

# Test state transition reporting.
def test_timeout_delta(node, machine, spin):
    
    # Initialize shorthand.
    model = machine.model
    
    # Add the machine to the node.
    node.machine = machine
    
    # Set up a single threaded executor to accurately measure timing.
    import rclpy.executors
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    # Set up a callback for the timer.
    future = rclpy.task.Future()
    clock = node.get_clock()
    def callback(*args, **kwargs): future.set_result(clock.now())
    
    # Set a timer for 100ms.
    delta_s = 0.100
    time_0 = clock.now()
    model.set_timeout(delta_s, callback=callback)
    
    # Wait for the timeout.
    executor.spin_until_future_complete(future)
    time_T = node.get_clock().now()
    
    # Verify that the elapsed time roughly matches the expected timeout 
    # interval (to within 2 ms).
    elapsed_s = (time_T - time_0).nanoseconds / 1e9
    assert abs(delta_s - elapsed_s) < 0.002
    
  

# Test state transition reporting.
# Tests expected state sequence and transition timing.
TIMEOUT_TOLERANCE_S = 0.00125 # 1.25ms maximum delay
def test_success_trial_sequence(node, machine, client_node, environment_server):
    
    # https://docs.ros.org/en/humble/Concepts/About-Executors.html
    
    # Initialize shorthand.
    model = machine.model
    timer = machine.model.timeout_timer
    
    # Add the machine to the node.
    node.machine = machine
    
    # Set up a single threaded executor to accurately measure timing.
    import rclpy.executors
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(client_node)
    
    # Start a block of trials and transition to intertrial state.
    model.start_block()
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    assert client_node.events[-1] == 'start_block'
    assert client_node.states[-1] == 'intertrial'
    
    # Wait for intertrial timeout and transition to move_a.
    executor.spin_until_future_complete(timer._future)
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    assert client_node.events[-1] == 'to_move_a'
    assert client_node.states[-1] == 'move_a'
    elapsed_s = timer._elapsed_ns / 1e9
    expected_s = model.parameters['timeout_s.intertrial']
    assert abs(expected_s - elapsed_s) < TIMEOUT_TOLERANCE_S
    
    # Indicate that the cursor has moved to the target.
    client_node.trigger('target_engaged')
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    assert client_node.events[-1] == 'target_engaged'
    assert client_node.states[-1] == 'hold_a'
    
    # Wait for hold_a timeout and transition to delay_a.
    executor.spin_until_future_complete(timer._future)
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    assert client_node.events[-1] == 'timeout'
    assert client_node.states[-1] == 'delay_a'
    elapsed_s = timer._elapsed_ns / 1e9
    expected_s = model.parameters['timeout_s.hold_a']
    assert abs(expected_s - elapsed_s) < TIMEOUT_TOLERANCE_S
    
    # Wait for delay_a timeout and transition to move_b.
    executor.spin_until_future_complete(timer._future)
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    assert client_node.events[-1] == 'timeout'
    assert client_node.states[-1] == 'move_b'
    elapsed_s = timer._elapsed_ns / 1e9
    expected_s = model.parameters['timeout_s.delay_a']
    assert abs(expected_s - elapsed_s) < TIMEOUT_TOLERANCE_S
    
    # Indicate that the cursor has moved to the target.
    client_node.trigger('target_engaged')
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    assert client_node.events[-1] == 'target_engaged'
    assert client_node.states[-1] == 'hold_b'
    
    # Wait for hold_b timeout and transition to move_c.
    executor.spin_until_future_complete(timer._future)
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    assert client_node.events[-1] == 'timeout'
    assert client_node.states[-1] == 'move_c'
    elapsed_s = timer._elapsed_ns / 1e9
    expected_s = model.parameters['timeout_s.hold_b']
    assert abs(expected_s - elapsed_s) < TIMEOUT_TOLERANCE_S
    
    # Indicate that the cursor has moved to the target.
    client_node.trigger('target_engaged')
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    assert client_node.events[-1] == 'target_engaged'
    assert client_node.states[-1] == 'hold_c'
    
    # Wait for hold_c timeout and transition to success.
    executor.spin_until_future_complete(timer._future)
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    assert client_node.events[-1] == 'timeout'
    assert client_node.states[-1] == 'success'
    elapsed_s = timer._elapsed_ns / 1e9
    expected_s = model.parameters['timeout_s.hold_c']
    assert abs(expected_s - elapsed_s) < TIMEOUT_TOLERANCE_S
    
    # Transition to intertrial.
    executor.spin_until_future_complete(timer._future)
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    executor.spin_once(timeout_sec=SPIN_TIMEOUT_SECONDS)
    assert client_node.events[-1] == 'end_trial'
    assert client_node.states[-1] == 'intertrial'
    
  

# __main__
if __name__ == '__main__':
    pytest.main(['test_integration.py'])
    
  


