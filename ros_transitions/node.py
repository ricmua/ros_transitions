""" This module defines a node_ that interfaces the ROS2 graph_ with a state 
    machine.

.. _node: https://docs.ros.org/en/galactic/Tutorials
          /Understanding-ROS2-Nodes.html
.. _graph: https://docs.ros.org/en/galactic/Tutorials
           /Understanding-ROS2-Nodes.html#the-ros-2-graph

Examples
--------

Create a pytransitions state machine based on the phases of matter. This 
example is derived from the pytransitions documentation_:

>>> import transitions
>>> class Matter: pass
>>> model = Matter()
>>> machine = transitions.Machine(model=model, states=['solid', 'liquid'])
>>> machine.add_transition('melt', source='solid', dest='liquid')
>>> machine.add_transition('freeze', source='liquid', dest='solid')
>>> machine.set_state('solid')

Next, initialize the ROS node:

>>> import rclpy
>>> import example_interfaces.msg
>>> import ros_transitions
>>> rclpy.init()
>>> node = ros_transitions.Node(machine=machine)

Test the initialized state machine and node by triggering a phase 
transition (via the model interface):

>>> result = model.melt()

This should cause the melt event and state transition to be logged.

In order to confirm that each state transition is accompanied by the 
publication of a ROS message, set up a second configured_ ROS2 environment, in 
order to create a client that listens for state changes. To monitor the state 
topic, run the following command at the shell command prompt::

  ros2 topic echo machine/state example_interfaces/msg/String

No feedback is returned. The client application should simply wait for messages. 

In the Python interpreter running in the first ROS2 environment, trigger a 
second and third phase transition:

>>> result = model.freeze()
>>> result = model.melt()

This should complete without error. The client application should show the 
following output:: 

  data: solid
  ---
  data: liquid
  ---

This indicates that the state machine transitioned from the liquid state to 
the solid state, and then back again. ROS2 messages were sent after each 
phase transition, and received by the ROS2 client. Event messages are also 
sent (on the ``machine/event`` topic), but are not shown in this example.

To clean up, run the following commands in the Python interpreter:

>>> rclpy.spin_once(node, timeout_sec=1)
>>> node.destroy_node()
>>> rclpy.shutdown()

Type Ctrl-C to kill the client application, and then exit both ROS2 
environments.

.. _configured: https://docs.ros.org/en/galactic/Tutorials/Configuring-ROS2-Environment.html
.. _doctests: https://docs.python.org/3/library/doctest.html#module-doctest
.. _documentation: https://github.com/pytransitions/transitions#basic-initialization
"""

# Copyright 2022 Carnegie Mellon University Neuromechatronics Lab (a.whit)
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
# 
# Contact: a.whit (nml@whit.contact)

# Import relevant ROS2 packages.
import rclpy
import rclpy.node

# Define ROS2 message types.
from example_interfaces.msg import String as state_message
from example_interfaces.msg import String as event_message

# Define the ROS2 Quality-of-Service pre-set profiles.
DEFAULT_QOS = rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value

# Define the node class.
class Node(rclpy.node.Node):
    """ ROS2 node for interfacing with a state machine.
    
    This class implements a ROS2 node_ that can mediate interaction between 
    a Python state machine and other nodes on the ROS2 graph_. The state 
    machine is implemented via the pytransitions_ package.

    .. _node: https://docs.ros.org/en/galactic/Tutorials
              /Understanding-ROS2-Nodes.html
    .. _graph: https://docs.ros.org/en/galactic/Tutorials
              /Understanding-ROS2-Nodes.html#the-ros-2-graph
    .. _pytransitions: https://github.com/pytransitions/transitions#-transitions
    
    .. todo:: Add parameter access.
    
    Parameters
    ----------
    *args :
        Arguments for `rclpy.node.Node`_.
        
        .. _rclpy.node.Node: https://docs.ros2.org/latest/api/rclpy/api/node.html
    node_name : str
        Node name_ of the node on the ROS2 graph. See the documentation for 
        `rclpy.node.Node`_.
        
        .. _name: http://wiki.ros.org/Names
        .. _rclpy.node.Node: https://docs.ros2.org/latest/api/rclpy
                             /api/node.html
    namespace : str
        Namespace_ of the node on the ROS2 graph. See the documentation for 
        `rclpy.node.Node`_.
        
        .. _Namespace: http://design.ros2.org/articles
                       /topic_and_service_names.html#namespaces
        .. _rclpy.node.Node: https://docs.ros2.org/latest/api/rclpy
                             /api/node.html
    machine : transitions.Machine
        A pytransitions `state machine`_. If left unspecified, initialization 
        of the state machine is delayed.
        
        .. _state machine: https://github.com/pytransitions/transitions/blob
                           /3836dc4ec5e09e101057bb335b940ef810b87513
                           /transitions/core.py#L470
    **kwargs :
        Keyword arguments for `rclpy.node.Node`_.
        
        .. _rclpy.node.Node: https://docs.ros2.org/latest/api/rclpy
                             /api/node.html
        
    """
    
    # In the current implementation, the ROS node is maintained as an object 
    # that is separate from the state machine and model. This is not strictly
    # necessary. It was here done to avoid clobbering member functions and
    # attribute names, since pytransitions initializes convenience functions 
    # and attributes in the model object. Maintaining two separate objects also 
    # avoids multiple inheritance issues.
    
    def __init__(self, *args, node_name='node',
                              namespace='machine',
                              machine=None, **kwargs):
        super().__init__(*args, node_name=node_name, 
                                namespace=namespace, **kwargs)
        self.initialize_publishers()
        if machine: self.machine = machine
        
    @property
    def machine(self):
        """ A pytransitions_ state machine.
        
        The ``initialize_machine`` member function is invoked when this 
        attribute is set.
        
        .. _pytransitions: https://github.com/pytransitions
                           /transitions#-transitions
        """
        self._machine = getattr(self, '_machine', None)
        return self._machine
    
    @machine.setter
    def machine(self, value):
        # This is useful for setting the state machine after initialization.
        # This helps to decouple the node and the state machine, which are also
        # intertwined with the model.
        self._machine = value
        self.initialize_machine()
        
    
    def initialize_publishers(self):
        """ Create publishers for reporting task state changes and events.
        
        Publishers for the following topics are initialized:
        
        * ``machine/state``
        * ``machine/event``
        
        If a ROS2 namespace is specified in this class' constructor, then 
        ``machine`` is replaced by that namespace in these topic names.
        
        The publishers use the "`system default`_" 
        `Quality-of-Service`_ settings.
        
        .. _system default: https://docs.ros.org/en/galactic/Concepts
                            /About-Quality-of-Service-Settings.html#qos-profiles
        .. _Quality-of-Service: https://docs.ros.org/en/galactic/Concepts
                                /About-Quality-of-Service-Settings.html
        """
        # Create a publisher for reporting task state changes.
        kwargs = dict(topic=f'state',
                     msg_type=state_message,
                     qos_profile=DEFAULT_QOS)
        self.state_publisher = self.create_publisher(**kwargs)
        
        # Create a publisher for reporting task events / triggers.
        kwargs = dict(topic=f'event',
                     msg_type=state_message,
                     qos_profile=DEFAULT_QOS)
        self.event_publisher = self.create_publisher(**kwargs)
        
    def initialize_machine(self):
        """
        Configure a pytransitions state machine for interfacing with this node.
        
        The state machine is configured to invoke callbacks in order to log 
        trigger events and states changes.
        """
        # Note that the pytransitions documentation discusses setting these 
        # callbacks during machine initialization. However, it seems clear from 
        # the pytransitions code (transitions/core.py) that accessor methods are
        # can be used to manipulate the callback lists after the class has been
        # constructed.
        # The ``send_event`` attribute must be True, in order to receive event
        # data. This is necessary for identifying (and suppressing log messages
        # for) internal transitions. See the log_state_change function for 
        # further information.
        # https://github.com/pytransitions/transitions#internal-transitions
        self.machine.send_event = True
        self.machine.prepare_event += [self.log_event]
        self.machine.after_state_change += [self.log_state_change]
        
    def log(self, message, severity=None):
        """ Record a message in the ROS log.
        
        Parameters
        ----------
        message : str
            The message to record.
        severity : str
            `ROS2 logging`_ severity level. Defaults to INFO_.
            
            .. _ROS2 logging: https://docs.ros.org/en/galactic/Concepts
                              /About-Logging.html
            .. _INFO: https://docs.ros2.org/dashing/api/rclpy/api
                      /logging.html#rclpy.logging.LoggingSeverity
        """
        log = self.get_logger()
        log_function = getattr(log, severity.lower()) if severity else log.info
        log_function(message)
        
    def log_state_change(self, event_data):
        """ Log changes to the state machine state.
        
        This is a callback_ function that the pytransitions core invokes after 
        each state transition. The state transition triggers two actions:
        
        * The state change is recorded to the ROS2 log.
        * The *new* state is published to the ``state`` ROS topic.
        
        `Internal transitions`_ are *NOT* logged. To suppress messages sent to 
        the internal log, adjust the ROS log severity_.
        
        .. _callback: https://github.com/pytransitions/transitions#callbacks-1
        .. _Internal transitions: https://github.com/pytransitions
                                  /transitions#internal-transitions
        .. _severity: https://docs.ros.org/en/galactic/Concepts
                      /About-Logging.html
        
        Parameters
        ----------
        event_data : pytransitions event_data
            Callback argument_ provided by the pytransitions core.
            
            .. _argument: https://github.com/pytransitions
                          /transitions#callbacks-1
        """
        # The pytransitions Machine must be configured such that 
        # ``send_event = True``.
        state = self.machine.model.state
        self.log(f'State: {state}')
        is_internal = not bool(event_data.transition.dest)
        message = state_message(data=state)
        if not is_internal: self.state_publisher.publish(msg=message)
        
    def log_event(self, event_data):
        """ Log events or triggers that cause changes in the state machine.
        
        This is a callback_ function that the pytransitions core invokes when  
        a relevant event occurs. The event triggers two actions:
                
        * The event is recorded to the ROS2 log.
        * The event is published to the ``event`` ROS topic.
        
        To suppress messages sent to the internal log, adjust the ROS log 
        severity_.
        
        .. _callback: https://github.com/pytransitions/transitions#callbacks-1
        .. _severity: https://docs.ros.org/en/galactic/Concepts
                      /About-Logging.html
        
        Parameters
        ----------
        event_data : pytransitions event_data
            Callback argument_ provided by the pytransitions core.
            
            .. _argument: https://github.com/pytransitions
                          /transitions#callbacks-1
        """
        # The pytransitions Machine must be configured such that 
        # ``send_event = True``.
        self.log(f'Event: {event_data.event.name}')
        message = event_message(data=event_data.event.name)
        self.event_publisher.publish(msg=message)
    
  

if __name__ == '__main__':
    import doctest
    option_flags = doctest.ELLIPSIS | doctest.NORMALIZE_WHITESPACE
    doctest.testmod(optionflags=option_flags)

