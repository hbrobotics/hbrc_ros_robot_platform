"""Serial Agent Program.

<!-- ===================================== 100 Characters ====================================== -->

# Serial Agent Program.

The serial agent mirrors traffic between robot computer running native ROS2 and a microcontroller.
The traffic between the robot computer and the microcontroller is encoded as byte streams over
a bidirectional serial line.  In addition, the serial agent generates a C code framework that
is used to encode/decode/manage the serial line message traffic.  The code is currently focused
on the FreeRTOS real time operating system, but other code generators are possible (e.g. Zephyr.)

## License

MIT License

Copyright 2021 Home Brew Robotics Club

Permission is hereby granted, free of charge, to any person obtaining a copy of this
software and associated documentation files (the "Software"), to deal in the Software
without restriction, including without limitation the rights to use, copy, modify,
merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be included in all or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.

## Overview

The documentation below discusses the command line arguments, class structure, execution
model, packet format, and code generation (TBD).

### Command Line

The current intention is do all configuration from the command line.  This avoids a
separate configuration file and instead allows all configuration to be done via ROS launch files.

There are 5 related commands that specify various mirroring operations that are very similar

* `--publisher=NODE_NAME:PRIORITY:IMPORT_PATH:TYPE_NAME:ROS_PATH`:
  The microcontroller can publish messages to a ROS Topic.

* `--subscription=NODE_NAME:PRIORITY:IMPORT_PATH:TYPE_NAME:ROS_PATH`:
  The microcontroller can subscribe ROS Topic messages.

* `--client=NODE_NAME:PRIORITY:IMPORT_PATH:TYPE_NAME:ROS_PATH`:
  The microcontroller is a client can request information from any other ROS server node.

* `--server=`NODE_NAME::PRIORITY:IMPORT_PATH:TYPE_NAME:ROS_PATH:PRIORITY`:
  The microcontroller is a server that respond with information from any other ROS server Node.

* `--timer=`NODE_NAME:PRIORITY:TIMER_NAME:RATE`:
  The microcontroller can receive periodic timer messages.

(TBD Parameter support)

The parameter to the right of the equal sign ("=") a list of colon (":") separated values
that are listed immediately below:

* `NODE_NAME`:
  This is the name of a `rospy` Node.  Since each Node can share multiple communication kinds,
  this node name occur more than one of the flags above (e.g. multiple publishers, subscriptions,
  etc.)  This node shows up as a separate node in the ROS `rqt` program.

* `IMPORT_PATH`: This name of a Python package that contains the needed message definitions
  (e.g. `std_msgs.msg`, `example_interfaces.srv`, etc.)

* `TYPE_NAME`: In rospy, the message format is specified by Python type.  In conjunction with the
  `IMPORT_PATH`, it is possible for Python to dynamically load in the message format information.
  (e.g. `String`, `AddTwoInts`, etc.)

* `ROSPATH`: This the ROS topic or service name (e.g. `sonars`, `motor/left`, `motor/right`, etc.)

* `TIMER_NAME`: Is the name of the timer.

* `RATE`: This species the timer period as a rate in Hz.

* `PRIORITY`: This specifies the message priority where the higher priority messages are always
  sent first.  At least one timer is required to ensure that the protocol will recover from
  dropped messages.  This timer needs to have the highest priority.  An emergency stop message
  probably has the next highest priority.

The next argument supports serial communication.  The serial communication can be done
with either a physical device (e.g. `/dev/ttyN`, `/dev/AMC`, etc.) or a named pipe/fifo (e.g.
`/tmp/agent2micro`, `/tmp/micro2agent`).  The following command line argument must be specified
at least once (for read/write) or twice (one for read, the other for write).

* `--io=FILE_NAME:FLAGS,...` :
   `FILE_NAME` is the file name for the device or named pipe.  `FLAGS` must be one or more of:

   * `create`: The pipe is created if it does not already exist.
   * 'device`: The a device is specified.
   * 'pipe`: The a pipe is specified
   * `read`: Agent reads from the device/pipe.
   * `write`: Agent write to the device/pipe.
   * More flags r

Code generation:

TBD


## Class Summary

The classes are:

* Agent: The top level class that reads the command line and create the associated ROS
  Node's, Client's, Publisher's, Server's, Subscriptions, Timer's etc.

* AgentNode: A subclass of rclpy.Node.  This is the rclpy approved way of managing ROS nodes.

* Packet: A class for encoding decoding packages over a serial line.

* Handle, TopicHandle, ServiceHandle: These 3 classes uniquely define either a ROS Topic
  (using TopicHandle) or a ROS service (using ServiceHandle).  These classes can
  read in the appropriate type information to support packet encoding and decoding.

* TimerHandle: Similar to the Handle base class, but for managing timers.

### Execution Model

The rclpy main loop supports multiple different execution strategies.  The one used for this
program is *MultiThreadedExectuor* with *ReentrantCallabkGroup*.  What this means is that under
the covers, rospy constructs a pool of Python threads and it allows them to run concurrently.
The threads are mostly used to process callbacks from the ROS communication protocols.  Due to
concurrent threads execution, locks and queues are needed to protect against accidentally
data corruption.  (A `# CONCURRENT` comment is used to flag methods that run concurrently.)

The client and server message traffic require a round trip message over the serial line.
The rospy callback that initiates this traffic will need to wait for a response message
to come back via the microcontroller serial line.  This will stall the Python thread running
the callback.  Ultimately, this means that enough Python threads need to be allocated to
ensure stalled threads do not freeze out the publish/subscribe message traffic.  (A `# CAN STALL`
comment is used to flag these methods.)

### Packet Format

There are two dedicated Python threads for interfacing to the serial line.  One thread sends
serial line traffic and the other receives serial line traffic.  These two threads communicate
via Python Queues.  There is a protocol similar to SLIP (Serial Line Internet Protocol) that
does the frames the messages.  There is a CRC (Cyclic Redundancy Check) to detect damaged
messages which then get transmitted again.  In order to simplify message buffer memory management
on the microcontroller, the total message byte length is sent at the front of each message.

Any messages that are the wrong length or do not match the CRC are dropped to force
and the transmission control system will get them transmitted again.  All of the
transmission control is managed by the robot host to simplify the microcontroller code.

### Code Generation

To be designed.

### Miscellaneous

This code is run through both `mypy` and `flake8 --max-length_line=100 -ignore=Q00`.

## References

* rclpy Documentation:
  * [https://docs.ros2.org/latest/api/rclpy/](General rclpy documentation)

* Asyncio:
  Python asynico has advantages/disadvantages and is semi-supported by ROS2.  The short summary
  is that it can work, but it has issues.  The ROS2 team is focusing is on multi-threading:
  * [https://github.com/ros2/rclpy/issues/279](See close Comment by sloretz)
  * [https://github.com/ros2/rclpy/blob/b72a05bd3fb3ac7d27a8da359571afbbfec07f19/
    rclpy/test/test_executor.py#L154](Note: URL spans 2 line: Test of asyncio in rclpy)
  * [https://answers.ros.org/question/362598/asyncawait-in-subscriber-callback/](random link)
  * [https://answers.ros.org/question/343279/ros2-how-to-implement-a-sync-service-client-in-a-node/]
    (another random link)

* Miscellaneous
  * [https://nicolovaligi.com/articles/
    concurrency-and-parallelism-in-ros1-and-ros2-application-apis/](A paper about this stuff)
  * [https://answers.ros.org/question/373169/
     mistakes-using-service-and-client-in-same-node-ros2-python](Informative)
"""

import os
import stat
import importlib
import struct
import sys
# import time
from dataclasses import dataclass
from typing import Any, Callable, ClassVar, Dict, List, IO, Optional, Tuple

import crcmod  # type:ignore

import rclpy  # type:ignore
from rclpy.client import Client  # type: ignore
from rclpy.executors import Executor  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.publisher import Publisher  # type: ignore
from rclpy.service import Service as Server  # type: ignore
from rclpy.subscription import Subscription  # type: ignore
from rclpy.task import Future  # type: ignore
from rclpy.timer import Timer  # type: ignore


# main():
def main(ros_arguments: Optional[List[str]] = None) -> int:
    """Run the main program."""
    # Initialize the Python ROS2 interface library:
    if ros_arguments is None:
        ros_arguments = []
    rclpy.init(args=ros_arguments)

    # Create the *executor*:
    executor: Executor = rclpy.executors.MultiThreadedExecutor()

    # Create the *agent* from command line *arguments*:
    arguments: List[str] = sys.argv[1:]
    agent: Agent = Agent(executor, arguments)

    # agent.timer_create("serial_agent", "timer1", 1.0)
    # agent.timer_create("serial_agent", "timer2", 5.0)

    print("Start executor.spin()")
    try:
        executor.spin()
    except KeyboardInterrupt:
        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Keyboard Interrupt")

    print("Destroying nodes:")
    agent.nodes_destroy()

    # Shut down the Python ROS2 interface library.
    rclpy.shutdown()

    return 0


@dataclass(order=True, frozen=True)
class ROSFieldType:
    """Class that represents information about a ROS Type."""

    ros_type: str  # ROS type (e.g. "bool", "int8", ..., "duration")
    cpp_type: str  # C++ type (e.g. "uint8_t", "int8_t", ..., "ros::Duration")
    python_type: str  # Python type (e.g. "bool", "int", ..., "rospy.Duration")
    struct_format: str  # struct type (e.g. "B", "b", ..., "ii")
    c_type: str  # C type (e.g. "unsigned char", "signed char", ..., "2 * signed")
    size: int  # Size in bytes (e.g. (1, 2, ..., 8)


# Kludge: Just access this function using a global declaration.
crc_compute: Callable[[bytes], int] = crcmod.mkCrcFun(0x11021, initCrc=0, xorOut=0xffff)


# Agent:
class Agent:
    """Class that implements a ROS serial agent Node."""

    # Agent.__init__():
    def __init__(self, executor: Executor, arguments: List[str]) -> None:
        """Init the serial agent."""
        # Collect the various objects in tables to detect accidental duplicates and
        # for final shut-down:
        self.agent_nodes: Dict[str, AgentNode] = {}
        self.executor: Executor = executor
        self.service_handles: Dict[HandleKey, ServiceHandle] = {}
        self.timer_handles: Dict[Tuple[str, str], TimerHandle] = {}
        self.topic_handles: Dict[HandleKey, TopicHandle] = {}

        self.clients_table: Dict[Tuple[str, str], Tuple[AgentNode, ServiceHandle]] = {}
        self.publishers_table: Dict[Tuple[str, str], Tuple[AgentNode, TopicHandle]] = {}
        self.servers_table: Dict[Tuple[str, str], Tuple[AgentNode, ServiceHandle]] = {}
        self.subscriptions_table: Dict[Tuple[str, str], Tuple[AgentNode, TopicHandle]] = {}
        self.timers_table: Dict[Tuple[str, str], Tuple[AgentNode, TimerHandle]] = {}

        self.packet_handles: List[Handle] = []
        self.packet_id: int = 0

        self.read_io_channel: Optional[IOChannel] = None
        self.write_io_channel: Optional[IOChannel] = None

        # Parse *arguments*:
        if not arguments:
            # Create some default arguments.
            arguments = [
                "--publisher=serial_agent:0:std_msgs.msg:String:topic",
                "--subscription=serial_agent:0:std_msgs.msg:String:topic",
                "--publisher=serial_agent:0:std_msgs.msg:String:echo",
                "--subscription=serial_agent:0:std_msgs.msg:String:echo",
                "--client=serial_agent:0:example_interfaces.srv:AddTwoInts:add_two_ints",
                "--server=serial_agent_server1:0:example_interfaces.srv:AddTwoInts:add_two_ints",
                "--timer=serial_agent_timer:10:timer:0.5",
                "--io=/tmp/agent2micro:create,pipe,write",
                "--io=/tmp/micro2agent:create,pipe,read",
            ]
        self.arguments_parse(arguments)
        # print(f"arguments: {arguments}")

    # Agent.agent_node_create_once():
    def agent_node_create_once(self, agent_node_name: str) -> "AgentNode":
        """Ensure that there is only one instance of an AgentNode."""
        agent_nodes: Dict[str, AgentNode] = self.agent_nodes
        agent_node: AgentNode
        if agent_node_name in agent_nodes:
            agent_node = agent_nodes[agent_node_name]
        else:
            agent_node = AgentNode(agent_node_name, self)
            agent_nodes[agent_node_name] = agent_node
            assert self.executor.add_node(agent_node), (
                f"Unable to add {agent_node_name} to executor")
        return agent_node

    # Agent.agent_node.lookup():
    def agent_node_lookup(self, agent_node_name: str) -> "AgentNode":
        """Lookup an AgentNode."""
        agent_nodes: Dict[str, AgentNode] = self.agent_nodes
        assert agent_node_name in agent_nodes, f"Unable to find '{agent_node_name}'"
        agent_node: AgentNode = agent_nodes[agent_node_name]
        return agent_node

    # Agent.arguments_parse():
    def arguments_parse(self, arguments: List[str]) -> None:
        """Parse the command line arguments."""
        # Sweep through the *arguments*:
        print("=>Agent.arguments_parse(*)")
        argument: str
        for argument in arguments:
            # Split out the *flag_name*, *flag_options*, *colon_split*:
            if not argument.startswith("--"):
                raise ValueError(f"Command line argument '{argument}' does not start with '--'")
            equals_split: Tuple[str, ...] = tuple(argument.split("="))
            assert len(equals_split) == 2, (
                f"More than one '=' in command line argument '{argument}'")
            flag_name: str
            flag_options: str
            flag_name, flag_options = equals_split
            colon_split: Tuple[str, ...] = tuple(flag_options.split(":"))

            # Dispatch on *flag_name*:
            agent_node_name: str
            priority: str
            priority_value: int
            if flag_name in ("--client", "--publisher", "--server", "--subscription"):
                # Unpack *colon_split* into *agent_name*, *priority*, *import_path*,
                # *type_name*, and *ros_path*:
                if len(colon_split) != 5:
                    raise ValueError(
                        f"Commad line argument '{argument}' has {len(colon_split)} instead "
                        "of the desired 5 -- NODE_NAME:PRIORITY:IMPORT_PATH:TYPE_NAME:ROSPATH")
                import_path: str
                type_name: str
                ros_path: str
                agent_node_name, priority, import_path, type_name, ros_path = colon_split

                # Verify that *priority* is an integer and fits into a byte:
                if not priority.isdigit():
                    raise ValueError(f"Priority '{priority}' is not an integer")
                priority_value = int(priority)
                if not 0 <= priority_value <= 255:
                    raise ValueError(f"Priority '{priority}' does not fit in a byte")

                # Create communication channel:
                if flag_name == "--publisher":
                    self.publisher_create(agent_node_name, import_path, type_name, ros_path)
                elif flag_name == "--subscription":
                    self.subscription_create(agent_node_name, import_path, type_name, ros_path)
                elif flag_name == "--client":
                    self.client_create(agent_node_name, import_path, type_name, ros_path)
                elif flag_name == "--server":
                    self.server_create(agent_node_name, import_path, type_name, ros_path)
                else:
                    assert False, f"'{flag_name}' is not an exceptable kind."
            elif flag_name == "--timer":
                # Unpack *colon_split* into *node_name*, *priority*, and *rate*:
                if len(colon_split) != 4:
                    raise ValueError(f"Command line argument '{argument}' has {len(colon_split)} "
                                     f"options instead of the desired 4 -- "
                                     "NODE_NAME:PRIORITY:TIMER_NAME:RATE")
                rate: str
                timer_name: str
                agent_node_name, priority, timer_name, rate = colon_split

                # Verity argument types:
                if not priority.isdigit():
                    raise ValueError(f"Command line argument '{argument}' priority is '{priority}' "
                                     "which is not an integer")
                priority_value = int(priority)
                rate_value: float
                try:
                    rate_value = float(rate)
                except ValueError:
                    raise ValueError(f"Command line argument '{argument}' rate is '{rate}' "
                                     "which is not an float")
                if rate_value <= 0.0:
                    raise ValueError(f"Command line argument '{argument}' rate is '{rate}' "
                                     "is not positive")

                # Create the timer:
                self.timer_create(agent_node_name, timer_name, 1 / rate_value)
            elif flag_name == "--io":
                print(f"processing '{argument}'")
                # Extract *colon_split* into *file_name* and *flags*:
                io_channel: IOChannel = IOChannel(argument, colon_split)
                if io_channel.read:
                    if self.read_io_channel:
                        raise ValueError(f"Command line argument '{argument}' tries to open "
                                         "more than one read I/O channel")
                    else:
                        self.read_io_channel = io_channel
                if io_channel.write:
                    if self.write_io_channel:
                        raise ValueError(f"Command line argument '{argument}' tries to open "
                                         "more than one writeI/O channel")
                    else:
                        self.write_io_channel = io_channel
            else:
                raise ValueError(f"Unrecognized flag '{flag_name}'")

        # Open the I/O channels:
        # print(f"read_io_channel:{self.read_io_channel}")
        # print(f"write_io_channel:{self.write_io_channel}")
        if self.read_io_channel and not self.write_io_channel:
            raise ValueError(f"Read channel '{self.read_io_channel.file_name}' is specified "
                             "but no write channel is specified")
        if self.write_io_channel and not self.read_io_channel:
            raise ValueError(f"Write channel '{self.write_io_channel.file_name}' is specified "
                             "but no read channel is specified")
        print("<=Agent.arguments_parse(*)")

    # Agent.client_create():
    def client_create(self, agent_node_name: str,
                      import_path: str, type_name: str, ros_path: str) -> None:
        """Create a client for a ROS service."""
        # print(f"=>client_create()")
        agent_node: AgentNode = self.agent_node_create_once(agent_node_name)
        service_handle: ServiceHandle = self.service_handle_get(import_path, type_name, ros_path)
        agent_node.client_create(service_handle)

        # Record information into the *clients_table*:
        key: Tuple[str, str] = (agent_node_name, ros_path)
        clients_table: Dict[Tuple[str, str],
                            Tuple[AgentNode, ServiceHandle]] = self.clients_table
        assert key not in clients_table, f"Duplicate client {key}"
        clients_table[key] = (agent_node, service_handle)
        # print(f"<=client_create(): key={key})")

    # Agent.packet_id_get():
    def packet_id_get(self, handle: "Handle") -> int:
        """Return the next packet id."""
        packet_id: int = self.packet_id
        packet_handles: List[Handle] = self.packet_handles
        assert len(packet_handles) == packet_id, "packet_handels is broken"
        packet_handles.append(handle)
        self.packet_id += 1
        return packet_id

    # Agent.nodes_destroy():
    def nodes_destroy(self) -> None:
        """Destroy all of the Agent nodes."""
        # print("=>Agent.destroy_nodes()")
        agent_node: Node
        for agent_node in self.agent_nodes.values():
            agent_node.destroy_node()
        # print("<=Agent.destroy_nodes()")

    # Agent.publisher_create():
    def publisher_create(self, agent_node_name: str,
                         import_path: str, type_name: str, ros_path: str) -> None:
        """Create a publisher."""
        # Create the publisher:
        agent_node: AgentNode = self.agent_node_create_once(agent_node_name)
        topic_handle: TopicHandle = self.topic_handle_get(import_path, type_name, ros_path)
        agent_node.publisher_create(topic_handle)

        # Record information into *publishers_table*:
        key: Tuple[str, str] = (agent_node_name, ros_path)
        publishers_table: Dict[Tuple[str, str],
                               Tuple[AgentNode, TopicHandle]] = self.publishers_table
        assert key not in publishers_table, f"Duplicate publisher {key}"
        publishers_table[key] = (agent_node, topic_handle)

    # Agent.server_create():
    def server_create(self, agent_node_name: str,
                      import_path: str, type_name: str, ros_path: str) -> None:
        """Create a server for a ROS service."""
        # Create the server:
        agent_node: AgentNode = self.agent_node_create_once(agent_node_name)
        service_handle: ServiceHandle = self.service_handle_get(import_path, type_name, ros_path)
        agent_node.server_create(service_handle)

        # Record informatin into the *servers_table*:
        key: Tuple[str, str] = (agent_node_name, ros_path)
        servers_table: Dict[Tuple[str, str],
                            Tuple[AgentNode, ServiceHandle]] = self.servers_table
        assert key not in servers_table, f"Duplicate server {key}"
        servers_table[key] = (agent_node, service_handle)

    # Agent.service_handle_get():
    def service_handle_get(self,
                           import_path: str, type_name: str, ros_path: str) -> "ServiceHandle":
        """Get a ServiceHandle for a ROS Server."""
        service_handles: Dict[HandleKey, ServiceHandle] = self.service_handles

        service_handle: ServiceHandle
        handle_key: HandleKey = HandleKey(import_path, type_name, ros_path)
        if handle_key not in service_handles:
            service_handle = ServiceHandle(import_path, type_name, ros_path, self)
            service_handles[service_handle.key_get()] = service_handle
        else:
            service_handle = service_handles[handle_key]
        return service_handle

    # Agent.subscription_create():
    def subscription_create(self, agent_node_name: str,
                            import_path: str, type_name: str, ros_path: str) -> None:
        """Create a subscription."""
        # print("=>agent.subscription_create()")
        agent_node: AgentNode = self.agent_node_create_once(agent_node_name)
        topic_handle: TopicHandle = self.topic_handle_get(import_path, type_name, ros_path)
        agent_node.subscription_create(topic_handle)

        # Record information into *subscriptions_table*:
        key: Tuple[str, str] = (agent_node_name, ros_path)
        subscriptions_table: Dict[Tuple[str, str],
                                  Tuple[AgentNode, TopicHandle]] = self.subscriptions_table
        assert key not in subscriptions_table, f"Duplicate subscription {key}"
        subscriptions_table[key] = (agent_node, topic_handle)
        # print("<=agent.subscription_create()")

    # Agent.timer_create():
    def timer_create(self, agent_node_name, timer_name: str, timer_period: float) -> None:
        """Create a timer."""
        # print("=>Agent.timer_create()")
        agent_node: AgentNode = self.agent_node_create_once(agent_node_name)
        timer_handle: TimerHandle = self.timer_handle_get(agent_node_name, timer_name)
        agent_node.timer_create(timer_handle, timer_period)
        # print("<=Agent.timer_create()")

    # Agent.timer_handle.get():
    def timer_handle_get(self, agent_node_name: str, timer_name: str) -> "TimerHandle":
        """Get a unique timer handle."""
        timer_handles: Dict[Tuple[str, str], TimerHandle] = self.timer_handles
        timer_handle_key: Tuple[str, str] = (agent_node_name, timer_name)

        timer_handle: TimerHandle
        if timer_handle_key not in timer_handles:
            timer_handle = TimerHandle(agent_node_name, timer_name, self)
            timer_handles[timer_handle_key] = timer_handle
        else:
            timer_handle = timer_handles[timer_handle_key]
        return timer_handle

    # Agent.topic_handle_get():
    def topic_handle_get(self, import_path: str, type_name: str, ros_path: str) -> "TopicHandle":
        """Get a unique TopicHandle for a ROS topic."""
        handle_key: HandleKey = HandleKey(import_path, type_name, ros_path)
        topic_handles: Dict[HandleKey, TopicHandle] = self.topic_handles

        topic_handle: TopicHandle
        if handle_key not in topic_handles:
            topic_handle = TopicHandle(import_path, type_name, ros_path, self)
            topic_handles[topic_handle.key_get()] = topic_handle
        else:
            topic_handle = topic_handles[handle_key]
        return topic_handle

    # Agent.client_lookup():
    def client_lookup(self, key: "Tuple[str, str]") -> "Tuple[AgentNode, ServiceHandle]":
        """Lookup a client."""
        clients_table: Dict[Tuple[str, str],
                            Tuple[AgentNode, ServiceHandle]] = self.clients_table
        assert key in clients_table, f"client key {key} is not one of {tuple(clients_table.keys())}"
        return clients_table[key]

    # Agent.publisher_lookup():
    def publisher_lookup(self, key: "Tuple[str, str]") -> "Tuple[AgentNode, TopicHandle]":
        """Lookup a publisher."""
        publishers_table: Dict[Tuple[str, str],
                               Tuple[AgentNode, TopicHandle]] = self.publishers_table
        assert key in publishers_table, f"Could not find publisher that matches {key}"
        return publishers_table[key]

    # Agent.server_lookup():
    def server_lookup(self, key: "Tuple[str, str]") -> "Tuple[AgentNode, ServiceHandle]":
        """Lookup a server."""
        servers_table: Dict[Tuple[str, str],
                            Tuple[AgentNode, ServiceHandle]] = self.servers_table
        assert key in servers_table, f"Could not find server that matches {key}"
        return servers_table[key]

    # Agent.subscription_lookup():
    def subscription_lookup(self, key: "Tuple[str, str]") -> "Tuple[AgentNode, TopicHandle]":
        """Lookup a subscription."""
        subscriptions_table: Dict[Tuple[str, str],
                                  Tuple[AgentNode, TopicHandle]] = self.subscriptions_table
        assert key in subscriptions_table, f"Could not find server that matches {key}"
        return subscriptions_table[key]

    # Agent.timer_lookup():
    def timer_lookup(self, key: Tuple[str, str]) -> "Tuple[AgentNode, TimerHandle]":
        """Lookup a timer."""
        timers_table: Dict[Tuple[str, str], Tuple[AgentNode, TimerHandle]] = self.timers_table
        assert key in timers_table, f"Could not find timer that matches {key}"
        return timers_table[key]


# AgentNode:
class AgentNode(Node):
    """A subclass of rclpy.Node that manages Nodes, callbacks, etc."""

    # AgentNode.__init__():
    def __init__(self, agent_node_name: str, agent: Agent):
        """Init AgentNode."""
        super().__init__(agent_node_name)
        self.agent: Agent = agent
        self.agent_node_name: str = agent_node_name
        self.clients_table: Dict[HandleKey, Client] = {}
        self.publishers_table: Dict[HandleKey, Publisher] = {}
        self.servers_table: Dict[HandleKey, Server] = {}
        self.subscriptions_table: Dict[HandleKey, Subscription] = {}
        self.timers_table: Dict[Tuple[str, str], Timer] = {}

    # AgentNode.client_create():
    def client_create(self, service_handle: "ServiceHandle") -> None:
        """Create a client for a ROS service."""
        clients_table: Dict[HandleKey, Client] = self.clients_table
        service_key: HandleKey = service_handle.key_get()
        assert service_key not in clients_table, f"Duplicate server {service_key}"
        client: Client = service_handle.client_create(self)
        clients_table[service_key] = client

    # AgentNode.publisher_create():
    def publisher_create(self, topic_handle: "TopicHandle") -> None:
        """Create a ROS topic publisher."""
        publishers_table: Dict[HandleKey, Publisher] = self.publishers_table
        topic_key: HandleKey = topic_handle.key_get()
        assert topic_key not in self.publishers_table, f"Duplicate publisher {topic_key}"
        publisher: Publisher = topic_handle.publisher_create(self)
        publishers_table[topic_key] = publisher

    # AgentNode.server_create():
    def server_create(self, service_handle: "ServiceHandle") -> None:
        """Create a server for a ROS service."""
        servers_table: Dict[HandleKey, Server] = self.servers_table
        service_key: HandleKey = service_handle.key_get()
        assert service_key not in self.servers_table, f"Duplicate server {service_key}"
        server: Server = service_handle.server_create(self)
        servers_table[service_key] = server

    # AgentNode.subscription_create():
    def subscription_create(self, topic_handle: "TopicHandle") -> None:
        """Create a ROS topic subscription."""
        subscriptions_table: Dict[HandleKey, Subscription] = self.subscriptions_table
        topic_key: HandleKey = topic_handle.key_get()
        assert topic_key not in self.subscriptions_table, f"Duplicate subscription {topic_key}"
        subscription: Subscription = topic_handle.subscription_create(self)
        subscriptions_table[topic_key] = subscription

    # AgentNode.timer_create():
    def timer_create(self, timer_handle: "TimerHandle", period: float) -> None:
        """Create a timer."""
        timers_table: Dict[Tuple[str, str], Timer] = self.timers_table
        timer_key: Tuple[str, str] = timer_handle.key_get()
        assert timer_key not in timers_table, f"Duplicate timer '{timer_key}'"
        timer: Timer = self.create_timer(period, timer_handle.callback)
        timers_table[timer_key] = timer


# encoder_create():
def encoder_create(start: int, escape: int, stop: int, twiddle: int) -> Tuple[Tuple[int, ...], ...]:
    """Return the escape encoder tuple table."""
    specials: Tuple[int, ...] = (start, escape, stop)
    encoder_list: List[Tuple[int, ...]] = []
    byte: int
    for byte in range(256):
        if byte in specials:
            encoder_list.append((escape, byte ^ twiddle))
        else:
            encoder_list.append((byte,))
    return tuple(encoder_list)


# Packet:
class Packet(object):
    """A Packet class converts between ROS topic and service messages and bytes.

    There are both full and partial packet formats.  The full packet format is:

    * Start: A reserved byte that only occurs at the beginning of packets. (1 byte)
    * Length: The total length of the packet in bytes (2 bytes, 14-bits only)
    * Packet Id: An identifier that specifies the data format of the packet content. (1 byte)
    * Structure: The remaining content is packed into a structure. (N bytes)
    * Strings: If there are any strings, each string has a length (1 byte) followed by
      that many 8-bit (ASCII) bytes.  (1 + LEN bytes)
    * Transmission Control: There are some bytes to control message re-transmits. (3-bytes for now)
    * CRC: There is a standard 16-bit cyclic redundancy check (2-bytes, reduced to 14-bits)
    * Stop: A reserved byte that only occurs at the end of packets. (1 byte)

    The varaious packet spans:

    * Partial Packet:
      A partial packet only has the packet id, strings, and structure fields present.
      These are output by various callbacks and sent to the serial line serializer.
    * Byte Escapting:
      The sequence of bytes that can be escaped by the esacpe byte (see below.)
    * CRC Computation:
      The CRC computate covers everything except the Start, CRC and Stop information.
      In other words it covers the partail packet plus length and transmission control.
    * Full Packet:
      This is the full packet from Start to Stop

    The crude ASCII art diagram shows the spans visually.

              1     2        1         *        *            3             2   1
           +-----+------+---------+---------+-------+--------------------+---+----+
           |Start|Length|Packet_ID|Structure|Strings|Transmission_Control|CRC|Stop|
           +-----+------+---------+---------+-------+--------------------+---+----+
           ^     ^      ^                           ^                    ^        ^
           |     |      |                           |                    |        |
           |     |      |<-----Partial Packet------>|                    |        |
           |     |      |                                                |        |
           |     |      |<-----Byte Escaping---------------------------->|        |
           |     |                                                       |        |
           |     |<------------CRC Computation-------------------------->|        |
           |                                                                      |
           |<------------------Full Packet--------------------------------------->|

    There are 3 characters reserved for message framing:

    * Start (0xfc)
    * Escape (0xfd)
    * Stop (0xfe)

    The Start character only occurs as the first packet byte and the Stop character only
    occurs as the last packet byte.  The Escape character only occurs as needed between
    the Start and Stop characters to deal with any data bytes which happen to match one
    of these three characters.  The Escape character basically says, set the higher bit
    of the next character to 1.

    The length field is organized as two 7-bit unsigned numbers (i.e. 0HHH HHHH, 0LLL LLLL.)
    The two values are combined to form a 14-bit unsigned number (i.e.  00HH HHHH HLLL LLLL)
    that specifies the remaining number of bytes.  The two zero are required to avoid
    having a length field that needs Escape characters.  The Length field specifies the
    entire packet length including all Escape characters.

    The transmssion control bytes currently are:

    * Current Sequence (8-bits):
      There is a sequence number that is incremented each time a new message is sent over the
      serial line.  Only the low 8-bits of this number is sent.  The first message is 1.

    * Counter sequence (8-bits):
      The lower 8-bits of the last successfully recevied message from counter direction.

    * Missing Messages Mask (8-bits):
      The mask specifies whether there are any missing messages past the current other sequence
      value.  If other sequence is N, the bits are for messages N+1 through N+8, where N+1 is
      the most significant bit and N+8 is the most signicant bit.  It is expected that the
      missing messages will be resent.

    The CRC is a standard 16-bit CCIT Cyclic Redundancy Check.  It is reduced to 14 bits
    by masking it with 0x7f7f.  This disables any chance of escape characters being needed.
    """

    ros_field_types: ClassVar[Dict[str, ROSFieldType]] = {
        # "Key": ROSTYPE("ROS Type", "C++ Type", "Python Type", "Struct Format", "C Type", Size)
        "bool": ROSFieldType("bool", "uint8_t", "bool", "B", "unsigned char", 1),
        "int8": ROSFieldType("int8", "int8_t", "int", "b", "signed char", 1),
        "uint8": ROSFieldType("uint8", "uint8_t", "int", "B", "unsigned char", 1),
        "int16": ROSFieldType("int16", "int16_t", "int", "h", "short", 2),
        "uint16": ROSFieldType("uint16", "uint16_t", "int", "H", "unsigned short", 2),
        "int32": ROSFieldType("int32", "int32_t", "int", "i", "int", 4),
        "uint32": ROSFieldType("uint32", "uint32_t", "int", "I", "unsigned int", 4),
        "int64": ROSFieldType("int64", "int64_t", "int", "q", "long long", 8),
        "uint64": ROSFieldType("uint64", "uint64_t", "int", "Q", "unsigned long long", 8),
        "float32": ROSFieldType("float32", "float", "float", "f", "float", 4),
        "float64": ROSFieldType("float64", "double", "float", "d", "double", 8),
        "string": ROSFieldType("string", "std::string", "bytes", "", "", -1),  # Special size
        "time": ROSFieldType("time", "ros::Time", "rospy.Time", "II", "!!", 8),  # 2 * unsigned int!
        "duration": ROSFieldType("duration",
                                 "ros::Duration", "rospy.Duration", "ii", "!!", 8),  # 2 * signed!
    }

    # Define the SLIP (originally stood for Serial Line Internet Protocol) characters.
    # The concept is the same, but different characters are used than the original SLIP protocol:
    slip_start: ClassVar[int] = 0xfc
    slip_escape: ClassVar[int] = 0xfd
    slip_stop: ClassVar[int] = 0xfe
    slip_twiddle: ClassVar[int] = 0x80  # Used to set the high order bit.
    encoder: ClassVar[Tuple[Tuple[int, ...], ...]] = encoder_create(
        slip_start, slip_escape, slip_stop, slip_twiddle)

    # The Python struct library is used to pack and unpack data to/from bytes:
    struct_endian: ClassVar[str] = "!"  # "!" stands for network which happens to be big-endian
    ros_type_convert: ClassVar[Dict[str, Tuple[str, str]]] = {
        "bool": ("?", "_Bool"),
        "byte": ("B", "unsigned char"),
        "char": ("c", "char"),
        "float32": ("f", "float"),
        "float64": ("d", "double"),
        "int8": ("b", "signed char"),
        "uint8": ("B", "unsigned char"),
        "int16": ("h", "short"),
        "uint16": ("H", "unsigned short"),
        "int32": ("i", "int"),
        "uint32": ("I", "unsigned int"),
        "int64": ("q", "long long"),
        "uint64": ("Q,", "unsigned long long"),
        # "string": ("p", "char p[]"), # Does not work!
        # "wstring": # not supported
    }

    # Packet.__init__():
    def __init__(self, packet_type: Any) -> None:
        """Initialize a Packet given a message instance."""
        # Create a *packet_instance* and extract information from it:
        packet_instance: Any = packet_type()

        # A ROS *packet_instance* must implement the *get_fields_and_types* method.
        # Get the *named_field_types* dictionary:
        named_field_types: Dict[str, str] = packet_instance.get_fields_and_field_types()

        # Grab the *sorted_names*:
        sorted_names: Tuple[str, ...] = tuple(sorted(named_field_types.keys()))

        # Split out *string_names* and *size_name_types* (a tuple needed to correctly
        # order the struct names to be from largest to smallest.):
        ros_field_types: Dict[str, ROSFieldType] = self.ros_field_types
        string_names: List[str] = []
        size_name_types: List[Tuple[int, str, str]] = []
        ros_field_type: ROSFieldType
        field_name: str
        field_type: str
        for field_name in sorted_names:
            field_type = named_field_types[field_name]
            assert field_type in ros_field_types, f"Unknown ROS type '{field_type}'"
            if field_type == "string":
                string_names.append(field_name)
            else:
                ros_field_type = ros_field_types[field_type]
                # Negate the struct size so that larger structs sort first.
                size_name_types.append((-ros_field_type.size, field_name, field_type))
        print(f"size_name_types={size_name_types}")

        # Assemble *struct_names* (largest struct size first, and then alphabetical there after).
        # At the same time construct *struct_format* which is the format string needed by the
        # Python struct library:
        struct_format: str = self.struct_endian
        struct_names: List[str] = []
        for _, field_name, field_type in sorted(size_name_types):
            struct_names.append(field_name)
            ros_field_type = ros_field_types[field_type]
            struct_format += ros_field_type.struct_format

        # Now compute the *struct_size* in bytes:
        struct_size: int = struct.calcsize(struct_format)

        # Load results into *self*:
        self.packet_type: Any = packet_type
        self.string_names: Tuple[str, ...] = tuple(string_names)
        self.struct_format: str = struct_format
        self.struct_names: Tuple[str, ...] = tuple(struct_names)
        self.struct_size: int = struct_size

    # Packet.__repr__()
    def __repr__(self) -> str:
        """Return a string representation of a Packet."""
        return (f"Packet({type(self.packet_type)}, '{self.struct_format}', "
                f"{self.struct_names}, {self.string_names}')")

    # Packet.full_decode():
    @classmethod
    def full_decode(cls, full_packet: bytes) -> Tuple[int, bytes, Tuple[int, int, int]]:
        """Convert a full packet into a partial packet with transmission control bytes."""
        print(f"=>Packet.full_decode({repr(full_packet)})")

        # Process the first 3 bytes (i.e. Start and Length):
        full_size: int = len(full_packet)
        if full_size < 3:
            raise ValueError(f"packet {repr(full_packet)} is too small")
        if full_packet[0] != cls.slip_start:
            raise ValueError(f"packet {repr(full_packet)} does not start with "
                             f"0x{cls.slip_start:02x}")

        # Extract and verify the *length*:
        high_length: int = full_packet[1]
        low_length: int = full_packet[2]
        if high_length >= 128 or low_length >= 128:
            raise ValueError(f"packet {repr(full_packet)} has actual length bytes of "
                             f"0x{high_length:02x} and 0x{low_length:02x} "
                             "neither of which should have bit 0x80 set")
        length: int = (high_length << 7) | low_length
        if full_size != length:
            raise ValueError(f"packet {repr(full_packet)} has actual length of {full_size},"
                             f"not the desired length of {length}")

        # Process the last three bytes (i.e. CRC and Stop):
        if full_packet[-1] != cls.slip_stop:
            raise ValueError(f"packet {repr(full_packet)} ends with 0x{full_packet[-1]:02x},"
                             f"not the desired 0x{cls.slip_stop}")
        crc_low: int = full_packet[-2]
        crc_high: int = full_packet[-3]
        if crc_high >= 128 or crc_low >= 128:
            raise ValueError(f"packet {repr(full_packet)} has actual length bytes of "
                             f"0x{crc_high:02x} and 0x{crc_low:02x} "
                             "neither of which should have bit 0x80 set")
        crc: int = (crc_high << 8) | crc_low

        # Verify the CRC:
        actual_crc: int = crc_compute(full_packet[3:-3]) & 0x7f7f
        if actual_crc != crc:
            raise ValueError(f"packet {repr(full_packet)} has an actual CRC of 0x{actual_crc:02x}, "
                             f"but the desired CRC is 0x{crc:02x}")

        # Extract the transmission *control* tuple.  This is three bytes of information that can
        # expand up to 6 bytes with escape characters (i.e. all 3 bytes have the 8th bit set.)
        # Rather unescaping the *full_packet* from the beginning, some cleverness is used instead.
        # Instead the last 6 bytes are grabbed, unescaped, and the last 3 bytes of the result
        # are the transmission control values:
        escaped_control: bytes = full_packet[-9:-3]
        unescaped_control: bytes = cls.unescape(escaped_control)
        if len(unescaped_control) < 3:
            raise ValueError(f"packet {repr(full_packet)} does not have 3 transmission "
                             "control bytes")
        control: Tuple[int, int, int] = (
            unescaped_control[-3], unescaped_control[-2], unescaped_control[-1])

        # Now the *control* values are entered into the *rescapded_control* to that
        # the final length the the escaped control bytes can be determined:
        encoder: Tuple[Tuple[int, ...], ...] = cls.encoder
        reescaped_control: bytearray = bytearray()
        for byte in control:
            reescaped_control.extend(encoder[byte])
        control_escaped_size: int = len(reescaped_control)

        # Extract *partial_packet* now that *control_escaped_size* is known:
        partial_packet: bytes = full_packet[3:-(control_escaped_size + 3)]

        # Now extract the *packet_id* from *partial_packet*:
        escaped_header: bytes = cls.unescape(partial_packet[:2])
        packet_id: int = escaped_header[0]

        print(f"<=Packet.full_decode({repr(full_packet)})"
              f"=>({repr(partial_packet)},{repr(control)})")
        return (packet_id, partial_packet, control)

    # Packet.full_encode():
    def full_encode(self,
                    partial_packet_bytes: bytes, control_bytes: Tuple[int, int, int]) -> bytes:
        """Convert a partial Packet into a full Packet."""
        # print("=>Packet.full_encode("
        #       f"{repr(partial_packet_bytes)}, {control_bytes})")

        # Create *crc_bytes* which contains all of the bytes to be CRC'ed.
        # This the *partial_packet* (already_escaped) and the *control_bytes*
        # (which are not escaped yet:
        crc_bytes: bytearray = bytearray(partial_packet_bytes)
        assert len(control_bytes) == 3, (
            "Length of transmission control bytes ({len(control_bytes)}) is not 3")
        encoder: Tuple[Tuple[int, ...], ...] = self.encoder
        control_byte: int
        for control_byte in control_bytes:
            crc_bytes.extend(encoder[control_byte])

        # Compute the *crc* and associated *crc_high* and *crc_low* bytes over *crc_bytes*:
        global crc_compute

        # crc_compute: Callable[[bytes], int] = self.crc_compute
        # crc_compute: Callable[[bytes], int] = crcmod.mkCrcFun(0x11021, initCrc=0, xorOut=0xffff)
        crc: int = crc_compute(bytes(crc_bytes)) & 0x7f7f
        crc_high: int = (crc >> 8) & 0x7f
        crc_low: int = crc & 0x7f

        # Compute *total_length* and associated the *high_length* and *low_length* bytes:
        front_length: int = 1 + 2  # Start(1) + Length(2)
        end_length: int = 2 + 1  # CRC(2) + End(1)
        total_length: int = front_length + len(crc_bytes) + end_length
        assert total_length <= 0x3fff, f"Packet Length is too long 0x{total_length:x} > 0x3fff"
        high_length: int = total_length >> 7
        low_length: int = total_length & 0x7f

        # Construct the *full_packet*:
        full_packet: bytearray = bytearray((self.slip_start, high_length, low_length))
        assert len(full_packet) == front_length, f"Packet header is {len(full_packet)} long"
        full_packet.extend(crc_bytes)
        assert len(full_packet) == front_length + len(crc_bytes), "broke here"
        full_packet.extend((crc_high, crc_low, self.slip_stop))
        full_packet_bytes: bytes = bytes(full_packet)
        assert len(full_packet) == total_length, (
            f"Actual packet length ({len(full_packet)}) is not desired length ({total_length})")
        # print("<=Packet.full_encode("
        #       f"{repr(partial_packet_bytes)}, {control_bytes})=>{repr(full_packet_bytes)}")
        return(full_packet_bytes)

    # Packet.partial_decode():
    def partial_decode(self, packet_bytes: bytes, index: int) -> Tuple[int, Any]:
        """Decode a strings and structs from a packet."""
        # print(f"=>Packet.decode({repr(packet_bytes)})")
        # Create the *packet_content* instance to put the data into:
        packet_content: Any = self.packet_type()
        packet_size: int = len(packet_bytes)
        assert index <= packet_size, "Packet.decode(): index error: bad start index"

        # Extract the *packet_id*:
        packet_id: int = packet_bytes[index]
        index += 1
        assert index <= packet_size, "Packet.decode(): index error: packet id"

        # Sweep through the *string_names* and stuff the associated strings into *packet_type*:
        string_name: str
        for string_name in self.string_names:
            # Grab the *string_length*:
            string_length: int = packet_bytes[index]
            index += 1
            assert index <= packet_size, "Packet.decode(): index error: string length"

            # Extract the *string* from the *packet_bytes*:
            end_index: int = index + string_length
            assert index <= packet_size, "Packet.decode(): index error: string character"
            byte: int
            string: str = "".join([chr(byte) for byte in packet_bytes[index:end_index]])

            # Stuff the *string* into the *packet_content*:
            assert string_name in self.string_names, (
                f"'string_name' is not present in {self.packet_type}")
            setattr(packet_content, string_name, string)
            index = end_index

        # Unpack the *struct_values* from the remainder of *packet_bytes*:
        struct_bytes: bytes = packet_bytes[index:]
        if len(struct_bytes) != self.struct_size:
            print(f"Actual struct bytes size ({len(struct_bytes)}) "
                  f"does not match desired size {self.struct_size}")
        struct_values: Tuple[int, ...] = struct.unpack(self.struct_format, struct_bytes)
        struct_names: Tuple[str, ...] = self.struct_names
        assert len(struct_values) == len(struct_names), "Packet.decode(): Struct mismatch"

        # Stuff *struct_values* into *packet_content*:
        struct_index: int
        struct_name: str
        for struct_index, struct_name in enumerate(struct_names):
            setattr(packet_content, struct_name, struct_values[struct_index])

        # print(f"<=Packet.decode({repr(packet_bytes)})=>({packet_id}, {packet_content}")
        return (packet_id, packet_content)

    # Packet.partial_encode():
    def partial_encode(self, packet_content: Any, packet_id: int) -> bytes:
        """Encode packet content into partial bytes packet."""
        # print(f"=>Packet.partial_encode({packet_content}, {packet_id})")
        assert isinstance(packet_content, self.packet_type), (
            f"Packet content is {type(packet_content)} rather than {type(self.packet_type)}")
        packet_bytes: bytearray = bytearray()

        # Start with the *packet_id*:
        encoder: Tuple[Tuple[int, ...], ...] = self.encoder
        assert 0 <= packet_id <= 255, f"Packet id ({packet_id} does not fit in a byte.)"
        packet_bytes.extend(encoder[packet_id])

        # Put all of the strings in next because they have a length byte:
        string_name: str
        for string_name in self.string_names:
            # Do some sanity checking:
            string_content: Any = getattr(packet_content, string_name)
            assert isinstance(string_content, str), (
                "{string_name} is {type(string_content)} instead of a string")

            # Ensure that *string_content* has a length that fits into a byte:
            string_size: int = len(string_content)
            if string_size > 255:
                print("Truncating string to 255 characters")
                string_content = string_content[:255]
                string_size = 255

            # Append *string_size* first, followed by the *content_content*:
            packet_bytes.extend(encoder[string_size])
            string_chr: str
            for string_chr in string_content:
                string_byte: int = ord(string_chr)
                if string_byte > 255:
                    print(f"Converting character 0x{string_byte:02x} to 0xff")
                    string_byte = 255
                packet_bytes.extend(encoder[string_byte])

        # Extract the *struct_values* from *packet_content* in *struct_names* order:
        struct_name: str
        struct_values: List[int] = [getattr(packet_content, struct_name)
                                    for struct_name in self.struct_names]

        # Convert *struct_values* to *struct_bytes* and then append them to *packet_bytes*:
        struct_bytes: bytes = struct.pack(self.struct_format, *struct_values)
        struct_byte: int
        for struct_byte in struct_bytes:
            packet_bytes.extend(encoder[struct_byte])

        final_packet_bytes: bytes = bytes(packet_bytes)
        # print(f"<=Packet.partail_encode({packet_content}, "
        #       f"{packet_id})=>{repr(final_packet_bytes)}")
        return final_packet_bytes

    # Packet.ros_field_types_get():
    @classmethod
    def ros_field_types_get(cls) -> Dict[str, ROSFieldType]:
        """Return the ROS Field Types table."""
        return cls.ros_field_types

    # Packet.unescape():
    @classmethod
    def unescape(cls, escaped_bytes: bytes) -> bytes:
        """Take a packet with escapes and remove them."""
        # print(f"=>Packet.unescape({repr(escaped_bytes)})")
        unescaped_bytes: bytearray = bytearray()
        slip_escape: int = cls.slip_escape
        slip_twiddle: int = cls.slip_twiddle
        escaped_byte: int
        escape_found: bool = False
        for escaped_byte in escaped_bytes:
            if escape_found:
                unescaped_bytes.append(escaped_byte ^ slip_twiddle)
                escape_found = False
            elif escaped_byte == slip_escape:
                escape_found = True
            else:
                unescaped_bytes.append(escaped_byte)
        # print(f"<=Packet.unescape({repr(escaped_bytes)})=>{repr(unescaped_bytes)}")
        return bytes(unescaped_bytes)


# Handle:
class Handle(object):
    """Base class for publish/subscribe topics and client/server services."""

    # Handle.__init__():
    def __init__(self, import_path: str, type_name: str, ros_path: str, agent: Agent) -> None:
        """Initialize Handle base class."""
        self.agent: Agent = agent
        self.import_path: str = import_path
        self.type_name: str = type_name
        self.key: HandleKey = HandleKey(import_path, type_name, ros_path)
        self.ros_path: str = ros_path

    # Handle.agent_get():
    def agent_get(self) -> Agent:
        """Return the Handle Agent."""
        agent: Optional[Agent] = self.agent
        assert isinstance(agent, Agent), "No agent is available"
        return agent

    # Handle.key_get():
    def key_get(self) -> "HandleKey":
        """Return the Handle key."""
        return self.key

    # Handle.match():
    def match(self, import_path: str, type_name: str, ros_path: str) -> bool:
        """Return True if TopicHandle's match."""
        return (self.import_path == import_path and (
                self.type_name == type_name and self.ros_path == ros_path))

    # Handle.packet_type_lookup():
    def packet_type_lookup(self, packet_id) -> Any:
        """Return the packet type for an a given packet ide."""
        raise NotImplementedError

    # Handle.type_import():
    def type_import(self, import_path: str, type_name: str, ros_path: str) -> Any:
        """Import a topic."""
        # Read the *import_path* into *module*:
        module: Any = importlib.import_module(import_path)
        # print(f"import_path='{import_path}' type(module)={type(module)}")

        # Get the Python *actual_type* object:
        actual_type: Any = getattr(module, type_name)
        # print(f"type_name='{type_name}' type(actual_type)={type(actual_type)}")

        return actual_type


# HandleKey:
@dataclass(order=True, frozen=True)
class HandleKey:
    """Class that represents a topic name."""

    import_path: str
    type_name: str
    ros_path: str


# IOChannel:
class IOChannel:
    """A handle for deal with serial I/O."""

    # IOChannel.__init__():
    def __init__(self, argument: str, colon_split: Tuple[str, ...]) -> None:
        """Init an IOChannel."""
        # Start with all value set to *False*:
        self.create: bool = False
        self.device: bool = False
        self.pipe: bool = False
        self.read: bool = False
        self.write: bool = False

        # Extract *file_name* and *flags* from *colon_split*:
        if len(colon_split) != 2:
            raise ValueError(f"Command line argument '{argument}' has {len(colon_split)} fields"
                             "instead of 2 -- FILE_NAME:FLAGS")
        flags: str
        file_name: str
        file_name, flags = colon_split

        # Process the *flags* treating *self* as a dictionary:
        flag: str
        for flag in flags.split(","):
            if hasattr(self, flag):
                setattr(self, flag, True)
            else:
                raise ValueError(f"Command line argument '{argument}' does not support "
                                 f"'{flag}' option")

        # Do some consistency checking:
        if self.device and self.pipe:
            raise ValueError(f"Command line argument '{argument}' can not specify "
                             "'device' and 'pipe' at the same time")
        if not self.device and not self.pipe:
            raise ValueError(f"Command line argument '{argument}' does not specify "
                             "one of 'device' or 'pipe'")
        if not self.read and not self.write:
            raise ValueError(f"Command line argument '{argument}' does not specify "
                             "one of 'read' or 'write'")
        if self.read and self.write and self.pipe:
            raise ValueError(f"Command line argument '{argument}' cannot both read "
                             "write a pipe")
        if self.create and self.device:
            raise ValueError(f"Command line argument '{argument}' cannot create device "
                             "'{self.file_name}'")

        # Create the named pipe (FIFO) it it does not already exist:
        if self.create and self.pipe:
            try:
                os.mkfifo(file_name)
            except FileExistsError:
                if not stat.S_ISFIFO(os.stat(file_name).st_mode):
                    raise RuntimeError(f"'{file_name}' already exists and is not a named pipe")

        # Finish loading up I/O handle.  Note that both *read_file* and *write_file* are only
        # opened on the first read or write.  This has to do with a peculiar "feature" of
        # named pipes.  Each end of a named pipe is needs to be opened before the two open()
        # calls will return.  The first open() does not return until after the second open()
        # is invoked:
        self.file_name: str = file_name
        self.read_file: Optional[IO[bytes]] = None
        self.write_file: Optional[IO[bytes]] = None

    # IOChannel.__str__():
    def __str__(self):
        """Return a text representation of an IOChannel."""
        return (f"IOCHannel(create={self.create}, device={self.device}, read={self.read}, "
                f"pipe={self.pipe} write={self.write}, file_name='{self.file_name}')")

    # IOChannel.open():
    def open(self):
        """Open an I/O channel."""
        # Now open it the channel:
        try:
            file_name: str = self.file_name
            if self.read and self.write:
                # "w+" is explained here:
                #     https://stackoverflow.com/questions/1466000/
                #         difference-between-modes-a-a-w-w-and-r-in-built-in-open-function
                read_write_file: IO[bytes] = open(file_name, "w+b")
                self.read_file = read_write_file
                self.write_file = read_write_file
            elif self.read:
                # This will stall until the other end of the pipe is opened for writing:
                self.read_file = open(file_name, "rb")
            else:
                # This will stall until the other end of the pipe is opened for reading:
                self.write_file = open(file_name, "wb")
        except FileNotFoundError:
            raise RuntimeError("File '{file_name}' does not exist")

    # IOChannel.data_read():
    def data_read(self, amount: int) -> bytes:
        """Read some data from an I/O channel."""
        if not self.read_file:
            self.open()  # This will stall until both sides of a named pipe are opened.
            if not self.read_file:
                raise RuntimeError("Can not read from '{self.file_name}'")
        return self.read_file.read(amount)

    # IOChannel.data_write(data: bytes) -> None:
    def data_write(self, data: bytes) -> None:
        """Write some data to an I/O channel."""
        if not self.write_file:
            self.open()  # This will stall until both sides of a named pipe are opened.
            if not self.write_file:
                raise RuntimeError("Can not write to '{self.file_name}'")
        self.write_file.write(data)


# TimerHandle:
class TimerHandle(object):
    """A handle for dealing with ROS timers."""

    # TimerHandle.__init__():
    def __init__(self,
                 agent_node_name: str, timer_name: str, agent: Optional[Agent] = None) -> None:
        """Init a TimeHandle."""
        self.agent: Optional[Agent] = agent
        self.agent_node_name: str = agent_node_name
        self.timer_name: str = timer_name
        self.key: Tuple[str, str] = (agent_node_name, timer_name)

    # TimerHandle.agent_get():
    def agent_get(self) -> Agent:
        """Return the Agent."""
        agent: Optional[Agent] = self.agent
        assert isinstance(agent, Agent), f"No agent specfied for TimerHandle({self.key})"
        return agent

    # TimerHandle.agent_node_create_once():
    def agent_node_create_once(self) -> AgentNode:
        """Ensure that an AgentNode has been created."""
        agent: Agent = self.agent_get()
        agent_node: AgentNode = agent.agent_node_create_once(self.agent_node_name)
        return agent_node

    # TimerHandle.callback():
    def callback(self) -> None:
        """Process timer callback."""
        print("")
        # print(f"=>TimerHandle('{key}').callback()")
        agent: Agent = self.agent_get()

        # Publish a message to the topic named 'topic':
        topic_agent_node: AgentNode
        topic_topic_handle: TopicHandle
        topic_agent_node, topic_topic_handle = agent.publisher_lookup(("serial_agent", "topic"))
        topic_publisher_count: int = topic_topic_handle.publisher_count_get()
        topic_message: Any = topic_topic_handle.message_create()
        topic_message.data = f"topic message {topic_publisher_count}"
        topic_topic_handle.publish(topic_message)

        # Publish a message to the topic named 'echo':
        echo_agent_node: AgentNode
        echo_topic_handle: TopicHandle
        echo_agent_node, echo_topic_handle = agent.publisher_lookup(("serial_agent", "echo"))
        echo_publisher_count: int = echo_topic_handle.publisher_count_get()
        echo_message: Any = echo_topic_handle.message_create()
        echo_message.data = f"echo message {2 * echo_publisher_count}"
        echo_topic_handle.publish(echo_message)

        # print(f"<=TimerHandle('{key}').callback()")

    # TimerHandle.match():
    def match(self, agent_node_name: str, timer_name: str) -> bool:
        """Return True if TimerHandle's match."""
        return self.agent_node_name == agent_node_name and self.timer_name == timer_name

    # TimerHandle.key_get():
    def key_get(self) -> Tuple[str, str]:
        """Return the TimerHandle key."""
        return self.key


# TopicHandle:
class TopicHandle(Handle):
    """Tracking class for publish/subscribe topic."""

    # TopicHandle.__init__():
    def __init__(self, import_path: str, type_name: str, ros_path: str, agent: Agent) -> None:
        """Initialize a TopicHandle."""
        # Initialize base class:
        super().__init__(import_path, type_name, ros_path, agent)

        # Import the relevant information about the topic:
        message_type: Any = self.type_import(import_path, type_name, ros_path)

        # Load values into the TopicHandle:
        self.key: HandleKey = HandleKey(import_path, type_name, ros_path)
        self.message_packet: Packet = Packet(message_type)
        self.message_type: Any = message_type
        self.publisher: Optional[Publisher] = None
        self.publisher_agent_node: Optional[AgentNode] = None
        self.publisher_count: int = 0
        self.publisher_packet_id: int = agent.packet_id_get(self)
        self.subscription_agent_node: Optional[AgentNode] = None
        self.subscription_count: int = 0
        self.subscription: Optional[Subscription] = None
        self.subscription_packet_id: int = agent.packet_id_get(self)

    # TopicHandle.__repr__():
    def __str__(self) -> str:
        """Return a string representation."""
        return (f"TopicHandle('{self.import_path}', '{self.type_name}', '{self.ros_path}', "
                f"{self.message_packet})")

    # TopicHandle.key_get():
    def key_get(self) -> HandleKey:
        """Return the TopicHandle key."""
        return self.key

    # TopicHandle.packet_type_lookup():
    def packet_type_lookup(self, packet_id) -> Any:
        """Return the packet type for an a given packet id."""
        packet_type: Any
        assert packet_id == self.publisher_packet_id or packet_id == self.subscription_packet_id, (
            f"Packet id {packet_id} does not match either "
            f"{self.publisher_packet_id} or {self.subscription_packet_id}")
        return self.message_type

    # TopicHandle.publish():
    def publish(self, message: Any) -> Publisher:
        """Publish a message to the topic."""
        # print(f"=>TopicHandle.publish('{message}')")
        publisher: Optional[Publisher] = self.publisher
        assert publisher, "Publisher not set yet."
        message_type: Any = self.message_type
        assert isinstance(message, message_type), f"Message is not of type {message_type}"
        print(f"Topic('{self.ros_path}): Published: {message}'")
        publisher.publish(message)
        self.publisher_count += 1
        # print(f"<=TopicHandle.publish('{message}')")

    # TopicHandle.publisher_count_get():
    def publisher_count_get(self) -> int:
        """Return the number of topic publications processed."""
        return self.publisher_count

    # TopicHandle.publisher_create():
    def publisher_create(self, publisher_agent_node: AgentNode) -> Publisher:
        """Create a publisher for a ROS topic."""
        # print(f"=>TopicHandle.publisher_create()")
        assert not self.publisher_agent_node, f"Duplicate publisher of {self.key}"
        self.publisher_agent_node = publisher_agent_node

        # Create and return the new *publisher*:
        publisher: Publisher = publisher_agent_node.create_publisher(
            self.message_packet.packet_type,
            self.ros_path,
            10)

        assert not self.publisher, f"Duplicate publisher for {self.key}"
        self.publisher = publisher

        # print(f"<=TopicHandle.publisher_create()=>{publisher}")
        return publisher

    # TopicHandle.message_create():
    def message_create(self) -> Any:
        """Create a new message object instance."""
        return self.message_type()

    # TopicHandle.subscription_create():
    def subscription_create(self, subscription_agent_node: AgentNode) -> Subscription:
        """Create a subcription for a ROS topic."""
        # print(f"=>TopicHandle.subscription_create()")
        assert not self.subscription_agent_node, (
            "TopicHandle.subscription_create(): subscription_agent_node already set")
        self.subscription_agent_node = subscription_agent_node

        # Create and return the new *subscription*:
        subscription: Subscription = subscription_agent_node.create_subscription(
            self.message_packet.packet_type,
            self.ros_path,
            self.subscription_callback,
            10)

        assert not self.subscription, f"duplicate subscription for {self.key}"
        self.subscription = subscription

        # print(f"<=TopicHandle.subscription_create()")
        return subscription

    # TopicHandle.subscription_callback():
    def subscription_callback(self, message: Any) -> None:
        """Process a subscription callback."""
        # print("=>TopicHandle.subscription_callback()")
        print(f"Topic('{self.ros_path}'): Got Message: {message}")

        agent: Optional[Agent] = self.agent
        assert isinstance(agent, Agent), "Agent is not set!"

        # Encode *message* into *packet_bytes*:
        assert isinstance(message, self.message_type), (
            f"message {message} does name match {self.message_type}")
        message_packet: Packet = self.message_packet
        partial_packet_bytes: bytes = message_packet.partial_encode(message,
                                                                    self.subscription_packet_id)
        # print(f"partial_packet={repr(partial_packet_bytes)}")
        control: Tuple[int, int, int] = (1, 2, 3)
        full_packet_bytes: bytes = message_packet.full_encode(partial_packet_bytes, control)
        extracted_packet_id: int
        extracted_partial_packet: bytes
        extracted_control: Tuple[int, int, int]
        extracted_packet_id, extracted_partial_packet, extracted_control = (
            message_packet.full_decode(full_packet_bytes))
        assert self.subscription_packet_id == extracted_packet_id, (
            f"Deisred packet id ({self.subscription_packet_id}) "
            f"is not actual packet id ({extracted_packet_id})")
        assert control == extracted_control, f"Mismatch control {control} != {extracted_control}"
        assert partial_packet_bytes == extracted_partial_packet, (
            "partial packet decode problem: "
            f"{repr(partial_packet_bytes)} != {repr(extracted_partial_packet)}")
        print(f"full_packet={repr(full_packet_bytes)}")

        # Decode *packet_types* into *unescaped_packet_bytes*:
        unescaped_bytes: bytes = message_packet.unescape(partial_packet_bytes)
        # print(f"unescaped_bytes={repr(unescaped_bytes)}")

        # Decode *unescaped_packet_bytes* into *decode_content:
        decode_packet_id: int
        decode_content: Any
        decode_packet_id, decode_content = message_packet.partial_decode(unescaped_bytes, 0)
        assert decode_packet_id == self.subscription_packet_id, (
            f"decode_packet_id ({decode_packet_id}) "
            f"does not match subscription_packet_id ({self.subscription_packet_id})")
        assert message.data == decode_content.data, (
            f"message.data ({message.data}) does not match "
            f"decode_content.data ({decode_content.data})")
        print("Yahoo! They match!")

        if self.ros_path == "echo":
            add_two_ints_agent_node: AgentNode
            add_two_ints_service_handle: ServiceHandle
            add_two_ints_agent_node, add_two_ints_service_handle = (
                agent.client_lookup(("serial_agent", "add_two_ints")))
            add_two_ints_count: int = add_two_ints_service_handle.request_count_get()
            add_two_ints_request: Any = add_two_ints_service_handle.request_create()
            add_two_ints_request.a = add_two_ints_count
            add_two_ints_request.b = add_two_ints_count + 1
            add_two_ints_service_handle.request_send(add_two_ints_request)
        # print("<=TopicHandle.subscription_callback()")


# ServiceHandle:
class ServiceHandle(Handle):
    """Tracking class for clientserver service."""

    # ServiceHandle.__init__():
    def __init__(self, import_path: str, type_name: str, ros_path: str, agent: Agent) -> None:
        """Initialize Handle class."""
        # Initialize base class:
        super().__init__(import_path, type_name, ros_path, agent)

        # Import the relevant information about the topic:
        service_type: Any = self.type_import(import_path, type_name, ros_path)
        request_type: Any = service_type.Request
        response_type: Any = service_type.Response

        # Load values into the TopicHandle:
        self.key: HandleKey = HandleKey(import_path, type_name, ros_path)
        self.service_type: Any = service_type
        self.request_count: int = 0
        self.request_packet_id: int = agent.packet_id_get(self)
        self.request_type: Any = request_type
        self.request_packet: Packet = Packet(request_type)
        self.response_count: int = 0
        self.response_packet_id: int = agent.packet_id_get(self)
        self.response_packet: Packet = Packet(response_type)
        self.response_type: Any = response_type

        self.client: Optional[Client] = None
        self.client_agent_node: Optional[AgentNode] = None
        self.server: Optional[Server] = None
        self.server_agent_node: Optional[AgentNode] = None

    # ServiceHandle.__repr__():
    def __str__(self) -> str:
        """Return a string representation."""
        return (f"TopicHandle('{self.import_path}', '{self.type_name}', '{self.ros_path}', "
                f"{self.request_packet}, {self.response_packet})")

    # ServiceHandle.client_create():
    def client_create(self, client_agent_node: AgentNode) -> Client:
        """Create a client for a ROS service."""
        # print("=>ServiceHandle.client_create()")

        assert not self.client_agent_node, f"Duplicate client {self.key}"
        self.client_agent_node = client_agent_node

        # Create and return the new *subscription*:
        assert not self.client, f"Duplicate client {self.key}"
        client: Client = client_agent_node.create_client(self.service_type, self.ros_path)
        self.client = client

        # print(f"<=ServiceHandle.client_create()=>{client}")
        return client

    # ServiceHandle.key_get():
    def key_get(self) -> HandleKey:
        """Return the ServiceHandle key."""
        return self.key

    # ServiceHandle.packet_type_lookup():
    def packet_type_lookup(self, packet_id) -> Any:
        """Return the packet type for an a given packet id."""
        packet_type: Any
        if packet_id == self.request_packet_id:
            packet_type = self.request_type
        elif packet_id == self.response_packet_id:
            packet_type = self.response_type
        else:
            assert False, (f"Packet id {packet_id} does not match either "
                           f"{self.request_packet_id} or {self.response_packet_id}")
        return packet_type

    # ServiceHandle.server_create():
    def server_create(self, server_agent_node: AgentNode) -> Client:
        """Create a server for a ROS service."""
        # print("=>ServiceHandle.server_create()")

        assert not self.server_agent_node, f"Duplicate server {self.key}"
        self.server_agent_node = server_agent_node

        # Create and return the new *subscription*:
        server: Server = server_agent_node.create_service(
            self.service_type, self.ros_path, self.request_callback)
        assert not self.server, f"Duplicate server {self.key}"

        # print(f"<=ServiceHandle.server_create()=>{server}")
        return server

    # ServiceHandle.request_count_get()
    def request_count_get(self) -> int:
        """Return the number of requests processed."""
        return self.request_count

    # ServiceHandle.request_create():
    def request_create(self) -> Any:
        """Return a ne request object."""
        return self.request_type()

    # ServiceHandle.request_callback():
    def request_callback(self, request: Any, response: Any) -> Any:
        """Process a server callback."""
        # print(f"=>SerialAgent.request_callback({request}, {response})")
        if self.ros_path == "add_two_ints":
            a: int = request.a
            b: int = request.b
            response.sum = a + b
            # print(f"============>>>>Server('{self.ros_path}'): Computed: {a} + {b} = {sum} ...")
            # time.sleep(3)
            # print(f"<<<<============Server('{self.ros_path}'): Computed: {a} + {b} = {sum}")
        else:
            assert False, f"Got request={request} response={response}"
        # print(f"<=SerialAgent.request_callback({request}, {response}) => {response}")
        return response

    # ServiceHandle.request_send():
    def request_send(self, request: Any) -> None:
        """Send a request to a server."""
        # print(f"=>ServiceHandle.request_send({request}):")
        client: Optional[Client] = self.client
        assert isinstance(client, Client), "No client"
        assert isinstance(request, self.request_type), (
            f"Request is {type(request)} not {type(self.request_type)}")
        response_future: Future = client.call_async(request)
        response_future.add_done_callback(self.response_callback)
        self.request_count += 1
        print(f"Service('{self.ros_path}'): Sent request: {request}")
        # print(f"<=ServiceHandle.request_send({request}):")

    # ServiceHandle.response_callback():
    def response_callback(self, response_future: Future) -> Any:
        """Deal with an asynchronous response to a server request."""
        # print("=>ServiceHandle.response_callback()")
        try:
            response: Any = response_future.result()
            self.response_count += 1
        except Exception as error:
            assert False, error
        else:
            assert isinstance(response, self.response_type)
        print(f"Service('{self.ros_path}'): Got response: {response}')")
        # print(f"<=ServiceHandle.response_callback()=>{response}")
        return response

    # ServiceHandle.response_count_get()
    def response_count_get(self) -> int:
        """Return the number of reponses proceesed."""
        return self.response_count

    # ServiceHandle.response_create():
    def response_create(self) -> Any:
        """Return a ne response object."""
        return self.response_type()


if __name__ == "__main__":
    main()
