#!/usr/bin/env python3
"""
Serial Agent Program.

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

Overview:

This mirrors traffic between a microcontroller and robot computer running ROS2 natively.

Class Summary:

* Agent: The top level class that reads the command line and create the associated ROS
  Node's, Client's, Publisher's, Server's, Subscriptions, Timer's etc.

* AgentNode: A subclass of rclpy.Node.  This is the approved way of managing ROS nodes.

* Packet: A class for encoding decodeing packeges over a serial line.

* Handle, TopicHandle, Service: These 3 classes uniquely define either a ROS Topic
  (using TopicHandle) or a ROS service (using Service Handle).  These classes can
  read in the appropriate type information to support packet encodeing and decodeing.

* TimerHandle: Simile to a Handle, but for managing timers.

"""
# <====================================== 100 Characters ========================================> #

# rclpy Documentation:
# * [https://docs.ros2.org/latest/api/rclpy/](General rclpy documentation)
#
# Asyncio:
#
# Python asynico has advantages/disadvantages and is semi-supported by ROS2:
# * [https://github.com/ros2/rclpy/issues/279](See close Comment by sloretz)
# * [https://github.com/ros2/rclpy/blob/b72a05bd3fb3ac7d27a8da359571afbbfec07f19/
#   rclpy/test/test_executor.py#L154](Test of asyncio in rclpy)
# * https://answers.ros.org/question/362598/asyncawait-in-subscriber-callback/
# * https://answers.ros.org/question/343279/ros2-how-to-implement-a-sync-service-client-in-a-node/
# Summary: It can work, it has issues. The ROS2 team is focusing is on multi-threading.

# Executors:
# * [https://github.com/ros2/examples/tree/master/rclpy/executors/examples_rclpy_executors]
#   (Executor Examples)

# Multi-Threading:
# *

# * [https://github.com/ros2/examples/blob/master/rclpy/executors/examples_rclpy_executors/
#   custom_executor.py](A PriorityExecutor using EStop as an example)

# https://answers.ros.org/question/377848/spinning-multiple-nodes-across-multiple-threads/
#
# Composition:
# Interesting, but mostly for C++:
# * https://answers.ros.org/question/351442/compose-two-components-into-a-single-process-in-python/
# *https://docs.ros.org/en/foxy/Tutorials/Composition.html

# Misc:
# * https://nicolovaligi.com/articles/concurrency-and-parallelism-in-ros1-and-ros2-application-apis/
# https://answers.ros.org/question/373169/mistakes-using-service-and-client-in-same-node-ros2-python

# Top level fuction (defined in __init__.py):
#     get_global_executor() -> Executor.
# Executor.add_node(node: Node) -> bool  # True on success, False otherwise.

#
# Serial Protocol description:
#
# Packets can be sent over either Linux pipes or over a serial line.  Linux pipes are assumed
# to be loss-less and error free.  Conversely, the serial line is assumed to have infrequent
# trasmission errors, byte insertions, and byte drops.  Thus the serail line protocol has an
# additional framing protocol, CRC (Cyclic Rendundancy Check), and retransmission information.
# Other than these difference packets are quite similar.
#
# The serial line framing characters are ESCAPE (0x80), START (0x81), and STOP (0x82).
# An entire packet starts with START, a bunch of data, followed by END.  In order,
# to avoid any confusion, when a byte of actual data matches one of the three framing
# characters, the single byte is replaced by two characters:
#
#     START (0xfc) => ESCAPE (0xfe) (0x7d)
#     ESCAPE (0xfd) => ESCAPE (0xfe) (0x7e)
#     STOP (0xfe) => ESCAPE (0xfe) (0x7f)
#
# This is for the serial line protocol only.
#
# The message content is organized as follows:
#
# * Length (2-bytes):
#   The two length bytes are two 7-bit unsigned numbers in big endian format (i.e. 0HHH HHHH,
#   0LLL LLLL.)  The two values are combined to form a 14-bit unsigned number (i.e.
#   00HH HHHH HLLL LLLL) that specifies the remaining number of bytes.
#
# * ID (1-byte):
#   This is a byte that specifies the message type (publish/subscribe/client/server and which
#   ROS path is used.  This table is created and maintained by the Serial Agent generator.
#
# * Strings:
#   A packet can have 0, 1 or more strings in it.  Each string has a maximum length of 255
#   bytes.  The number if strings is fixed and specifed by the ID.
#
# * Struct:
#   The struct is a bunch of 8, 16, 32 byte values is an order specified by the
#   defining ID.  They are packed and unpacked using the Python `struct` library.
#   Big endian format is used.
#
# The serial line protocol encodes the entire packet using the framing bytes.
# The length field specifies all of the bytes including the any ESCAPE'd bytes.
#
# In addition, are some 3 transmission control bytes followed by a 2 byte CRC:
#
# The transmssion control bytes are:
#
# * Sequence (8-bits):
#   There is a sequence number that is incremented each time a new message is sent over the
#   serial line.  Only the low 8-bits of this number is sent.  The first message is 0.
#
# * Other sequence (8-bits):
#   The lower 8-bits of the last successfully recevied message from counter direction.
#
# * Missing Messages Mask (8-bits):
#   The mask specifies whether there are any missing messages past the current other sequence
#   value.  If other sequence is N, the bits are for messages N+1 through N+8, where N+1 is
#   the most significant bit and N+8 is the most signicant bit.  It is expected that the
#   missing messages will be retransmitted.
#
# The CRC is a standard 16-bit CCIT CRC.  It is computed over the


import importlib
import struct
import sys
from dataclasses import dataclass
from typing import Any, Callable, ClassVar, Dict, List, Optional, Tuple

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
    # This *main* routine is based off of:
    #
    #    https://answers.ros.org/question/377848/spinning-multiple-nodes-across-multiple-threads/

    # Initialize the Python ROS2 interface library:
    if ros_arguments is None:
        ros_arguments = []
    rclpy.init(args=ros_arguments)

    # Create the *executor*:
    executor: Executor = rclpy.executors.MultiThreadedExecutor()

    # Create the *agent* from command line *arguments*:
    arguments: List[str] = sys.argv[1:]
    agent: Agent = Agent(executor, arguments)

    agent.timer_create("serial_agent", "timer1", 2.0)
    agent.timer_create("serial_agent", "timer2", 5.0)

    # Create the timer_node:
    # timer_node: Node = rclpy.create_node("timer_node")
    # print("Added 'timer_node' to executor")
    # executor.add_node(timer_node)

    print("Start executor.spin()")
    # agent_node: AgentNode = agent.agent_node_create_once("timer")
    # rclpy.spin(agent_node)

    executor.spin()

    # Start the *spin_thread* that does the spinning:
    # spin_thread: Thread = threading.Thread(target=executor.spin, daemon=True)
    # spin_thread.start()

    # rate: Rate = timer_node.create_rate(2)

    # Manually spin on the *timer_node* *rate* to catch control-C and shutdown gracefully:
    # try:
    #    while rclpy.ok():
    #         print("Rate tick")
    #         rate.sleep()
    # except KeyboardInterrupt:
    #     pass

    # Destroy all of the nodes:
    # agent.destroy_nodes()
    # timer_node.destroy_node()

    # Shut down the Python ROS2 interface library.
    rclpy.shutdown()
    # spin_thread.join()

    return 0


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

        # Parse *arguments*:
        if not arguments:
            # Create some default arguments.
            arguments = [
                "serial_agent:publish:std_msgs.msg:String:topic",
                "serial_agent:subscribe:std_msgs.msg:String:topic",
                "serial_agent:publish:std_msgs.msg:String:echo",
                "serial_agent:subscribe:std_msgs.msg:String:echo",
                "serial_agent:client:example_interfaces.srv:AddTwoInts:add_two_ints",
                "serial_agent_server1:server:example_interfaces.srv:AddTwoInts:add_two_ints",
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
        # Sweep through the *arguments:
        # print("=>Agent.arguments_parse(*)")
        argument: str
        for argument in arguments:
            if argument.startswith("-"):
                # Process flags here:
                pass
            else:
                # Split *registration* into *kind*, *import_path*, *type_name*, and *ros_path*:
                registration: List[str] = argument.split(":")
                assert len(registration) == 5, f"Registration '{registration} is wrong'"
                kind: str
                agent_node_name: str
                import_path: str
                type_name: str
                ros_path: str
                acceptable_kinds: Tuple[str, ...] = ("publish", "subscribe", "client", "server")
                agent_node_name, kind, import_path, type_name, ros_path = registration

                topic_handle: TopicHandle
                assert kind in acceptable_kinds, f"'{kind}' not one of {acceptable_kinds}"
                if kind == "publish":
                    self.publisher_create(agent_node_name, import_path, type_name, ros_path)
                elif kind == "subscribe":
                    self.subscription_create(agent_node_name, import_path, type_name, ros_path)
                elif kind == "client":
                    self.client_create(agent_node_name, import_path, type_name, ros_path)
                elif kind == "server":
                    self.server_create(agent_node_name, import_path, type_name, ros_path)
                else:
                    assert False, f"'{kind}' is not an exceptable kind."
        # print("<=Agent.arguments_parse(*)")

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


# Packet:
class Packet(object):
    """Class that describes the format of packet."""

    slip_escape: ClassVar[int] = 0x80  # This must be 0x80
    slip_start: ClassVar[int] = 0x81
    slip_stop: ClassVar[int] = 0x21
    slip_specials: ClassVar[Tuple[int, ...]] = (slip_escape, slip_start, slip_stop)
    struct_endian: str = "!"  # "!" stands for network which happens to be big-endian
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
    crc_compute: ClassVar[Callable[[bytes], int]]
    crc_compute = crcmod.mkCrcFun(0x11021, initCrc=0, xorOut=0xffff)

    # Packet.__init():
    def __init__(self, packet_type: Any) -> None:
        """Initialize a Packet given a message instance."""
        packet_instance: Any = packet_type()
        name_types: Dict[str, str] = packet_instance.get_fields_and_field_types()
        sorted_names: Tuple[str, ...] = tuple(sorted(name_types.keys()))

        # Sweep through the *struct_names* to create *struct_format*, which is used
        # pack and unpack the non-string component of messages:

        # The Python struct library is used to encode and decode most ROS messages.
        # The exception is for strings which are variable length.  So we split *sorted_names*
        # into *struct_names* and *string_names*:
        struct_names: List[str] = []
        string_names: List[str] = []
        name: str
        for name in sorted_names:
            if name_types[name] == "string":
                string_names.append(name)
            else:
                struct_names.append(name)

        # Now sweep through *struct_names* and create the *struct_format* string:
        struct_format: str = self.struct_endian
        struct_name: str
        # ros_type_convert: Dict[str, Tuple[str, str]] = self.ros_type_convert
        for struct_name in struct_names:
            ros_type: str = name_types[struct_name]
            struct_letter: str
            struct_letter, _ = self.ros_type_convert[ros_type]
            struct_format += struct_letter

        # Load results into *self*:
        self.packet_type: Any = packet_type
        self.struct_format: str = struct_format
        self.struct_names: Tuple[str, ...] = tuple(struct_names)
        self.string_names: Tuple[str, ...] = tuple(string_names)

    # Packet.__repr__()
    def __repr__(self) -> str:
        """Return a string representation of a Packet."""
        return (f"Packet({type(self.packet_type)}, '{self.struct_format}', "
                f"{self.struct_names}, {self.string_names}')")

    # Packet.encode():
    def encode(self, packet_content: Any, packet_id: int) -> bytes:
        """Encode packet content into bytes."""
        assert isinstance(packet_content, self.packet_type), (
            f"Packet content is {type(packet_content)} rather than {type(self.packet_type)}")
        packet_bytes: bytearray = bytearray()

        # Start with the *packet_id*:
        assert 0 <= packet_id <= 255, f"id value ({id}) does not fit into a byte"
        packet_bytes.append(packet_id & 0x255)

        # Put all of the strings in next because they have a length byte:
        string_name: str
        for string_name in self.string_names:
            assert string_name in packet_content, (
                "String name '{string_name} not one of {string_names}'")
            string_content: str = packet_content[string_name]
            string_size: int = len(string_content)
            if string_size > 255:
                print("Truncating string to 255 characters")
                string_content = string_content[:255]
                string_size = 255
            packet_bytes.append(string_size)
            c: str
            packet_bytes += bytearray([min(ord(c), 255)
                                       for c in string_content])

        # Put the struct in next:
        struct_name: str
        struct_values: List[int] = [packet_content[struct_name]
                                    for struct_name in self.struct_names]
        packet_bytes += struct.pack(self.struct_format, *struct_values)
        return bytes(packet_bytes)

    def decode(self, packet_bytes: bytes) -> Tuple[int, Any]:
        """Decode a bucket of bytes."""
        # Create the xxx:
        packet_content: Any = self.packet_type()

        index: int = 0
        packet_id: int = packet_bytes[index]
        packet_id += 0  # Ignore for now
        index += 1

        # Sweep through the *string_names* and stuff the associated strings into *packet_type*:
        string_name: str
        for string_name in self.string_names:
            length: int = packet_bytes[index]
            index += 1
            end_index: int = index + length
            byte: int
            string: str = "".join([chr(byte) for byte in packet_bytes[index:end_index]])
            setattr(packet_content, string_name, string)
            index = end_index

        # Unpack the *struct_values* from the remainder of *packet_bytes*:
        struct_values: Tuple[int, ...] = struct.unpack(self.struct_format, packet_bytes[index:])
        struct_names: Tuple[str, ...] = self.struct_names
        assert len(struct_values) == len(struct_names), "Struct mismatch"

        # Stuff *struct_values* into *packet_content*:
        struct_index: int
        struct_name: str
        for struct_index, struct_name in enumerate(struct_names):
            setattr(packet_content, struct_name, struct_values[struct_index])

        return packet_content


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
        topic_publish_count: int = topic_topic_handle.publish_count_get()
        topic_message: Any = topic_topic_handle.message_create()
        topic_message.data = f"topic message {topic_publish_count}"
        topic_topic_handle.publish(topic_message)

        # Publish a message to the topic named 'echo':
        echo_agent_node: AgentNode
        echo_topic_handle: TopicHandle
        echo_agent_node, echo_topic_handle = agent.publisher_lookup(("serial_agent", "echo"))
        echo_publish_count: int = echo_topic_handle.publish_count_get()
        echo_message: Any = echo_topic_handle.message_create()
        echo_message.data = f"echo message {2 * echo_publish_count}"
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
        self.publish_count: int = 0
        self.publisher: Optional[Publisher] = None
        self.publisher_agent_node: Optional[AgentNode] = None
        self.subscription_count: int = 0
        self.subscription: Optional[Subscription] = None
        self.subscription_agent_node: Optional[AgentNode] = None

    # TopicHandle.__repr__():
    def __str__(self) -> str:
        """Return a string representation."""
        return (f"TopicHandle('{self.import_path}', '{self.type_name}', '{self.ros_path}', "
                f"{self.message_packet})")

    # TopicHandle.key_get():
    def key_get(self) -> HandleKey:
        """Return the TopicHandle key."""
        return self.key

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
        self.publish_count += 1
        # print(f"<=TopicHandle.publish('{message}')")

    # TopicHandle.publish_count_get():
    def publish_count_get(self) -> int:
        """Return the number of topic publications processed."""
        return self.publish_count

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
        assert not self.subscription_agent_node
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
        self.request_type: Any = request_type
        self.response_type: Any = response_type
        self.request_packet: Packet = Packet(request_type)
        self.request_count: int = 0
        self.response_packet: Packet = Packet(response_type)
        self.response_count: int = 0

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
        # print(f"=>SerialAgent.server_callback({request}, {response})")
        if self.ros_path == "add_two_ints":
            a: int = request.a
            b: int = request.b
            response.sum = a + b
            print(f"Server('{self.ros_path}'): Computed: {a} + {b} = {sum}")
        else:
            assert False, f"Got request={request} response={response}"
        # print(f"<=SerialAgent.server_callback({request}, {response}) => {response}")
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
