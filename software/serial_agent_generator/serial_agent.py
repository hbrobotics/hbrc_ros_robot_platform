#!/usr/bin/env python

# MIT License
# 
# Copyright 2021 Home Brew Robotics Club
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this
# software and associated documentation files (the "Software"), to deal in the Software
# without restriction, including without limitation the rights to use, copy, modify,
# merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to the following
# conditions:
#
# The above copyright notice and this permission notice shall be included in all or
# substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
# FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
# OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

# <====================================== 100 Characters ========================================> #

import crcmod  # type:ignore
import importlib
import rclpy  # type:ignore
import sys
import time

from rclpy.client import Client  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.publisher import Publisher  # type: ignore
from rclpy.service import Service as Server  # type: ignore
from rclpy.subscription import Subscription  # type: ignore
from rclpy.task import Future  # type: ignore
from typing import Any, Callable, ClassVar, Dict, List, Optional, Set, Tuple

# Types for debugging only:
from std_msgs.msg import String  # type: ignore
from example_interfaces.srv import AddTwoInts  # type: ignore


def main(ros_arguments: List[str] = None) -> int:
    # Deal with command line *arguments*:
    
    if ros_arguments is None:
        ros_arguments = []

    arguments: List[str] = sys.argv[1:]

    # Initialize the Python ROS2 interface library.
    rclpy.init(args=ros_arguments)

    # Create the SerialAgent Node:
    serial_agent: SerialAgent = SerialAgent(arguments)

    # Wait for all requested servers to become avaialble:
    serial_agent.wait_for_servers()

    # Start the serial agent Node.  All activity occurs via callback routines:
    print('Starting rclpy.spin()')
    rclpy.spin(serial_agent)

    # Destroy the serial agent node explicitly.
    serial_agent.destroy_node()

    # Shut down the Python ROS2 interface library.
    rclpy.shutdown()

    return 0

# SerialAgent:
class SerialAgent(Node):
    """Class that implements a ROS serial agent Node."""

    def __init__(self, arguments: List[str]) -> None:
        """Initalize the serial agent:"""

        # Initalize the Node base class:
        super().__init__("SerialAgent")

        if not arguments:
            # Create some default arguments.
            arguments = [
                "publish:std_msgs.msg:String:topic",
                "subscribe:std_msgs.msg:String:topic",
                "client:example_interfaces.srv:AddTwoInts:add_two_ints",
                "server:example_interfaces.srv:AddTwoInts:add_two_ints",
                "publish:std_msgs.msg:String:echo",
                "subscribe:std_msgs.msg:String:echo",
            ]
        print(f"arguments: {arguments}")

        # Collect the various Handle's in lists:
        self.topic_handles: Dict[str, TopicHandle] = {}
        self.service_handles: Dict[str, ServiceHandle] = {}

        # Collect the publisher, subscriptions, clients, and servers:
        self.clients_table: Dict[str, Client] = {}
        self.publishers_table: Dict[str, Publisher] = {}
        self.servers_table: Dict[str, Server] = {}
        self.subscriptions_table: Dict[str, Subscription] = {}
        self.timer_counter: int = 0

        # Sweep through the *arguments:
        argument: str
        for argument in arguments:
            if argument.startswith("-"):
                # Process flags here:
                pass
            else:
                # Split *registration* into *kind*, *import_path*, *type_name*, and *ros_path*:
                registration: List[str] = argument.split(":")
                assert len(registration) == 4, f"Registration '{registration} is wrong'"
                kind: str
                import_path: str
                type_name: str
                ros_path: str
                acceptable_kinds: Tuple[str, ...] = ("publish", "subscribe", "client", "server")
                kind, import_path, type_name, ros_path = registration

                topic_handle: TopicHandle
                assert kind in acceptable_kinds, f"'{kind}' not one of {acceptable_kinds}"
                if kind == "publish":
                    self.publisher_create(import_path, type_name, ros_path)
                elif kind == "subscribe":
                    self.subscription_create(import_path, type_name, ros_path)
                elif kind == "client":
                    self.client_create(import_path, type_name, ros_path)
                elif kind == "server":
                    self.server_create(import_path, type_name, ros_path)
                else:
                    assert False, f"'{kind}' is not an exceptable kind."

        timer_period: float = 4.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # print(f"type(self.timer)={type(self.timer)}")

    # SerialAgent.client_create():
    def client_create(self, import_path: str, type_name: str, ros_path: str) -> None:
        """Create a client for a ROS service."""

        # Check for duplicates before creating the *client*:
        clients_table: Dict[str, Client] = self.clients_table
        assert ros_path not in clients_table, "Do not create two clients for '{ros_path}'"
        service_handle: ServiceHandle = self.service_handle_create(import_path, type_name, ros_path)
        client: Client = service_handle.client_create(self)
        clients_table[ros_path] = client

    # SerialAgent.publisher_create():
    def publisher_create(self, import_path: str, type_name: str, ros_path: str) -> None:
        """Create a publisher."""

        publishers_table: Dict[str, Publisher] = self.publishers_table
        assert ros_path not in publishers_table, "Do not create two publishers for '{ros_path}'"
        topic_handle: TopicHandle = self.topic_handle_create(import_path, type_name, ros_path)
        publisher: Publisher = topic_handle.publisher_create(self)
        publishers_table[ros_path] = publisher            

    # SerialAgent.server_create():
    def server_create(self, import_path: str, type_name: str, ros_path: str) -> None:
        """Create a server for a ROS service."""

        servers_table: Dict[str, Server] = self.servers_table
        assert ros_path not in servers_table, "Do not create two servers for '{ros_path}'"
        service_handle: ServiceHandle = self.service_handle_create(import_path, type_name, ros_path)
        server: Server = service_handle.server_create(self)
        servers_table[ros_path] = server            

    # SerialAgent.server_callback():
    def server_callback(self, service_handle: "ServiceHandle", request: Any, response: Any) -> Any:
        """Process a server callback."""

        # print(f"=>SerialAgent.server_callback({request}, {response})")
        if service_handle.ros_path == "add_two_ints":
            a: int = request.a
            b: int = request.b
            sum: int = a + b
            response.sum = sum
            print(f"Server('{service_handle.ros_path}'): Computed: {a} + {b} = {sum}")
        else:
            assert False, f"Got request={request} response={response}"
        # print(f"<=SerialAgent.server_callback({request}, {response}) => {response}")
        return response

    # SerialAgent.service_handle_create():
    def service_handle_create(self,
                              import_path: str, type_name: str, ros_path: str) -> "ServiceHandle":
        """Return unique ServiceHandle."""

        # Create *service_handle*:
        service_handle: ServiceHandle = ServiceHandle(import_path, type_name, ros_path)

        # If there is a duplicate, use the *previous_service_handle*:
        service_handles: Dict[str, ServiceHandle] = self.service_handles
        if ros_path in service_handles:
            previous_service_handle: ServiceHandle = service_handles[ros_path]
            assert previous_service_handle.match(service_handle), "Mismatched service handles"
            service_handle = previous_service_handle
        else:
            service_handles[ros_path] = service_handle

        # Note only the first *service_handle* that matches the arguments is returned:
        return service_handle
        
    # SerialAgent.subscription_callback():
    def subscription_callback(self, subscription_handle: "TopicHandle", message: Any) -> None:
        """Callback for when a Topic subscription message comes in."""

        # print("=>SerialAgent.subscription_callback()")
        assert isinstance(subscription_handle, TopicHandle)
        ros_path: str = subscription_handle.ros_path
        print(f"Topic('{ros_path}'): Got: {message}")

        # Temporary code for development only:
        if ros_path == "topic":
            # Publish a message to "echo" ROS topic.
            echo_ros_path: str = "echo"
            echo_publisher_handle: TopicHandle = self.topic_handles[echo_ros_path]
            echo_message: Any = echo_publisher_handle.message_create()
            assert isinstance(echo_message, String)
            echo_message.data = f"Echo '{message.data}'"
            echo_publisher_handle.publish(echo_message)
        elif ros_path == "echo":
            # Call a client:
            add_two_ints_ros_path: str = "add_two_ints"
            add_two_ints_service_handle: ServiceHandle = self.service_handles[add_two_ints_ros_path]
            request: Any = add_two_ints_service_handle.request_create()
            request.a = self.timer_counter
            request.b = self.timer_counter + 1
            add_two_ints_service_handle.request_send(request)
        # print("<=SerialAgent.subscription_callback()")

    # SerialAgent.subscription_create():
    def subscription_create(self, import_path: str, type_name: str, ros_path: str) -> None:
        """Create a subscription."""

        subscriptions_table: Dict[str, Subscription] = self.subscriptions_table
        assert ros_path not in subscriptions_table, (
            "Do not create two subscriptions for '{ros_path}'")
        topic_handle: TopicHandle = self.topic_handle_create(import_path, type_name, ros_path)
        subscription: Subscription = topic_handle.subscription_create(self)
        subscriptions_table[ros_path] = subscription

    # SerialAgent.timer_callback():
    def timer_callback(self) -> None:
        """Process timer events."""

        print("")  # Put out a blank line to make the message log easier to read.
        print("=>SerialAgent.timer_callback()")

        # Lookup the *topic_publisher_handle*:
        topic_handles: Dict[str, TopicHandle] = self.topic_handles
        topic_ros_path: str = "topic"
        assert topic_ros_path in topic_handles
        topic_publisher_handle: TopicHandle = self.topic_handles[topic_ros_path]

        # Send an topic_message to *topic_publsher_handle*:
        topic_message: Any = topic_publisher_handle.message_create()
        assert isinstance(topic_message, String)
        topic_message.data = f"timer_counter={self.timer_counter}"
        topic_publisher_handle.publish(topic_message)

        self.timer_counter += 1
        print("<=SerialAgent.timer_callback()")

    # SerialAgent.topic_handle_create():
    def topic_handle_create(self, import_path: str, type_name: str, ros_path: str) -> "TopicHandle":
        """Return unique TopicHandle."""

        # Create *topic_handle*:
        topic_handle: TopicHandle = TopicHandle(import_path, type_name, ros_path)

        # If there is a duplicate, use the *previous_topic_handle*:
        topic_handles: Dict[str, TopicHandle] = self.topic_handles
        if ros_path in topic_handles:
            previous_topic_handle: TopicHandle = topic_handles[ros_path]
            assert previous_topic_handle.match(topic_handle), "Mismatched topic handles"
            topic_handle = previous_topic_handle
        else:
            topic_handles[ros_path] = topic_handle

        # Note only the first *topic_handle* that matches the arguments is returned:
        return topic_handle

    # SerialAgent.wait_for_servers():
    def wait_for_servers(self) -> None:
        """Wait for the servers needed by all clients."""

        clients_table: Dict[str, Client] = self.clients_table
        client_waits: Set[str] = set(clients_table.keys())
        delay: float = 0.0
        while client_waits:
            client_name: str
            # Iterate over an immutable tuple rather than a mutable set:
            print(f"Waiting for {client_waits} server(s) to become available...")
            for client_name in tuple(client_waits):
                client: Client = clients_table[client_name]
                if client.wait_for_service(timeout_sec=0.1):
                    client_waits.remove(client_name)
            delay = min(delay + 0.1, 1.0)
            time.sleep(delay)

# Packet:
class Packet(object):
    """Class that describes the format of packet."""

    slip_escape: ClassVar[int] = 0x80  # This must be 0x80
    slip_start: ClassVar[int] = 0x81
    slip_stop: ClassVar[int] = 0x21
    slip_specials: ClassVar[Tuple[int, ...]] = (slip_escape, slip_start, slip_stop)
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
        "int64": ("q","long long"),
        "uint64": ("Q,", "unsigned long long"),
        # "string": ("p", "char p[]"), # Does not work!
        # "wstring": # not supported
    }
    crc_compute: ClassVar[Callable[[bytes], int]] = (
            crcmod.mkCrcFun(0x11021, initCrc=0, xorOut=0xffff))

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
        struct_format: str = "!"  # "!" stands for network which is big-endian
        struct_name: str
        ros_type_convert: Dict[str, Tuple[str, str]] = self.ros_type_convert
        for struct_name in struct_names:
            ros_type: str  = name_types[struct_name] 
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


# Handle:
class Handle(object):
    """Base class for publish/subscribe topics and client/server services."""

    # Handle.__init__():
    def __init__(self, import_path: str, type_name: str, ros_path: str) -> None:
        """Initialize Handle base class."""

        self.import_path: str = import_path
        self.type_name: str = type_name
        self.ros_path: str = ros_path

    # Handle.match():
    def match(self, handle: "Handle") -> bool:
        """Return True if TopicHandle's match."""

        return (
            self.import_path == handle.import_path and
            self.type_name == handle.type_name and
            self.ros_path == handle.ros_path)

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


# TopicHandle:
class TopicHandle(Handle):
    """Tracking class for publish/subscribe topic."""

    # TopicHandle.__init__():
    def __init__(self, import_path: str, type_name: str, ros_path: str) -> None:
        """Initialize TopicHandle."""

        # Initialize base class:
        super().__init__(import_path, type_name, ros_path)

        # Import the relevant information about the topic:
        message_type: Any = self.type_import(import_path, type_name, ros_path)

        # Load values into the TopicHandle:
        self.message_type: Any = message_type
        self.message_packet: Packet = Packet(message_type)

        # The following fields get filled in later:
        self.publisher: Optional[Publisher] = None
        self.subscription: Optional[Subscription] = None
        self.serial_agent: Optional[SerialAgent] = None

    # TopicHandle.__repr__():
    def __str__(self) -> str:
        """Return a string representation."""

        return (f"TopicHandle('{self.import_path}', '{self.type_name}', '{self.ros_path}', "
                f"{self.message_packet})")

    # TopicHandle.publish():
    def publish(self, message: Any) -> Publisher:
        """Publish a message to the topic."""

        publisher: Optional[Publisher] = self.publisher
        assert publisher, "Publisher net set yet."
        message_type: Any = self.message_type
        assert isinstance(message, message_type), f"Message is not of type {message_type}"
        print(f"Topic('{self.ros_path}): Published: {message}'")
        publisher.publish(message)

    # TopicHandle.publisher_create():
    def publisher_create(self, serial_agent: SerialAgent) -> Publisher:
        """Create a publisher for a ROS topic."""

        print(f"=>TopicHandle.publisher_create()")
        # It is pointless to check of duplicate SerialAgent's, since there is only one:
        self.serial_agent = serial_agent

        # Ensure that no attempt is made to create a duplicate *publisher*:
        assert not self.publisher, "Publisher already present."

        # Create and return the new *publisher*:
        publisher: Publisher = serial_agent.create_publisher(
            self.message_packet.packet_type,
            self.ros_path,
            10)
        self.publisher = publisher
        print(f"<=TopicHandle.publisher_create()=>{publisher}")
        return publisher

    # TopicHandle.subscription_create():
    def subscription_create(self, serial_agent: SerialAgent) -> Subscription:
        """Create a subcription for a ROS topic."""

        # print("=>TopicHandle.subscription_create()")
        # It is pointless to check of duplicate SerialAgent's, since there is only one:
        self.serial_agent = serial_agent

        # Ensure that no attempt is made to create a duplicate *subscription*:
        assert not self.subscription, "Publisher already present."

        # Create and return the new *subscription*:
        subscription: Subscription = serial_agent.create_subscription(
            self.message_packet.packet_type,
            self.ros_path,
            self.subscription_callback,
            10)
        # print(f"=>TopicHandle.subscription_create()=>{subscription}")
        return subscription

    # TopicHandle.message_create():
    def message_create(self) -> Any:
        """Create a new message object instance."""

        return self.message_type()

    # TopicHandle.subscription_callback():
    def subscription_callback(self, message: Any) -> None:
        """Forward a subscription callback to serial agent. """

        # print("")
        # print("=>TopicHandle.subscription_callback()")
        serial_agent: Optional[SerialAgent] = self.serial_agent
        assert isinstance(serial_agent, SerialAgent)
        serial_agent.subscription_callback(self, message)
        # print("<=TopicHandle.subscription_callback()")


# ServiceHandle:
class ServiceHandle(Handle):
    """Tracking class for clientserver service."""

    # ServiceHandle.__init__():
    def __init__(self, import_path: str, type_name: str, ros_path: str) -> None:
        """Initialize Handle class."""

        # Initialize base class:
        super().__init__(import_path, type_name, ros_path)

        # Import the relevant information about the topic:
        service_type: Any = self.type_import(import_path, type_name, ros_path)
        request_type: Any = service_type.Request
        response_type: Any = service_type.Response

        # Load values into the TopicHandle:
        self.service_type: Any = service_type
        self.request_type: Any = request_type
        self.response_type: Any = response_type
        self.request_packet: Packet = Packet(request_type)
        self.response_packet: Packet = Packet(response_type)

        # These field get filled in later:
        self.client: Optional[Client] = None
        self.server: Optional[Server] = None
        self.serial_agent: Optional[SerialAgent] = None

    # ServiceHandle.__repr__():
    def __str__(self) -> str:
        """Return a string representation."""

        return (f"TopicHandle('{self.import_path}', '{self.type_name}', '{self.ros_path}', "
                f"{self.request_packet}, {self.response_packet})")
    
    # ServiceHandle.client_create():
    def client_create(self, serial_agent: SerialAgent) -> Client:
        """Create a client for a ROS service."""

        print("=>ServiceHandle.client_create()")

        # It is pointless to check of duplicate SerialAgent's, since there is only one:
        self.serial_agent = serial_agent

        # Ensure that no attempt is made to create a duplicate *subscription*:
        assert not self.client, "Client already present."

        # Create and return the new *subscription*:
        client: Client = serial_agent.create_client(self.service_type, self.ros_path)

        # The SerialAgent.wait_for_servers() is called seperately to ensure all servers
        # are running before the event loop is started.
        self.client = client

        print(f"<=ServiceHandle.client_create()=>{client}")
        return client

    # ServiceHandle.server_callback():
    def server_callback(self, request: Any, response: Any) -> Any:
        """Process a server request."""

        # print("")
        # print("=>ServiceHandle.server_callback()")
        # Verify that *request* and *response* have the correct types:
        assert isinstance(request, self.request_type), (
            f"request type is {type(request)} instead of {self.request_type}")
        assert isinstance(response, self.response_type), (
            f"response type is {type(response)} instead of {self.response_type}")

        # Forward callback back up to *SerialAgent.server_callback()*:
        serial_agent: Optional[SerialAgent] = self.serial_agent
        assert isinstance(serial_agent, SerialAgent), "SerialAgent not present yet."
        serial_agent.server_callback(self, request, response)
        # print(f"<=ServiceHandle.server_callback()=>{response}")
        return response

    # ServiceHandle.server_create():
    def server_create(self, serial_agent: SerialAgent) -> Client:
        """Create a server for a ROS service."""

        print("=>ServiceHandle.srver_create()")

        # It is pointless to check of duplicate SerialAgent's, since there is only one:
        self.serial_agent = serial_agent

        # Ensure that no attempt is made to create a duplicate *subscription*:
        assert not self.server, "Server already present."

        # Create and return the new *subscription*:
        server: Server = serial_agent.create_service(
            self.service_type, self.ros_path, self.server_callback)

        # The SerialAgent.wait_for_servers() is called seperately to ensure all servers
        # are running before the event loop is started.
        self.server = server

        print(f"<=ServiceHandle.srver_create()=>{server}")
        return server

    # ServiceHandle.request_create():
    def request_create(self) -> Any:
        """Return a ne request object."""

        return self.request_type()

    # ServiceHandle.request_send():
    def request_send(self, request: Any) -> None:
        """Send a request to a server."""

        # print(f"=>ServiceHandle.request_send({request}):")
        assert isinstance(request, self.request_type)
        client: Optional[Client] = self.client
        assert isinstance(client, Client)
        response_future: Future = client.call_async(request)
        response_future.add_done_callback(self.response_callback)
        print(f"Service('{self.ros_path}'): Sent request: {request}")
        # print(f"<=ServiceHandle.request_send({request}):")

    # ServiceHandle.response_callback():
    def response_callback(self, response_future: Future) -> Any:
        """Deal with an asynchronous response to a server request."""
        
        # print("=>ServiceHandle.response_callback()")
        try:
            response: Any = response_future.result()
        except Exception as error:
            assert False
        else:
            assert isinstance(response, self.response_type)
        print(f"Service('{self.ros_path}'): Got response: {response}')")
        # print(f"<=ServiceHandle.response_callback()=>{response}")
        return response

    # ServiceHandle.response_create():
    def response_create(self) -> Any:
        """Return a ne response object."""

        return self.response_type()


if __name__ == "__main__":
    main()

