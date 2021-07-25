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

## Command Line

The current intention is do all configuration from the command line.  This avoids a
separate configuration file and instead allows all configuration to be done via ROS launch files.

There are 5 related commands that specify various mirroring operations that are very similar
to one another:

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

* `PRIORITY`: This specifies the message priority where the higher priority messages are always
  sent first.  At least one timer is required to ensure that the protocol will recover from
  dropped messages.  This timer needs to have the highest priority.  An emergency stop message
  probably has the next highest priority.

* `IMPORT_PATH`: This name of a Python package that contains the needed message definitions
  (e.g. `std_msgs.msg`, `example_interfaces.srv`, etc.)

* `TYPE_NAME`: In rospy, the message format is specified by Python type.  In conjunction with the
  `IMPORT_PATH`, it is possible for Python to dynamically load in the message format information.
  (e.g. `String`, `AddTwoInts`, etc.)

* `ROSPATH`: This the ROS topic or service name (e.g. `sonars`, `motor/left`, `motor/right`, etc.)

* `TIMER_NAME`: Is the name of the timer.

* `RATE`: This species the timer period as a rate in Hz.


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

* Handles:

  There is a class hierarchy that represents different message types:

  * Handle: The Base class for the following sub-classes:

    * PingHandle: A message used to keep the serial line alive and trigger re-transmission.

    * TimerHandle: A generic timer event handle.

    * ROSHandle: The class for all ROS topic and service messages.

      * TopicHandle: The class that represents ROS publish/subscribe topics.

      * ServiceHandle: The class that represents ROS client/server topics.

* SerialLine: Represents a serial line.

  * IOChannel:  Represents either a read or write serial line.

* Packet: A class for encoding decoding packages over a serial line.

## Execution Model

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

## Packet Format

There are two dedicated Python threads for interfacing to the serial line.  One thread sends
serial line traffic and the other receives serial line traffic.  These two threads communicate
via Python Queues.  There is a protocol similar to SLIP (Serial Line Internet Protocol) that
does the frames the messages.  There is a CRC (Cyclic Redundancy Check) to detect damaged
messages which then get transmitted again.  In order to simplify message buffer memory management
on the microcontroller, the total message byte length is sent at the front of each message.

Any messages that are the wrong length or do not match the CRC are dropped to force
and the transmission control system will get them transmitted again.  All of the
transmission control is managed by the robot host to simplify the microcontroller code.

See the `Packet` class for way more information.

## Code Generation

To be designed.

## Miscellaneous

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

import fcntl
import importlib
import os
import queue
import stat
import struct
import sys
import time
from dataclasses import dataclass
from queue import Queue
from typing import Any, Callable, ClassVar, Dict, IO, List, Optional, Tuple

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
    arguments: Tuple[str, ...] = tuple(sys.argv[1:])
    agent: Agent = Agent(executor, arguments)

    serial_line: Optional[SerialLine] = agent.serial_line
    assert isinstance(serial_line, SerialLine), "Not a SerialLine"
    print("Creating reader task")
    executor.create_task(serial_line.reader)
    executor.create_task(serial_line.writer)

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


# DecodedPacket:
@dataclass(frozen=True)
class DecodedPacket:
    """Class that represents a decoded packet."""

    # The "from_" prefix refers to packet received from the microcontroller to the agent.
    from_sequence: int  # The from packet sequence index
    from_packet_id: int  # The from packet id from the microcontroller.
    from_partial_packet: bytes  # The partial packet from the microcontroller.
    # The "to_" prefix refers to packets previously sent from agent to microcontroller.
    to_sequence: int  # Highest packet seq. sent to microcontroller has no missing packets below.
    to_missing: int  # A mask of missing packets that the microcontroller has not yet received.


# ROSFieldType:
@dataclass(order=True, frozen=True)
class ROSFieldType:
    """Class that represents information about a ROS Type."""

    ros_type: str  # ROS type (e.g. "bool", "int8", ..., "duration")
    cpp_type: str  # C++ type (e.g. "uint8_t", "int8_t", ..., "ros::Duration")
    python_type: str  # Python type (e.g. "bool", "int", ..., "rospy.Duration")
    struct_format: str  # struct type (e.g. "B", "b", ..., "ii")
    c_type: str  # C type (e.g. "unsigned char", "signed char", ..., "2 * signed")
    size: int  # Size in bytes (e.g. (1, 2, ..., 8)


# SLIP:
@dataclass(order=True, frozen=True)
class SLIP:
    """Class that contains the SLIP-like (Serial Line Interget Prococol) constants."""

    start: int  # Start byte
    escape: int  # Escape byte
    stop: int  # Stop byte
    minimum: int  # Minimum of (start, escape, stop)
    twiddle: int  # Slip twiddle bit


# Kludge: Just access this function using a global declaration.
crc_compute: Callable[[bytes], int] = crcmod.mkCrcFun(0x11021, initCrc=0, xorOut=0xffff)


# Agent:
class Agent:
    """Class that implements a ROS serial agent Node."""

    # This is the only place the *slip* object is defined.
    slip: ClassVar[SLIP] = SLIP(start=0xfc, escape=0xfd, stop=0xfe, twiddle=0x80, minimum=0xfc)

    # Agent.__init__():
    def __init__(self, executor: Executor, arguments: Tuple[str, ...]) -> None:
        """Init the serial agent."""
        # Collect the various objects in tables to detect accidental duplicates and
        # for final shut-down:
        print("=>Agent.__init__()")
        self.agent_nodes: Dict[str, AgentNode] = {}
        self.executor: Executor = executor
        self.service_handles: Dict[ROSKey, ServiceHandle] = {}
        self.timer_handles: Dict[Tuple[str, str], TimerHandle] = {}
        self.topic_handles: Dict[ROSKey, TopicHandle] = {}

        self.clients_table: Dict[Tuple[str, ROSKey], Tuple[AgentNode, ServiceHandle]] = {}
        self.publishers_table: Dict[Tuple[str, ROSKey], Tuple[AgentNode, TopicHandle]] = {}
        self.servers_table: Dict[Tuple[str, ROSKey], Tuple[AgentNode, ServiceHandle]] = {}
        self.subscriptions_table: Dict[Tuple[str, ROSKey], Tuple[AgentNode, TopicHandle]] = {}
        self.timers_table: Dict[Tuple[str, str], Tuple[AgentNode, TimerHandle]] = {}

        # Table to map a packet id to a tagged handle:
        self.packet_id_to_handle: List[Tuple[Handle, str]] = []
        self.packet_id: int = 0

        # The first packet id is for ping packets:
        self.ping_handle: PingHandle = PingHandle(0, 0)
        self.ping_packet_id: int = self.packet_id_get(self.ping_handle, "")
        self.ping_packet: bytes = bytes(self.ping_packet_id)  # Empty packet with just a packet id.

        # Define this before creating *ping_packet*:
        self.prioritized_packets: List[List[bytes]] = []

        # Command line argument stuff:
        self.is_agent: bool = False
        self.is_micro: bool = False
        self.is_stand_alone: bool = False
        self.read_io_channel: Optional[IOChannel] = None
        self.write_io_channel: Optional[IOChannel] = None

        # The queue that runs evertyhing:
        self.pending_queue: Queue[bytes] = Queue()

        # Configrute evertyhing using *argumentes*:
        self.arguments_parse(arguments)

        # Create the *serial_line*:
        read_io_channel: Optional[IOChannel] = self.read_io_channel
        write_io_channel: Optional[IOChannel] = self.write_io_channel
        if not read_io_channel:
            raise ValueError("No read channel specified")
        if not write_io_channel:
            raise ValueError("No write channel specified")
        self.serial_line: SerialLine = SerialLine(
            read_io_channel, write_io_channel, tuple(self.packet_id_to_handle),
            self.pending_queue, self.prioritized_packets, self.ping_packet)

        # Compute the *maximum_priority*:
        packet_id_to_handle: List[Tuple[Handle, str]] = self.packet_id_to_handle
        handle: Handle
        tag: str
        max_priority: int = 0
        index: int
        handle_tag: Tuple[Handle, str]
        for index, handle_tag in enumerate(packet_id_to_handle):
            handle, tag = handle_tag
            priority: int = handle.priority_get(tag)
            if False:
                print(f"Agent.__init__():ID[{index}]: '{tag}':{priority}:{handle}")
            max_priority = max(max_priority, priority)

        # Ensure that that there is an empty buffer for each priority level:
        while len(self.prioritized_packets) < max_priority + 1:
            self.prioritized_packets.append([])
        print("Agent.__init__(): "
              f"max_priority={max_priority} prioritized_packets={self.prioritized_packets}")
        print("=>Agent.__init__()")

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

    # Agent.arguments_empty_expand():
    def arguments_empty_expand(self, arguments: Tuple[str, ...]) -> Tuple[str, ...]:
        """Expand an empty arguments list."""
        if not arguments:
            # Create some default arguments.
            arguments += (
                "--publisher=serial_agent:1:std_msgs.msg:String:topic",
                "--subscription=serial_agent:0:std_msgs.msg:String:topic",
                "--publisher=serial_agent:1:std_msgs.msg:String:echo",
                "--subscription=serial_agent:0:std_msgs.msg:String:echo",
                "--client=serial_agent:0:example_interfaces.srv:AddTwoInts:add_two_ints",
                "--server=serial_agent_server1:1:example_interfaces.srv:AddTwoInts:add_two_ints",
            )
            if self.is_stand_alone:
                # In stand alone mode, just dump output to a file:
                arguments += (
                    "--timer=serial_agent_timer:10:timer:0.5",
                    "--io=/tmp/agent2micro:device,write",  # Dump to file rather than pipe.
                    "--io=/tmp/micro2agent:create,pipe,read",
                )
            elif self.is_agent:
                # In agnent mode, use two pipes:
                arguments += (
                    "--timer=serial_agent_timer:10:timer:0.5",
                    "--io=/tmp/agent2micro:create,pipe,write",
                    "--io=/tmp/micro2agent:create,pipe,read",
                )
            elif self.is_micro:
                # In *is_micro_mode*, there is no timer and pipe reads/write are flipped:
                arguments += (
                    "--io=/tmp/agent2micro:create,pipe,read",
                    "--io=/tmp/micro2agent:create,pipe,write",
                )
        return arguments

    # Agent.argument_files_read():
    def argument_files_read(self, arguments: Tuple[str, ...]) -> Tuple[str, ...]:
        """Read in arguments files that start with '@'."""
        expanded_arguments: List[str] = []
        argument: str
        for argument in arguments:
            if argument.startswith("@"):
                # Open *arguments_file_name* for reading:
                arguments_file_name: str = argument[1:]
                if not os.path.exists(arguments_file_name):
                    raise ValueError(f"@{argument} does not exist")

                # Read in *argument_lines*:
                arguments_file: IO[str]
                lines: List[str]
                with open(arguments_file_name, "r") as arguments_file:
                    lines = arguments_file.read().splitlines()

                # Append all non-empty and non-comment lines:
                line: str
                for line in lines:
                    if line and not line.startswith("#"):
                        expanded_arguments.append(line.rstrip())
            else:
                expanded_arguments.append(argument)
        return tuple(expanded_arguments)

    # Agent.arguments_io_process():
    def arguments_io_process(self, argument: str, flag_name: str,
                             colon_split: Tuple[str, ...]) -> None:
        """Process the --io flag."""
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

    # Agent.arguments_message_process():
    def arguments_message_process(self, argument: str, flag_name: str,
                                  colon_split: Tuple[str, ...]) -> None:
        """Process --publisher, --subscription, --client, and --server flags."""
        # Unpack *colon_split* into *agent_name*, *priority*, *import_path*,
        # *type_name*, and *ros_path*:
        if len(colon_split) != 5:
            raise ValueError(
                f"Commad line argument '{argument}' has {len(colon_split)} instead "
                "of the desired 5 -- NODE_NAME:PRIORITY:IMPORT_PATH:TYPE_NAME:ROSPATH")

        # Unpack *colon_split*:
        agent_name: str
        priority: str
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
        ros_key: ROSKey = ROSKey(import_path, type_name, ros_path)
        if flag_name == "--publisher":
            self.publisher_create(agent_node_name, priority_value, ros_key)
        elif flag_name == "--subscription":
            self.subscription_create(agent_node_name, priority_value, ros_key)
        elif flag_name == "--client":
            self.client_create(agent_node_name, priority_value, ros_key)
        elif flag_name == "--server":
            self.server_create(agent_node_name, priority_value, ros_key)

    # Agent.arguments_mode_extract():
    def arguments_mode_extract(self, arguments: Tuple[str, ...]) -> Tuple[str, ...]:
        """Process the --agent and --micro flags."""
        print("=>Agent.mode_extract({arguments})")
        remaining_arguments: Tuple[str, ...] = ()
        arguement: str
        for argument in arguments:
            if argument == "--is-agent":
                self.is_agent = True
            elif argument == "--is-micro":
                self.is_micro = True
            else:
                remaining_arguments += (argument,)

        # Perform mode sanity checks:
        self.is_stand_alone = False
        if self.is_agent and self.is_micro:
            raise ValueError("Setting both --agent and --micro is not allowed.")
        elif not self.is_agent and not self.is_micro:
            # If neither are specified, default to *is_agent* and mark *is_stand_alone*:
            self.is_agent = True
            # This flag is only used in Agent.empty_expand().
            self.is_stand_alone = True
        print(f"<=Agent.mode_extract({arguments})=>{remaining_arguments}")
        return tuple(remaining_arguments)

    # Agent.arguments_parse():
    def arguments_parse(self, arguments: Tuple[str, ...]) -> None:
        """Parse the command line arguments."""
        # Prescan arguments to set *is_agent* and *is_micro* flags.
        # Also, deal with indirect flags file (i.e. "@args_file_name"):
        original_arguments: Tuple[str, ...] = arguments
        print(f"=>Agent.arguments_parse({original_arguments})")
        arguments = self.argument_files_read(arguments)
        print(f"Agent.arguments_parse: after files_read: arguments={arguments}")
        arguments = self.arguments_mode_extract(arguments)
        print(f"Agent.arguments_parse: after mode extract: arguments={arguments}")
        arguments = self.arguments_empty_expand(arguments)
        print(f"Agent.arguments_parse: after empty expand: arguments={arguments}")

        # Now do the full scan of *culled_arguments*:
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
                self.arguments_message_process(argument, flag_name, colon_split)
            elif flag_name == "--timer":
                self.arguments_timer_process(argument, flag_name, colon_split)
            elif flag_name == "--io":
                self.arguments_io_process(argument, flag_name, colon_split)
            else:
                raise ValueError(f"Unrecognized flag '{flag_name}'")
        self.arguments_sanity_check()
        print(f"<=Agent.arguments_parse({original_arguments})")

    # Agent.arguments_sanity_check():
    def arguments_sanity_check(self):
        """Perform a sanity check of the command line arguments."""
        # Open the I/O channels:
        # print(f"read_io_channel:{self.read_io_channel}")
        # print(f"write_io_channel:{self.write_io_channel}")
        if self.read_io_channel and not self.write_io_channel:
            raise ValueError(f"Read channel '{self.read_io_channel.file_name}' is specified "
                             "but no write channel is specified")
        if self.write_io_channel and not self.read_io_channel:
            raise ValueError(f"Write channel '{self.write_io_channel.file_name}' is specified "
                             "but no read channel is specified")

    # Agent.arguments_timer_process():
    def arguments_timer_process(self, argument: str, flag_name: str,
                                colon_split: Tuple[str, ...]) -> None:
        """Process the --timer flag."""
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
        self.timer_create(agent_node_name, priority_value, timer_name, 1 / rate_value)

    # Agent.client_create():
    def client_create(self, agent_node_name: str, priority: int, ros_key: "ROSKey") -> None:
        """Create a client for a ROS service."""
        # print(f"=>client_create()")
        if self.is_agent:
            agent_node: AgentNode = self.agent_node_create_once(agent_node_name)
            service_handle: ServiceHandle = self.service_handle_get(ros_key)
            agent_node.client_create(service_handle)

            # Record information into the *clients_table*:
            key: Tuple[str, ROSKey] = (agent_node_name, ros_key)
            clients_table: Dict[Tuple[str, ROSKey],
                                Tuple[AgentNode, ServiceHandle]] = self.clients_table
            if key in clients_table:
                raise RuntimeError(f"Agent.client_create(): Duplicate client {key}")
            clients_table[key] = (agent_node, service_handle)
        # print(f"<=client_create(): key={key})")

    # Agent.packet_id_get():
    def packet_id_get(self, handle: "Handle", tag: str) -> int:
        """Return the next packet id."""
        # Update *packet_id*:
        # print(f"=>Agent.packet_id_get({handle}, '{tag}')")
        packet_id: int = self.packet_id
        self.packet_id += 1

        # Tack *handle* onto the *packet_id_to_handle*:
        packet_id_to_handle: List[Tuple[Handle, str]] = self.packet_id_to_handle
        packet_id_to_handle.append((handle, tag))
        assert len(packet_id_to_handle) == packet_id + 1, "Agent.packet_id_get()"
        # print(f"<=Agent.packet_id_get({handle}, '{tag}')=>{packet_id}")
        return packet_id

    # Agent.nodes_destroy():
    def nodes_destroy(self) -> None:
        """Destroy all of the Agent nodes."""
        # print("=>Agent.destroy_nodes()")
        agent_node: Node
        for agent_node in self.agent_nodes.values():
            agent_node.destroy_node()

        serial_line: Optional[SerialLine] = self.serial_line
        if serial_line:
            serial_line.shutdown()
        # print("<=Agent.destroy_nodes()")

    # Agent.publisher_create():
    def publisher_create(self, agent_node_name: str, priority: int, ros_key: "ROSKey") -> None:
        """Create a publisher."""
        # Create the publisher:
        if self.is_agent:
            agent_node: AgentNode = self.agent_node_create_once(agent_node_name)
            topic_handle: TopicHandle = self.topic_handle_get(ros_key)
            agent_node.publisher_create(topic_handle)
            topic_handle.priority_set("pub", priority)

            # Record information into *publishers_table*:
            key: Tuple[str, ROSKey] = (agent_node_name, ros_key)
            publishers_table: Dict[Tuple[str, ROSKey],
                                   Tuple[AgentNode, TopicHandle]] = self.publishers_table
            if key in publishers_table:
                raise RuntimeError(f"Agent.publisher_create(): Duplicate publisher {key}")
            publishers_table[key] = (agent_node, topic_handle)

    # Agent.server_create():
    def server_create(self, agent_node_name: str, priority: int, ros_key: "ROSKey") -> None:
        """Create a server for a ROS service."""
        # Create the server:
        if self.is_agent:
            agent_node: AgentNode = self.agent_node_create_once(agent_node_name)
            service_handle: ServiceHandle = self.service_handle_get(ros_key)
            agent_node.server_create(service_handle)
            service_handle.priority_set("resp", priority)

            # Record information into the *servers_table*:
            key: Tuple[str, ROSKey] = (agent_node_name, ros_key)
            servers_table: Dict[Tuple[str, ROSKey],
                                Tuple[AgentNode, ServiceHandle]] = self.servers_table
            if key in servers_table:
                raise RuntimeError(f"Agent.server_create(): Duplicate server {key}")
            servers_table[key] = (agent_node, service_handle)

    # Agent.service_handle_get():
    def service_handle_get(self, ros_key: "ROSKey") -> "ServiceHandle":
        """Get a ServiceHandle for a ROS Server."""
        service_handles: Dict[ROSKey, ServiceHandle] = self.service_handles
        if ros_key not in service_handles:
            service_handles[ros_key] = ServiceHandle(ros_key, self)
        service_handle: ServiceHandle = service_handles[ros_key]
        return service_handle

    # Agent.slip_get():
    @classmethod
    def slip_get(cls) -> SLIP:
        """Return the SLIP constants."""
        return cls.slip

    # Agent.subscription_create():
    def subscription_create(self, agent_node_name: str, priority: int, ros_key: "ROSKey") -> None:
        """Create a subscription."""
        # print("=>agent.subscription_create()")
        if self.is_agent:
            agent_node: AgentNode = self.agent_node_create_once(agent_node_name)
            topic_handle: TopicHandle = self.topic_handle_get(ros_key)
            agent_node.subscription_create(topic_handle)

            # Record information into *subscriptions_table*:
            key: Tuple[str, ROSKey] = (agent_node_name, ros_key)
            subscriptions_table: Dict[Tuple[str, ROSKey],
                                      Tuple[AgentNode, TopicHandle]] = self.subscriptions_table
            if key in subscriptions_table:
                raise RuntimeError(f"Agent.subscription_create(): Duplicate subscription {key}")
            subscriptions_table[key] = (agent_node, topic_handle)
        # print("<=agent.subscription_create()")

    # Agent.timer_create():
    def timer_create(self, agent_node_name, priority: int,
                     timer_name: str, timer_period: float) -> None:
        """Create a timer."""
        # print("=>Agent.timer_create()")
        if self.is_agent:
            agent_node: AgentNode = self.agent_node_create_once(agent_node_name)
            timer_handle: TimerHandle = self.timer_handle_get(agent_node_name, priority, timer_name)
            agent_node.timer_create(timer_handle, timer_period)
        # print("<=Agent.timer_create()")

    # Agent.timer_handle.get():
    def timer_handle_get(self, agent_node_name: str, priority: int,
                         timer_name: str) -> "TimerHandle":
        """Get a unique timer handle."""
        timer_handles: Dict[Tuple[str, str], TimerHandle] = self.timer_handles
        timer_handle_key: Tuple[str, str] = (agent_node_name, timer_name)

        timer_handle: TimerHandle
        if timer_handle_key not in timer_handles:
            timer_handle = TimerHandle(agent_node_name, priority, timer_name, self)
            timer_handles[timer_handle_key] = timer_handle
        else:
            timer_handle = timer_handles[timer_handle_key]
        return timer_handle

    # Agent.topic_handle_get():
    def topic_handle_get(self, ros_key: "ROSKey") -> "TopicHandle":
        """Get a unique TopicHandle for a ROS topic."""
        topic_handles: Dict[ROSKey, TopicHandle] = self.topic_handles
        if ros_key not in topic_handles:
            topic_handles[ros_key] = TopicHandle(ros_key, self)
        topic_handle: TopicHandle = topic_handles[ros_key]
        return topic_handle

    # Agent.client_lookup():
    def client_lookup(self, key: "Tuple[str, ROSKey]") -> "Tuple[AgentNode, ServiceHandle]":
        """Lookup a client."""
        clients_table: Dict[Tuple[str, ROSKey],
                            Tuple[AgentNode, ServiceHandle]] = self.clients_table
        assert key in clients_table, f"client key {key} is not one of {tuple(clients_table.keys())}"
        return clients_table[key]

    # Agent.publisher_lookup():
    def publisher_lookup(self, key: "Tuple[str, ROSKey]") -> "Tuple[AgentNode, TopicHandle]":
        """Lookup a publisher."""
        publishers_table: Dict[Tuple[str, ROSKey],
                               Tuple[AgentNode, TopicHandle]] = self.publishers_table
        assert key in publishers_table, f"Could not find publisher that matches {key}"
        return publishers_table[key]

    # Agent.server_lookup():
    def server_lookup(self, key: "Tuple[str, ROSKey]") -> "Tuple[AgentNode, ServiceHandle]":
        """Lookup a server."""
        servers_table: Dict[Tuple[str, ROSKey],
                            Tuple[AgentNode, ServiceHandle]] = self.servers_table
        assert key in servers_table, f"Could not find server that matches {key}"
        return servers_table[key]

    # Agent.subscription_lookup():
    def subscription_lookup(self, key: "Tuple[str, ROSKey]") -> "Tuple[AgentNode, TopicHandle]":
        """Lookup a subscription."""
        subscriptions_table: Dict[Tuple[str, ROSKey],
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
        self.clients_table: Dict[ROSKey, Client] = {}
        self.publishers_table: Dict[ROSKey, Publisher] = {}
        self.servers_table: Dict[ROSKey, Server] = {}
        self.subscriptions_table: Dict[ROSKey, Subscription] = {}
        self.timers_table: Dict[Tuple[str, str], Timer] = {}

    # AgentNode.client_create():
    def client_create(self, service_handle: "ServiceHandle") -> None:
        """Create a client for a ROS service."""
        clients_table: Dict[ROSKey, Client] = self.clients_table
        ros_key: ROSKey = service_handle.ros_key
        if ros_key in clients_table:
            raise RuntimeError(f"Duplicate server {ros_key}")
        client: Client = service_handle.client_create(self)
        clients_table[ros_key] = client

    # AgentNode.publisher_create():
    def publisher_create(self, topic_handle: "TopicHandle") -> None:
        """Create a ROS topic publisher."""
        publishers_table: Dict[ROSKey, Publisher] = self.publishers_table
        topic_key: ROSKey = topic_handle.key_get()
        assert topic_key not in self.publishers_table, f"Duplicate publisher {topic_key}"
        publisher: Publisher = topic_handle.publisher_create(self)
        publishers_table[topic_key] = publisher

    # AgentNode.server_create():
    def server_create(self, service_handle: "ServiceHandle") -> None:
        """Create a server for a ROS service."""
        servers_table: Dict[ROSKey, Server] = self.servers_table
        service_key: ROSKey = service_handle.key_get()
        assert service_key not in self.servers_table, f"Duplicate server {service_key}"
        server: Server = service_handle.server_create(self)
        servers_table[service_key] = server

    # AgentNode.subscription_create():
    def subscription_create(self, topic_handle: "TopicHandle") -> None:
        """Create a ROS topic subscription."""
        subscriptions_table: Dict[ROSKey, Subscription] = self.subscriptions_table
        ros_key: ROSKey = topic_handle.ros_key
        if ros_key in self.subscriptions_table:
            raise RuntimeError(f"Duplicate subscription {ros_key}")
        subscription: Subscription = topic_handle.subscription_create(self)
        subscriptions_table[ros_key] = subscription

    # AgentNode.timer_create():
    def timer_create(self, timer_handle: "TimerHandle", period: float) -> None:
        """Create a timer."""
        timers_table: Dict[Tuple[str, str], Timer] = self.timers_table
        timer_key: Tuple[str, str] = timer_handle.key_get()
        assert timer_key not in timers_table, f"Duplicate timer '{timer_key}'"
        timer: Timer = self.create_timer(period, timer_handle.callback)
        timers_table[timer_key] = timer


# encoder_create():
def xencoder_create(slip: SLIP) -> Tuple[Tuple[int, ...], ...]:
    """Return the escape encoder tuple table."""
    twiddle: int = slip.twiddle
    escape: int = slip.escape
    specials: Tuple[int, ...] = (slip.start, escape, slip.stop)
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
    slip: SLIP = Agent.slip_get()
    # encoder: ClassVar[Tuple[Tuple[int, ...], ...]] = encoder_create(slip)

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
    def __init__(self, packet_type: Any, packet_id: int) -> None:
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
        # print(f"size_name_types={size_name_types}")

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
        self.packet_id: int = packet_id
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
    def full_decode(cls, full_packet: bytes) -> DecodedPacket:
        """Convert a full packet into a DecodedPacket."""
        # print(f"=>Packet.full_decode({repr(full_packet)})")

        slip: SLIP = Agent.slip_get()

        # Process the first 3 bytes (i.e. Start and Length):
        full_size: int = len(full_packet)
        if full_size < 3:
            raise ValueError(f"packet {repr(full_packet)} is too small")
        if full_packet[0] != slip.start:
            raise ValueError(f"packet {repr(full_packet)} does not start with {hex(slip.start)}")

        # Extract and verify the *length*:
        high_length: int = full_packet[1]
        low_length: int = full_packet[2]
        if high_length >= 128 or low_length >= 128:
            raise ValueError(f"packet {repr(full_packet)} has actual length bytes of "
                             f"{hex(high_length)} and {hex(low_length)} "
                             "neither of which should have bit 0x80 set")
        length: int = (high_length << 7) | low_length
        if full_size != length:
            raise ValueError(f"packet {repr(full_packet)} has actual length of {full_size},"
                             f"not the desired length of {length}")

        # Process the last three bytes (i.e. CRC and Stop):
        if full_packet[-1] != slip.stop:
            raise ValueError(f"packet {repr(full_packet)} ends with {hex(full_packet[-1])},"
                             f"not the desired {hex(slip.stop)}")
        crc_low: int = full_packet[-2]
        crc_high: int = full_packet[-3]
        if crc_high >= 128 or crc_low >= 128:
            raise ValueError(f"packet {repr(full_packet)} has actual length bytes of "
                             f"{hex(crc_high)} and {hex(crc_low)} "
                             "neither of which should have bit 0x80 set")
        crc: int = (crc_high << 8) | crc_low

        # Verify the CRC:
        actual_crc: int = crc_compute(full_packet[3:-3]) & 0x7f7f
        if actual_crc != crc:
            raise ValueError(f"packet {repr(full_packet)} has an actual CRC of {hex(actual_crc)}, "
                             f"but the desired CRC is {hex(crc)}")

        # Unpack *full_packet* (which is still escaped) into *packet_id* (1 byte),
        # *partial_packet* (variable length) and *control* (3 bytes):
        unescaped_packet: bytes = Packet.unescape(full_packet)
        packet_id: int = unescaped_packet[3]  # Skip over start and length.
        control: bytes = unescaped_packet[-6:-3]  # 3 bytes before CRC and stop.
        partial_packet: bytes = unescaped_packet[3:-6]  # The remaining bytes between.

        # Unpack the values from *control*:
        from_sequence: int
        to_sequence: int
        to_mask: int
        from_sequence, to_sequence, to_mask = control

        decoded_packet: DecodedPacket = DecodedPacket(from_sequence, packet_id,
                                                      partial_packet, to_sequence, to_mask)
        # print(f"<=Packet.full_decode({repr(full_packet)})=>({decoded_packet})")
        return decoded_packet

    # Packet.escape():
    @classmethod
    def escape(self, unescaped_packet: bytes) -> bytes:
        """Convert an unescaped packet into an escaped one."""
        # Grap some values from *slip*:
        slip: SLIP = self.slip
        start: int = slip.start
        escape: int = slip.escape
        stop: int = slip.stop
        twiddle: int = slip.stop
        # We assume that *start*, *escape*, and *stop* both together and ordered:

        escaped_packet: bytearray = bytearray()
        for byte in unescaped_packet:
            if start <= byte <= stop:
                escaped_packet.append(escape)
                escaped_packet.append(byte ^ twiddle)
            else:
                escaped_packet.append(byte)
        return bytes(escaped_packet)

    # Packet.full_encode():
    @classmethod
    def full_encode(cls,
                    partial_packet: bytes, control_bytes: Tuple[int, int, int]) -> bytes:
        """Convert a partial Packet into a full Packet."""
        # print("=>Packet.full_encode("
        #       f"{repr(partial_packet_bytes)}, {control_bytes})")

        # Create *crc_bytes* which contains all of the bytes to be CRC'ed.
        # This the *partial_packet* (already_escaped) and the *control_bytes*
        # (which are not escaped yet:

        if len(control_bytes) != 3:
            raise RuntimeError("Packet.full_encode: "
                               f"Length of control bytes ({len(control_bytes)}) is not 3")
        byte: int
        for byte in control_bytes:
            if not (0 <= byte <= 255):
                raise RuntimeError("Packet.full_encode: "
                                   f"byte {hex(byte)} from {control_bytes} is not 8-bits")
        crc_bytes: bytearray = bytearray(Packet.escape(partial_packet + bytes(control_bytes)))

        # Compute the *crc* and associated *crc_high* and *crc_low* bytes over *crc_bytes*:
        global crc_compute
        crc: int = crc_compute(bytes(crc_bytes)) & 0x7f7f
        crc_high: int = (crc >> 8) & 0x7f
        crc_low: int = crc & 0x7f

        # Compute *total_length* and associated the *high_length* and *low_length* bytes:
        front_length: int = 1 + 2  # Start(1) + Length(2)
        end_length: int = 2 + 1  # CRC(2) + End(1)
        total_length: int = front_length + len(crc_bytes) + end_length
        assert total_length <= 0x3fff, f"Packet Length is too long {hex(total_length)} > 0x3fff"
        high_length: int = total_length >> 7
        low_length: int = total_length & 0x7f

        # Construct the *full_packet*:
        slip: SLIP = cls.slip
        full_packet: bytearray = bytearray((slip.start, high_length, low_length))
        assert len(full_packet) == front_length, f"Packet header is {len(full_packet)} long"
        full_packet.extend(crc_bytes)
        assert len(full_packet) == front_length + len(crc_bytes), "broke here"
        full_packet.extend((crc_high, crc_low, slip.stop))
        full_packet_bytes: bytes = bytes(full_packet)
        assert len(full_packet) == total_length, (
            f"Actual packet length ({len(full_packet)}) is not desired length ({total_length})")
        # print("<=Packet.full_encode("
        #       f"{repr(partial_packet_bytes)}, {control_bytes})=>{repr(full_packet_bytes)}")
        return(full_packet_bytes)

    # Packet.packet_id_get():
    def packet_id_get(self) -> int:
        """Return the packet id."""
        return self.packet_id

    # Packet.type_get():
    def type_get(self) -> Any:
        """Return the packet type."""
        return self.packet_type

    # Packet.partial_decode():
    def partial_decode(self, packet_bytes: bytes, handle: "Handle", tag: str) -> Tuple[int, Any]:
        """Decode partial packet containing a packet id, struct, and strings."""
        print(f"=>Packet.decode({repr(packet_bytes)}, {handle}, '{tag}')")

        # Grab some information from *handle*:
        struct_size: int = self.struct_size_get(tag)
        struct_format: str = self.struct_format_get(tag)
        string_names: Tuple[str, ...] = self.string_names
        struct_names: Tuple[str, ...] = self.struct_names
        packet_type: Any = self.packet_type
        packet_content: Any = packet_type()

        # Extract the *packet_id*:
        packet_size: int = len(packet_bytes)
        index: int = 0
        next_index: int = index + 1
        if next_index > packet_size:
            raise RuntimeError("Packet {repr(packet_bytes)}[{index}]: No packet id.")
        packet_id: int = packet_bytes[index]
        index = next_index

        # Grab the *struct_bytes*:
        next_index = index + struct_size
        if next_index >= packet_size:
            raise RuntimeError(f"Packet {repr(bytes)}[{index}]: "
                               f"{struct_size} bytes need for struct are not present")
        struct_bytes: bytes = packet_bytes[index:index + struct_size]

        # Unpack *struct_bytes* into *struct_values* and stuff into *packet_content*:
        try:
            struct_values: Tuple[int, ...] = tuple(struct.unpack(struct_format, struct_bytes))
        except struct.error as error:
            raise RuntimeError(f"Packet {repr(bytes)}[{index}]: "
                               f"Struct unpack failed with '{error}'")
        if len(struct_values) != len(struct_names):
            raise RuntimeError(f"Packet {repr(bytes)}[{index}]: "
                               f"Unpacked struct has {len(struct_values)} "
                               f"rather than {len(struct_names)} values")
        struct_index: int
        struct_name: str
        for struct_index, struct_name in enumerate(struct_names):
            setattr(packet_content, struct_name, struct_values[struct_index])
        index = next_index

        # Sweep through the *string_names* and stuff the associated strings into *packet_type*:
        string_name: str
        for string_name in string_names:
            # Grab the *string_length*:
            next_index = index + 1
            if next_index > packet_size:
                raise RuntimeError(f"Packet {repr(packet_bytes)}[{index}]: "
                                   "Missing string length for string '{string_name}'")
            string_length: int = packet_bytes[index]
            index = next_index

            # Extract the *string* from the *packet_bytes*:
            next_index = index + string_length
            if next_index > packet_size:
                raise RuntimeError(f"Packet {repr(packet_bytes)}[{index}]: "
                                   f"{string_length} bytes for string '{string_name}'")

            # Convert the packet_bytes into a string using the latin-1 encoding.
            # This encoding is 1-to-1 with byte values for the first 256 bytes.
            string: str = str(packet_bytes[index:next_index], encoding="latin-1")

            # Stuff the *string* into the *packet_content*:
            assert string_name in string_names, (
                f"'string_name' is not present in {packet_type}")
            setattr(packet_content, string_name, string)
            index = next_index

        print(f"=>Packet.decode({repr(packet_bytes)}, {handle}, '{tag}')=>"
              f"({packet_id}, {packet_content}")
        return (packet_id, packet_content)

    # Packet.partial_packet_id_get():
    @classmethod
    def partial_packet_id_get(cls, partial_packet: bytes) -> int:
        """Return the packet id from an escaped partial packet."""
        # For partial packet id is the first byte (that may be escaped):
        if len(partial_packet) < 2:
            raise ValueError(f"Partial packet {repr(partial_packet)} is too short.")
        slip: SLIP = cls.slip
        packet_id: int = partial_packet[0]
        if partial_packet[0] == slip.escape:
            packet_id = partial_packet[1] ^ slip.twiddle
        return packet_id

    # Packet.partial_encode():
    def partial_encode(self, packet_content: Any, handle: "Handle", tag: str) -> bytes:
        """Encode packet content into partial bytes packet that is not escaped yet."""
        # print(f"=>Packet.partial_encode({packet_content}, *, '{tag}')")

        # Grab some values from *handle*:
        packet_type: Any = self.packet_type
        packet_id: int = self.packet_id
        string_names: Tuple[str, ...] = self.string_names_get(tag)
        struct_format: str = self.struct_format
        struct_names: Tuple[str, ...] = self.struct_names_get(tag)
        if not isinstance(packet_content, packet_type):
            raise RuntimeError(f"Packet.partial: packet_content={packet_content}, "
                               f"should be {packet_type} instead")

        # Start with the *packet_id*:
        slip: SLIP = self.slip
        assert 0 <= packet_id < slip.minimum, f"Packet id ({packet_id} is too big.)"
        packet_bytes: bytearray = bytearray()
        packet_bytes.append(packet_id)

        # Extract the *struct_values* from *packet_content*, convert them into *struct_bytes*,
        # and append to *packet_bytes*:
        struct_name: str
        struct_values: List[int] = [getattr(packet_content, struct_name)
                                    for struct_name in struct_names]
        struct_bytes: bytes = struct.pack(struct_format, *struct_values)
        packet_bytes.extend(struct_bytes)

        # Next output each string:
        string_name: str
        for string_name in string_names:
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
            packet_bytes.append(string_size)
            string_chr: str
            for string_chr in string_content:
                string_byte: int = ord(string_chr)
                if string_byte > 255:
                    print(f"Converting character {hex(string_byte)} to 0xff")
                    string_byte = 255
                packet_bytes.append(string_byte)

        final_packet_bytes: bytes = bytes(packet_bytes)
        # print(f"<=Packet.partial_encode({packet_content}, *, '{tag}') => "
        #       f"{repr(final_packet_bytes)}")
        return final_packet_bytes

    # Packet.ros_field_types_get():
    @classmethod
    def ros_field_types_get(cls) -> Dict[str, ROSFieldType]:
        """Return the ROS Field Types table."""
        return cls.ros_field_types

    # Packet.string_names_get():
    def string_names_get(self, tag: str) -> Tuple[str, ...]:
        """Return the string field names."""
        return self.string_names

    # Packet.struct_format_get():
    def struct_format_get(self, tag: str) -> str:
        """Return the format string."""
        return self.struct_format

    # Packet.struct_names_get():
    def struct_names_get(self, tag: str) -> Tuple[str, ...]:
        """Return the struct field names."""
        return self.struct_names

    # Packet.struct_size_get():
    def struct_size_get(self, tag: str) -> int:
        """Return the number of bytes needed to contain the struct."""
        return self.struct_size

    # Packet.unescape():
    @classmethod
    def unescape(cls, escaped_bytes: bytes) -> bytes:
        """Take a packet with escapes and remove them."""
        # print(f"=>Packet.unescape({repr(escaped_bytes)})")
        slip: SLIP = cls.slip
        escape: int = slip.escape
        twiddle: int = slip.twiddle
        unescaped_bytes: bytearray = bytearray()
        escaped_byte: int
        escape_found: bool = False
        for escaped_byte in escaped_bytes:
            if escape_found:
                unescaped_bytes.append(escaped_byte ^ twiddle)
                escape_found = False
            elif escaped_byte == escape:
                escape_found = True
            else:
                unescaped_bytes.append(escaped_byte)
        # print(f"<=Packet.unescape({repr(escaped_bytes)})=>{repr(unescaped_bytes)}")
        return bytes(unescaped_bytes)


# Handle:
class Handle(object):
    """Base class for all dispatchable messages."""

    # Handle.__init__():
    def __init__(self) -> None:
        """Init the base Dispatch."""
        pass

    # Handle.forward():
    def forward(self, content: Any, tag: str) -> None:
        """Forward a Message content."""
        raise NotImplementedError(f"Handle.forward({self}): not implemented.")

    # Handle.packet_get():
    def packet_get(self, tag: str) -> Packet:
        """Return the packet id."""
        raise NotImplementedError(f"Handle.packet_get({self}): not implemented")

    # Handle.priority_get():
    def priority_get(self, tag: str) -> int:
        """Return the Handle priority."""
        raise NotImplementedError(f"Handle.packet_get({self}): not implemented")

    # Handle.priority_set():
    def priority_set(self, tag: str, priority: int) -> None:
        """Set the priority."""
        raise NotImplementedError(f"Handle.priority_set({self}): not implemented")


# PingHandle:
class PingHandle(Handle):
    """Class for ping messages for serial line transmission error recovery."""

    # PingHandle.__init__():
    def __init__(self, packet_id: int, priority: int) -> None:
        """Initialzie PingHandle."""
        super().__init__()
        self.packet: Packet = Packet(PingHandleType, packet_id)
        self.priority: int = 0

    # PingHandle.packet_get():
    def packet_get(self, tag: str) -> Packet:
        """Return priority for a PingHandle's."""
        return self.packet

    # PingHandle.priority_get():
    def priority_get(self, tag: str) -> int:
        """Return the PingHandle priority."""
        return self.priority

    # PingHandle.priority_set():
    def priority_set(self, tag: str, priority: int) -> None:
        """Return the PingHandle priority."""
        self.priority = priority


# PingHandleType:
class PingHandleType(object):
    """..."""

    # PingHandleType.__init__():
    def __init__(self):
        """Init PingFoo."""
        pass

    # PingHandleType.get_fields_and field_types():
    def get_fields_and_field_types(self) -> Dict[str, Any]:
        """Return an empty dictionary."""
        return {}


# ROSHandle:
class ROSHandle(Handle):
    """Base class for publish/subscribe topics and client/server services."""

    # ROSHandle.__init__():
    def __init__(self, ros_key: "ROSKey", agent: Agent) -> None:
        """Initialize Handle base class."""
        super().__init__()
        self.ros_key = ros_key
        self.agent = agent

    # ROSHandle.agent_get():
    def agent_get(self) -> Agent:
        """Return the Handle Agent."""
        agent: Optional[Agent] = self.agent
        assert isinstance(agent, Agent), "No agent is available"
        return agent

    # ROSHandle.match():
    def match(self, ros_key: "ROSKey") -> bool:
        """Return True if TopicHandle's match."""
        return self.ros_key == ros_key

    # ROSHandle.packet_type_get():
    def packet_type_get(self, tag: str) -> Any:
        """Return the packet type for an a given packet tag."""
        raise NotImplementedError

    # ROSHandle.type_import():
    def type_import(self, ros_key: "ROSKey") -> Any:
        """Import a topic."""
        # Read the *import_path* into *module*:
        module: Any = importlib.import_module(ros_key.import_path)
        # print(f"import_path='{import_path}' type(module)={type(module)}")

        # Get the Python *actual_type* object:
        actual_type: Any = getattr(module, ros_key.type_name)
        # print(f"type_name='{type_name}' type(actual_type)={type(actual_type)}")

        return actual_type


# ROSKey:
@dataclass(order=True, frozen=True)
class ROSKey:
    """Class that represents a ROS topic/service name."""

    import_path: str
    type_name: str
    ros_path: str


# IOChannel:
class IOChannel:
    """A handle for dealing with serial I/O."""

    # IOChannel.__init__():
    def __init__(self, argument: str, colon_split: Tuple[str, ...]) -> None:
        """Init an IOChannel."""
        # Start with all value set to *False*:
        print(f"=>IOChannel.__init__(*, '{argument},', '{colon_split}')")
        self.create: bool = False
        self.device: bool = False
        self.log: bool = False
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
            print("IOChannel.__init__(): Create Pipe")
            if os.path.exists(file_name):
                if not stat.S_ISFIFO(os.stat(file_name).st_mode):
                    print(f"IOChannel.__init__(): '{file_name}' exists and is not a named pipe")
                    try:
                        os.remove(file_name)
                        if os.path.exists(file_name):
                            raise RuntimeError(f"'{file_name}' not succesfully deleted")
                    except PermissionError:
                        raise RuntimeError(f"'{file_name}' not removed due to permission error")
            else:
                try:
                    os.mkfifo(file_name)
                except FileExistsError:
                    raise RuntimeError(f"'{file_name}' already exists and is not a named pipe")
                except PermissionError:
                    raise RuntimeError(f"Unable to create '{file_name}' named pipe")

        # Save the *file_name*:
        self.file_name: str = file_name
        print(f"<=IOChannel.__init__(*, '{argument},', '{colon_split}')")

    # IOChannel.__str__():
    def __str__(self):
        """Return a text representation of an IOChannel."""
        return (f"IOCHannel(create={self.create}, device={self.device}, read={self.read}, "
                f"pipe={self.pipe} write={self.write}, file_name='{self.file_name}')")

    # IOChannel.open_for_reading():
    def open_for_reading(self) -> IO[bytes]:
        """Open IOChannel for reading in raw (unbuffered) mode."""
        # Do a sanity check:
        file_name: str = self.file_name
        if not self.read:
            raise RuntimeError("IOChannel('{file_name}') is not enabled for reading")

        # Open and return the raw unbuffered *read_file*:
        read_file: IO[bytes]
        try:
            # If *file_name* is a pipe, this open will block until another process
            # opens the other end for writing:
            read_file = open(file_name, "rb", buffering=0)
        except FileNotFoundError:
            raise RuntimeError("File '{file_name}' can not be opened.")
        return read_file

    # IOChannel.open_for_writing():
    def open_for_writing(self, force: bool = False) -> IO[bytes]:
        """Open IOChannel for writing in raw (unbuffered) mode."""
        # Do a sanity check:
        file_name: str = self.file_name
        if not self.write and not force:
            raise RuntimeError("IOChannel('{file_name}') is not enabled for writing")

        # Open and return the raw unbuffered *write_file*:
        write_file: IO[bytes]
        try:
            write_file = open(file_name, "wb", buffering=0)
        except FileNotFoundError:
            raise RuntimeError("File '{file_name}' can not be opened.")
        return write_file


# TimerHandle:
class TimerHandle(Handle):
    """A handle for dealing with ROS timers."""

    # TimerHandle.__init__():
    def __init__(self, agent_node_name: str, priority: int,
                 timer_name: str, agent: Optional[Agent] = None) -> None:
        """Init a TimeHandle."""
        super().__init__()
        self.agent: Optional[Agent] = agent
        self.agent_node_name: str = agent_node_name
        self.priority: int = priority
        self.key: Tuple[str, str] = (agent_node_name, timer_name)
        self.timer_name: str = timer_name

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
        agent_node_name: str = self.agent_node_name
        print(f"=>TimerHandle('{agent_node_name}').callback()")
        agent: Agent = self.agent_get()

        # Publish a message to the topic named 'topic':
        topic_agent_node: AgentNode
        topic_topic_handle: TopicHandle
        topic_key: Tuple[str, ROSKey] = (
            "serial_agent", ROSKey("std_msgs.msg", "String", "topic"))
        topic_agent_node, topic_topic_handle = agent.publisher_lookup(topic_key)
        topic_publisher_count: int = topic_topic_handle.publisher_count_get()
        topic_message: Any = topic_topic_handle.message_create("pub")
        topic_message.data = f"topic message {topic_publisher_count}"
        topic_topic_handle.publish(topic_message)
        print("Published message to Topic('topic')")

        # Publish a message to the topic named 'echo':
        # echo_agent_node: AgentNode
        # echo_topic_handle: TopicHandle
        # echo_agent_node, echo_topic_handle = agent.publisher_lookup(("serial_agent", "echo"))
        # echo_publisher_count: int = echo_topic_handle.publisher_count_get()
        # echo_message: Any = echo_topic_handle.message_create()
        # echo_message.data = f"echo message {2 * echo_publisher_count}"
        # echo_topic_handle.publish(echo_message)
        # print("Published message to Topic('echo')")

        print(f"<=TimerHandle('{agent_node_name}').callback()")

    # TimerHandle.key_get():
    def key_get(self) -> Tuple[str, str]:
        """Return the TimerHandle key."""
        return self.key

    # TimerHandle.match():
    def match(self, agent_node_name: str, timer_name: str) -> bool:
        """Return True if TimerHandle's match."""
        return self.agent_node_name == agent_node_name and self.timer_name == timer_name

    # TimerHandle.priority_get():
    def priority_get(self, tag: str) -> int:
        """Return the priority for a TimerHandle."""
        return self.priority


# TopicHandle:
class TopicHandle(ROSHandle):
    """Tracking class for publish/subscribe topic."""

    # TopicHandle.__init__():
    def __init__(self, ros_key: ROSKey, agent: Agent) -> None:
        """Initialize a TopicHandle."""
        # Initialize base class:
        super().__init__(ros_key, agent)

        # Load everything up:
        publisher_packet_id: int = agent.packet_id_get(self, "pub")
        subscription_packet_id: int = agent.packet_id_get(self, "sub")
        message_type: Any = self.type_import(ros_key)
        self.message_type: Any = message_type
        self.publisher: Optional[Publisher] = None
        self.publisher_agent_node: Optional[AgentNode] = None
        self.publisher_count: int = 0
        self.publisher_packet: Packet = Packet(message_type, publisher_packet_id)
        self.publisher_priority: int = 0
        self.subscription: Optional[Subscription] = None
        self.subscription_agent_node: Optional[AgentNode] = None
        self.subscription_count: int = 0
        self.subscription_packet: Packet = Packet(message_type, subscription_packet_id)
        self.subscription_priority: int = 0

    # TopicHandle.__repr__():
    def __str__(self) -> str:
        """Return a string representation."""
        return (f"TopicHandle('{self.ros_key}')")

    # TopicHandle.key_get():
    def key_get(self) -> ROSKey:
        """Return the TopicHandle key."""
        return self.ros_key

    # TopicHandle.message_create():
    def message_create(self, tag: str) -> Any:
        """Create a new message object instance."""
        packet: Packet = self.packet_get(tag)
        message_type: Any = packet.type_get()
        message_instance: Any = message_type()
        return message_instance

    # TopicHandle.packet_get():
    def packet_get(self, tag: str) -> Packet:
        """Return the associated packet."""
        if tag not in ("pub", "sub"):
            raise RuntimeError(f"TopicHandle.packet_get(): '{tag}' is not either 'pub' or 'sub'")
        packet: Packet = self.publisher_packet if tag == "pub" else self.subscription_packet
        return packet

    # TopicHandle.priority_get():
    def priority_get(self, tag: str) -> int:
        """Return the priority."""
        if tag not in ("pub", "sub"):
            raise RuntimeError(f"TopicHandle.packet_get(): '{tag}' is not either 'pub' or 'sub'")
        priority: int = self.publisher_priority if tag == "pub" else self.subscription_priority
        return priority

    # TopicHandle.priority_set():
    def priority_set(self, tag: str, priority: int) -> None:
        """Set the priority."""
        if tag not in ("pub", "sub"):
            raise RuntimeError(f"TopicHandle.packet_get(): '{tag}' is not either 'pub' or 'sub'")
        if tag == "pub":
            self.publisher_priority = priority
        else:
            self.subscription_priority = priority

    # TopicHandle.publish():
    def publish(self, message: Any) -> Publisher:
        """Publish a message to the topic."""
        # print(f"=>TopicHandle.publish('{message}')")
        publisher: Optional[Publisher] = self.publisher
        assert publisher, "Publisher not set yet."
        message_type: Any = self.message_type
        assert isinstance(message, message_type), f"Message is not of type {message_type}"
        # print(f"Topic('{self.ros_path}): Published: {message}'")
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
        assert not self.publisher_agent_node, f"Duplicate publisher of {self.ros_key}"
        self.publisher_agent_node = publisher_agent_node

        # Create and return the new *publisher*:
        publisher: Publisher = publisher_agent_node.create_publisher(
            self.message_type,
            self.ros_key.ros_path,
            10)

        assert not self.publisher, f"Duplicate publisher for {self.ros_key}"
        self.publisher = publisher

        # print(f"<=TopicHandle.publisher_create()=>{publisher}")
        return publisher

    # TopicHandle.subscription_create():
    def subscription_create(self, subscription_agent_node: AgentNode) -> Subscription:
        """Create a subcription for a ROS topic."""
        # print(f"=>TopicHandle.subscription_create()")
        assert not self.subscription_agent_node, (
            "TopicHandle.subscription_create(): subscription_agent_node already set")
        self.subscription_agent_node = subscription_agent_node

        # Create and return the new *subscription*:
        subscription: Subscription = subscription_agent_node.create_subscription(
            self.message_type,
            self.ros_key.ros_path,
            self.subscription_callback,
            10)

        assert not self.subscription, f"duplicate subscription for {self.ros_key}"
        self.subscription = subscription

        # print(f"<=TopicHandle.subscription_create()")
        return subscription

    # TopicHandle.subscription_callback():
    def subscription_callback(self, message: Any) -> None:
        """Process a subscription callback."""
        ros_path: str = self.ros_key.ros_path
        print(f"=>TopicHandle.subscription_callback('{ros_path}')")
        print(f"Topic('{ros_path}'): Got Message: {message}")

        agent: Optional[Agent] = self.agent
        assert isinstance(agent, Agent), "Agent is not set!"

        # Encode *message* into *packet_bytes*:
        assert isinstance(message, self.message_type), (
            f"message {message} does name match {self.message_type}")
        subscription_packet: Packet = self.subscription_packet
        partial_packet_bytes: bytes = subscription_packet.partial_encode(message, self, "sub")
        pending_queue: Queue[bytes] = self.agent.pending_queue
        print("Sending subscription to writer...")
        pending_queue.put(partial_packet_bytes)

        """
        # print(f"partial_packet={repr(partial_packet_bytes)}")
        control: Tuple[int, int, int] = (1, 2, 3)
        full_packet_bytes: bytes = message_packet.full_encode(partial_packet_bytes, control)
        decoded_packet: DecodedPacket = message_packet.full_decode(full_packet_bytes)
        assert self.subscription_packet_id == decoded_packet.from_packet_id, (
            f"Desired packet id ({self.subscription_packet_id}) "
            f"is not actual packet id ({decoded_packet.from_packet_id})")
        extracted_control: Tuple[int, int, int] = (decoded_packet.from_sequence,
                                                   decoded_packet.to_sequence,
                                                   decoded_packet.to_missing)
        assert control == extracted_control, f"Mismatch control {control} != {extracted_control}"
        assert partial_packet_bytes == decoded_packet.from_partial_packet, (
            "partial packet decode problem: "
            f"{repr(partial_packet_bytes)} != {repr(decoded_packet.from_partial_packet)}")
        # print(f"full_packet={repr(full_packet_bytes)}")

        # Decode *packet_types* into *unescaped_packet_bytes*:
        unescaped_bytes: bytes = message_packet.unescape(partial_packet_bytes)
        # print(f"unescaped_bytes={repr(unescaped_bytes)}")

        # Decode *unescaped_packet_bytes* into *decode_content:
        decode_packet_id: int
        decode_content: Any
        decode_packet_id, decode_content = (
            message_packet.partial_decode(unescaped_bytes, self, "tag"))
        assert decode_packet_id == self.subscription_packet_id, (
            f"decode_packet_id ({decode_packet_id}) "
            f"does not match subscription_packet_id ({self.subscription_packet_id})")
        assert message.data == decode_content.data, (
            f"message.data ({message.data}) does not match "
            f"decode_content.data ({decode_content.data})")
        # print("Yahoo! They match!")

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
        """
        print(f"<=TopicHandle.subscription_callback('{ros_path}')")


# ServiceHandle:
class ServiceHandle(ROSHandle):
    """Tracking class for clientserver service."""

    # ServiceHandle.__init__():
    def __init__(self, ros_key: ROSKey, agent: Agent) -> None:
        """Initialize Handle class."""
        # Initialize base class:
        super().__init__(ros_key, agent)

        # Import the relevant information about the topic:
        service_type: Any = self.type_import(ros_key)
        request_type: Any = service_type.Request
        response_type: Any = service_type.Response

        request_packet_id: int = agent.packet_id_get(self, "req")
        response_packet_id: int = agent.packet_id_get(self, "resp")

        # Load values into the TopicHandle:
        self.client: Optional[Client] = None
        self.client_agent_node: Optional[AgentNode] = None
        self.request_count: int = 0
        self.request_type: Any = request_type
        self.request_packet: Packet = Packet(request_type, request_packet_id)
        self.request_packet_id: int = request_packet_id
        self.request_priority: int = 0
        self.response_count: int = 0
        self.response_packet: Packet = Packet(response_type, response_packet_id)
        self.response_packet_id: int = response_packet_id
        self.response_type: Any = response_type
        self.response_priority: int = 0
        self.server: Optional[Server] = None
        self.server_agent_node: Optional[AgentNode] = None
        self.service_type: Any = service_type

    # ServiceHandle.__repr__():
    def __str__(self) -> str:
        """Return a string representation."""
        return (f"TopicHandle('{self.ros_key}')")

    # ServiceHandle.client_create():
    def client_create(self, client_agent_node: AgentNode) -> Client:
        """Create a client for a ROS service."""
        # print("=>ServiceHandle.client_create()")

        if self.client_agent_node:
            raise RuntimeError(f"Duplicate client {self.ros_key}")
        self.client_agent_node = client_agent_node

        # Create and return the new *subscription*:
        if self.client:
            raise RuntimeError(f"Duplicate client {self.ros_key}")
        self.client = client_agent_node.create_client(self.service_type, self.ros_key.ros_path)

        # print(f"<=ServiceHandle.client_create()=>{self.client}")
        return self.client

    # ServiceHandle.key_get():
    def key_get(self) -> ROSKey:
        """Return the ServiceHandle key."""
        return self.ros_key

    # ServiceHandle.packet_get():
    def packet_get(self, tag: str) -> Packet:
        """Return the packet type for an a given packet id."""
        if tag not in ("req", "resp"):
            raise RuntimeError(f"ServiceHandle.packet_get(): '{tag}' is not 'req' or 'resp'")
        packet: Packet = self.request_packet if tag == "req" else self.response_packet
        return packet

    # ServiceHandle.priority_set():
    def priority_set(self, tag: str, priority: int) -> None:
        """Set the priority."""
        if tag not in ("req", "resp"):
            raise RuntimeError(f"TopicHandle.packet_get(): '{tag}' is not either 'req' or 'resp'")
        if tag == "req":
            self.client_priority = priority
        else:
            self.server_priority = priority

    # ServiceHandle.priority_get():
    def priority_get(self, tag: str) -> int:
        """Return the priority."""
        if tag not in ("req", "resp"):
            raise RuntimeError(f"ServiceHandle.priority_get(): '{tag}' is not 'req' or 'resp'")
        priority: int = self.request_priority if tag == "req" else self.response_priority
        return priority

    # ServiceHandle.server_create():
    def server_create(self, server_agent_node: AgentNode) -> Client:
        """Create a server for a ROS service."""
        # print("=>ServiceHandle.server_create()")

        if self.server_agent_node:
            raise RuntimeError(f"Duplicate server {self.ros_key}")
        self.server_agent_node = server_agent_node

        # Create and return the new *subscription*:
        server: Server = server_agent_node.create_service(
            self.service_type, self.ros_key.ros_path, self.request_callback)
        if self.server:
            raise RuntimeError(f"Duplicate server {self.ros_key}")

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
        if self.ros_key.ros_path == "add_two_ints":
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
        # print(f"Service('{self.ros_path}'): Sent request: {request}")
        # print(f"<=ServiceHandle.request_send({request}):")

    # ServiceHandle.response_callback():
    def response_callback(self, response_future: Future) -> Any:
        """Deal with an asynchronous response to a server request."""
        # print("=>ServiceHandle.response_callback()")
        try:
            response: Any = response_future.result()
            self.response_count += 1
        except Exception as error:
            assert False, f"response callback {error}"
        else:
            assert isinstance(response, self.response_type)
        # print(f"Service('{self.ros_path}'): Got response: {response}')")
        # print(f"<=ServiceHandle.response_callback()=>{response}")
        return response

    # ServiceHandle.response_count_get()
    def response_count_get(self) -> int:
        """Return the number of responses processed."""
        return self.response_count

    # ServiceHandle.response_create():
    def response_create(self) -> Any:
        """Return a ne response object."""
        return self.response_type()


# SerialLine:
class SerialLine(object):
    """Class to that messages I/O the microcontroller serial line.

    This class manages the serial line communication.  It consists of two threads:
    * reader:
      This simple thread blocks waiting for serial data from the microcontroller, which it
      then reads and forwards to the writer thread.
    * writer: This the workhouse thread reads requests from a pending queue and performs actions
      that result in serial data being sent to the microcontroller.  This thread does packet
      prioritization, missing packet recovery, etc.
    """

    # SerialLine.__init__():
    def __init__(self, read_io_channel: IOChannel, write_io_channel: IOChannel,
                 packet_id_to_handle: Tuple[Tuple[Handle, str], ...], pending_queue: "Queue[bytes]",
                 prioritized_packets: List[List[bytes]], ping_packet: bytes) -> None:
        """Init the SerialLine."""
        # Load values into *self*:
        self.acknowledged_to_sequence: int = 0
        self.byte_time: float = 10 * (1.0 / 115200.)  # 8N1 (i.e. 10-bits/byte)
        self.from_packets: List[bytes] = []
        self.last_control: Tuple[int, int] = (0, 0)
        self.packet_id_to_handle: Tuple[Tuple[Handle, str], ...] = packet_id_to_handle
        self.pending_queue: Queue[bytes] = pending_queue
        self.ping_packet: bytes = ping_packet
        self.prioritized_packets: List[List[bytes]] = prioritized_packets
        self.read_file: Optional[IO[bytes]] = None
        self.read_io_channel: IOChannel = read_io_channel
        self.slip: SLIP = Agent.slip_get()
        self.to_packet_cache: Dict[int, bytes] = {}
        self.to_sequence: int = 0
        self.to_packets: List[bytes] = []
        self.write_file: Optional[IO[bytes]] = None
        self.write_io_channel: IOChannel = write_io_channel

    # SerialLine.reader():
    def reader(self) -> None:
        """Forward full packets from microcontroller serial line input to a pending queue."""
        print("=>SerialLine.reader()")
        # Grab some values from *self*:
        slip: SLIP = self.slip
        pending_queue: Queue[bytes] = self.pending_queue
        read_io_channel: IOChannel = self.read_io_channel

        # Open *in_file* in unbuffered raw mode.  If *read_io_channel* points to a pipe (FIFO),
        # this will block until some other thread/process opens the pipe for writing:
        read_file: IO[bytes]
        with read_io_channel.open_for_reading() as read_file:
            # Compute the *block_flags* and *non_block_flags* needed to switching between
            # blocking/non-blocking IO:
            self.read_file = read_file
            print("SerialLine.reader(): read_file set")
            fd: int = read_file.fileno()
            block_flags: int = fcntl.fcntl(fd, fcntl.F_GETFL)
            non_block_flags: int = block_flags | os.O_NONBLOCK
            print("SerialLine.reader(): "
                  f"fd={fd} block_flags={hex(block_flags)} non_block_flags={hex(non_block_flags)}")

            # Read bytes from *read_file* until it is closed.
            start: int = slip.start
            stop: int = slip.stop
            input_buffer: bytearray = bytearray()
            in_packet: bool = False
            packet_buffer: bytearray = bytearray()
            done: bool = False
            while not done:
                # Read some *data* into *buffer*, first byte blocks, renaming bytes non-blocking:
                input_buffer.clear()
                amount: int
                for amount, flags in ((1, block_flags), (10000, non_block_flags)):
                    # print(f"SerialLine.reader(): Setting flags to {hex(flags)}")
                    if fcntl.fcntl(fd, fcntl.F_SETFL, flags) != 0:
                        raise OSError()
                    # print(f"SerialLine.reader(): Reading amount {amount}")
                    data: bytes = read_file.read(amount)
                    if len(data) == 0:
                        # On end-of-file, send a 1 byte stop message to *pending_queue*:
                        print("SerialLine.reader(): End-of-file")
                        pending_queue.put(bytes([slip.stop]))
                        done = True
                        break
                    input_buffer.extend(data)

                # Process the bytes that have arrived:
                byte: int
                for byte in input_buffer:
                    if byte == start:
                        in_packet = True
                        del packet_buffer[:]
                        packet_buffer.append(byte)
                    elif in_packet:
                        packet_buffer.append(byte)
                        if byte == stop:
                            in_packet = False
                            pending_queue.put(bytes(packet_buffer))

        # Shut down occurs here:
        print("<=SerialLine.reader()")

    # SerialLine.writer():
    def writer(self) -> None:
        """Deal with messages both sent to and received from the microcontroller."""
        # This is the main loop a dedicated thread that unloads the *pending_queue*,
        # which contains both incoming and outgoing packets.
        #
        # Incoming packets are from the reader thread and the are fully framed packets that
        # contain, start, length, packet id, structs, string, transmission control, CRC,
        # and stop information.
        #
        # The outgoing packets are just partial packets that just contain a packet id, struct,
        # and strings.  This packets are prioritized for transmission order.  The transmission
        # control information from the incoming packets is used to trigger re-transmission of
        # previously sent messages.
        #
        # The outgoing packet is sent and an estimated transmission time is computed.
        # Only after the estimated transmission time has elapsed, does another packet get
        # selected and sent.  This means that only one message is being transmitted at a
        # time and allows higher priority messages to get sent before lower priority messages.

        # Open *write_file*:
        print("=>SerialLine.writer()")
        write_file: IO[bytes]
        with self.write_io_channel.open_for_writing() as write_file:
            # print("=>SerialLine.writer(): write_file is open")
            self.write_file = write_file  # Needed for keyboard interrupt shut-down.
            done: bool = False
            send_done_time: float = 0.0
            ping_timeout: float = 1.0  # Should be configurable!
            while not done:
                # print("SerialLine.writer(): start loop")
                # Drain the pending queue:
                from_packets: List[bytes]
                to_packets: List[bytes]
                now: float = time.time()
                time_out: float = send_done_time - now
                from_packets, to_packets, done = self.pending_queue_drain(time_out)
                if not done:
                    # Forward *from_packets* and prioritize *to_packets*:
                    self.from_packets_forward(from_packets)
                    self.to_packets_prioritize(to_packets)

                    # If not already transmitting a packet, pick one and send it:
                    if now > send_done_time:
                        # Do resend packets 1st, prioritized packets 2nd, pings 3rd:
                        send_packet: Optional[bytes] = self.resend_packet_select()
                        if not send_packet:
                            send_packet = self.prioritized_packet_select()
                        if not send_packet and now > send_done_time + ping_timeout:
                            send_packet = self.ping_packet

                        # Send *send_packet* (if present) and estimate the *send_done_time*:
                        send_done_time = 0.0  # Forces huge drain time_out (i.e. forces a block).
                        if send_packet:
                            self.packet_send(send_packet, write_file)
                            write_file.write(send_packet)
                            send_done_time = now + len(send_packet) * self.byte_time

        # Do any shutdown stuff here:
        self.write_file = None
        print("<=SerialLine.writer()")

    # SerialLine.from_packets_forward():
    def from_packets_forward(self, from_packets: List[bytes]) -> Tuple[DecodedPacket, ...]:
        """Forward packets from the microcontroller towards the destinations."""
        packet_id_to_handle: Tuple[Tuple[Handle, str], ...] = self.packet_id_to_handle
        packet_id_to_handle_size: int = len(packet_id_to_handle)
        decoded_packets: List[DecodedPacket] = []
        from_packet: bytes
        for from_packet in from_packets:
            # Decode *from_packet* look up associated *handle* and *tag* using the *from_packet_id*:
            decoded_packet: DecodedPacket = Packet.full_decode(from_packet)
            decoded_packets.append(decoded_packet)
            from_packet_id: int = decoded_packet.from_packet_id
            if from_packet_id >= packet_id_to_handle_size:
                raise RuntimeError(f"Invalid packet_id: {from_packet_id}")
            handle: Handle
            tag: str
            handle, tag = packet_id_to_handle[from_packet_id]
            from_partial_packet: bytes = decoded_packet.from_partial_packet
            result_packet: Packet = handle.packet_get(tag)
            result: Any
            _, result = result_packet.partial_decode(from_partial_packet, handle, tag)
            handle.forward(result, tag)
        return tuple(decoded_packets)

    # SerialLine.resend_packet_select():
    def resend_packet_select(self) -> Optional[bytes]:
        """Seleect a packet to resend."""
        # For now, there are no resends.
        # assert False, "SerialLine.resend_packet_select()"
        return None

    # SerialLine.prioritized_packet_select():
    def prioritized_packet_select(self) -> Optional[bytes]:
        """Select the highest priority packet to send."""
        prioritized_packets: List[List[bytes]] = self.prioritized_packets
        packet: Optional[bytes] = None
        packets: List[bytes]
        for packets in reversed(prioritized_packets):
            if packets:
                packet = packets.pop(0)
        return packet

    # SerialLine.pending_queue_drain():
    def pending_queue_drain(
            self, time_out: Optional[float] = None) -> Tuple[List[bytes], List[bytes], bool]:
        """Drain the pending queue and transfer into prioritized pending packets."""
        # Unpack some values from *self*:
        # print(f"=>SerialLine.pending_queue_drain(*, {time_out})")
        pending_queue: Queue[bytes] = self.pending_queue
        to_packets: List[bytes] = self.to_packets
        from_packets: List[bytes] = self.from_packets
        slip: SLIP = self.slip
        start: int = slip.start
        stop: int = slip.stop

        # Read in packets from queue until it is empty:
        from_packets.clear()
        to_packets.clear()
        block: bool = True
        done: bool = False
        while True:
            # Get the next *packet*:
            packet: bytes
            if time_out is not None and time_out < 0.0:
                time_out = None
            # print("================> SerialLine.pending_queue_drain(): "
            #       f"block={block} time_out={time_out}")
            try:
                packet = pending_queue.get(block=block, timeout=time_out)
            except queue.Empty:
                # *pending_queue* is currently drained:
                # print("<================ SerialLine.pending_queue_drain(): Queue empty")
                break
            # print(f"<================ SerialLine.pending_queue_drain(): Got {repr(packet)}")
            block = False

            # Do an initial sort of *packet* based on the first byte:
            if not packet or packet[0] == stop:
                # A shut down is requested.
                done = True
            elif packet[0] == start:
                from_packets.append(packet)
            else:
                to_packets.append(packet)
        # print(f"<=SerialLine.pending_queue_drain(*, {time_out}) => *, *, {done}")
        return from_packets, to_packets, done

    # SerialLine.packet_send():
    def packet_send(self, to_packet: bytes, write_file: IO[bytes]) -> None:
        """Encode a packet and send it."""
        to_sequence: int = self.to_sequence + 1
        self.sequence = to_sequence
        encoded_to_packet: bytes = Packet.full_encode(to_packet, (to_sequence & 0xff, 0, 0))
        self.to_packet_cache[to_sequence] = encoded_to_packet
        write_file.write(encoded_to_packet)

    # SerialLine.from_packets_process():
    def from_packets_process(self, received_full_packets: List[bytes]) -> None:
        """Process any packets received from microcontroller."""
        # Drain *control_queue* and return updated *last_control*:
        assert False, "SerialLine.from_packets_process()"

    # SerialLine.shutdown():
    def shutdown(self):
        """Shutdown the serial line."""
        print("=>SerialLine.shutdown()")
        read_file: Optional[IO[bytes]] = self.read_file
        if read_file:
            print("SerialLine.shutdown(): Closing read_file")
            read_file.close()
        else:
            print("SerialLine.shutdown(): Reader stalled in open()")
            read_io_channel: Optional[IOChannel] = self.read_io_channel
            if read_io_channel and read_io_channel.pipe:
                print("SerialLine.shutdown(): Opening other end of pipe")
                write_file: IO[bytes]
                with read_io_channel.open_for_writing(force=True) as write_file:
                    print("SerialLine.shutdown: Opened other end of pipe")
                    write_file.write(bytes())
                print("SerialLine.shutdown(): Closed other end of pipe")

        # An empty packet gracefully shuts down the writer.
        self.pending_queue.put(bytes())
        print("<=SerialLine.shutdown()")

    # SerialLine.from_packets_decode():
    def from_packets_decode(
            self, from_full_packets: List[bytes]) -> Tuple[DecodedPacket, ...]:
        """Return the packets that can be decoded."""
        decoded_packets: List[DecodedPacket] = []
        from_full_packet: bytes
        for from_full_packet in from_full_packets:
            # Try to decode *full_packet*:
            try:
                decoded_packet: DecodedPacket = Packet.full_decode(from_full_packet)
            except ValueError as value_error:
                print(f"Problem with received packet{repr(from_full_packet)}: {value_error}")
            else:
                decoded_packets.append(decoded_packet)
        return tuple(decoded_packets)

    # SerialLine.missing_to_sequences_get():
    def missing_to_sequences_get(
            self, decoded_packets: Tuple[DecodedPacket, ...]) -> Tuple[int, ...]:
        """Return the sequence numbers for messages that have not been acknowledged yet."""
        missing_to_sequences: List[int] = []
        decoded_packet: DecodedPacket
        if decoded_packets:
            # Find the *best_decoded_packet* that has the highest *to_sequence*:
            best_decoded_packet: DecodedPacket = decoded_packets[0]
            for decoded_packet in decoded_packets[1:]:
                if decoded_packet.to_sequence > best_decoded_packet.to_sequence:
                    best_decoded_packet = decoded_packet

            # Extract the *missing_to_sequences*::
            to_sequence: int = best_decoded_packet.to_sequence
            to_missing: int = best_decoded_packet.to_missing
            for index in range(8):
                if to_missing & (1 << index):
                    missing_to_sequences.append(to_sequence + 1 + index)
        return tuple(missing_to_sequences)

    # SerialLine.masked_sequence_recover():
    def masked_sequence_recover(self, masked_sequence: int, close_sequence: int) -> int:
        """Extend high order bits of a sequence using a close sequence."""
        # *masked_sequence* is sequence that has been down to the low order 8-bits
        # Recover the higher order bits by using *close_sequence*:
        high_mask: int = (close_sequence | 0xff) + 1
        low_mask: int = high_mask - 0x100
        high_sequence: int = high_mask | masked_sequence
        low_sequence: int = low_mask | masked_sequence
        high_error: int = abs(high_sequence - masked_sequence)
        low_error: int = abs(low_sequence - masked_sequence)
        # Now update *recovered_sequence* with the extended bits:
        recovered_sequence: int = high_sequence if high_error < low_error else low_sequence
        return recovered_sequence

    # SerialLine.to_packet_cache_cull():
    def to_packet_cache_clear(self, cull_to_sequence: int) -> None:
        """Clear the sent packet cache up to specified sequence."""
        to_packet_cache: Dict[int, bytes] = self.to_packet_cache
        to_sequence: int
        for to_sequence in sorted(to_packet_cache.keys()):
            if to_sequence < cull_to_sequence and len(to_packet_cache) > 1:
                del to_packet_cache[to_sequence]

    # SerialLine.to_packets_prioritize():
    def to_packets_prioritize(self, to_packets: List[bytes]) -> None:
        """Sort packets into priority queues."""
        # print(f"=>SerialLine.to_packets_prioritize({to_packets})")
        # Grap *prioritized_packets* and *packet_id_to_handle*:
        prioritized_packets: List[List[bytes]] = self.prioritized_packets
        prioritized_packets_size: int = len(prioritized_packets)
        packet_id_to_handle: Tuple[Tuple[Handle, str], ...] = self.packet_id_to_handle

        # Sort *to_packets* into the correct priortized lists:
        to_packet: bytes
        for to_packet in to_packets:
            if to_packet:
                # Using *to_packet_id*, lookup the *priority*:
                to_packet_id: int = to_packet[0]
                hangle: Handle
                tag: str
                handle, tag = packet_id_to_handle[to_packet_id]
                priority: int = handle.priority_get(tag)
                if priority >= prioritized_packets_size:
                    raise RuntimeError(f"SerialLine.to_packets_prioritize(): Priority ({priority}) "
                                       f"is too high {prioritized_packets_size}")

                # Append packet to the corrent prioritized packets list:
                prioritized_packets[priority].append(to_packet)
        # print(f"<=SerialLine.to_packets_prioritize({to_packets})")

    """
    def misc(self):
        if False:
            if partial_packet:
                # Unpack the *packet_id* from the front of the partial packet:
                assert isinstance(partial_packet, bytes), (
                    f"Received bad partial packet {partial_packet}")
                packet_id: int = Packet.unescape(partial_packet[:2])[0]
                priority_queues[packet_id].append(partial_packet)
                partial_packet = None  # Forget the packet we just got off the queue:

            # Unpack the *packet_id* from the front of the partial packet:
            assert isinstance(partial_packet, bytes), (
                f"Received bad partial packet {partial_packet}")
            packet_id: int = Packet.unescape(partial_packet[:2])[0]
            priority_queues[packet_id].append(partial_packet)
            partial_packet = None  # Forget the packet we just got off the queue:

            # Grab the highest priority *partial_packet*:
            index: int
            for index in range(priority_queues_size -1, -1):
                priority_queue: List[int] = priority_queues[index]
                if priority_queue:
                    partial_packet = priority_queue.pop(0)
                    break

            # Send partial_packet on its way:
            if partial_packet:
                send_sequence: int = self.sent_sequence
                full_packet: bytes = Packet.full_encode(partial_packet, (send_sequence, 0, 0))
                # write_io_channel.write(full_packet)
                self.sent_messages[send_sequence] = full_packet
                self.send_sequence = send_sequence + 1

                # Delay until the packet should be clear of the buffer:
                time.sleep(len(full_packet) & byte_type)
    """


if __name__ == "__main__":
    main()
