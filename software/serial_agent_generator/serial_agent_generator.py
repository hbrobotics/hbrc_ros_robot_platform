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

# <====================================== 100 Characters ======================================= > #

import shutil
import subprocess
import sys

from pathlib import Path
from typing import Dict, IO, List, Tuple

def main() -> int:
    # Parse command line arguments:
    args: List[str, ] = sys.argv[1:]
    # print(f"args={args}")
    if len(args) == 0:
        print(f"Usage: serial_agent.py [WORKSPACE_DIR] [PACKAGE_NAME]")

    workspace_directory: Path = Path("/tmp/dev_ws") if len(args) <= 1 else Path(args[0])
    package_name: str = "serial_agent" if len(args) <= 2 else args[1]
    # print(f"workspace_directory='{workspace_directory}' package_name='{package_name}'")

    # Create the various directories:
    ros2_package_directory: Path = workspace_directory / "src" / package_name
    python_package_directory: Path = ros2_package_directory / package_name
    python_package_directory.mkdir(parents=True, exist_ok=True)
        
    # Write out the `package.xml` and `__init__.py` files:
    package_xml_create(ros2_package_directory, package_name)
    init_py_create(python_package_directory)
    setup_cfg_create(ros2_package_directory, package_name)
    setup_py_create(ros2_package_directory, package_name)
    resource_create(ros2_package_directory, package_name)

    # Read in 
    serial_agent_py_source: Path = Path("serial_agent.py")
    serial_agent_py_destination: Path = python_package_directory / "serial_agent.py"

    if True:
        shutil.copyfile(serial_agent_py_source, serial_agent_py_destination)
        print(f"{serial_agent_py_source} copied to {serial_agent_py_destination}")
        return 0

    # Temporary testing of SerialAgentGenerator class:
    serial_agent_generator: SerialAgentGenerator = SerialAgentGenerator()
    # Temporary:
    serial_agent_generator.publisher_add("std_msgs.msg", "String", "topic")
    serial_agent_generator.subscription_add("std_msgs.msg", "String", "topic")
    serial_agent_generator.client_add("example_interfaces", "AddTwoInts", "add_two_ints")
    serial_agent_generator.server_add("example_interfaces", "AddTwoInts", "add_two_ints")
    serial_agent_generator.timer_add("add_two_ints", "String", 4.0)
    serial_agent_generator.serial_agent_py_create(ros2_package_directory, package_name)

    return 0


class ImportManager(object):
    """Class to keep track of import statements.

    It is really easy for different packages to conflict on the type names that
    they create.  This package attempts to detect these errors and fail with a
    a semi useful error message.
    """

    # ImportManager.__init__():
    def __init__(self) -> None:
        """Initialize ImportManager"""
        self.all_tags: Tuple[str, ...] = ()  # All allowed tags
        self.tag_to_import_lines: Dict[str, List[str]] = {}  # tag => lines
        self.tag_to_comment_line: Dict[str, str] = {}  # tag => comment line
        self.import_lines_to_tag: Dict[str, str] = {}  # import_line => tag
        self.global_to_import_line: Dict[str, str] = {}  # global_name => import_line

    # ImportManager.tag_create():
    def tag_create(self, tag: str, comment_line: str) -> None:
        """Create an import tag."""
        assert tag not in self.all_tags, "Duplicate tag '{tag}' creation."
        self.all_tags += (tag,)
        self.tag_to_comment_line[tag] = comment_line
        self.tag_to_import_lines[tag] = []

    # ImportManager.insert():
    def insert(self, tag: str, global_name: str, import_line: str, comment: str) -> None:
        if global_name == "AddTwoInts":
            print(f">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> {tag}")
        assert tag in self.all_tags, f"Tag '{tag}' is not one of {self.all_tags}."
        if global_name in self.global_to_import_line:
            # Duplicate global name.  It may be OK:
            previous_import_line: str = self.global_to_import_line[global_name]
            assert import_line  == previous_import_line, (
                f"'{global_name}' is defined by both '{previous_import_line}' and '{import_line}'")
        else:
            if import_line not in self.tag_to_import_lines[tag]:
                self.global_to_import_line[global_name] = import_line
                self.import_lines_to_tag[import_line] = tag
                if comment:
                    import_line += f"  # {comment}"
                self.tag_to_import_lines[tag].append(import_line)

    # ImportManager.simple_import():
    def simple_import(self, tag: str, global_name: str, comment: str = "") -> None:
        """Generate a simple import statement."""
        import_line: str = f"import {global_name}"
        self.insert(tag, global_name, import_line, comment)

    # ImportManager.import_as():
    def import_as(self, tag: str, import_name: str, global_name: str, comment: str = "") -> None:
        """Generate an import/as statement."""
        import_line: str = f"import {import_name} as {global_name}"
        self.insert(tag, global_name, import_line, comment)

    # ImportManager.from_import():
    def from_import(self, tag: str, import_name: str, global_name: str, comment: str = "") -> None:
        """Generate an from/import statement."""
        assert global_name != "topic", f"tag='{tag}"  # Debug only
        import_line: str = f"from {import_name} import {global_name}"
        self.insert(tag, global_name, import_line, comment)

    # ImportManager.from_import_as():
    def from_import_as(self, tag: str, import_name: str,
                       actual_name: str, global_name: str, comment: str = "") -> None:
        """Generate a from/import/as statement."""
        import_line: str = f"from {import_name} import {actual_name} as {global_name}"
        self.insert(tag, global_name, import_line, comment)

    # ImportManager.import_lines_get():
    def import_lines_get(self) -> List[str]:
        """Return a list of all of the imports."""
        # Standard import lines go first:
        import_lines: List[str] = [
            "",
            "# Type hint imports:",
            "from typing import Any, Callable, Dict, List, Optional, Set",
        ]

        # Append the remaining import lines on a tag by tag basis:
        tag: str
        for tag in self.all_tags:
            tag_import_lines: List[str] = self.tag_to_import_lines[tag]
            if tag_import_lines:
                comment_line: str = self.tag_to_comment_line[tag]
                import_lines.extend(["", comment_line] + sorted(self.tag_to_import_lines[tag]))

        return import_lines

    # ImportManager.service_types_import():
    def service_types_import(self, service_package: str, service_type: str) -> Tuple[str, str]:
        """Import the types needed for a service."""

        # Ensure that the service message type is imported.
        # self.from_import("server", service_package, service_type, "type: ignore")

        # Also ensure that associated Request and Response types are imported:
        request_type: str = f"{service_type}.Request"
        response_type: str = f"{service_type}.Response"
        self.from_import("service", f"{service_package}.srv", service_type, "type: ignore")
        return request_type, response_type


# TaggedLines:
class TaggedLines(object):
    """Class for managing the tagged lines."""
    
    # TaggedLines.__init__():
    def __init__(self) -> None:
        """Initialize the InitManager."""

        self.tags: Tuple[str, ...] = ()
        self.lines: Dict[str, List[str]] = {}
        self.prefix_lines: Dict[str, List[str]] = {}

    # TaggedLines.tag_create():
    def tag_create(self, tag: str, prefix_lines: List[str]) -> None:
        """Create a new tag tagged buffer."""

        assert tag not in self.tags, f"Defining a duplicate tag: '{tag}'"
        self.tags += (tag,)
        self.lines[tag] = []
        self.prefix_lines[tag] = prefix_lines

    # TaggedLine.prefix_set():
    def prefix_set(self, tag: str, prefix_lines: List[str]) -> None:
        """Set the prefix lines."""
        assert tag in self.tags, f"Unknown tag '{tag}'"
        self.prefix_lines[tag] = prefix_lines

        # Force the prefix to show up.
        self.lines[tag].append("")

    # TaggedLines.extend():
    def extend(self, tag: str, lines: List[str]) -> None:
        """Append some lines to a tagged buffer."""
        assert tag in self.tags, f"Unknonw tag '{tag}'"
        self.lines[tag].extend(lines)

    # TaggegLines.lines_get():
    def lines_get(self) -> List[str]:
        """Return the tagged lines as a single list of lines."""
        all_lines: List[str] = []
        tag: str
        for tag in self.tags:
            lines: List[str] = self.lines[tag]
            # Empty lines a the end for rce
            if lines:
                # Empty lines are trimmed.  Thus, a prefix only can be output:
                while lines and lines[0] == "":
                    lines.pop(0)
                all_lines.extend([""] + self.prefix_lines[tag] + lines)
        return all_lines

# SerialAgentGenerator:
class SerialAgentGenerator(object):
    """Class for generating a serial agent."""

    # SerialAgentGenerator.__init__():
    def __init__(self) -> None:
        """Initialize SerialAgentGenerator."""

        # Create all of the import tags for *import_manager*:
        import_manager: ImportManager = ImportManager()
        import_manager.tag_create("required", "# Required imports:")
        import_manager.tag_create("publisher", "# Publisher required imports:")
        import_manager.tag_create("subscription", "# Subscription required imports:")
        import_manager.tag_create("message", "# Message type required imports:")
        import_manager.tag_create("client", "# Client required imports:")
        import_manager.tag_create("server", "# Server required imports:")
        import_manager.tag_create("service", "# Service type required imports:")
        import_manager.tag_create("timer", "# Timer required imports:")
        self.import_manager: ImportManager = import_manager

        # Fill in the requried imports:
        import_manager.simple_import("required", "rclpy", "type: ignore")
        import_manager.from_import_as("required", "rclpy.impl.rcutils_logger",
                                      "RcutilsLogger", "Logger", "type: ignore")
        import_manager.from_import("required", "rclpy.node", "Node", "type: ignore")
        
        # Various sections of generated code.
        indent: str = " " * 8
        init_tagged_lines: TaggedLines = TaggedLines()
        init_tagged_lines.tag_create("publisher", [])
        init_tagged_lines.tag_create("subscription", [])
        init_tagged_lines.tag_create("client", [])
        init_tagged_lines.tag_create("server", [])
        init_tagged_lines.tag_create("timer", [])

        # This comes last:
        init_tagged_lines.tag_create("server_wait", [])  # Prefix lines filled elsewhere

        self.init_tagged_lines: TaggedLines = init_tagged_lines

        # Callback methods are added to this list:
        callback_tagged_lines: TaggedLines = TaggedLines()
        callback_tagged_lines.tag_create("subscription", [])
        callback_tagged_lines.tag_create("client", [])
        callback_tagged_lines.tag_create("server", [])
        callback_tagged_lines.tag_create("timer", [])
        self.callback_tagged_lines: TaggedLines = callback_tagged_lines

        # Copyright lines for the beginning of the file:
        self.copyright_lines: List[str] = [
            "# Code generated by ros2_serial_manager.py.",
            "# Generated code is not really covered by copyright law.",
        ]

        # Create the main() function:
        dq1: str = "\""
        dq3: str = dq1 * 3
        self.main_lines: List[str] = [
            "",
            "",
            "# main():",
            "def main(args: Optional[List[str]] = None) -> None:",
            "    # Initialize the Python ROS2 interface library.",
            "    rclpy.init(args=args)",
            "",
            "    # Create the serial agent Node:",
            "    serial_agent: SerialAgent = SerialAgent()",
            "",
            "    # Start the serial agent Node.  All activity occurs via callback routines:",
            "    print('Starting rclpy.spin()')",
            "    rclpy.spin(serial_agent)",
            "",
            "    # Destroy the serial agent node explicitly.",
            "    serial_agent.destroy_node()",
            "",
            "    # Shut down the Python ROS2 interface library.",
            "    rclpy.shutdown()",
        ]

        # Serial Agent class lines:
        self.class_header_lines: List[str] = [
            "",
            "",
            "# SerialAgent:",
            "class SerialAgent(Node):",
            f"    {dq3}Serial Agent for managing ROS2 topics and services.{dq3}",
        ]

        self.init_start_lines: List [str] = [
            "",
            "    # SerialAgent.init():",
            "    def __init__(self) -> None:",
            f"        {dq3}Initialize SerialAgent class.{dq3}",
            "",
            "        # Initialize the Node super-class:",
            "        super().__init__('serial_agent')",
        ]

        # The boiler plate code to call main() goes here:
        self.main_call_lines: List[str] = [
            "",
            "",
            "# Call main():",
            "if __name__ == '__main__':",
            "    main(args=None)",
        ]



    # SerialAgent.serial_agent_pyt_create():
    def serial_agent_py_create(self, ros2_package_directory: Path, package_name: str) -> None:
        """Write out `serial_agent.py` file."""
        dq1: str = "\""
        dq3: str = dq1 * 3

        # Joint all of the import lines into *all_import_lines*:
        all_import_lines: List[str] = self.import_manager.import_lines_get()
        all_init_lines: List[str] = self.init_start_lines + self.init_tagged_lines.lines_get()
        all_callback_lines: List[str] = self.callback_tagged_lines.lines_get()

        # Join all of the lines that make up the class into *all_class_lines*:
        all_class_lines: List[str] = (
            self.class_header_lines +
            all_init_lines +
            all_callback_lines
        )

        # Join all the lines together into *all_lines* and create a single
        all_lines: List[str] = (
            self.copyright_lines +
            all_import_lines +
            self.main_lines +
            all_class_lines +
            self.main_call_lines +
            [""]  # Trick to get the final new-line on the file content
        )

        # Generate the `serial_agent.py` file:
        all_content: str = "\n".join(all_lines)
        serial_agent_py_path: Path = ros2_package_directory / package_name / "serial_agent.py"
        serial_agent_py_file: IO[str]
        with open(serial_agent_py_path, "w") as serial_agent_py_file:
            serial_agent_py_file.write(all_content)

    # SerialAgent.publisher_add():
    def publisher_add(self, topic_package_name: str, topic_type: str, topic_name: str) -> None:
        """Add a publisher to the serial agent."""
        
        # Make sure the Publisher type is imported:
        import_manager: ImportManager = self.import_manager
        import_manager.from_import("publisher", "rclpy.publisher", "Publisher", "type: ignore")
        import_manager.from_import("message", topic_package_name, topic_type, "type: ignore")


        # Make sure the publisher tables are created:
        init_tagged_lines: TaggedLines = self.init_tagged_lines
        init_tagged_lines.prefix_set("publisher", [
            f"        # Create all topic Publisher's here:",
            f"        self.publishers_table: Dict[str, Publisher] = {{}}",
            f"        self.publishers_counter: Dict[str, int] = {{}}",
        ])
        

        # Add publisher here:
        subscription_callback_name: str = f"{topic_name}_subscription_callback"
        init_tagged_lines.extend("publisher", [
            f"        self.publishers_table['{topic_name}'] = self.create_publisher(",
            f"            {topic_type}, '{topic_name}', 10)",
            f"        self.publishers_counter['{topic_name}'] = 0",
        ])
            

    # SerialAgent.timer_add():
    def timer_add(self, timer_name: str, timer_type: str, period: float) -> None:
        """Add a timer callback."""

        # Ensure that the Timer type is imported:
        import_manager: ImportManager = self.import_manager
        import_manager.from_import("timer", "rclpy.timer", "Timer", "type: ignore")

        # Create the timer here:
        callback_name: str = f"{timer_name}_timer"
        init_tagged_lines: TaggedLines = self.init_tagged_lines
        init_tagged_lines.prefix_set("timer", [
            f"        # Create all Timer's here:",
            f"        self.timers_table: Dict[str, Timer] = {{}}",
            f"        self.timers_counter: Dict[str, int] = {{}}",
        ])
        init_tagged_lines.extend("timer", [
            f"        self.timers_table['{timer_name}'] = (",
            f"            self.create_timer({period}, self.{callback_name}))",
            f"        self.timers_counter['{timer_name}'] = 0",
        ])
        
        # Create the callback counter and function:
        dq1: str = "\""
        dq3: str = dq1 * 3
        callback_tagged_lines: TaggedLines = self.callback_tagged_lines
        callback_tagged_lines.extend("timer", [
            "",
            f"    # SerialAgent.{callback_name}:",
            f"    def {callback_name}(self) -> None:",
            f"        {dq3}Callback for {timer_name} Timer.{dq3}",
            "",
            f"        count: int = self.timers_counter['{timer_name}']",
            f"        message: {timer_type} = {timer_type}()",
            f"        message.data = f'Counter={{count}}'",
            "",
            f"        # Temporary:",
            f"        publisher: Publisher = self.publishers_table['topic']",
            f"        publisher.publish(message)",
            f"        print('\\nTimer({dq1}{timer_name}{dq1}): '",
            f"              f'Publish {{message}} to Topic({dq1}{timer_name}{dq1})')",
            "",
            f"        self.timers_counter['{timer_name}'] = count + 1",
            f"        # print('<=SerialAgent.{callback_name}()')",
        ])

    # SerialAgent.client_add():
    def client_add(self, service_package: str, service_type: str, client_name: str) -> None:
        """Add a client to the serial agent."""
    
        # Ensure that the Client type is imported:
        import_manager: ImportManager = self.import_manager
        import_manager.from_import("client", "rclpy.task", "Future", "type: ignore")
        import_manager.from_import("client", "rclpy.client", "Client", "type: ignore")

        request_type: str
        response_type: str
        request_type, response_type = import_manager.service_types_import(
            service_package, service_type)

        # Ensure that the client wait code is added to the __init__ startup.
        init_tagged_lines: TaggedLines = self.init_tagged_lines
        init_tagged_lines.prefix_set("server_wait", [
            "        # Wait for all requested client services to become available:",
            "        logger: Logger = self.get_logger()",
            "        while self.client_waits:",
            "            client_name: str",
            "            for client_name in tuple(self.client_waits):",
            "                client: Client = self.clients_table[client_name]",
            "                if client.wait_for_service():",
            "                    self.client_waits.remove(client_name)",
            "            if not self.client_waits:",
            "                logger.info(f'Waiting for {{client_waits}} to become available')",
        ])

        # Create the client and add it to the clients dictionariers:
        init_tagged_lines.prefix_set("client", [
            f"        # Client send rouines are defined here:",
        ])
        init_tagged_lines.prefix_set("client", [
            f"        # Create all service Client's here:",
            f"        self.clients_table: Dict[str, Client] = {{}}",
            f"        self.clients_counter: Dict[str, int] = {{}}",
            f"        self.client_waits: Set[str] = set()",
        ])
        init_tagged_lines.extend("client", [
            f"        self.clients_table['{client_name}'] = self.create_client(",
            f"            {service_type}, '{client_name}')",
            f"        self.clients_counter['{client_name}'] = 0",
            f"        self.client_waits.add('{client_name}')",
        ])

        # Generate a request routine with a callback for the eventual response:
        dq1: str = "\""
        dq3: str = dq1 * 3
        request_send_name: str = f"{client_name}_request_send"
        response_callback_name: str = f"{client_name}_response_callback"
        indent: str = " " * (len(request_send_name) + 1)
        callback_tagged_lines: TaggedLines = self.callback_tagged_lines
        callback_tagged_lines.extend("client", [
            "",
            f"    # SerialAgent.{request_send_name}()",
            f"    # Debugging:",
            f"    def {request_send_name}(self, a: int, b: int,",
            f"        {indent}response_callback: Callable[[Any], None]) -> None:",
            f"        {dq3}Initiate a request/response to/from {client_name} server.{dq3}",
            "",
            f"        request: {request_type} = {request_type}()",
            f"        request.a = a",
            f"        request.b = b",
            f"        client: Client = self.clients_table['{client_name}']",
            f"        response_future: Future = client.call_async(request)",
            f"        # Debugging:",
            f"        print(f'response_future={{response_future}})')",
            f"        response_future.add_done_callback(response_callback)",
            f"        print('Client({dq1}{client_name}{dq1}): '",
            f"              f'Sent {dq1}{{request}}{dq1}')",
        ])

        callback_tagged_lines.extend("client", [
            "",
            f"    # SerialAgent.{response_callback_name}():",
            f"    # Debugging:",
            f"    def {response_callback_name}(self, response_future: Future) -> None:",
            f"        {dq3}Response callback routine for {client_name}.{dq3}",
            "",
            f"        # Debugging:",
            f"        try:",
            f"            response: Any = response_future.result()",
            f"        except Exception as error:",
            f"            assert False, 'Fixme!'",
            f"        else:",
            f"            assert isinstance(response, {response_type})",
            f"            print('Client({dq1}{client_name}{dq1}): '",
            f"                  f'Got response={dq1}{{response}}{dq1}')",
        ])

    # SerialAgent.server_add():
    def server_add(self, service_package_name: str, service_type: str, server_name: str) -> None:
        """Add a server to the serial agent."""

        # Ensure that the Server type is imported.
        # Note rclpy.service.Service is changed to Server to be more consistent.
        import_manager: ImportManager = self.import_manager
        import_manager.from_import_as(
            "server", "rclpy.service", "Service", "Server", "type: ignore")

        # Ensure that the service Request and Response types are imported.
        request_type: str
        response_type: str
        request_type, response_type = import_manager.service_types_import(
            service_package_name, service_type)

        # Ensure that the server dictionaries are first initalized to empty:
        init_tagged_lines: TaggedLines = self.init_tagged_lines
        init_tagged_lines.prefix_set("server", [
            "        # Create all service Server's here:",
            "        self.servers_table: Dict[str, Server] = {}",
            "        self.servers_counter: Dict[str, int] = {}",
        ])

        # Create the server and add it to the servers dictionary:
        callback_name: str = f"{server_name}_server_callback"
        init_tagged_lines.extend("server", [
            f"        # Create all topic Subscription's here:",
            f"        self.servers_table['{server_name}'] = self.create_service(",
            f"            {service_type}, '{server_name}', self.{callback_name})",
            f"        self.servers_counter['{server_name}'] = 0",
        ])

        # Create the callback method for the server:
        dq1: str = "\""
        dq3: str = dq1 * 3
        indent: str = " " * len(callback_name)
        callback_tagged_lines: TaggedLines = self.callback_tagged_lines
        callback_tagged_lines.extend("server", [
            "",
            f"    # SerialAgent.{callback_name}() -> {response_type}:",
            f"    def {callback_name}(self,",
            f"         {indent}request: {request_type},",
            f"         {indent}response: {response_type}) -> None:",
            "",
            f"        {dq3}Callback for '{server_name}' service request.{dq3}",
            f"        # Debugging only:",
            f"        print(f'Service({dq1}{server_name}{dq1}): Got {dq1}{{request}}{dq1}')",
            f"        response.sum = request.a + request.b",
            f"        print(f'Service({dq1}{server_name}{dq1}): Sent {dq1}{{response}}{dq1}')",
            f"        return response",
        ])

    # SerialAgent.subscription_add():
    def subscription_add(self, subscription_package_name: str,
                         subscription_type: str, subscription_name: str) -> None:
        """Add a subscription to the serial agent."""

        # Make sure the Subscription type is imported:
        import_manager: ImportManager = self.import_manager
        import_manager.from_import(
            "subscription", "rclpy.subscription", "Subscription", "type: ignore")
        import_manager.from_import(
            "message", subscription_package_name, subscription_type, "type: ignore")

        # Ensure that the subscriptions dictionary is initalized to empty:
        init_tagged_lines: TaggedLines = self.init_tagged_lines
        init_tagged_lines.prefix_set("subscription", [
                "        # Create all topic Subscription's here:",
                "        self.subscriptions_table: Dict[str, Subscription] = {}",
                "        self.subscriptions_counter: Dict[str, int] = {}",
        ])

        # Create the subscription and add it to the subscriptions dictionary:
        callback_name: str = f"{subscription_name}_subscription_callback"
        init_tagged_lines.extend("subscription", [
            f"        self.subscriptions_table['{subscription_name}'] = self.create_subscription(",
            f"            {subscription_type}, '{subscription_name}', self.{callback_name}, 10)",
            f"        self.subscriptions_counter['{subscription_name}'] = 0",
        ])

        # Create the callback method for the subscription:
        dq1: str = "\""
        dq3: str = dq1 * 3
        callback_tagged_lines: TaggedLines = self.callback_tagged_lines
        callback_tagged_lines.extend("subscription", [
            "",
            f"    # SerialAgent.{callback_name}():",
            f"    def {callback_name}(self, message: {subscription_type}) -> None:",
            f"        {dq3}Callback for '{subscription_name}' topic subscription.{dq3}",
            f"        count: int = self.subscriptions_counter['{subscription_name}']",
            f"        count += 1",
            f"        self.subscriptions_counter['{subscription_name}'] = count",
            f"        # Debgging only:",
            f"        print(f'Subscription({dq1}{subscription_name}{dq1}): '"
            f"              f'Got {dq1}{{message.data}}{dq1}')",
            "",
            f"        # Debugging:",
            f"        a: int = count",
            f"        b: int = count + 1",
            f"        self.add_two_ints_request_send(a, b, self.add_two_ints_response_callback)",
        ])


def resource_create(ros2_package_directory: Path, package_name: str) -> None:
    """Create the resource directory an fill it in."""
    resource_directory: Path = ros2_package_directory / "resource"
    resource_directory.mkdir(parents=True, exist_ok=True)

    # Write out an empty file:
    resource_package_path: Path = resource_directory / package_name
    resource_package_file: IO[str]
    with open(resource_package_path, "w") as resource_package_file:
        pass


def setup_py_create(ros2_package_directory: Path, package_name: str) -> None:
    """Create the `setup.py` file. """
    email: str = email_get()
    maintainer: str = maintainer_get()
    setup_py_lines: Tuple[str, ...] = (
        "from setuptools import setup",
        "",
        f"package_name = '{package_name}'",
        "",
        "setup(",
        "    name=package_name,",
        "    version='0.0.0',",
        "    packages=[package_name],",
        "    data_files=[",
        "        ('share/ament_index/resource_index/packages',",
        "            ['resource/' + package_name]),",
        "        ('share/' + package_name, ['package.xml']),",
        "    ],",
        "    install_requires=['setuptools'],",
        "    zip_safe=True,",
        f"    maintainer='{maintainer}',",
        f"    maintainer_email='{email}',",
        "    description='ROS2 Serial line package',",
        "    license='MIT',",
        "    tests_require=['pytest'],",
        "    entry_points={",
        "        'console_scripts': [",
        f"            'serial_agent = {package_name}.serial_agent:main',",
        "        ],",
        "    },",
        ")",
        "",
    )
    setup_py_content: str ="\n".join(setup_py_lines)

    # Write *setup_py_lines_content* to *setup_py_path:
    setup_py_path: Path = ros2_package_directory / "setup.py"
    setup_py_file: IO[str]
    with open(setup_py_path, "w") as setup_py_file:
        setup_py_file.write(setup_py_content)


def package_xml_create(ros2_package_directory: Path, package_name: str) -> None:
    """Create the `package.xml` file."""
    # Get the *maintainer* and *email*:
    email: str = email_get()
    maintainer: str = maintainer_get()

    # Create the *package_xml_text* content.  Use single quotes for strings since XML
    # files tend to use double quotes a lot:
    package_xml_lines: Tuple[str, ...] = (
        '<?xml version="1.0"?>',
        ('<?xml-model href="http://download.ros.org/schema/package_format3.xsd"'
         ' schematypens="http://www.w3.org/2001/XMLSchema"?>'),
        '<package format="3">',
        f'  <name>{package_name}</name>',
        '  <version>0.0.0</version>',
        '  <description>Package for interacting with serial line.</description>',
        f'  <maintainer email="{email}">{maintainer}</maintainer>',
        '  <license>MIT</license>',
        '  <exec_depend>rclpy</exec_depend>',
        '  <exec_depend>std_msgs</exec_depend>',
        '',
        '  <test_depend>ament_copyright</test_depend>',
        '  <test_depend>ament_flake8</test_depend>',
        '  <test_depend>ament_pep257</test_depend>',
        '  <test_depend>python3-pytest</test_depend>',
        '',
        '  <export>',
        '    <build_type>ament_python</build_type>',
        '  </export>',
        '</package>',
        ''
    )
    package_xml_content: str = "\n".join(package_xml_lines)

    # Write out the *package_xml_content* to the `package.xml` file:
    package_xml_path: Path = ros2_package_directory / "package.xml"
    package_xml_file: IO[str]
    with open(package_xml_path, "w") as package_xml_file:
        package_xml_file.write(package_xml_content)


def init_py_create(python_package_directory: Path) -> None:
    """Create the empty `__init__.py` file."""
    init_py_path: Path = python_package_directory / "__init__.py"
    init_py_file: IO[str]
    with open(init_py_path, "w") as init_py_file:
        pass  # An empty file is what is desired


def setup_cfg_create(ros_package_directory: Path, package_name: str) -> None:
    """Create the `setup.cfg` file."""
    # Create the *setup_cfg_content*:
    setup_cfg_lines: Tuple[str, ...] = (
        "[develop]",
        f"script-dir=$base/lib/{package_name}",
        "[install]",
        f"install-scripts=$base/lib/{package_name}",
        "",
    )
    setup_cfg_content: str = "\n".join(setup_cfg_lines)

    # Write *setup_cfg_content* to *set_up_cfg_file*:
    setup_cfg_path: Path = ros_package_directory / "setup.cfg"
    setup_cfg_file: IO[str]
    with open(setup_cfg_path, "w") as setup_cfg_file:
        setup_cfg_file.write(setup_cfg_content)


def email_get() -> str:
    """Return the maintainer E-mail from git."""
    done: subprocess.CompletedProcess = subprocess.run(
        ["git", "config", "--get", "user.email"], capture_output=True, encoding="UTF-8")
    assert done.returncode == 0, "git config --get user.name has error code of {done.returncode}"
    return done.stdout.strip("\n")


def maintainer_get() -> str:
    """Return the maintainer name from git."""
    done: subprocess.CompletedProcess = subprocess.run(
        ["git", "config", "--get", "user.name"], capture_output=True, encoding="UTF-8")
    assert done.returncode == 0, "git config --get user.name has error code of {done.returncode}"
    return done.stdout.strip("\n")


if __name__ == "__main__":
    main()


# [Multiple futures await](https://stackoverflow.com/questions/46785274/asyncio-speculatively-await-multiple-futures)

#   client = MinimalClientAsync()
#   while rclpy.ok():
#       rclpy.spin_once()
#       response_futures: List[asyncio.Future] = 
#       done_futures: List[asyncio.Future]
#       pending_futures: List[asyncio.Future]
#       done_futures, pending_futures = (
#           await asyncio.wait(response_futures, return_when=FIRST_COMPLETED))
#       # Ignore pending_futures
#       done_future: asyncio.Future
#       for done_future in done_futures:
#            try:
#                response = done_future.result()
#            except Exception as e:
#                # Remove from all_futures
#                # do something error like here
#            else:
#                # Remove from all_futures
#                # Response is valid here
#                # Do someithing with response

# request = example_interfaces.srv._add_two_ints.AddTwoInts_Request(1, 2)
# request.get_fields_and_types()  => {'a': 'int64', 'b': 'int64'}
#
# response = example_interfaces.srv._add_two_ints.AddTwoInts_Response()
# response.get_fields_and_types() => {'sum': 'int64'}
