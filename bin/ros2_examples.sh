#!/usr/bin/env bash

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
# The above copyright notice and this permission notice shall be included in all copies
# or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
# FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
# OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

# <--------------------------------------- 100 characters ---------------------------------------> #

# Ensure that environment variables are properly set.
echo "================> Setup the environment variables"
readonly ROS2_DIST="foxy"
source "/opt/ros/${ROS2_DIST}/setup.bash"

# Turn on error checking here because `/opt/ros/.../setup.bash` is not error free:
set -euxo pipefail

readonly DEV_WS="/tmp/dev_ws"
readonly SRC="${DEV_WS}/src"

function build_workspace {
    echo "================> Create The development workspace."
    unset AMENT_PREFIX_PATH
    rm -rf "${DEV_WS}"
    mkdir -p "${SRC}"
    ( cd "$DEV_WS" && rosdep install -i --from-path src --rosdistro "${ROS2_DIST}" -y )
    ( cd "$DEV_WS" && colcon build --packages-select py_pubsub )
}

function publish_subscribe_examples {
    echo "================> Create py_pubsub Python packaage."
    # Define some constants:
    local PKG_NAME="py_pubsub"
    local NODE_NAME="minimal_pubsub"
    local DESCRIPTION="Examples of minimal publisher-subscriber using rclpy."

    # Construct the package directory tree:
    ( cd "${SRC}" ; ros2 pkg create --build-type ament_python "${PKG_NAME}" )

    # Define a some URL's:
    local HOST="https://raw.githubusercontent.com/"
    local ROOT_PATH="ros2/examples/foxy/rclpy/topics"
    local PUB_PATH="${ROOT_PATH}/minimal_publisher/examples_rclpy_minimal_publisher/"
    local SUB_PATH="${ROOT_PATH}/minimal_subscriber/examples_rclpy_minimal_subscriber/"
    local PUB_MOD="publisher_member_function"
    local SUB_MOD="subscriber_member_function"
    local PUB_URL="${HOST}${PUB_PATH}${PUB_MOD}.py"
    local SUB_URL="${HOST}${SUB_PATH}${SUB_MOD}.py"
    local PY_DIR="${SRC}/${PKG_NAME}/${PKG_NAME}"
    echo "PUB_URL=${PUB_URL}"    
    echo "SUB_URL=${SUB_URL}"

    # Fetch the publisher and subscriber Python files:
    (cd "${PY_DIR}" && wget "${PUB_URL}" )
    (cd "${PY_DIR}" && wget "${SUB_URL}" )

    # Update the setup configuration files:
    ws_update.py "${DEV_WS}" "${DESCRIPTION}"
}

# Creates a the service member function:
function service_member_function_py_create {
cat >> service_member_function.py <<EOF
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
   main()
EOF
}

function client_member_function_py_create {
cat >> client_member_function.py <<EOF
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_two_ints: for %d + %d = %d' %
                    (minimal_client.req.a, minimal_client.req.b, response.sum))
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
}

function service_client_examples {
    echo "================> Create py_pubsub Python packaage."
    # Define some constants:
    local PKG_NAME="py_srvcli"
    local NODE_NAME="minimal_srvcli"
    local DESCRIPTION="Examples of minimal server-client using rclpy."

    # Construct the package directory tree:
    ( cd "${SRC}" && \
      ros2 pkg create --build-type ament_python "${PKG_NAME}" )

    (cd "${SRC}/${PKG_NAME}/${PKG_NAME}" && service_member_function_py_create )
    (cd "${SRC}/${PKG_NAME}/${PKG_NAME}" && client_member_function_py_create )

    # Update the setup configuration files:
    ws_update.py "${DEV_WS}" "${DESCRIPTION}"
    # Define some constants:
    local PKG_NAME="py_srvcli"
    local NODE_NAME="minimal_srvcli"
    local DESCRIPTION="Examples of custom interfaces."
}

function add_three_ints_srv_create {
    cat >> AddThreeInts.srv <<EOF
int64 a
int64 b
int64 c
---
int64 sum
EOF
}

function tutorial_interfaces_examples {
    echo "================> Create the tutorial interfaces package."
    # Define some constants:
    local PKG_NAME="tutorial_interfaces"
    #local NODE_NAME="minimal_srvcli"
    local DESCRIPTION="Examples of minimal server-client using rclpy."

    # Construct the package directory tree:
    # Note that ROS2 currnently only builds new messages and services in C++.
    # Thus, this package is create using CMake.
    ( cd "${SRC}" && ros2 pkg create --build-type ament_cmake "${PKG_NAME}" )

    # Create the `msg` and `srv` sub-directories:
    ( cd "${SRC}/${PKG_NAME}" && mkdir msg && mkdir srv )

    # Create the `.msg` and `.srv` files:
    echo "int64 num" > "${SRC}/${PKG_NAME}/msg/Num.msg"
    
    (cd "${SRC}/${PKG_NAME}/srv" && add_three_ints_srv_create )

    # Update the workspace:
    ws_update.py "${DEV_WS}" "${DESCRIPTION}"
}

function python_parmaters_node_py_create {
    cat >> python_parameters_node.py <<EOF
import rclpy
import rclpy.node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter('my_parameter', 'world')

    def timer_callback(self):
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

        self.get_logger().info('Hello %s!' % my_param)

        my_new_param = rclpy.parameter.Parameter(
            'my_parameter',
            rclpy.Parameter.Type.STRING,
            'world'
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)

def main():
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
EOF
}

function python_parameters_examples {
    echo "================> Create the python parameters examples package."
    # Define some constants:
    local PKG_NAME="python_parameters"
    #local NODE_NAME="minimal_srvcli"
    local DESCRIPTION="Examples of ROS2 parameters using python."
    
    # Construct the directory tree:
    (cd "${SRC}" && \
	 ros2 pkg create --build-type ament_python "${PKG_NAME}" --dependencies rclpy
    )

    # Install the `python_parameters_node.py` program.
    (cd "${SRC}/${PKG_NAME}/${PKG_NAME}" && python_parmaters_node_py_create )

    # Update the workspace:  # This actually messes up package.xml
    ws_update.py "${DEV_WS}" "${DESCRIPTION}"
}

build_workspace

declare -a packages

publish_subscribe_examples
packages+=("py_pubsub")

service_client_examples
packages+=("py_srvcli")

tutorial_interfaces_examples
packages+=("tutorial_interfaces")

python_parameters_examples
packages+=("python_parameters")

# Temporarity disable bash tracing:
set +eux

# Make sure that the underlay is set up properly.
source "/opt/ros/${ROS2_DIST}/setup.bash"
cd "$DEV_WS"
source "${DEV_WS}/install/setup.bash"

# Reenable bash tracing:
set -euxo pipefail

# Build the examples:
( cd "$DEV_WS" && rosdep install -i --from-path src --rosdistro "${ROS2_DIST}" -y )
( cd "$DEV_WS" && colcon build --packages-select ${packages[*]} )

tree "${DEV_WS}"

#  [Define custom messages in python package (ROS2)](https://answers.ros.org/question/350084/define-custom-messages-in-python-package-ros2/)
# [Publishing in Python with custom message fails.](https://github.com/ros2/ros2/issues/682)
