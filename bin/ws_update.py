#!/usr/bin/env python

# MIT License
#
# Copyright 2020 Home Brew Robotics Club
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

# This program is meant to update various configuration files associated with a ROS2 workspace.
# It basically scans through various ROS2 python files and updates the `setup.py` file and
# `package.xml` file within a ROS2 workspace.

import subprocess
import sys
from pathlib import Path
from typing import Dict, IO, List, Set, Tuple


def main(tracing: str = "") -> int:
    """Run main program to update ROS2 configuration files.

    Usage:
        ros2_ws_update.py WORKSPACE_DIR [DESCRIPTION]

    where:
        WORKSPACE_DIR is a full path to the top level ROS2 workspace.
        DESCRIPTION is a description string that gets stuffed into the various configuration files.

    This program `WORSPACE_DIR/src/` looking for top-level directories with a `package.xml` file.
    When present, both the `package.xml` and `setup.py` files are updated by scanning for
    Python files.

    """
    # Tracing can be manually enabled setting *tracing* to a non-empty string.
    next_tracing: str = tracing + " " if tracing else ""
    if tracing:
        print(f"{tracing}=>main()")
    result: int = 0

    # Parse the command line arguments:
    arguments: List[str] = sys.argv[1:]
    if len(arguments) < 1:
        print("Usage: ros2_ws_update.py WORKSPACE_DIR [DESCRIPTION]")
        result = 1
    else:
        workspace_path: Path = Path(arguments[0])
        description: str = arguments[1] if len(arguments) >= 2 else ""
        workspace_update(workspace_path, description, tracing=next_tracing)

    if tracing:
        print(f"{tracing}<=main()=>{result}")
    return result


def workspace_update(workspace_path: Path, description: str, tracing: str = "") -> None:
    """Update the configuration files for a ROS2 workspace."""
    next_tracing: str = tracing + " " if tracing else ""
    if tracing:
        print(f"{tracing}=>workspace_update({workspace_path}, '{description}')")

    # Verify that *workspace_path* is a directory and fail if it is not one:
    assert workspace_path.is_dir(), f"{workspace_path} is not a directory"

    # Verify that *required_sub_directories* actually exist:
    required_sub_directories: Set[str] = {"build", "install", "log", "src"}
    actual_sub_directories: Set[str] = {
        Path(file_name).name for file_name in workspace_path.glob("*")
    }
    missing_sub_directories: Set[str] = required_sub_directories - actual_sub_directories
    assert not missing_sub_directories, (
        f"Sub-directories {missing_sub_directories} are missing from {workspace_path}")

    # Find and Update each *package_path*:
    package_paths: Tuple[Path, ...] = packages_find(workspace_path, tracing=next_tracing)
    package_path: Path
    for package_path in package_paths:
        package_update(package_path, description, tracing=next_tracing)

    if tracing:
        print(f"{tracing}=>workspace_update({workspace_path}, '{description}')")


def packages_find(workspace_path: Path, tracing: str = "") -> Tuple[Path, ...]:
    """Find the various packages within a ROS2 workspace."""
    # next_tracing: str = tracing + " " if tracing else ""
    if tracing:
        print(f"{tracing}=>packages_find({workspace_path})")
    src_path: Path = workspace_path / "src"
    assert src_path.is_dir(), f"{src_path} is not a directory"

    all_packages: Set[Path] = set()
    package_path: Path
    for package_path in src_path.glob("*/package.xml"):
        all_packages.add(package_path.parent)
    for package_path in src_path.glob("*/CMakeLists.txt"):
        all_packages.add(package_path.parent)
    final_packages: Tuple[Path, ...] = tuple(sorted(all_packages))

    if tracing:
        print(f"{tracing}<=packages_find({workspace_path})=>{final_packages}")
    return tuple(final_packages)


def package_update(package_path: Path, description: str, tracing: str = "") -> None:
    """Update the configuration files for a package."""
    next_tracing: str = tracing + " " if tracing else ""
    if tracing:
        print(f"{tracing}=>package_update({package_path}, '{description}')")
    python_package_paths: Tuple[Path, ...] = (
        python_packages_find(package_path, tracing=next_tracing))

    import_names: Set[str] = set()
    has_param_talker: bool = False
    python_package_path: Path
    for python_package_path in python_package_paths:
        python_paths: Tuple[Path, ...] = python_paths_find(python_package_path,
                                                           tracing=next_tracing)
        python_path: Path
        for python_path in python_paths:
            import_names |= python_imports_get(python_path, tracing=next_tracing)

        label_entries: Dict[str, str] = python_label_entries_find(
            python_paths, tracing=next_tracing)
        has_param_talker |= "param_talker" in label_entries
        setup_update(package_path, description, label_entries, tracing=next_tracing)

    # Kludge: trim import names to zero for parameters package.
    if has_param_talker:
        import_names = set()
    if (package_path / "package.xml").exists():
        package_xml_update(package_path, import_names, description, tracing=next_tracing)
    if (package_path / "CMakeLists.txt").exists():
        package_cmakelists_update(package_path, tracing=next_tracing)

    if tracing:
        print(f"{tracing}<=package_update({package_path}, '{description}')")


def python_packages_find(package_path: Path, tracing: str = "") -> Tuple[Path, ...]:
    """Find the various python packages within a ROS2 package."""
    # next_tracing: str = tracing + " " if tracing else ""
    if tracing:
        print(f"{tracing}=>python_packages_find({package_path})")
    package_name: str = package_path.name
    python_package_paths: List[Path] = []
    init_path: Path
    for init_path in package_path.glob(f"{package_name}/__init__.py"):
        python_package_paths.append(init_path.parent)
    python_package_paths.sort()

    if tracing:
        print(f"{tracing}<=python_packages_find({package_path})=>{python_package_paths}")
    return tuple(python_package_paths)


def python_paths_find(python_package_path: Path, tracing: str = "") -> Tuple[Path, ...]:
    """Find all of the python files inside of a Python package."""
    # next_tracing: str = tracing + " " if tracing else ""
    if tracing:
        print(f"{tracing}=>python_paths_find({python_package_path})")
    python_path: Path
    python_files: List[Path] = [
        python_path
        for python_path in python_package_path.glob("*.py")
        if python_path.name != "__init__.py"
    ]
    python_files.sort()
    if tracing:
        print(f"{tracing}<=python_paths_find({python_package_path})=>{python_files}")
    return tuple(python_files)


def python_imports_get(python_path: Path, tracing: str = "") -> Set[str]:
    """Scan the python files searching for imported files to keep track of."""
    # next_tracing: str = tracing + " " if tracing else ""
    if tracing:
        print(f"{tracing}=>python_imports_get({python_path})")

    imports_set: Set[str] = set()
    python_file: IO[str]
    with open(python_path) as python_file:
        python_lines: List[str] = python_file.read().split("\n")
        python_line: str
        index: int
        for python_line in python_lines:
            if python_line.startswith("import "):
                imports_set.add(python_line[7:])
            elif python_line.startswith("from "):
                dot_index: int = python_line.find('.')
                space_index: int = python_line.find(' ')
                if dot_index >= 0:
                    imports_set.add(python_line[5:dot_index])
                elif space_index >= 0:
                    imports_set.add(python_line[5:space_index])

    # This is kludge for now.  What we want to do is to restrict the imports to ROS2 packages.
    imports_set -= {"os", "sys"}
    if tracing:
        print(f"{tracing}<=python_imports_get({python_path})=>{imports_set}")
    return imports_set


def package_cmakelists_update(package_path: Path, tracing: str = "") -> None:
    """Update the CMakeLists.txt file."""
    # next_tracing: str = tracing + " " if tracing else ""
    if tracing:
        print(f"{tracing}=>package_cmakelists_update({package_path})")
    cmake_lists_text_path: Path = package_path / "CMakeLists.txt"

    lines: Tuple[str, ...] = ()
    assert cmake_lists_text_path.exists(), f"'{cmake_lists_text_path}' does not exist"
    cmake_lists_file: IO[str]
    with open(cmake_lists_text_path) as cmake_lists_file:
        lines = tuple(cmake_lists_file.read().split("\n"))

    message_paths: Tuple[Path, ...] = tuple(sorted(package_path.glob("*/*.msg")))
    service_paths: Tuple[Path, ...] = tuple(sorted(package_path.glob("*/*.srv")))

    updated_lines: List[str] = []
    line: str
    delete_mode: bool = False
    for line in lines:
        if delete_mode:
            # Delete lines until the closing `'` is found:
            if line.endswith(")"):
                delete_mode = False
        elif line.startswith("find_package(rosidl_default_generators REQUIRED)"):
            # Delete stale stuff
            delete_mode = True
        elif line.startswith("ament_package()"):
            # Append the `find_package(...) right before `ament_package()`:
            updated_lines.append("find_package(rosidl_default_generators REQUIRED)")
            updated_lines.append("")
            updated_lines.append("rosidl_generate_interfaces(${PROJECT_NAME}")

            # Append the message names:
            message_path: Path
            for message_path in message_paths:
                updated_lines.append(f'  "msg/{message_path.name}"')

            # Append the service names:
            service_path: Path
            for service_path in service_paths:
                updated_lines.append(f'  "srv/{service_path.name}"')
                updated_lines.append(")")

            # Append the `ament_package()`:
            updated_lines.append("")
            updated_lines.append(line)
        else:
            # Some other line.
            updated_lines.append(line)

    updated_contents: str = "\n".join(updated_lines)

    # For debugging only:
    # print("================================================================")
    # print(f"{cmake_lists_text_path}:")
    # print(updated_contents)
    # print("================================================================")

    # Update the file contents:
    with open(cmake_lists_text_path, "w") as cmake_lists_file:
        cmake_lists_file.write(updated_contents)

    if tracing:
        print(f"{tracing}<=package_cmakelists_update({package_path})")


def package_xml_update(package_path: Path, import_names: Set[str],
                       description: str, tracing: str = "") -> None:
    """Update the package.xml file."""
    # Update a ROS2 XML package file contents.
    # next_tracing: str = tracing + " " if tracing else ""
    if tracing:
        print(f"{tracing}=>package_xml_update('{package_path}', {import_names}, '{description}')")

    maintainer: str = maintainer_get()
    email: str = email_get()
    xml_path: Path = package_path / "package.xml"
    msg_path: Path = package_path / "msg"
    srv_path: Path = package_path / "srv"
    rosidl_needed: bool = msg_path.is_dir() or srv_path.is_dir()
    if tracing:
        print(f"{tracing}xml_path='{xml_path}'")
        print(f"{tracing}msg_path='{msg_path}'")
        print(f"{tracing}srv_path='{srv_path}'")
        print(f"{tracing}rosidl_needed={rosidl_needed}")

    # Reading xml_lines.
    xml_file: IO[str]
    xml_lines: List[str] = []
    with open(xml_path) as xml_file:
        xml_lines = xml_file.read().split('\n')

    # Find the <depend>DEPEND_NAME</depend> entries.
    depend_names: Set[str] = set()
    xml_line: str
    for xml_line in xml_lines:
        if xml_line.startswith("  <depend>") and xml_line.endswith("</depend>"):
            depend_name: str = xml_line[10:-9]
            depend_names.add(depend_name)

    # Remove duplicates from *import_names*:
    import_names -= depend_names

    updated_xml_lines: List[str] = []
    for xml_line in xml_lines:
        if xml_line.startswith("  <description>"):
            updated_xml_lines.append(f"  <description>{description}</description>")
        elif xml_line.startswith("  <maintainer email="):
            updated_xml_lines.append(f'  <maintainer email="{email}">{maintainer}</maintainer>')
        elif xml_line.startswith("  <license>"):
            updated_xml_lines.append("  <license>MIT</license>")
            import_name: str
            for depend_name in sorted(depend_names):
                updated_xml_lines.append(f"  <depend>{depend_name}</depend>")
            for import_name in sorted(import_names):
                updated_xml_lines.append(f"  <exec_depend>{import_name}</exec_depend>")
            if rosidl_needed:
                updated_xml_lines.extend([
                    "  <build_depend>rosidl_default_generators</build_depend>",
                    "  <exec_depend>rosidl_default_runtime</exec_depend>",
                    "  <member_of_group>rosidl_interface_packages</member_of_group>",
                ])
        elif xml_line.startswith("  <depend>"):
            pass  # Delete this previously installed line.
        elif xml_line.startswith("  <exec_depend>"):
            pass  # Delete this previously installed line.
        elif xml_line.startswith("  <build_depend>rosidl_default_generators</build_depend>"):
            pass  # Delete this previously installed line.
        elif xml_line.startswith("  <exec_depend>rosidl_default_runtime</exec_depend>"):
            pass  # Delete this previously installed line.
        elif xml_line.startswith("  <member_of_group>rosidl_interface_packages</member_of_group>"):
            pass  # Delete this previously installed line.
        else:
            updated_xml_lines.append(xml_line)

    # Write out everything.
    updated_xml: str = "\n".join(updated_xml_lines)
    with open(xml_path, "w") as xml_file:
        xml_file.write(updated_xml)

    # For debugging only:
    print("================================================================")
    print(f"{xml_path}")
    print(updated_xml)
    print("================================================================")

    if tracing:
        print(f"{tracing}<=package_xml_update('{xml_path}', {import_names}, '{description}')")


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


def setup_update(package_path: Path, description: str,
                 label_entries: Dict[str, str], tracing: str = "") -> None:
    """Update the setup.py file."""
    # next_tracing: str = tracing + " " if tracing else ""
    if tracing:
        print(f"{tracing}=>setup_update({package_path}, '{description}', {label_entries})")

    setup_path: Path = package_path / "setup.py"
    assert setup_path.exists(), f"{setup_path} does not exist"

    # Reading in the *setup_lines* of code from *setup_path*:
    setup_file: IO[str]
    setup_lines: List[str] = []
    with open(setup_path) as setup_file:
        setup_lines = setup_file.read().split('\n')

    maintainer: str = maintainer_get()
    email: str = email_get()

    package_name: str = package_path.name
    updated_setup_lines: List[str] = []
    setup_line: str
    for setup_line in setup_lines:
        if setup_line.startswith("    maintainer="):
            updated_setup_lines.append(f"    maintainer='{maintainer}',")
        elif setup_line.startswith("    maintainer_email="):
            updated_setup_lines.append(f"    maintainer_email='{email}',")
        elif setup_line.startswith("    description="):
            updated_setup_lines.append(f"    description='{description}',")
        elif setup_line.startswith("    license="):
            updated_setup_lines.append("    license='MIT',")
        elif setup_line.startswith("        'console_scripts': ["):
            updated_setup_lines.append(setup_line)
            label: str
            module_name: str
            for label, module_name in label_entries.items():
                updated_setup_lines.append(
                    f"            '{label} = {package_name}.{module_name}:main',")
        elif setup_line.endswith(":main',"):
            pass  # Delete any prexisting entry line
        else:
            updated_setup_lines.append(setup_line)

    updated_setup: str = "\n".join(updated_setup_lines)
    with open(setup_path, "w") as setup_file:
        setup_file.write(updated_setup)

    # For debugging only:
    # print("================================================================")
    # print(f"{setup_path}")
    # print(updated_setup)
    # print("================================================================")

    if tracing:
        print(f"{tracing}<=setup_update({package_path}, '{description}', {label_entries})")


def python_label_entries_find(
        python_paths: Tuple[Path, ...], tracing: str = "") -> Dict[str, str]:
    """Scan python files for entry label pairs."""
    # next_tracing: str = tracing + " " if tracing else ""
    if tracing:
        print(f"{tracing}=>python_entries_find({python_paths})")

    label_entries: Dict[str, str] = {}
    python_path: Path
    for python_path in python_paths:
        label: str = ""
        python_module_name: str = ""
        python_file: IO[str]
        with open(python_path) as python_file:
            python_line: str
            for python_line in python_file.read().split("\n"):
                if python_line.startswith("def main("):
                    python_file_name: str = python_path.name
                    assert python_file_name.endswith(".py"), (
                        f"{python_file_name} does not end in .py")
                    python_module_name = python_path.name[:-3]
                elif python_line.find("create_client(") >= 0:
                    label = "client"
                elif python_line.find("create_publisher(") >= 0:
                    label = "talker"
                elif python_line.find(".declare_parameter(") >= 0:
                    label = "param_talker"
                elif python_line.find("create_service(") >= 0:
                    label = "service"
                elif python_line.find("create_subscription(") >= 0:
                    label = "listener"
        if label and python_module_name:
            assert label not in label_entries, f"Duplicate lable '{label}'."

            label_entries[label] = python_module_name

    if tracing:
        print(f"{tracing}<=python_entries_find({python_paths} => {label_entries}")
    return label_entries


if __name__ == "__main__":
    main(tracing=" ")

# [ROS Python Launch Files](https://www.youtube.com/watch?v=RDoig5qEHRM)

# Note that PKG only occurs onece
# mkdir -p WS/PKG/launch
# cat >> WS/src/PKG/launch/launch.py <<EOF  # Note it is not PKG/PKG
# from launch import LaunchDescription
# from launch_ros.actions import Node
#
# def generate_launch_description():
#    return LaunchDescription([
#        Node(
#           package="py_pubsub",
#           executable="talker",
#           output="screen",
#        ),
#        Node(
#           package="py_pubsub",
#           executable="listener",
#           output="screen",
#        ),
#    ])
# EOF

# in setyp.py
# Add: imports:
# from setuptools import setup  # Already presetn
# import os  # For os is for os.join() only
# from glob import glob
# 
# in data_files add:
#    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

# Run `colcon build`
#
# source intall/setup.bash
# ros2 launch py_pubsub launch.py

# [https://roboticsbackend.com/ros2-launch-file-example/](https://roboticsbackend.com/ros2-launch-file-example/)

# [Writing a ROS 2 launch file](https://github.com/bponsler/ros2-support/blob/master/tutorials/ros2-launch.md)

# [ROS2 Launch File Migrator](https://github.com/aws-robotics/ros2-launch-file-migrator)
# Look at the test fixtures.

# [ROS1 roslaunch](http://wiki.ros.org/roslaunch)
# [ROS1 roslaunch XML](http://wiki.ros.org/roslaunch/XML/launch)
