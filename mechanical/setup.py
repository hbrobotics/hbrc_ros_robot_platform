# MIT License
#
# Copyright (c) 2019 Wayne C. Gramlich
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# <======================================= 100 characters =======================================> #
import os
import setuptools  # type: ignore
from typing import Any, Dict, IO

def long_description_read() -> str:
    readme_file: IO[Any]
    with open("README.md") as readme_file:
        long_description: str = readme_file.read()
    return long_description


# Arguments to *setup*() are in alphabetical order:
setuptools.setup(
    author="Wayne Gramlich",
    author_email="Wayne@Gramlich.Net",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    description="Bill Of Materials Manager",
    entry_points={
        "console_scripts": [
            "scad_models=scad_models.scad_models:main",
        ],
    },
    include_package_data=True,
    install_requires=([]),
    license="MIT",
    long_description=long_description_read(),
    long_description_content_type="text/markdown",
    name=("scad_models"),
    packages=[
        "scad_models",
    ],
    python_requires=">=3.6",
    url="https://github.com/hbrobotics/hbrc_ros_robot_platform",
    version="0.0.1",
)
