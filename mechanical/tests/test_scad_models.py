"""test_romi_mode: Unit tests for the Romi stuff."""

# Copyright (c) 2019 Wayne C. Gramlich (Wayne@Gramlich.Net)
#
# MIT License
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

from scad_models.scad_models import OtherPi, RaspberryPi3, Romi
from scad_models.scad import If2D, If3D, ScadProgram


# test_raspi3b():
def test_raspi3b():
    """Test RaspberryPi3 class."""
    raspi3b: RaspberryPi3 = RaspberryPi3()
    scad_program: ScadProgram = ScadProgram("Top Level program")
    if2d: If2D = If2D("If2D If/Then/Else", "false", [])
    if3d: If3D = If3D("If3D If/Then/Else", "false", [])
    raspi3b.scad_program_append(scad_program, if2d, if3d)


# test_romi():
def test_romi():
    """Test the Romi class."""
    romi: Romi = Romi()
    romi.debugging = False
    scad_program: ScadProgram = ScadProgram("Top Level program")
    if2d: If2D = If2D("If2D If/Then/Else", "false", [])
    if3d: If3D = If3D("If3D If/Then/Else", "false", [])
    romi.scad_program_append(scad_program, if2d, if3d)
    romi.holes_slots_rectangles_write()


# test_other_pi():
def test_other_pi() -> None:
    """Test OtherPi class."""
    other_pi: OtherPi = OtherPi()
    scad_program: ScadProgram = ScadProgram("Top Level program")
    if2d: If2D = If2D("If2D If/Then/Else", "false", [])
    if3d: If3D = If3D("If3D If/Then/Else", "false", [])
    other_pi.scad_program_append(scad_program, if2d, if3d)
