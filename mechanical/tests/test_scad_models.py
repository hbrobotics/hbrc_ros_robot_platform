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

from scad_models.scad_models import (HR2, MasterPCB, OtherPi, RaspberryPi3, RomiBase,
                                     RomiExpansionPlate, RomiMagnet, RomiMotor, RomiMotorHolder,
                                     RomiWheelAssembly)
from scad_models.scad import ScadProgram


# test_hr2():
def test_hr2():
    """Test the HR2 class."""
    scad_program: ScadProgram = ScadProgram("Top Level Program")
    master_pcb: MasterPCB = MasterPCB(scad_program)
    other_pi: OtherPi = OtherPi(scad_program)
    romi_base: RomiBase = RomiBase(scad_program)
    romi_magnet: RomiMagnet = RomiMagnet(scad_program)
    romi_base.holes_slots_rectangles_write()
    romi_motor_holder: RomiMotorHolder = RomiMotorHolder(scad_program)
    romi_wheel_assembly: RomiWheelAssembly = RomiWheelAssembly(scad_program, romi_base,
                                                               romi_motor_holder, romi_magnet)
    hr2: HR2 = HR2(scad_program, romi_base, romi_wheel_assembly, other_pi, master_pcb)
    hr2 = hr2


# test_raspi3b():
def test_raspi3b():
    """Test RaspberryPi3 class."""
    scad_program: ScadProgram = ScadProgram("Top Level Program")
    raspi3b: RaspberryPi3 = RaspberryPi3(scad_program)
    raspi3b = raspi3b


# test_romi_expansion_plate():
def test_romi_expansion_plate():
    """Test RomiExpanstionPlate class."""
    scad_program: ScadProgram = ScadProgram("Top Level Program")
    romi_expansion_plate: RomiExpansionPlate = RomiExpansionPlate(scad_program)
    romi_expansion_plate = romi_expansion_plate


# test_romi_motor():
def test_romi_motor():
    """Test RomiMotor class."""
    scad_program: ScadProgram = ScadProgram("Top Level Program")
    romi_motor: RomiMotor = RomiMotor(scad_program)
    romi_motor = romi_motor


# test_other_pi():
def test_other_pi() -> None:
    """Test OtherPi class."""
    scad_program: ScadProgram = ScadProgram("Top Level program")
    other_pi: OtherPi = OtherPi(scad_program)
    other_pi = other_pi
