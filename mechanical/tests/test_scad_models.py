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

from scad_models.scad_models import (BaseDXF, EncoderBoard, HR2, MasterBoard, OtherPi,
                                     RaspberryPi3, RomiBase, RomiExpansionPlate, RomiMagnet,
                                     RomiMotor, RomiMotorHolder, RomiWheelAssembly)
from scad_models.scad import (P3D, ScadProgram)
from typing import Any, List, Tuple


# test_hr2():
def test_hr2():
    """Test the HR2 class."""
    scad_program: ScadProgram = ScadProgram("Top Level Program")
    base_dxf: BaseDXF = BaseDXF()
    pi_offset: P3D = P3D(0.0, 0.0, 0.0)
    romi_base_keys: List[Tuple[Any, ...]] = [
        ("Circle", "BATTERY: Upper Hole (0, 1)", -35.000, 17.000, 2.300, 2.3000)
    ]
    master_board: MasterBoard = MasterBoard(scad_program, base_dxf, pi_offset, romi_base_keys)
    other_pi: OtherPi = OtherPi(scad_program)
    romi_base: RomiBase = RomiBase(scad_program, base_dxf)
    romi_magnet: RomiMagnet = RomiMagnet(scad_program, base_dxf)
    romi_base.holes_slots_rectangles_write()
    romi_motor_holder: RomiMotorHolder = RomiMotorHolder(scad_program, base_dxf)
    encoder_board: EncoderBoard = EncoderBoard(scad_program, base_dxf)
    romi_wheel_assembly: RomiWheelAssembly = RomiWheelAssembly(scad_program, romi_base,
                                                               romi_motor_holder, romi_magnet,
                                                               encoder_board)
    hr2: HR2 = HR2(scad_program, romi_base, romi_wheel_assembly, master_board, other_pi, pi_offset)
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
    base_dxf: BaseDXF = BaseDXF()
    romi_motor: RomiMotor = RomiMotor(scad_program, base_dxf)
    romi_motor = romi_motor


# test_other_pi():
def test_other_pi() -> None:
    """Test OtherPi class."""
    scad_program: ScadProgram = ScadProgram("Top Level program")
    other_pi: OtherPi = OtherPi(scad_program)
    other_pi = other_pi
