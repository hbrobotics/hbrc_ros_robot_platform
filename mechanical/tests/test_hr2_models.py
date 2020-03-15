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

from math import cos, pi, sin
from scad_models.hr2_models import (BaseDXF, HR2Robot, OtherPi,
                                    RaspberryPi3, RectangularConnector,
                                    RomiExpansionPlate, RomiMotor, Spacer)
from scad_models.scad import (Color, CornerCube, Difference3D, LinearExtrude, Module3D,
                              P2D, P3D, Polygon, Scad3D, ScadProgram, Square)
from typing import Any, IO, List, Tuple


# test_hr2_robot():
def test_hr2_robot():
    """Test the HR2 class."""
    scad_program: ScadProgram = ScadProgram("Top Level Program")
    hr2_robot: HR2Robot = HR2Robot(scad_program)
    hr2_robot = hr2_robot


# test_raspi3b():
def test_raspi3b():
    """Test RaspberryPi3 class."""
    scad_program: ScadProgram = ScadProgram("Top Level Program")
    raspi3b: RaspberryPi3 = RaspberryPi3(scad_program)
    raspi3b = raspi3b


# test_rectangular_connector():
def test_rectangular_connector():
    """Test RectangularConnector class."""
    # Create a *scad_program* to stuff everything into:
    scad_program: ScadProgram = ScadProgram("Top Level Program")

    # Create the *pcb_polygon*:
    pcb_polygon: Polygon = Polygon("PCB Polygon", [], lock=False)

    # Create an *pcb_exterior* and append to *pcb_polygon*:
    pcb_exterior: Square = Square("PCB Exterior Square", 100.0, 100.0, center=P2D(45.0, 45.0))
    pcb_polygon.append(pcb_exterior)

    offset_pitch: float = 10.0
    rectangular_connectors: List[Scad3D] = []
    pcb_height: float = 1.0

    # Do of *vertical_rotate* of 0, 45, and 90 degrees:
    vertical_rotate: float
    for vertical_rotate in (0.0, pi / 4.0, pi / 2.0):
        vertical_name: str = str(int(vertical_rotate * 180.0 / pi))
        position: int = 1
        for is_top in (True, False):
            z: float = pcb_height if is_top else 0.0
            top_name: str = 'T' if is_top else 'B'
            male_female_index: int
            for male_female_index, male_pin_height in enumerate([0.0, 2.54]):
                male_female_name: str = "MF"[male_female_index]
                right_angle_length: float
                right_angle_index: int
                for right_angle_index, right_angle_length in enumerate([0.0, 3.0]):
                    # See origin rotate math above:
                    offset: float = float(position) * offset_pitch
                    position += 1
                    x: float = offset * cos(vertical_rotate)
                    y: float = offset * sin(vertical_rotate)
                    right_angle_name: str = "VR"[right_angle_index]
                    full_name = f"{top_name}{vertical_name}{right_angle_name}{male_female_name}"
                    center: P3D = P3D(x, y, z)

                    # Create the rectangular connector:
                    rectangular_connector: RectangularConnector
                    rectangular_connector = RectangularConnector(scad_program, f"{full_name}",
                                                                 1, 2, 2.54, 2.54,
                                                                 male_pin_height=male_pin_height,
                                                                 center=center,
                                                                 cut_out=True,
                                                                 right_angle_length=(
                                                                     right_angle_length),
                                                                 vertical_rotate=vertical_rotate,
                                                                 is_top=is_top,
                                                                 insulation_color="Orange",
                                                                 pcb_polygon=pcb_polygon)
                    rectangular_connectors.append(rectangular_connector.module.use_module_get())

    # Create the final *green_pcb* from *pcb_polygon* by extruding, trimming and coloring:
    pcb_polygon.lock()
    extruded_pcb: LinearExtrude = LinearExtrude("Extruded PCB", pcb_polygon, 1.0)
    # Trim off the edges along the edges of the PCB to expose the PCB holes.
    south_trim_corner_cube: CornerCube = CornerCube("South Trim Corner Cube",
                                                    P3D(-20.0, -10.0, -1.0),
                                                    P3D(100.0, 0.0, 2.0))
    west_trim_corner_cube: CornerCube = CornerCube("West Trim Corner Cube",
                                                   P3D(-10.0, -20.0, -1.0),
                                                   P3D(0.0, 100.0, 2.0))
    trimmed_pcb: Difference3D = Difference3D("Trimmed PCB", extruded_pcb,
                                             [south_trim_corner_cube, west_trim_corner_cube])
    green_pcb: Color = Color("Green PCB", trimmed_pcb, "Green")

    # Create final *module* and append to *scad_program*:
    module: Module3D = Module3D("Rectangular Module",
                                [green_pcb] + rectangular_connectors, lock=True)
    scad_program.append(module)
    scad_program.if3d.name_match_append("connectors", module, ["Connectors PCB"])
    scad_lines: List[str] = []
    scad_program.scad_lines_append(scad_lines, "")
    scad_lines.append("")
    scad_program_text: str = '\n'.join(scad_lines)
    scad_program_file: IO[Any]
    with open("rectangular_conector.scad", "w") as scad_program_file:
        scad_program_file.write(scad_program_text)


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


# test_spacer():
def test_spacer():
    """Test Spacer Class."""
    scad_program: ScadProgram = ScadProgram("Top Level Program")
    washers: List[Tuple[float, float, str]] = [
        (1.0, 3.5, "Green"),
        (0.5, 4.0, "")
    ]
    spacer: Spacer = Spacer(scad_program, "Test Spacer with Washers",
                            10.0, "M2.5", bottom_washers=washers, top_washers=washers,
                            top_height=5.0)
    spacer = spacer


# test_other_pi():
def test_other_pi() -> None:
    """Test OtherPi class."""
    scad_program: ScadProgram = ScadProgram("Top Level program")
    other_pi: OtherPi = OtherPi(scad_program)
    other_pi = other_pi
