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
                                     RaspberryPi3, RectangularConnector, RomiBase,
                                     RomiExpansionPlate, RomiMagnet, RomiMotor,
                                     RomiMotorHolder, RomiWheelAssembly)
from scad_models.scad import (Color, CornerCube, Difference3D, LinearExtrude, Module3D,
                              P2D, P3D, Polygon, Scad3D, ScadProgram, Square)
from typing import Any, IO, List, Tuple


# test_hr2():
def test_hr2():
    """Test the HR2 class."""
    scad_program: ScadProgram = ScadProgram("Top Level Program")
    base_dxf: BaseDXF = BaseDXF()
    pi_offset: P3D = P3D(0.0, 0.0, 0.0)
    romi_base_keys: List[Tuple[Any, ...]] = [
        ("Circle", "BATTERY: Upper Hole (0, 1)", -35.000, 17.000, 2.300, 2.3000)
    ]
    master_board_dz: float = 0.0
    master_board: MasterBoard = MasterBoard(scad_program, base_dxf,
                                            master_board_dz, pi_offset, romi_base_keys)
    other_pi: OtherPi = OtherPi(scad_program)
    romi_base: RomiBase = RomiBase(scad_program, base_dxf)
    romi_magnet: RomiMagnet = RomiMagnet(scad_program, base_dxf)
    romi_base.holes_slots_rectangles_write()
    romi_motor: RomiMotor = RomiMotor(scad_program, base_dxf)
    romi_motor_holder: RomiMotorHolder = RomiMotorHolder(scad_program, base_dxf)
    east_encoder_board: EncoderBoard = EncoderBoard(scad_program, base_dxf, True)
    east_romi_wheel_assembly: RomiWheelAssembly
    east_romi_wheel_assembly = RomiWheelAssembly(scad_program, "East Wheel Assembly",
                                                 romi_motor, romi_motor_holder,
                                                 romi_magnet, east_encoder_board)
    west_encoder_board: EncoderBoard = EncoderBoard(scad_program, base_dxf, False)
    west_romi_wheel_assembly: RomiWheelAssembly
    west_romi_wheel_assembly = RomiWheelAssembly(scad_program, "West Wheel Assembly",
                                                 romi_motor, romi_motor_holder,
                                                 romi_magnet, west_encoder_board)
    hr2: HR2 = HR2(scad_program, romi_base, east_romi_wheel_assembly, west_romi_wheel_assembly,
                   master_board, other_pi, pi_offset)
    hr2 = hr2


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

    connector_quad_pitch: float = 25.0
    rectangular_connectors: List[Scad3D] = []
    pcb_height: float = 1.0
    is_vertical: bool
    for is_vertical in (True, False):
        vertical_name: str = 'V' if is_vertical else 'H'
        dx: float = 0.0 if is_vertical else connector_quad_pitch
        dy: float = connector_quad_pitch if is_vertical else 0.0
        right_angle_length: float
        for is_top in (True, False):
            z: float = pcb_height if is_top else 0.0
            top_name: str = 'T' if is_top else 'B'
            dx_dy_adjust: float = -0.5 + (0.0 if is_top else 0.5)
            male_female_index: int
            for male_female_index, male_pin_height in enumerate([0.0, 2.54]):
                male_female_name: str = "MF"[male_female_index]
                if male_pin_height > 0.0:
                    dx_dy_adjust += 0.25
                right_angle_index: int
                index: int
                for right_angle_index, right_angle_length in enumerate([0.0, -3.0, 3.0]):
                    x: float = (float(right_angle_index + 1) + dx_dy_adjust) * dx
                    y: float = (float(right_angle_index + 1) + dx_dy_adjust) * dy
                    right_angle_name: str = "LCR"[right_angle_index]
                    full_name = f"{top_name}{vertical_name}{right_angle_name}{male_female_name}"
                    center: P3D = P3D(x, y, z)

                    # Create the rectangular connector:
                    rectangular_connector: RectangularConnector
                    rectangular_connector = RectangularConnector(scad_program, f"{full_name}",
                                                                 1, 2, 2.54, 2.54,
                                                                 male_pin_height=male_pin_height,
                                                                 center=center,
                                                                 right_angle_length=(
                                                                     right_angle_length),
                                                                 is_vertical=is_vertical,
                                                                 is_top=is_top,
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


# test_other_pi():
def test_other_pi() -> None:
    """Test OtherPi class."""
    scad_program: ScadProgram = ScadProgram("Top Level program")
    other_pi: OtherPi = OtherPi(scad_program)
    other_pi = other_pi
