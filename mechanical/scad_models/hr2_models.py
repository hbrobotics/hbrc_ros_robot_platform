# This file is licensed using the "MIT License" below:
#
# ##################################################################################################
#
# MIT License
#
# Copyright 2019, 2020 Home Brew Robotics Club
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
#
# ##################################################################################################
# <======================================= 100 characters =======================================> #
"""Code genarates an OpenSCAD model for HR2 (HBRC ROS Robot)."""

from scad_models.scad import (Color, Circle, CornerCube, Cylinder, If2D, Difference2D, KicadPcb,
                              LinearExtrude, Module2D, Module3D, P2D, P3D, Polygon, Rotate3D,
                              Scad2D, Scad3D, SimplePolygon, ScadProgram, Square,
                              Translate3D, UseModule3D, Union3D, Variable2D)
from typing import Any, Dict, IO, List, Optional, Set, Tuple
from math import acos, asin, atan2, cos, degrees, nan, pi, sin, sqrt


# DXF:
class DXF:
    """Represents a .DXF file for getting dimensions from.

    Pololu provides `.dxf` files to grab locations out of.  This class
    basically provides a way to convert points read directly out of a
    `.dxf` file and convert them to points in the correct units (mm
    rather than inches) and offset such that the robot origin is at the
    center of the base circle and at the center of the wheel shaft.

    Figuring out where everything is located is done with a combination
    of reading drawing in section 6 of the "Pololu Romi Chassis User's
    Guide" and extracting values from the `romi-chassis.dxf` file
    available from the Pololu web site.  `romi-chassis.dxf` is also
    stored in the git repository to make sure it does not go away.

    It is important to notice that the section 6 drawings are
    upside-down from the `romi-chassis.dxf`.  Sigh.  For the modeling
    of this platform, the `.dxf` file orientation is used.  Sigh.
    Also, the `.dxf` file seems to be in units of inches rather than
    millimeters.  The rest of this code uses millimeters, so we always
    immdiately convert the inches coordinate values to millimeters.

    There are two `.dxf` files:
    * `romi-chassis.dxf`: The primary `.dxf` file for the Romi chasis
       base assembly.
    * `romi-chassis-expansion-plate.dxf`: The `.dxf` file for the Romi
       expansion plate.
    Both of these files can be found in the `dxf` sub-directory.

    This class has 6 read-only attributes:
    * *inches2mm* (*float*): The constant 25.4 to convert inches to mm.
    * *name* (*str*): The name of the DXF object.
    * *offset_side_y* (*float*): The `.dxf` X coordinate of side view
      that corresponds to the Y "center".  For the base plate this
      corresponds to center of wheel axle.  For the expansion plate
      this corrsponds to the top long edge.  The expansion plate
      `.dxf` file uses the term "profile view" instead of "side view".
    * *offset_top_y* (*float*): The `.dxf` Y coordinate of corresponds
      to the Y "center" in the top view.  Again, this value corrsponds
      to the wheel axle center for the Romi Base and the top long edge
      of the expansion plate.
    * *offset_x* (*float*): The X coordinate of the X "center" the both
      the side and front views.  For the Romi base, this is the center
      of circular base and for the expansion plate, this is the center
      of the long edge.
    * *offset_z* (*float8): The Y corrdinate in the `.dxf` file for
      the Z "center" for both the front view and side (or profile)
      view.  For the Romi base this is center of the wheel axle and
      for the expansion base, this is bottom edge.

    The *DXF* class is sub-classed by *BaseDXF* and *ExpansionDXF*.
    The *__init__* methods for these two sub-classes comute the correct
    values for the attributes listed above.  Other that that the
    methods of the *DXF* class are used:
    * *hole_locate*: Locate the center and diameter of a hole and
      return it as a *Circle* object.
    * *point_locate*: Locate a trop view point and return it as *P2D*
      object.
    * *rectangle_locate*: Locoate a rectangle and return it as
      *Square* object.  The rectangle is identified using opposite
      corners.
    * *side_point_locate*: Locate a point on the side view.  The Y
      coordinate is stored in the *x* attribute and the Z coordinate
      is stored in the *y* attribute of the returned *P2D*.
    * *x_locate*: Locate an X coordinate from the top or front view.
    * *y_locate*: Locate a Y coordinate from the top view.
    * *z_locate*: Locate a Z coordinate from the front or side view.
    """

    # DXF.__init__():
    def __init__(self, name: str, offset_x: float, offset_top_y: float,
                 offset_side_y: float, offset_z: float) -> None:
        """Initialize a DXF object."""
        # Stuff arguments into *dxf* (i.e. *self*):
        # dxf: Dxf = self
        self.inches2mm: float = 25.4
        self.name: str = name
        self.offset_side_y: float = offset_side_y
        self.offset_top_y: float = offset_top_y
        self.offset_x: float = offset_x
        self.offset_z: float = offset_z

    # DXF.hole_locate():
    def hole_locate(self, name: str, dxf_x1: float, dxf_y1: float,
                    dxf_x2: float, dxf_y2: float) -> Circle:
        """Return a located top view hole using opposite corners."""
        # Grab some values from *dxf* (i.e. *self*):
        dxf: DXF = self
        x1: float = dxf.x_locate(dxf_x1)
        y1: float = dxf.y_locate(dxf_y1)
        x2: float = dxf.x_locate(dxf_x2)
        y2: float = dxf.y_locate(dxf_y2)

        # Compute *dx*, *dy*, *center*, and *diameter*:
        dx: float = abs(x2 - x1)
        dy: float = abs(y2 - y1)
        center: P2D = P2D((x1 + x2) / 2.0, (y1 + y2) / 2.0)
        diameter: float = (dx + dy) / 2.0

        # Construct and return the *hole*:
        hole: Circle = Circle(name, diameter, 8, center)
        return hole

    # DXF.point_locate():
    def point_locate(self, dxf_x: float, dxf_y: float) -> P2D:
        """Return a top view located point."""
        # Using *dxf* (i.e. *self*) convert *dxf_x* and *dxf_y* into a *point* and return it.
        dxf: DXF = self
        x: float = dxf.x_locate(dxf_x)
        y: float = dxf.y_locate(dxf_y)
        point: P2D = P2D(x, y)
        return point

    # DXF.rectangle_locate():
    def rectangle_locate(self, name: str, dxf_x1: float, dxf_y1: float,
                         dxf_x2: float, dxf_y2: float) -> Square:
        """Return a located rectangle on top view as a Square.

        Args:
            *name* (*str*): The name to assign to the returned *Square*.
            *dxf_x1* (*float*): One of sides of the rectangle in inches.
            *dxf_y1* (*float*): Either the top/bottom edge of the
                rectangle in inches.
            *dxf_x1* (*float*): The other sides of the rectangle in
                inches.
            *dxf_y2* (*float*): The other top/bottom edge of the
                rectangle in inches:

        Returns:
            (*Square*) Returns a *Square* that represents the rectangle.

        """
        # Use *dxf* (i.e. *self*) to convert *dxf_x1*, *dxf_y1*, *dxf_x2*, and *dxf_y2*
        # into *x1*, *y1*, *x2*, and *y2* (i.e. millemeters with corrector X/Y origin.
        dxf: DXF = self
        x1: float = dxf.x_locate(dxf_x1)
        y1: float = dxf.y_locate(dxf_y1)
        x2: float = dxf.x_locate(dxf_x2)
        y2: float = dxf.y_locate(dxf_y2)

        # Compute *dx*, *dy*, and *center*:
        dx: float = abs(x2 - x1)
        dy: float = abs(y2 - y1)
        center: P2D = P2D((x1 + x2) / 2.0, (y1 + y2) / 2.0)

        # Create and return the *rectangle* (which is represented as a *Square* object):
        rectangle: Square = Square(name, dx, dy, center)
        return rectangle

    # DXF.slot_locate()
    def slot_locate(self, name: str, dxf_x1: float, dxf_y1: float, dxf_x2: float, dxf_y2: float,
                    corner_count: int = 3) -> Square:
        """Return a horizontal/vertical slot with rounded corners."""
        dxf: DXF = self
        x1: float = dxf.x_locate(dxf_x1)
        y1: float = dxf.y_locate(dxf_y1)
        x2: float = dxf.x_locate(dxf_x2)
        y2: float = dxf.y_locate(dxf_y2)

        # Create the *slot*:
        dx: float = abs(x2 - x1)
        dy: float = abs(y2 - y1)
        center: P2D = P2D((x1 + x2) / 2.0, (y1 + y2) / 2.0)
        corner_radius: float = min(dx, dy) / 2.0
        slot: Square = Square(name, dx, dy, center=center,
                              corner_radius=corner_radius, corner_count=corner_count)
        return slot

    # DXF.x_locate():
    def x_locate(self, dxf_x: float) -> float:
        """Return a X coordinate from top or front view."""
        # Grab some values from *dxf* (i.e. *self*):
        dxf: DXF = self
        inches2mm: float = dxf.inches2mm
        offset_x: float = dxf.offset_x
        x: float = dxf_x * inches2mm - offset_x
        return x

    # DXF.y_locate():
    def y_locate(self, dxf_y: float) -> float:
        """Return a Y coordinate from top view."""
        # Grab some values from *dxf* (i.e. *self*):
        dxf: DXF = self
        inches2mm: float = dxf.inches2mm
        offset_top_y: float = dxf.offset_top_y
        y: float = dxf_y * inches2mm - offset_top_y
        return y

    # DXF.z_locate():
    def z_locate(self, dxf_y: float) -> float:
        """Return a Z coordinate from front/side view."""
        # Grab some values from *dxf* (i.e. *self*):
        dxf: DXF = self
        inches2mm: float = dxf.inches2mm
        offset_z: float = dxf.offset_z

        # Convert from inches to mm:
        z = dxf_y * inches2mm - offset_z
        return z


# BaseDXF:
class BaseDXF(DXF):
    """Represents the `romi-chasis.dxf` file."""

    # BaseDXF.__init__():
    def __init__(self):
        """Initialize BaseDXF for Romi Base Chassis."""
        # Compute *offset_x* using the two edges of the top-most castor slot:
        inches2mm: float = 25.4
        caster_west_x: float = -3.995811 * inches2mm
        caster_east_x: float = -3.739909 * inches2mm
        offset_x: float = (caster_west_x + caster_east_x) / 2.0

        # Compute *offset_top_y* which is the Y offset for the Top view using the motor shaft.
        # (Do not use the wheel shaft since it has a flat cut in it!):
        motor_shaft_north_y: float = 2.967165 * inches2mm
        motor_shaft_south_y: float = 2.908110 * inches2mm
        offset_top_y: float = (motor_shaft_north_y + motor_shaft_south_y) / 2.0

        # Compute the *offset_side_y* using the hole above the wheel shaft.  The wheel
        # shaft can not be used since it has a flat on it:
        motor_shaft_north_y: float = 3.610941 * inches2mm
        motor_shaft_south_y: float = 3.489772 * inches2mm
        offset_side_y: float = (motor_shaft_north_y + motor_shaft_south_y) / 2.0

        # Compute the *offset_z* using the wheel shaft.  The wheel shaft has a flat
        # on it, so this extracted using the side view.
        wheel_shaft_top_z = -2.759752 * inches2mm
        wheel_shaft_bottom_z = -2.643004 * inches2mm
        offset_z: float = (wheel_shaft_top_z + wheel_shaft_bottom_z) / 2.0
        wheel_shaft_diameter: float = abs(wheel_shaft_top_z - wheel_shaft_bottom_z)

        # Initialize the parent *DXF* class:
        super().__init__("Romi Base DXF", offset_x, offset_top_y, offset_side_y, offset_z)

        # Save *wheel_shaft_diameter* into *base_dxf* (i.e. *self*):
        # base_dxf: BaseDXF = self
        self.wheel_shaft_diameter: float = wheel_shaft_diameter


# ExpansionDXF:
class ExpansionDXF(DXF):
    """Represents the `romi-chasis-expansion-plate.dxf` file."""

    # ExpansionDXF.
    def __init__(self) -> None:
        """Initialze the ExpansionDXF for the Expansion Plate."""
        # Compute *offset_x* using the two outer-most side edges:
        inches2mm: float = 25.4
        side_west_x: float = -5.113197 * inches2mm
        side_east_x: float = 0.613193 * inches2mm
        offset_x: float = (side_west_x + side_east_x) / 2.0

        # Compute the *offset_top_y* using the top-most edge of the top view:
        offset_top_y: float = 3.808358 * inches2mm

        # Compute the *offset_side_y* using the right most edge of the Profile (i.e. side) View:
        offset_side_y: float = 4.940795 * inches2mm

        # Compute the *offset_z*: using the bottom most edge of the Profile (i.e. side) View:
        offset_z: float = -2.107961 * inches2mm

        # Initialize the parent *DXF* class:
        super().__init__("Romi Base DXF", offset_x, offset_top_y, offset_side_y, offset_z)


# EncoderBoard:
class EncoderBoard:
    """Represents a motor encoder board."""

    # EncoderBoard.__init__():
    def __init__(self, scad_program: ScadProgram, base_dxf: BaseDXF) -> None:
        """Initialize the EncoderBoard and append to ScadProgram."""
        # Grab some X/Y/Z coordinates from *base_dxf*:
        motor_casing_east_x: float = base_dxf.x_locate(-5.253299)
        motor_casing_north_y: float = base_dxf.y_locate(3.382512)
        north_electrical_north_y: float = base_dxf.y_locate(3.208303)
        north_electrical_south_y: float = base_dxf.y_locate(3.188610)
        motor_shart_north_y: float = base_dxf.y_locate(2.967165)
        motor_shart_south_y: float = base_dxf.y_locate(2.908110)
        south_electrical_north_y: float = base_dxf.y_locate(2.686650)
        south_electrical_south_y: float = base_dxf.y_locate(2.666957)
        motor_casing_south_y: float = base_dxf.y_locate(2.492748)
        motor_casing_top_z: float = base_dxf.z_locate(-1.658071)
        electrical_top_z: float = base_dxf.z_locate(-1.842126)
        electrical_bottom_z: float = base_dxf.z_locate(-1.926776)
        motor_casing_bottom_z: float = base_dxf.z_locate(-2.110835)

        # Compute some constants:
        north_electrical_dy: float = abs(north_electrical_north_y - north_electrical_south_y)
        north_electrical_y: float = (north_electrical_north_y + north_electrical_south_y) / 2.0
        south_electrical_dy: float = abs(south_electrical_north_y - south_electrical_south_y)
        south_electrical_y: float = (south_electrical_north_y + south_electrical_south_y) / 2.0
        electrical_dy: float = (north_electrical_dy + south_electrical_dy) / 2.0
        electrical_dz: float = abs(electrical_top_z - electrical_bottom_z)
        motor_casing_dy: float = abs(motor_casing_north_y - motor_casing_south_y)
        motor_casing_dz: float = abs(motor_casing_top_z - motor_casing_bottom_z)
        motor_shaft_diameter: float = (motor_shart_north_y + motor_shart_south_y) / 2.0
        motor_shaft_z: float = motor_casing_bottom_z + motor_casing_dz / 2.0
        pcb_translate: P3D = P3D(motor_casing_east_x, 0.0, motor_shaft_z)

        # The PCB is designed flat with the motor shaft in the center and then rotated on end
        # and translated into position.  This tends to swap X and Z coordinates:
        pcb_west_x: float = -motor_casing_dz / 2.0 - 3.0  # Note the X/Z coordinate swap
        pcb_east_x: float = motor_casing_dz / 2.0
        pcb_header_dx: float = 5.0
        pcb_corner_x: float = pcb_west_x + pcb_header_dx
        pcb_dy_extra: float = 7.0 * 2.54
        pcb_north_y: float = (motor_casing_dy + pcb_dy_extra) / 2.0
        pcb_north_corner_y: float = motor_casing_dy / 2.0
        pcb_south_corner_y: float = -pcb_north_corner_y
        pcb_south_y: float = -pcb_north_y

        self.pcb_north_y: float = pcb_north_y
        self.pcb_south_y: float = pcb_south_y
        pcb_height: float = 1.0
        pcb_shaft_diameter_extra: float = 3.0
        pcb_shaft_hole_diameter: float = motor_shaft_diameter + pcb_shaft_diameter_extra
        pcb_slot_extra: float = 0.400
        pcb_slot_dx: float = electrical_dz + pcb_slot_extra
        pcb_slot_dy: float = electrical_dy + pcb_slot_extra
        pcb_north_slot_center: P2D = P2D(0.0, north_electrical_y)
        pcb_south_slot_center: P2D = P2D(0.0, south_electrical_y)

        # Create the *pcb_extorior* to allow enough space for the *MasterBoard* to slide up
        # to approximately the motor shaft Z:
        #
        #    A---H
        #    |   |
        #    |   G--F
        #    |      |
        #    |   D--E
        #    |   |
        #    B---C
        #
        pcb_exterior: SimplePolygon = SimplePolygon("Encoder PCB Exterior Polygon", [
            P2D(pcb_west_x, pcb_north_y),           # A
            P2D(pcb_west_x, pcb_south_y),           # B
            P2D(pcb_corner_x, pcb_south_y),         # C
            P2D(pcb_corner_x, pcb_south_corner_y),  # D
            P2D(pcb_east_x, pcb_south_corner_y),    # E
            P2D(pcb_east_x, pcb_north_corner_y),    # F
            P2D(pcb_corner_x, pcb_north_corner_y),  # G
            P2D(pcb_corner_x, pcb_north_y)          # H
        ], lock=True)

        # Create the *pcb_polygon* centered with the motor shaft hole at the origin (0,0):
        pcb_shaft_hole: Circle = Circle("Encoder Shaft Hole", pcb_shaft_hole_diameter, 16)
        pcb_north_electrical_slot: Square = Square("Encoder North Electrical Slot",
                                                   pcb_slot_dx, pcb_slot_dy,
                                                   center=pcb_north_slot_center,
                                                   corner_radius=pcb_slot_extra/2.0,
                                                   corner_count=1)
        pcb_south_electrical_slot: Square = Square("Encoder South Electrical Slot",
                                                   pcb_slot_dx, pcb_slot_dy,
                                                   center=pcb_south_slot_center,
                                                   corner_radius=pcb_slot_extra/2.0,
                                                   corner_count=1)
        # Create the *pcb_polygon* from the *pcb_exterior*, *pcb_shaft_hole*,
        # *pcb_north_electrical_slot* and *pcb_south_electrical_slot*.  Leave it unlocked
        # so that the electrical connectors can be added:
        pcb_polygon: Polygon = Polygon("Encoder PCB Polygon", [
            pcb_exterior,
            pcb_shaft_hole,
            pcb_north_electrical_slot,
            pcb_south_electrical_slot
        ], lock=False)

        # Create *north_header* and *south_header* for (Digikey: 2057-PH1RB-03-UA-ND (Adam Tech))
        # and make sure the header pin holes are appended to *pcb_polygon*:
        offset: float = 1.5 * 2.54
        degrees90: float = pi / 2.0
        north_header_center: P3D = P3D(pcb_west_x + 0.5 * 2.54, pcb_north_y - offset, 0.0)
        north_header = RectangularConnector(scad_program, "North Encoder Header",
                                            1, 3, 2.50, 3.90 - 2.50, male_pin_height=6.00,
                                            right_angle_length=(3.05 + 0.127),
                                            center=north_header_center,
                                            insulation_color="SkyBlue",
                                            pcb_polygon=pcb_polygon,
                                            vertical_rotate=degrees90, is_top=False)
        north_header_use_module: Scad3D = north_header.module.use_module_get()

        south_header_center: P3D = P3D(pcb_west_x + 0.5 * 2.54, pcb_south_y + offset, 0.0)
        south_header = RectangularConnector(scad_program, "South Encoder Header",
                                            1, 3, 2.50, 3.90 - 2.50, male_pin_height=6.00,
                                            right_angle_length=(3.05 + 0.127),
                                            center=south_header_center,
                                            insulation_color="Lime",
                                            pcb_polygon=pcb_polygon,
                                            vertical_rotate=degrees90, is_top=False)
        south_header_use_module: Scad3D = south_header.module.use_module_get()

        # Now that the connector holes have been added to *pcb_polygon*, it can be locked,
        # turned into a *encode_pcb_module*, append it to *scad_program* and make
        # it independently viewable as a 2D object:
        pcb_polygon.lock()
        encoder_pcb_module: Module2D = Module2D("Encoder PCB Module", [pcb_polygon])
        scad_program.append(encoder_pcb_module)
        scad_program.if2d.name_match_append("encoder_pcb", encoder_pcb_module, ["Encoder PCB"])

        # Now create *encoder_pcb* and *colored_pcb*:
        extruded_pcb: LinearExtrude = LinearExtrude("Extruded Encoder PCB", pcb_polygon, pcb_height)
        colored_pcb: Scad3D = Color("Colored Encoder PCB", extruded_pcb, "Fuchsia")

        # Create the *encoder_union* so we can do the final rotate and translate operations:
        encoder_union: Union3D = Union3D("Encoder Union", [
            colored_pcb,
            north_header_use_module,
            south_header_use_module,
        ])

        # Now rotate to vertical and traslate to the left motor:
        rotated_encoder_pcb: Rotate3D = Rotate3D("Rotated Encoder PCB", encoder_union,
                                                 pi/2.0, P3D(0.0, 1.0, 0.0))
        translated_encoder_pcb: Translate3D = Translate3D("Translated Encoder PCB",
                                                          rotated_encoder_pcb,
                                                          pcb_translate)

        # Create *module*, append to *scad_program*, and save into *encoder_board* (i.e. *self*):
        module: Module3D = Module3D("EncoderBoard Module", [
            translated_encoder_pcb,
        ])
        scad_program.append(module)
        scad_program.if3d.name_match_append("encoder_board", module, ["Encoder Board"])
        # encoder_board: EncoderBoard = self
        self.module: Module3D = module


# HCSR04:
class HCSR04:
    """Represents an HC-SR04 sonar module."""

    # HCSR04.__init__():
    def __init__(self, scad_program: ScadProgram) -> None:
        """Initialize anHCSR04."""
        # Create the PCB:
        pcb_dy: float = 20.60
        pcb_dx: float = 44.50
        pcb_dz: float = 1.20
        pcb_square: Square = Square("HCSR04 PCB", pcb_dx, pcb_dy, center=P2D(0.0, pcb_dy / 2.0))
        extruded_pcb: LinearExtrude = LinearExtrude("Extruded HCSR04 PCB", pcb_square, pcb_dz)
        colored_pcb: Color = Color("Colored HRCSR04", extruded_pcb, "Green")

        # Create the sonar cylinders:
        sonar_diameter: float = 16.00
        sonar_radius: float = sonar_diameter / 2.0
        sonar_dz: float = 12.00
        sonar_left_x: float = pcb_dx / 2.0 - sonar_radius
        sonar_right_x: float = -pcb_dx / 2.0 + sonar_radius
        sonar_start_z: float = pcb_dz
        sonar_end_z: float = sonar_start_z + sonar_dz
        left_start: P3D = P3D(sonar_left_x, pcb_dy/2, sonar_start_z)
        left_end: P3D = P3D(sonar_left_x, pcb_dy/2, sonar_end_z)
        right_start: P3D = P3D(sonar_right_x, pcb_dy/2, sonar_start_z)
        right_end: P3D = P3D(sonar_right_x, pcb_dy/2, sonar_end_z)
        left_cylinder: Cylinder = Cylinder("Left Sonar Transducer",
                                           sonar_diameter, left_start, left_end, 16)
        right_cylinder: Cylinder = Cylinder("Left Sonar Transducer",
                                            sonar_diameter, right_start, right_end, 16)
        colored_left_cylinder: Color = Color("Colored Left Sonar Transducer",
                                             left_cylinder, "Silver")
        colored_right_cylinder: Color = Color("Colored Right Sonar Transducer",
                                              right_cylinder, "Silver")
        hcsr04_union: Union3D = Union3D("HCSR04 Union", [
            colored_pcb,
            colored_left_cylinder,
            colored_right_cylinder
        ])

        # Now tilt the sonar up followed by rotating it to vertical:
        x_axis: P3D = P3D(1.0, 0.0, 0.0)
        degrees90: float = pi / 2.0
        vertical_hcsr04: Rotate3D = Rotate3D("Veritcal HC-SR04", hcsr04_union, degrees90, x_axis)
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        east_facing_hcsr04: Rotate3D = Rotate3D("East Facing HC-S404",
                                                vertical_hcsr04, degrees90, z_axis)

        # Create *module*, append to *scad_program*, and save into *hcsr04* (i.e. *self*):
        module: Module3D = Module3D("HR SR04 Sonar Module", [east_facing_hcsr04])
        scad_program.append(module)
        scad_program.if3d.name_match_append("sonar", module, ["HC-SR04 Sonar"])
        # hcsr04: HCSR04 = self
        self.module: Module3D = module


# HR2BaseAssembly:
class HR2BaseAssembly:
    """Represents the HR2 base with motor holders and spacers."""

    # HR2BaseAssembly.__init__():
    def __init__(self, scad_program: ScadProgram, base_dxf: BaseDXF,
                 pi_z: float, master_board_z: float, arm_z: float) -> None:
        """Initialize a HR2BaseAssembly."""
        # Grab some Z values via *base_dxf*:
        base_battery_top_z: float = base_dxf.z_locate(-2.701374)
        base_top_z: float = base_dxf.z_locate(-3.095083)

        romi_base: RomiBase = RomiBase(scad_program, base_dxf)
        romi_motor_holder: RomiMotorHolder = RomiMotorHolder(scad_program, base_dxf)
        west_romi_motor_holder: UseModule3D = romi_motor_holder.module.use_module_get()
        degrees180: float = pi
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        east_romi_motor_holder: Rotate3D = Rotate3D("East Romi Motor Holder",
                                                    west_romi_motor_holder, degrees180, z_axis)

        # Grab the *romi_base_keys* and build *romi_base_keys_table*:
        spacer_male_height: float = 30.0
        romi_base_keys: List[Tuple[Any, ...]] = romi_base.keys_get()
        romi_base_key_table: Dict[str, Tuple[Any, ...]] = {}
        romi_base_key: Tuple[Any, ...]
        for romi_base_key in romi_base_keys:
            name: str = romi_base_key[1]
            romi_base_key_table[name] = romi_base_key

        # Construct *spacers* using data in *spacer_tuples*.  The spacers attached to the
        # battery holes are Male-Female so that the male end can be screwed into a hex nut
        # in the battery box.  The other spacers are Female-Female, since there is no associated
        # hex nut indentation under the hole:
        spacer_tuples: List[Tuple[str, str, float, float, float]] = [
            ("Pi NE", "BATTERY: Upper Slot (7, 1)",
             base_battery_top_z, pi_z, spacer_male_height),
            ("Pi NW", "BATTERY: Upper Slot (2, 1)",
             base_battery_top_z, pi_z, spacer_male_height),
            ("Pi SE", "RIGHT: LOWER Small Hex Slot (3, 0)",
             base_top_z, pi_z, 0.0),
            ("Pi SW", "LEFT: LOWER Small Hex Slot (3, 0)",
             base_top_z, pi_z, 0.0),
            ("MasterBoard NE", "BATTERY: Upper Hole (9, 2)",
             base_battery_top_z, master_board_z, spacer_male_height),
            ("MasterBoard NW", "BATTERY: Upper Hole (0, 2)",
             base_battery_top_z, master_board_z, spacer_male_height),
            # ("MasterBoard SE", "RIGHT: Misc Small Upper Right 90deg",
            #  base_top_z, master_board_z, 0.0),
            # ("MasterBoard SW", "LEFT: Misc Small Upper Right 90deg",
            #  base_top_z, master_board_z, 0.0),
            ("MasterBoard SE", "RIGHT: Vector Hole 8",
             base_top_z, master_board_z, 0.0),
            ("MasterBoard SW", "LEFT: Vector Hole 8",
             base_top_z, master_board_z, 0.0),
        ]
        spacer_tuple: Tuple[str, str, float, float, float]
        # Set *debug_dz* to non-zero to provide a little gap on top and bottom for visualization:
        debug_dz: float = 0.0 + 0.250
        spacers: List[Scad3D] = []
        for spacer_tuple in spacer_tuples:
            # Unpack *spacer_tuple*:
            spacer_name: str
            key_name: str
            bottom_z: float
            top_z: float
            male_height: float
            spacer_name, key_name, bottom_z, top_z, male_height = spacer_tuple

            # Lookup the *key_name* and extract (*key_x*, *key_y*) location:
            key: Tuple[Any, ...] = romi_base_key_table[key_name]
            key_x = key[2]
            key_y = key[3]

            # Construct the *spacer* and append the *UseModule3D* to *spacers*:
            spacer_height: float = abs(top_z - bottom_z) - 2.0 * debug_dz
            spacer_bottom_center: P3D = P3D(key_x, key_y, bottom_z + debug_dz)
            spacer: Spacer = Spacer(scad_program, f"{spacer_name} Spacer",
                                    spacer_height, "M2", diameter=3.50,
                                    bottom_height=male_height,
                                    bottom_center=spacer_bottom_center)
            spacers.append(spacer.module.use_module_get())

        # Create *module*, append to *scad_program* and save into *hr2_base_assembly* (i.e. *self*):
        module: Module3D = Module3D("HR2 Base Assembly",
                                    spacers + [
                                        romi_base.module.use_module_get(),
                                        west_romi_motor_holder,
                                        east_romi_motor_holder])
        scad_program.append(module)
        # hr2_base_assembly: HR2BaseAssembly = self
        self.module = module
        self.romi_base: RomiBase = romi_base
        self.spacer_tuples: List[Tuple[str, str, float, float, float]] = spacer_tuples
        scad_program.if3d.name_match_append("hr2_base_assembly", module, ["HR2 Base Assembly"])

    # HR2BaseAssembly.romi_base_keys_get():
    def romi_base_keys_get(self) -> List[Tuple[Any, ...]]:
        """Return the Romi Base Keys table."""
        # Grab and return *romi_base_keys* via *romi_base_assembly* (i.e. *self*):
        hr2_base_assembly: HR2BaseAssembly = self
        romi_base: RomiBase = hr2_base_assembly.romi_base
        romi_base_keys: List[Tuple[Any, ...]] = romi_base.keys_get()
        return romi_base_keys


# HR2ArmAssembly:
class HR2ArmAssembly:
    """Represents the HR2 Robot with the arm attached."""

    def __init__(self, scad_program: ScadProgram, hr2_nucleo_assembly: "HR2NucleoAssembly",
                 romi_expansion_plate: "RomiExpansionPlate", arm_z: float) -> None:
        """Initialize HR2ArmAssembly."""
        # Create the *expansion_plate*, rotate it 180 degress, and move it up to the right height:
        arm_plate: UseModule3D = romi_expansion_plate.module.use_module_get()
        colored_arm_plate: Color = Color("Colored Arm Plate", arm_plate, "Purple")
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        degrees180: float = pi
        rotated_arm_plate: Rotate3D = Rotate3D("Rotated Arm Plate",
                                               colored_arm_plate, degrees180, z_axis)
        translated_arm_plate: Translate3D = Translate3D("Translated Arm Plate",
                                                        rotated_arm_plate,
                                                        P3D(0.0, 0.0, arm_z))
        # Assembly all of the parts into a single *arm_assembly*:
        hr2_nucleo_assembly_use_module: UseModule3D = hr2_nucleo_assembly.module.use_module_get()
        module: Module3D = Module3D("HR2 Arm Assembly", [
            hr2_nucleo_assembly_use_module,
            translated_arm_plate,
        ])
        scad_program.append(module)
        # hr2_arm_assembly: HR2ArmAssembly = self
        self.module: Module3D = module
        scad_program.if3d.name_match_append("hr2_arm_assembly", module, ["Romi Base with Arm"])


# HR2MasterAssembly:
class HR2MasterAssembly:
    """Represents the HR2 Base with Pi & Master PCB."""

    # HR2MasterAssembly.__init__():
    def __init__(self, scad_program: ScadProgram, hr2_pi_assembly: "HR2PiAssembly",
                 base_dxf: BaseDXF, master_board_z: float, arm_z, pi_offset: P3D,
                 nucleo144: "Nucleo144", romi_base_keys: List[Tuple[Any, ...]],
                 romi_expansion_plate_keys: List[Tuple[Any, ...]]) -> None:
        """Initialize the HR2MasterAssembly."""
        master_board: MasterBoard = MasterBoard(scad_program, base_dxf, nucleo144,
                                                pi_offset, master_board_z, arm_z,
                                                romi_base_keys, romi_expansion_plate_keys)

        # Create *module*, append to *scad_program* and save into *hr2_master_assembly*
        # (i.e. *self*):
        module: Module3D = Module3D("HR2 Master Assembly", [
            hr2_pi_assembly.module.use_module_get(),
            master_board.module.use_module_get()])
        scad_program.append(module)
        # hr2_master_assembly: HR2MasterAssembly = self
        self.module = module
        scad_program.if3d.name_match_append("hr2_master_assembly", module, ["HR2 Base Assembly"])


# HR2NucleoAssembly:
class HR2NucleoAssembly:
    """Represents the HR2 up to the Nucleo144."""

    # HR2NucleoAssembly.__init__():
    def __init__(self, scad_program: ScadProgram, hr2_wheel_assembly: "HR2WheelAssembly",
                 nucleo144: "Nucleo144", master_board_z: float) -> None:
        """Initialize the HR2NucleoAssembly."""
        # Create *module*, append to *scad_program* and save into *hr2_nucleo_assembly*
        # (i.e. *self*):
        module: Module3D = Module3D("HR2 Nucleo Assembly", [
            hr2_wheel_assembly.module.use_module_get(),
            nucleo144.module.use_module_get(),
        ])
        scad_program.append(module)
        # hr2_master_assembly: HR2MasterAssembly = self
        self.module = module
        scad_program.if3d.name_match_append("hr2_nucleo_assembly",
                                            module, ["HR2 Nucleo144 Assembly"])


# HR2PiAssembly:
class HR2PiAssembly:
    """Represents the HR2 Base with Pi mounted."""

    # HR2PiAssembly.__init__():
    def __init__(self, scad_program: ScadProgram, hr2_base_assembly: HR2BaseAssembly,
                 pi_offset: P3D, master_board_z: float) -> None:
        """Initialize the HR2BaseAssembly."""
        # Define some constants
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        degrees90: float = pi / 2.0

        # Create the *other_pi* and rotate and translate to the correct location:
        other_pi: OtherPi = OtherPi(scad_program)
        other_pi_use_module: UseModule3D = other_pi.module.use_module_get()
        rotated_other_pi: Rotate3D = Rotate3D("Rotated Other Pi",
                                              other_pi_use_module, degrees90, z_axis)
        translated_other_pi: Translate3D = Translate3D("Translated Other Pi",
                                                       rotated_other_pi, pi_offset)
        # Create the *raspberry_pi3*:
        raspberry_pi3: RaspberryPi3 = RaspberryPi3(scad_program)
        raspberry_pi3_use_module: UseModule3D = raspberry_pi3.module.use_module_get()
        rotated_raspberry_pi3: Rotate3D = Rotate3D("Rotated Raspberry Pi3",
                                                   raspberry_pi3_use_module, degrees90, z_axis)
        translated_raspberry_pi3: Translate3D = Translate3D("Translated Raspberry Pi3",
                                                            rotated_raspberry_pi3, pi_offset)

        # Create *module*, append to *scad_program* and save into *hr2_base_assembly* (i.e. *self*):
        module: Module3D = Module3D("HR2 Pi Assembly", [
            hr2_base_assembly.module.use_module_get(),
            translated_other_pi,
            translated_raspberry_pi3])
        scad_program.append(module)
        # hr2_pi_assembly: HR2PiAssembly = self
        self.module = module
        self.hr2_base_assembly: HR2BaseAssembly = hr2_base_assembly
        scad_program.if3d.name_match_append("hr2_pi_assembly", module, ["HR2 Base Assembly"])


# HR2Robot:
class HR2Robot:
    """Represents the entire HR2 robot."""

    def __init__(self, scad_program: ScadProgram) -> None:
        """Initialize an HR2Robot."""
        base_dxf: BaseDXF = BaseDXF()
        base_top_z: float = base_dxf.z_locate(-2.701374)

        # romi_base_key: Tuple[Any, ...]
        # The tuple is stored as:
        #    ("Polygon Type", "Name", X, Y, DX, DY, Corner, Radius, Corner_Count),
        # where all non-strings are floats except the Corner_Count:
        # romi_base_keys_table: Dict[str, Tuple[Any, ...]] = {romi_base_key[1]: romi_base_key
        #                                                     for romi_base_key in romi_base_keys}
        # pi_slot_ne: Tuple[Any, ...] = romi_base_keys_table["BATTERY: Upper Slot (7, 1)"]
        # pi_slot_nw: Tuple[Any, ...] = romi_base_keys_table["BATTERY: Upper Slot (2, 1)"]
        # pi_slot_se: Tuple[Any, ...] = romi_base_keys_table["RIGHT: LOWER Small Hex Slot (3, 0)"]
        # pi_slot_sw: Tuple[Any, ...] = romi_base_keys_table["LEFT: LOWER Small Hex Slot (3, 0)"]
        # pi_x: float = (pi_slot_se[2] + pi_slot_sw[2]) / 2.0
        # pi_y: float = (pi_slot_ne[3] + pi_slot_se[3]) / 2.0
        #
        # It turns out that the code above computes the origin, so we'll just *pi_x* and
        # *pi_y* to 0.0.
        # *pi_dz* is selected to make all the master pcb pins fit:
        pi_x: float = 0.0
        pi_y: float = 0.0
        pi_z: float = base_top_z + 11.00
        pi_offset: P3D = P3D(pi_x, pi_y, pi_z)
        master_board_z: float = pi_z + 8.000
        arm_z: float = master_board_z + 26.00

        # Create the *nucleo144* before *master_board* so it can be passed in:
        nucleo144_offset: P3D = P3D(pi_x + 6.5, pi_y - 1.0, master_board_z + 13.00)
        degrees90 = pi / 2.0
        nucleo144: Nucleo144 = Nucleo144(scad_program, -degrees90, nucleo144_offset)

        # Create the *romi_expansion_plate* before *master_board* so it can be passed in:
        romi_expansion_plate: RomiExpansionPlate = RomiExpansionPlate(scad_program)
        romi_expansion_plate_keys: List[Tuple[Any, ...]] = romi_expansion_plate.keys_get()

        # Create the *hr2_base_assembly* object that can accept the various PCB's and assemblies
        # that go on top of it:
        hr2_base_assembly: HR2BaseAssembly = HR2BaseAssembly(scad_program, base_dxf,
                                                             pi_z, master_board_z, arm_z)
        hr2_pi_assembly: HR2PiAssembly = HR2PiAssembly(scad_program, hr2_base_assembly,
                                                       pi_offset, master_board_z)
        romi_base_keys: List[Tuple[Any, ...]] = hr2_base_assembly.romi_base_keys_get()
        hr2_master_assembly: HR2MasterAssembly = HR2MasterAssembly(scad_program, hr2_pi_assembly,
                                                                   base_dxf, master_board_z,
                                                                   arm_z,
                                                                   pi_offset, nucleo144,
                                                                   romi_base_keys,
                                                                   romi_expansion_plate_keys)
        hr2_wheel_assembly: HR2WheelAssembly = HR2WheelAssembly(scad_program,
                                                                hr2_master_assembly, base_dxf)

        hr2_nucleo_assembly: HR2NucleoAssembly = HR2NucleoAssembly(scad_program,
                                                                   hr2_wheel_assembly,
                                                                   nucleo144,
                                                                   master_board_z)
        hr2_arm_assembly: HR2ArmAssembly = HR2ArmAssembly(scad_program, hr2_nucleo_assembly,
                                                          romi_expansion_plate, arm_z)
        hr2_arm_assembly = hr2_arm_assembly


# HR2WheelAssembly:
class HR2WheelAssembly:
    """Represents HR2 with both wheels assemblies installed."""

    # HR2WheelAssemlby.__init__():
    def __init__(self, scad_program: ScadProgram,
                 hr2_master_assembly: HR2MasterAssembly, base_dxf: BaseDXF) -> None:
        """Initialzie HR2WheelAssembly."""
        # Create the *west_romi_wheel_assembly* and associated *UseModule3D*:
        west_romi_wheel_assembly: RomiWheelAssembly = RomiWheelAssembly(scad_program, base_dxf)
        west_romi_wheel_assembly_use_module: UseModule3D
        west_romi_wheel_assembly_use_module = west_romi_wheel_assembly.module.use_module_get()

        # Create *east_romi_wheel_assembly*:
        degrees180: float = pi
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        east_romi_wheel_assembly: Rotate3D = Rotate3D("East RomiWheel Assembly",
                                                      west_romi_wheel_assembly_use_module,
                                                      degrees180, z_axis)

        # Create the *module* and append it to *scad_program*:
        module: Module3D = Module3D("HR2 Wheel Assembly", [
            hr2_master_assembly.module.use_module_get(),
            west_romi_wheel_assembly_use_module,
            east_romi_wheel_assembly
        ])
        scad_program.append(module)
        self.module: Module3D = module
        self.hr2_master_assembly: HR2MasterAssembly = hr2_master_assembly
        scad_program.if3d.name_match_append("hr2_wheel_assembly", module, ["HR2 Wheel Assembly"])


# Nucleo144:
class Nucleo144:
    """Represents a STM32 Nucleo-144 development board."""

    # Nucleo144.__init__():
    def __init__(self, scad_program: ScadProgram,
                 z_axis_rotate: float, nucleo144_offset: P3D) -> None:
        """Initialize Nucleo144 and append to ScadProgram."""
        # Define various constants, particularly the various X/Y/Z coordinates of component
        # position on the Nucleo144.  The ethernet RJ45 connector locations are done using
        # numbers measured with calipers.

        # There are two documents that have dimensions and both are used.  They can be
        # found in `.../synchro/pcbs/docs`:
        # * `nucleo144.pdf`: Origin is on the west edge and near the center,
        #    but not exactly in the center:
        # * `stm32_nucleo_144_manual.pdf` (figure 7): No real origin, just a a bunch
        #    random dimensions all over the place.
        # The origin for this board is in the exact center:

        # All units are in thousandths of an inch and need to be converted to millimeters:
        def mil(mil: float) -> float:
            return mil * 0.0254

        # Misc. constants:
        morpho_pin_columns: int = 35
        morpho_pin_rows: int = 2
        degrees90: float = pi / 2.0
        mount_hole_diameter: float = 3.20
        # morpho_total_pins: int = morpho_pin_rows * morpho_pin_columns

        # X Coordinates:
        pcb_dx: float = 70.0
        pcb_east_x: float = pcb_dx / 2.0
        pcb_west_x: float = -pcb_east_x
        self.pcb_dx: float = pcb_dx

        # Connector X coordinates:
        cn7_zio_pin1_x: float = pcb_west_x + mil(2227)
        cn7_zio_center_x: float = cn7_zio_pin1_x + mil(50)
        cn8_zio_pin1_x: float = pcb_west_x + mil(427)
        cn8_zio_center_x: float = cn8_zio_pin1_x + mil(50)
        cn9_zio_pin1_x: float = pcb_west_x + mil(427)
        cn9_zio_center_x: float = cn9_zio_pin1_x + mil(50)
        cn10_zio_pin1_x: float = pcb_west_x + mil(2227)
        cn10_zio_center_x: float = cn10_zio_pin1_x + mil(50)
        cn11_morpho_pin1_x: float = pcb_west_x + mil(127)
        cn11_morpho_center_x: float = cn11_morpho_pin1_x + mil(50)
        cn12_morpho_pin1_x: float = pcb_west_x + mil(2527)
        cn12_morpho_center_x: float = cn12_morpho_pin1_x + mil(50)

        # Mounting hole X coordinates:
        nw_mount_hole_x: float = pcb_west_x + mil(427)
        ne_mount_hole_x: float = pcb_west_x + mil(2327)
        sw_mount_hole_x: float = pcb_west_x + mil(477)
        se_mount_hole_x: float = pcb_west_x + mil(2277)
        center_mount_hole_x: float = pcb_west_x + mil(1727)

        # Miscellaneous X coordinates:
        ethernet_east_x: float = pcb_east_x - 17.50  # Calipers
        ethernet_west_x: float = pcb_west_x + 36.00  # Calipers

        # Y Coordinates:
        pcb_dy: float = mil(2002) + mil(2243)
        pcb_north_y: float = pcb_dy / 2.0
        pcb_south_y: float = -pcb_north_y
        pcb_pseudo_origin_y: float = pcb_south_y + mil(2002)

        # Connector Y coordinates:
        cn7_zio_pin1_y: float = pcb_pseudo_origin_y + mil(1958)
        cn7_zio_y_pins: int = 10
        cn7_zio_center_y: float = cn7_zio_pin1_y - float(cn7_zio_y_pins - 1) * mil(100) / 2.0
        cn8_zio_pin1_y: float = pcb_pseudo_origin_y + mil(1598)
        cn8_zio_y_pins: int = 8
        cn8_zio_center_y: float = cn8_zio_pin1_y - float(cn8_zio_y_pins - 1) * mil(100) / 2.0
        cn9_zio_pin1_y: float = pcb_pseudo_origin_y + mil(698)
        cn9_zio_y_pins: int = 15
        cn9_zio_center_y: float = cn9_zio_pin1_y - float(cn9_zio_y_pins - 1) * mil(100) / 2.0
        cn10_zio_pin1_y: float = pcb_pseudo_origin_y + mil(898)
        cn10_zio_y_pins: int = 17
        cn10_zio_center_y: float = cn10_zio_pin1_y - float(cn10_zio_y_pins - 1) * mil(100) / 2.0
        cn11_morpho_pin1_y: float = pcb_pseudo_origin_y + mil(1998)
        cn11_morpho_y_pins: int = 35
        cn11_morpho_center_y: float = (cn11_morpho_pin1_y -
                                       float(cn11_morpho_y_pins - 1) * mil(100) / 2.0)
        cn12_morpho_pin1_y: float = pcb_pseudo_origin_y + mil(1998)
        cn12_morpho_y_pins: int = 35
        cn12_morpho_center_y: float = (cn12_morpho_pin1_y -
                                       float(cn12_morpho_y_pins - 1) * mil(100) / 2.0)
        # Miscellaneous Y coordinates:
        ethernet_south_y: float = pcb_south_y - 2.50   # Calipers
        ethernet_north_y: float = pcb_south_y + 13.00  # Calipers

        # Mounting_hole_coordinates:
        center_mount_hole_y: float = pcb_pseudo_origin_y + mil(98)
        ne_mount_hole_y: float = pcb_pseudo_origin_y + mil(2098)
        nw_mount_hole_y: float = pcb_pseudo_origin_y + mil(2148)
        sw_mount_hole_y: float = pcb_pseudo_origin_y - mil(852)
        se_mount_hole_y: float = pcb_pseudo_origin_y - mil(852)
        # There are 2 ground pin pairs at the bottom of each morpho connector:
        ground_south_y: float = cn12_morpho_pin1_y - float(cn12_morpho_y_pins) * mil(100)

        # Z Cordinates:
        pcb_dz: float = 1.60  # Calipers
        ethernet_top_z: float = pcb_dz + 6.50
        ethernet_bottom_z: float = -2.50
        self.ethernet_bottom_z: float = ethernet_bottom_z
        zio_pin_dz: float = 7.60 + pcb_dz
        zio_insulation_dz: float = 8.85

        # Create *pcb_polygon* and load it with *external_outline*:
        pcb_polygon: Polygon = Polygon("Nucleo144 PCB Polygon", [], lock=False)
        external_outline: Square = Square("Nucleo144 PCB Outline", pcb_dx, pcb_dy)
        pcb_polygon.append(external_outline)

        # Create the *zio7_connector*, *zio8_connector*, *zio9_connector*, and *zio10connector*:
        cn7_zio_connector: RectangularConnector
        cn7_zio_connector = RectangularConnector(scad_program, "CN7 Zio Connector",
                                                 2, 10, zio_insulation_dz, zio_pin_dz,
                                                 center=P3D(cn7_zio_center_x,
                                                            cn7_zio_center_y, pcb_dz),
                                                 insulation_color="SlateBlue",
                                                 cut_out=True,
                                                 pcb_polygon=pcb_polygon,
                                                 vertical_rotate=degrees90)
        cn8_zio_connector: RectangularConnector
        cn8_zio_connector = RectangularConnector(scad_program, "CN8 Zio Connector",
                                                 2, 8, zio_insulation_dz, zio_pin_dz,
                                                 center=P3D(cn8_zio_center_x,
                                                            cn8_zio_center_y, pcb_dz),
                                                 insulation_color="SlateBlue",
                                                 cut_out=True,
                                                 pcb_polygon=pcb_polygon,
                                                 vertical_rotate=degrees90)
        cn9_zio_connector: RectangularConnector
        cn9_zio_connector = RectangularConnector(scad_program, "CN9 Zio Connector",
                                                 2, 15, zio_insulation_dz, zio_pin_dz,
                                                 center=P3D(cn9_zio_center_x,
                                                            cn9_zio_center_y, pcb_dz),
                                                 insulation_color="SlateBlue",
                                                 cut_out=True,
                                                 pcb_polygon=pcb_polygon,
                                                 vertical_rotate=degrees90)
        cn10_zio_connector: RectangularConnector
        cn10_zio_connector = RectangularConnector(scad_program, "CN10 Zio Connector",
                                                  2, 17, zio_insulation_dz, zio_pin_dz,
                                                  center=P3D(cn10_zio_center_x,
                                                             cn10_zio_center_y, pcb_dz),
                                                  insulation_color="SlateBlue",
                                                  cut_out=True,
                                                  pcb_polygon=pcb_polygon,
                                                  vertical_rotate=degrees90)

        # Create the *east_morpho_connector* and *west_morpho_connector*:
        # Digikey: SAM1066-40-ND; pcb_pin=2.79  insulation=2.54  mating_length=15.75
        # Digikey: S2212EC-40-ND; pcb_pin=3.05  insulation=2.50  mating_length=8.08  price=$1.15/1
        # In additon, create the two ground connectors:
        cn12_morpho_center: P3D = P3D(cn12_morpho_center_x, cn12_morpho_center_y, 0.0)
        self.cn12_morpho_center: P3D = cn12_morpho_center
        cn12_morpho_connector: RectangularConnector
        cn12_morpho_connector = RectangularConnector(scad_program, "CN12 East Morpho Connector",
                                                     morpho_pin_rows, morpho_pin_columns,
                                                     2.54, 2.79,
                                                     male_pin_height=8.08,
                                                     center=cn12_morpho_center,
                                                     insulation_color="DarkRed",
                                                     pcb_polygon=pcb_polygon,
                                                     is_top=False, vertical_rotate=degrees90)
        cn11_morpho_center: P3D = P3D(cn11_morpho_center_x, cn11_morpho_center_y, 0.0)
        self.cn11_morpho_center: P3D = cn11_morpho_center
        cn11_morpho_connector: RectangularConnector
        cn11_morpho_connector = RectangularConnector(scad_program, "CN11 West Morpho Connector",
                                                     morpho_pin_rows, morpho_pin_columns,
                                                     2.54, 2.79, male_pin_height=6.00,
                                                     center=cn11_morpho_center,
                                                     insulation_color="DarkRed",
                                                     pcb_polygon=pcb_polygon,
                                                     is_top=False, vertical_rotate=degrees90)
        east_ground_connector: RectangularConnector
        west_ground_connector: RectangularConnector
        east_ground_connector = RectangularConnector(scad_program, "East Ground Connector",
                                                     1, 2, 2.54, 2.79, male_pin_height=5.08,
                                                     center=P3D(cn12_morpho_center_x,
                                                                ground_south_y, 0.0),
                                                     pcb_polygon=pcb_polygon, is_top=False)
        west_ground_connector = RectangularConnector(scad_program, "West Ground Connector",
                                                     1, 2, 2.54, 2.79, male_pin_height=5.08,
                                                     center=P3D(cn11_morpho_center_x,
                                                                ground_south_y, 0.0),
                                                     pcb_polygon=pcb_polygon, is_top=False)

        # Create the *ethernet_connector*, using measurements taken via calipers:
        ethernet_corner_cube: CornerCube = CornerCube("Ethernet Connector Corner Cube",
                                                      P3D(ethernet_west_x,
                                                          ethernet_south_y,
                                                          ethernet_bottom_z),
                                                      P3D(ethernet_east_x,
                                                          ethernet_north_y,
                                                          ethernet_top_z))
        colored_ethernet: Color = Color("Colored Ethenet Connector",
                                        ethernet_corner_cube,
                                        "Silver")

        # Create a list of *mount_hole_keys* that specify where each mount holes goes:
        mount_hole_keys: List[Tuple[str, float, float, float]] = [
            ("NE Mount Hole H2", mount_hole_diameter, ne_mount_hole_x, ne_mount_hole_y),
            ("NW Mount Hole H3", mount_hole_diameter, nw_mount_hole_x, nw_mount_hole_y),
            ("SE Mount Hole H4", mount_hole_diameter, se_mount_hole_x, se_mount_hole_y),
            ("SW Mount Hole H5", mount_hole_diameter, sw_mount_hole_x, sw_mount_hole_y),
            ("Center Mount Hole H1", mount_hole_diameter, center_mount_hole_x, center_mount_hole_y),
        ]

        # Interate across *mount_hole_keys* and append each *mount_hole* to *pcb_polygon*:
        mount_hole_key: Tuple[str, float, float, float]
        for mount_hole_key in mount_hole_keys:
            mount_hole_name: str = mount_hole_key[0]
            mount_hole_diameter = mount_hole_key[1]
            mount_hole_x: float = mount_hole_key[2]
            mount_hole_y: float = mount_hole_key[3]
            mount_hole: Circle = Circle(mount_hole_name, mount_hole_diameter, 16,
                                        P2D(mount_hole_x, mount_hole_y))
            pcb_polygon.append(mount_hole)

        # To improve visibility cut a hole into the center of the PCB:
        pcb_cut_out: Square = Square("PCB Cut-Out", .575 * pcb_dx, .75 * pcb_dy)
        pcb_polygon.append(pcb_cut_out)

        # We are done adding holes to the *pcb_polygon* so we can lock it and create
        # a *pcb_module* that cand be displayed by name in using the `openscad` program:
        pcb_polygon.lock()
        pcb_module: Module2D = Module2D("Nucleo144 PCB Module", [pcb_polygon])
        scad_program.append(pcb_module)
        scad_program.if2d.name_match_append("nucleo144_pcb", pcb_module, ["Nucleo144 PCB"])

        # Now create *nucleo144_pcb* and color it:
        extruded_pcb: LinearExtrude = LinearExtrude("Extruded Nucleo144 PCB", pcb_polygon, pcb_dz)
        nucleo144_pcb: Color = Color("White Nucleo144 PCB", extruded_pcb, "White")

        # Now create *nucleo144_union* consisting of the connectors and the *nucleo144*_pcb*:
        nucleo144_union: Union3D = Union3D("Nucleao 144 Union", [
            cn7_zio_connector.module.use_module_get(),
            cn8_zio_connector.module.use_module_get(),
            cn9_zio_connector.module.use_module_get(),
            cn10_zio_connector.module.use_module_get(),
            cn11_morpho_connector.module.use_module_get(),
            cn12_morpho_connector.module.use_module_get(),
            east_ground_connector.module.use_module_get(),
            west_ground_connector.module.use_module_get(),
            colored_ethernet,
            nucleo144_pcb,
            ])

        # Perform the final *z_axis_rotation* and *nucleo144_offset*:
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        rotated_nucleo144: Rotate3D = Rotate3D("Rotated Nucleo144", nucleo144_union,
                                               z_axis_rotate, z_axis)
        translated_nucleo144: Translate3D = Translate3D("Translated Nucleo144",
                                                        rotated_nucleo144, nucleo144_offset)

        # Now create *module*, append it to *scad_program*":
        module: Module3D = Module3D("Nucleo144 Module", [translated_nucleo144])
        scad_program.append(module)
        self.module: Module3D = module
        self.mount_hole_keys: List[Tuple[str, float, float, float]] = mount_hole_keys
        self.offset: P3D = nucleo144_offset
        scad_program.if3d.name_match_append("nucleo144", module, ["Nucleo144 Board"])


# MasterBoard:
class MasterBoard:
    """Represents Master PCB that the various Pi boards mount to."""

    # MasterBoard.__init__():
    def __init__(self, scad_program: ScadProgram, base_dxf: BaseDXF, nucleo144: Nucleo144,
                 pi_offset: P3D, master_board_bottom_z: float, arm_z: float,
                 romi_base_keys: List[Tuple[Any, ...]],
                 romi_expansion_plate_keys: List[Tuple[Any, ...]]) -> None:
        """Initialize the MasterBoard."""
        # Now Grab the X/Y coordinates:
        # Start with some X coordinates:
        # holder_inside_tab_dx: float = 2.60  # Measured with calipers.
        motor_casing_east_x: float = base_dxf.x_locate(-5.253299)
        wheel_well_west_x: float = base_dxf.x_locate(-6.326134)
        # holder_east_x: float = base_dxf.x_locate(-2.031646) - holder_inside_tab_dx
        magnet_east_x: float = base_dxf.x_locate(-2.652508)
        magnet_west_x: float = base_dxf.x_locate(-5.083217)
        # holder_west_x: float = base_dxf.x_locate(-5.704091) + holder_inside_tab_dx
        holder_slop_dx: float = 0.500
        wheel_well_east_x: float = base_dxf.x_locate(-1.409591)
        # wheel_well_dx: float = abs(wheel_well_east_x - wheel_well_west_x)
        # print(f"wheel_well_dx={wheel_well_dx}")

        # Now grab some Y coordinates:
        # Currently set *pcb_dy* equal to the area covering the 4 Raspberry Pi holes:
        pcb_north_y: float = 36.00
        # pcb_south_y: float = -46.00
        # pcb_dy: float = 7.00 + 58.00 + 7.00
        holder_north_y: float = base_dxf.y_locate(3.410067)
        motor_casing_north_y: float = base_dxf.y_locate(3.382512)
        motor_casing_south_y: float = base_dxf.y_locate(2.492748)
        holder_south_y: float = base_dxf.y_locate(2.465193)
        holder_slop_dy: float = 0.500
        motor_casing_dy: float = abs(motor_casing_north_y - motor_casing_south_y) / 2.0

        # Find the bottom edge of the motor casing.  Arbitrarily set Y to 0.0 since it is unneeded.
        # The Y coordinate cooresponds to the Z height.
        # motor_holder_top_z: float = base_dxf.z_locate(-1.559654)
        # motor_casing_top_z: float = base_dxf.z_locate(-1.658071)
        pcb_bottom_z: float = master_board_bottom_z
        pcb_dz: float = 1.00
        pcb_top_z: float = pcb_bottom_z + pcb_dz

        # Miscellaneous constants:
        degrees90: float = pi / 2.0

        # Create *pcb_polygon* and fill it with the *external_polygon* and start filling
        # it up with various holes:
        pcb_polygon: Polygon = Polygon("Master PCB Polygon", [], lock=False)

        # The outline of the master PCB is show approximately below as crude ASCII artwork.
        # The top and bottom are arcs centered around the center of the board at Z.
        #
        #         ~~~~~~~~F             C~~~~~~~~
        #        /        |             |        \
        #       /         |             |         \
        #      /          |             |          \
        #     G--H        E-------------D        A--B
        #        |                               |
        #        |                               |
        #        I----J                     X----Y
        #             |                     |
        #             |                     |
        #             +----------Z----------W
        #             |                     |
        #             |                     |
        #        L----K                     V----U
        #        |                               |
        #        |            P-----Q            |
        #     N--M            |     |            T--S
        #      \              |     |              /
        #       \             |     |             /
        #        \            |     |            /
        #         ~~~~~~~~~~~~O     R~~~~~~~~~~~~
        #
        #
        # The wheel well distance (|BS| =|GN|) is 72mm.  The diameter of the board is 163mm.
        # This numers are taken from the Romi Chasis User's Manual.  Thus we know the *radius*
        # is |ZB|/2 =163mm/2 = 81.5mm.
        # Also the distance of B above Z in the Y axis By = |BR|/2 = 72mm/2 = 36mm.
        diameter: float = 163.0
        radius: float = diameter / 2.0
        wheel_well_dy: float = 72.0
        half_wheel_well_dy: float = wheel_well_dy / 2.0
        arm_well_dx: float = 15.0
        half_arm_well_dx: float = arm_well_dx / 2.0
        arm_well_y: float = -40.0  # =Py=Qy

        # We need to compute the *wheel_well_angle* (= <BZW) and the *cut_out_angle* (=CZW).
        # The math is the same for both, so we will just walk through the <BZW computation.
        # Using @ to be the *wheel_well_angle*, R=*radius*=|ZB|
        #
        # (1) By = R*sin(@)
        # (2) sin(@) = By/R
        # (3) @ = asin(By/R)
        wheel_well_angle: float = asin(half_wheel_well_dy / radius)

        # We need to compute the *arm_well_angle* (= <TZW)
        # Again the math is the basically same, we just adjust the angles appropriately
        # them correctly:
        arm_well_angle: float = asin(half_arm_well_dx / radius)

        # By trial and error, the Pi connector *cutout_dx* (=|GE|) = 60mm and the *cut_out_y*
        # Fy (=|FZ|) is 32.5mm.  The same basic math yields the *cut_out_angle*
        # (Dx=R*cos(@) => @=acos(Dx/R) ):
        cut_out_y: float = 32.5
        cut_out_dx: float = 60.0
        half_cut_out_dx: float = cut_out_dx / 2.0
        cut_out_angle: float = acos(half_cut_out_dx / radius)

        # Some additional useful constants:
        origin2d: P2D = P2D(0.0, 0.0)
        degrees180: float = pi
        degrees270: float = 1.5 * pi
        degrees360: float = 2.0 * pi
        corner_radius: float = 1.5

        # Create *external_polygon* and fill draw the diagram above proceed from point A
        # through point P.  There is an arc centered on Z from to B to C and from J to K.
        external_polygon: SimplePolygon = SimplePolygon("Master PCB External Simple Polygon",
                                                        [], lock=False)

        external_polygon.corner_arc_append(                                         # A
            P2D(wheel_well_east_x, pcb_north_y), corner_radius, "SE")

        external_polygon.rounded_arc_append("B-to-C", "BH+EV+", origin2d, radius,   # Arc
                                            wheel_well_angle,                       # from B
                                            cut_out_angle, corner_radius)           # to C

        external_polygon.corner_arc_append(                                         # D
            P2D(half_cut_out_dx, cut_out_y), corner_radius, "NW")
        external_polygon.corner_arc_append(                                         # E
            P2D(-half_cut_out_dx, cut_out_y), corner_radius, "EN")

        external_polygon.rounded_arc_append("F-to-G", "BV-EH+", origin2d, radius,   # Arc
                                            degrees180 - cut_out_angle,             # from F
                                            degrees180 - wheel_well_angle,          # to G
                                            corner_radius)

        external_polygon.corner_arc_append(                                         # H
            P2D(wheel_well_west_x, half_wheel_well_dy), corner_radius, "WS")
        external_polygon.corner_arc_append(                                         # I
            P2D(wheel_well_west_x,
                holder_north_y + holder_slop_dy), corner_radius, "NE")
        external_polygon.corner_arc_append(                                         # J
            P2D(magnet_west_x + holder_slop_dx,
                holder_north_y + holder_slop_dy), corner_radius, "WS")
        external_polygon.corner_arc_append(                                         # K
            P2D(magnet_west_x + holder_slop_dx,
                holder_south_y - holder_slop_dy), corner_radius, "NW")
        external_polygon.corner_arc_append(                                         # L
            P2D(wheel_well_west_x,
                holder_south_y - holder_slop_dy), corner_radius, "ES")
        external_polygon.corner_arc_append(                                         # M
            P2D(wheel_well_west_x, -half_wheel_well_dy), corner_radius, "NW")

        external_polygon.rounded_arc_append("N-to-G", "BH-EV-", origin2d, radius,   # Arc
                                            degrees180 + wheel_well_angle,          # from N
                                            degrees270 - arm_well_angle,            # to G
                                            corner_radius)

        external_polygon.corner_arc_append(                                         # P
            P2D(-half_arm_well_dx, arm_well_y), corner_radius, "SE")
        external_polygon.corner_arc_append(                                         # Q
            P2D(half_arm_well_dx, arm_well_y), corner_radius, "WS")

        external_polygon.rounded_arc_append("R-to-S", "BV+EH-", origin2d, radius,   # Arc
                                            degrees270 + arm_well_angle,            # from R
                                            degrees360 - wheel_well_angle,          # to S
                                            corner_radius)

        external_polygon.corner_arc_append(                                         # T
            P2D(wheel_well_east_x, -half_wheel_well_dy), corner_radius, "EN")
        external_polygon.corner_arc_append(                                         # U
            P2D(wheel_well_east_x, holder_south_y - holder_slop_dy),
            corner_radius, "SW")
        external_polygon.corner_arc_append(                                         # V
            P2D(magnet_east_x - holder_slop_dx, holder_south_y - holder_slop_dy),
            corner_radius, "EN")
        #                                                                           # W not used
        external_polygon.corner_arc_append(                                         # X
            P2D(magnet_east_x - holder_slop_dx, holder_north_y + holder_slop_dy),
            corner_radius, "SE")
        external_polygon.corner_arc_append(                                         # Y
            P2D(wheel_well_east_x, holder_north_y + holder_slop_dy),
            corner_radius, "WN")

        # Lock up *external_polygon* and append it to *pcb_polygon*:
        external_polygon.lock()
        pcb_polygon.append(external_polygon)

        # The Rev. A PCB is resized to be a little bit smaller than a Bantam Labs PCB
        # which is 4 x 5 inches.  The motor wells a present, but the the wheel wells
        # are not.  There is no need for weird arcs or rounded corners:
        #
        #        G~~~~~~~~F             C~~~~~~~~B
        #        |        |             |        |
        #        |        |             |        |
        #        |        |             |        |
        #        |        E-------------D        |
        #        |                               |
        #        |                               |
        #        H----I                     T----A
        #             |                     |
        #             |                     |
        #             +----------Z----------+
        #             |                     |
        #             |                     |
        #        K----J                     S----R
        #        |                               |
        #        |            N-----O            |
        #        |            |     |            |
        #        |            |     |            |
        #        |            |     |            |
        #        |            |     |            |
        #        L------------M     P------------Q
        #

        # Some bantam constants:
        bantam_trim: float = 3.0
        bantam_dx: float = 5.0 * 25.4 - bantam_trim
        bantam_dy: float = 4.0 * 25.4 - bantam_trim
        # We need to ffset the board center down a little to have room for the ST-link board:
        bantam_y_offset: float = -10.00
        bantam_n: float = bantam_dy/2.0 + bantam_y_offset
        bantam_s: float = -bantam_dy/2.0 + bantam_y_offset

        # Create the *bantam_polygon* for the outside:
        motor_dx: float = magnet_east_x - holder_slop_dx
        holder_dy: float = holder_north_y + holder_slop_dy
        bantam_exterior: SimplePolygon = SimplePolygon("Bantam Exterior Polygon", [], lock=False)
        # bantam_exterior.point_append(P2D(bantam_dx/2.0, wheel_well_dy/2.0))      # A
        bantam_exterior.point_append(P2D(bantam_dx/2.0, bantam_n))                 # B

        bantam_exterior.point_append(P2D(half_cut_out_dx, bantam_n))               # C
        bantam_exterior.point_append(P2D(half_cut_out_dx, cut_out_y))              # D
        bantam_exterior.point_append(P2D(-half_cut_out_dx, cut_out_y))             # E
        bantam_exterior.point_append(P2D(-half_cut_out_dx, bantam_n))              # F

        bantam_exterior.point_append(P2D(-bantam_dx/2.0, bantam_n))                # G

        bantam_exterior.point_append(P2D(-bantam_dx/2.0, holder_dy))               # H
        bantam_exterior.point_append(P2D(-motor_dx, holder_dy))                    # I
        bantam_exterior.point_append(P2D(-motor_dx, -holder_dy))                   # J
        bantam_exterior.point_append(P2D(-bantam_dx/2.0, -holder_dy))              # K

        bantam_exterior.point_append(P2D(-bantam_dx/2.0, bantam_s))                # L

        bantam_exterior.point_append(P2D(-half_arm_well_dx, bantam_s))             # M
        bantam_exterior.point_append(P2D(-half_arm_well_dx, arm_well_y))           # N
        bantam_exterior.point_append(P2D(half_arm_well_dx, arm_well_y))            # O
        bantam_exterior.point_append(P2D(half_arm_well_dx, bantam_s))              # P

        bantam_exterior.point_append(P2D(bantam_dx/2.0, bantam_s))                 # Q
        bantam_exterior.point_append(P2D(bantam_dx/2.0, -holder_dy))               # R
        bantam_exterior.point_append(P2D(motor_dx, -holder_dy))                    # S
        bantam_exterior.point_append(P2D(motor_dx, holder_dy))                     # T
        bantam_exterior.point_append(P2D(bantam_dx/2.0, holder_dy))                # A

        # Lock *bantam_exteior* and append it to the newly created *bantam_polygon*:
        bantam_exterior.lock()
        bantam_polygon: Polygon = Polygon("Master PCB Polygon", [], lock=False)
        bantam_polygon.append(bantam_exterior)

        # Use *romi_base_keys* to build *romi_base_keys_table*:
        romi_base_keys_table: Dict[str, Tuple[Any, ...]] = {}
        romi_base_key: Tuple[Any, ...]
        romi_base_key_name: str
        for romi_base_key in romi_base_keys:
            romi_base_key_name = romi_base_key[1]
            romi_base_keys_table[romi_base_key_name] = romi_base_key

        # Create the *romi_base_mounting holes* which are the holes for mounting the
        # master board to the Romi base:
        spacer_tuples: List[Tuple[str, str, str]] = [
            ("NE MasterBoard", "BATTERY: Upper Hole (9, 2)", "H6"),
            ("NW MasterBoard", "BATTERY: Upper Hole (0, 2)", "H7"),
            # ("SE MasterBoard", "RIGHT: Misc Small Upper Right 90deg", "H8"),
            # ("SW MasterBoard", "LEFT: Misc Small Upper Right 90deg", "H9"),
            ("SE MasterBoard", "RIGHT: Vector Hole 8", "H8"),
            ("SW MasterBoard", "LEFT: Vector Hole 8", "H9"),
        ]

        # Put in a spacer_hole for each *spacer_tuple*:
        spacer_hole_diameter: float = 2.2
        kicad_mounting_holes: Dict[str, Tuple[P2D, float]] = {}
        kicad_hole_name: str
        spacer_tuple: Tuple[str, str, str]
        for spacer_tuple in spacer_tuples:
            spacer_name: str
            spacer_name, romi_base_key_name, kicad_hole_name = spacer_tuple
            romi_base_key = romi_base_keys_table[romi_base_key_name]
            romi_base_key_x: float = romi_base_key[2]
            romi_base_key_y: float = romi_base_key[3]
            # romi_base_key_diameter: float = romi_base_key[4]  # Use *space_hole_diameter* instead!
            kicad_mounting_holes[kicad_hole_name] = (P2D(romi_base_key_x, romi_base_key_y),
                                                     spacer_hole_diameter)

            # Consturct the *spacer_hole*:
            spacer_hole_center: P2D = P2D(romi_base_key_x, romi_base_key_y)
            spacer_hole: Circle = Circle(f"{spacer_name} Spacer Hole",
                                         spacer_hole_diameter, 16,
                                         center=spacer_hole_center)
            pcb_polygon.append(spacer_hole)

        # Grab the *arm_plate_keys* and build *arm_plate_keys_table*:
        arm_plate_keys: List[Tuple[Any, ...]] = romi_expansion_plate_keys
        arm_plate_key_table: Dict[str, Tuple[Any, ...]] = {}
        arm_plate_key: Tuple[Any, ...]
        for arm_plate_key in arm_plate_keys:
            name = arm_plate_key[1]
            arm_plate_key_table[name] = arm_plate_key

        # Define all of the arm spacer hole locations
        # Note: due to 180 degree rotation large hole origin is in upper right:
        arm_spacer_tuples: List[Tuple[str, str, str, str]] = [
            ("SE Spacer", "LEFT: Middle Triple Hole", "H10", "H14"),
            ("SW Spacer", "RIGHT: Middle Triple Hole", "H11", "H15"),
            ("NE Spacer", "Angle Hole[0,0]", "H12", "H16"),
            ("NW Spacer", "Angle Hole[5,0]", "H13", "H17"),
        ]

        # Iterate over *arm_spacer_tuples* and append the results to *arm_spacers*:
        arm_spacers: List[Scad3D] = []
        arm_spacer_tuple: Tuple[str, str, str, str]
        for arm_spacer_tuple in arm_spacer_tuples:
            # Unpack *arm_spacer_tuple*:
            arm_spacer_name: str
            arm_key_name: str
            arm_kicad_name1: str
            arm_kicad_name2: str
            arm_spacer_name, arm_key_name, arm_kicad_name1, arm_kicad_name2 = arm_spacer_tuple

            # Lookup up *arm_key_name* and extract the (*arm_key_x*, *arm_key_y*) location:
            assert arm_key_name in arm_plate_key_table, (f"'{arm_key_name}' not in "
                                                         f"[{list(arm_plate_key_table.keys())}]")
            arm_key: Tuple[Any, ...] = arm_plate_key_table[arm_key_name]
            # Note that the expansion plate is rotated around the Z axis by 180 degrees.
            # This means that the coordinates need to be reflected with minus signs:
            arm_key_x: float = -arm_key[2]
            arm_key_y: float = -arm_key[3]
            arm_key_diameter: float = arm_key[4]
            kicad_mounting_holes[arm_kicad_name1] = (P2D(arm_key_x, arm_key_y), arm_key_diameter)
            kicad_mounting_holes[arm_kicad_name2] = (P2D(-arm_key_x, -arm_key_y), arm_key_diameter)

            # Construct the *arm_spacer* and append to the *spacers*:
            arm_spacer_height: float = 30.0
            arm_spacer_bottom_center: P3D = P3D(arm_key_x, arm_key_y, pcb_top_z)
            arm_spacer = Spacer(scad_program, f"{arm_spacer_name} Arm Spacer",
                                arm_spacer_height, "M2", diameter=3.50,
                                bottom_center=arm_spacer_bottom_center)
            arm_spacers.append(arm_spacer.module.use_module_get())

            # Now add the mounting holes to the PCB.
            arm_spacer_hole1: Circle = Circle(f"{arm_spacer_name} Arm Spacer Hole 1",
                                              2.4, 16, P2D(arm_key_x, arm_key_y))
            arm_spacer_hole2: Circle = Circle(f"{arm_spacer_name} Arm Spacer Hole 2",
                                              2.4, 16, P2D(-arm_key_x, -arm_key_y))
            pcb_polygon.append(arm_spacer_hole1)
            pcb_polygon.append(arm_spacer_hole2)
        assert len(arm_spacers), "Something failed"

        # Create the *pi_receptacle_2x20* and append its mounting holes to *pcb_polygon*:
        receptacle_height: float = 4.57
        pcb_pin_height: float = 2.92
        # insulation_height: float = 2.54
        # horizontal_height: float = pcb_height + insulation_height + header_height
        receptacle_center: P3D = P3D(pi_offset.x - 49.0/2.0, pi_offset.y, master_board_bottom_z)
        receptcale_2x20: RectangularConnector
        receptacle_2x20 = RectangularConnector(scad_program, "Pi", 2, 20,
                                               receptacle_height, pcb_pin_height,
                                               cut_out=True,
                                               pcb_polygon=pcb_polygon,
                                               pin_dx_dy=0.640,
                                               center=receptacle_center,
                                               insulation_color="Maroon",
                                               vertical_rotate=degrees90, is_top=False)

        # Create the 4 EncoderBoard 1x4 Female connectors and append the mounting holes
        # to *pcb_polygon*:
        encoder_receptacles_use_modules: List[Scad3D] = []
        names: List[str] = ["South West",
                            "South East",
                            "North West",
                            "North East"]
        pin_pitch: float = 2.54  # .1in = 2.54mm
        # encoder_board_thickness: float = 1.0
        x_center_offset: float = abs(motor_casing_east_x) + pin_pitch / 2.0
        y_center_offset: float = motor_casing_dy + (1.5 + .5) * pin_pitch
        index: int
        for index, name in enumerate(names):
            x_offset: float = -x_center_offset if index & 1 == 0 else x_center_offset
            y_offset: float = -y_center_offset if index & 2 == 0 else y_center_offset
            center: P3D = P3D(x_offset, y_offset, pcb_top_z)
            encoder_receptacle_1x3: RectangularConnector
            # SLW-103-01-T-S:
            encoder_receptacle_1x3 = RectangularConnector(scad_program,
                                                          f"{name} Encoder Receptacle",
                                                          1, 3, 4.57, 2.92,
                                                          center=center,
                                                          cut_out=True,
                                                          pcb_polygon=pcb_polygon,
                                                          insulation_color="Olive",
                                                          vertical_rotate=degrees90)
            encoder_receptacles_use_modules.append(encoder_receptacle_1x3.module.use_module_get())

        # *nucleo144_offset* is the offset from the robot origin to the bottom center of
        # the Nucleo144 board.  We use these offsets to place the various holes Nucleo144
        # mounting holes, female morpho connectors, etc.

        # Install the *nucleo144* mounting holes into *pcb_polygon* and create a list
        # of *nucleo144_spacer*:
        # Set *spacer_debug_dz* to non zero to show some gap space above and below the spacer:
        spacer_debug_dz: float = 0.0 + 0.250
        nucleo144_offset: P3D = nucleo144.offset
        spacer_height: float = abs(nucleo144_offset.z - pcb_top_z) - 2.0 * spacer_debug_dz
        nucleo144_spacers: List[Scad3D] = []
        nucleo144_mount_hole_keys: List[Tuple[str, float, float, float]]
        nucleo144_mount_hole_keys = nucleo144.mount_hole_keys
        nucleo144_mount_index: int
        for nucleo144_mount_index, nucleo144_mount_hole_key in enumerate(nucleo144_mount_hole_keys):
            # The first 4 keys are for the 4 M3.0 corner spacers.  The fifth key is for the
            # alignment screw which is going to M2.0 (~=#2)
            is_screw: bool = nucleo144_mount_index == 4
            nucleo144_mount_hole_name: str = nucleo144_mount_hole_key[0]
            nucleo144_mount_hole_diameter: float = 2.20 if is_screw else 3.20
            # Swap *nucleo144_mount_hole_x* and *nucleo144_mount_hole_y*
            # due to the 90 degree rotation:
            nucleo144_mount_hole_y: float = nucleo144_mount_hole_key[2]
            nucleo144_mount_hole_x: float = nucleo144_mount_hole_key[3]
            hole_x: float = nucleo144_offset.x + nucleo144_mount_hole_x
            # Subtle: The Nucleo144 90 degree rotation causes the Y coordinate to need
            # to be negative:
            hole_y: float = nucleo144_offset.y - nucleo144_mount_hole_y
            hole: P2D = P2D(hole_x, hole_y)
            nucleo144_mount_hole: Circle = Circle(nucleo144_mount_hole_name,
                                                  nucleo144_mount_hole_diameter, 16, hole)
            pcb_polygon.append(nucleo144_mount_hole)

            # See if we have a hole that needs to be mirrored to the KiCAD PCB:
            last_space_index: int = nucleo144_mount_hole_name.rfind(' ')
            if (last_space_index >= 0 and (nucleo144_mount_hole_name[-1] != ' ' and
                                           nucleo144_mount_hole_name[last_space_index+1] == 'H')):
                # There might be a hole name of the form Hnumber" ad the end of the name:
                kicad_hole_name = nucleo144_mount_hole_name[last_space_index+1:]
                if kicad_hole_name[1:].isdigit():
                    assert kicad_hole_name not in kicad_mounting_holes, (
                        f"Nucleo Mount Hole '{kicad_hole_name}' is a duplicate")
                    mount_hole_diameter: float = (2.20 if kicad_hole_name == "H1"
                                                  else nucleo144_mount_hole_diameter)
                    # print(f">>>>>>>>>>>>>>>Kicad Hole:'{kicad_hole_name} "
                    #       f"diameter:{mount_hole_diameter}'")
                    kicad_mounting_holes[kicad_hole_name] = (hole, mount_hole_diameter)

            # Create the spacer (or alignment screw):
            spacer_bottom_center: P3D = P3D(hole_x, hole_y, pcb_top_z + spacer_debug_dz)
            if is_screw:
                # Install a vertical *alignment_screw*:
                screw_start: P3D = P3D(hole_x, hole_y, pcb_bottom_z - 1.0)
                screw_end: P3D = P3D(hole_x, hole_y, nucleo144_offset.z + 3.0)
                alignment_screw: Cylinder = Cylinder("Nucleo144 Alignment Screw",
                                                     3.0, screw_start, screw_end, 8)
                nucleo144_spacers.append(alignment_screw)
            else:
                # Install a vertical mounting spacer:
                spacer: Spacer = Spacer(scad_program, f"{nucleo144_mount_hole_name} Spacer",
                                        spacer_height, "M3", diameter=4.50,
                                        bottom_center=spacer_bottom_center)
                nucleo144_spacers.append(spacer.module.use_module_get())

        # Place the two "morpho" connectors.  On the Nucleo144 theya are called "east" and "west",
        # but since the Nucleo144 board has been rotated by 90 degrees, they actually become
        # "north" and "south".  Thus, there is some swapping between X and Y coordinates to
        # make this work:
        cn11_morpho_center: P3D = nucleo144.cn11_morpho_center
        north_morpho_center: P3D = P3D(nucleo144_offset.x + cn11_morpho_center.y,
                                       nucleo144_offset.y + cn11_morpho_center.x,
                                       pcb_top_z)
        north_morpho_connector: RectangularConnector
        north_morpho_connector = RectangularConnector(scad_program,
                                                      "North Morpho Connector",
                                                      2, 36, 5.00, 2.92,
                                                      center=north_morpho_center,
                                                      cut_out=True,
                                                      pcb_polygon=pcb_polygon,
                                                      insulation_color="Teal")
        cn12_morpho_center: P3D = nucleo144.cn12_morpho_center
        south_morpho_center: P3D = P3D(nucleo144_offset.x + cn12_morpho_center.y,
                                       nucleo144_offset.y + cn12_morpho_center.x,
                                       pcb_top_z)
        south_morpho_connector: RectangularConnector
        south_morpho_connector = RectangularConnector(scad_program,
                                                      "South Morpho Connector",
                                                      2, 36, 5.00, 2.92,
                                                      center=south_morpho_center,
                                                      cut_out=True,
                                                      pcb_polygon=pcb_polygon,
                                                      insulation_color="Teal")

        # Create some HC-SR04 sonars:
        hcsr04: HCSR04 = HCSR04(scad_program)
        # srf02: SRF02 = SRF02(scad_program)
        hcsr04_use_module: UseModule3D = hcsr04.module.use_module_get()
        # srf02_use_module: UseModule3D = srf02.module.use_module_get()
        sonars: List[Scad3D] = []
        degrees2radians: float = pi / 180.0
        sonar_poses: List[Tuple[float, float, float, bool]] = [
            # (*rim_angle*, *beam_angle* (negative => same), *sonar_offset*, *on_bottom*)
            ((90.0 - 2.3 * 22.5) * degrees2radians,
             (90.0 - 1.5 * 22.5) * degrees2radians, 0.0, True),
            ((90.0 - 1.9 * 22.5) * degrees2radians,
             (90.0 - 0.5 * 22.5) * degrees2radians, 0.0, False),
            # No rear center sonar:
            ((90.0 + 1.9 * 22.5) * degrees2radians,
             (90.0 + 0.5 * 22.5) * degrees2radians, 0.0, False),
            ((90.0 + 2.3 * 22.5) * degrees2radians,
             (90.0 + 1.5 * 22.5) * degrees2radians, 0.0, True),
            # Front Sonars:
            ((270.0 - 2 * 22.5) * degrees2radians, -1.0, 0.0, False),
            ((270.0 - 1 * 22.5) * degrees2radians, -1.0, 0.0, True),
            ((270.0) * degrees2radians, -1.0, 35.0, False),
            ((270.0 + 1 * 22.5) * degrees2radians, -1.0, 0.0, True),
            ((270.0 + 2 * 22.5) * degrees2radians, -1.0, 0.0, False)]
        x_axis: P3D = P3D(1.0, 0.0, 0.0)
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        sonars_radius: float = 77.0
        rim_angle: float
        beam_angle: float
        radius_offset: float
        on_bottom: bool
        for rim_angle, beam_angle, sonar_offset, on_bottom in sonar_poses:
            if beam_angle < 0.0:
                beam_angle = rim_angle
            angle_text: str = f"{rim_angle * 180.0 / pi}deg:{on_bottom}"
            sonar_z: float = pcb_top_z
            sonar: Scad3D = hcsr04_use_module

            # Deal with *on_bottom* first:
            if on_bottom:
                sonar_z = pcb_bottom_z
                sonar = Rotate3D(f"Bottom Sonar ({angle_text})", sonar, degrees180, x_axis)

            # Now rotate the *sonar* by *beam_angle* along the *z_axis*:
            sonar = Rotate3D(f"Rotated Sonar ({angle_text})", sonar, beam_angle, z_axis)

            # Now position *sonar* to *sonar_position* along the rim and tack onto *sonars* list:
            sonar_x: float = (sonars_radius - sonar_offset) * cos(rim_angle)
            sonar_y: float = (sonars_radius - sonar_offset) * sin(rim_angle)
            sonar_position: P3D = P3D(sonar_x, sonar_y, sonar_z)
            sonar = Translate3D(f"Translated Sonar ({angle_text})", sonar, sonar_position)
            sonars.append(sonar)

            # In addition, output 4 PCB holes for the sonars:
            pin_index: int
            for pin_index in range(4):
                pin_angle: float = beam_angle + pi / 2.0
                pin_offset: float = float(pin_index) * 2.54 - 1.5 * 2.54
                pin_x: float = sonar_x + pin_offset * cos(pin_angle)
                pin_y: float = sonar_y + pin_offset * sin(pin_angle)
                pin_hole: Circle = Circle(f"Sonar {angle_text} Pin {pin_offset}",
                                          1.27, 8, P2D(pin_x, pin_y))
                pcb_polygon.append(pin_hole)

        # This is where we build the *STLink* board and associated connectors.
        # All the work is done inside the *STLink* initializer.
        st_link_offset: P3D = P3D(0.0, -45.0, pcb_bottom_z)
        st_link_connectors: List[Scad3D] = []
        st_link: STLink = STLink(scad_program, st_link_offset, pcb_polygon, st_link_connectors)
        st_link_use_module: UseModule3D = st_link.module.use_module_get()
        translated_st_link: Scad3D = Translate3D("Translated ST Link",
                                                 st_link_use_module,
                                                 st_link_offset + P3D(0.0, 0.0, -13.0))

        # We need to cut out the two slots needed for the two Raspberry Pi flex connectors:
        # The camera slot is 17mm x 2mm and it is centered at (45, 11.5) from the lower left corner.
        # The lcd slot is 5mm x 17mm and it is centered at (2.5, 19.5 + 17/2) from the lower left
        # corner.
        pi_dx: float = 65.00
        pi_dy: float = 56.50
        camera_slot_dx: float = 2.00
        camera_slot_dy: float = 17.00
        camera_slot_center_dx: float = 45.00
        camera_slot_center_dy: float = 11.50
        lcd_slot_dx: float = 5.00
        lcd_slot_dy: float = 17.00
        lcd_slot_center_dx: float = 0.0 + lcd_slot_dx / 2.0
        lcd_slot_center_dy: float = 19.50 + lcd_slot_dy / 2.0

        # Note: the Pi board is rotated 90 degrees such that the SW corner becomes the SE corner.
        # This rotation causes the X and Y values to get swapped and sign changed as appropriate:
        rotated_pi_corner_x: float = pi_offset.x + pi_dy / 2.0
        rotated_pi_corner_y: float = pi_offset.y - pi_dx / 2.0
        camera_slot_center_x: float = rotated_pi_corner_x - camera_slot_center_dy
        camera_slot_center_y: float = rotated_pi_corner_y + camera_slot_center_dx
        lcd_slot_center_x: float = rotated_pi_corner_x - lcd_slot_center_dy
        # +4.00mm moves the slot out from under the morpho connector:
        lcd_slot_center_y: float = rotated_pi_corner_y + lcd_slot_center_dx + 4.00

        # Create the two Raspberry Pi Hat slots and apppend to *pcb_polygon*:
        lcd_slot: Square = Square("Pi Hat LCD Cable Slot", lcd_slot_dx, lcd_slot_dy,
                                  center=P2D(lcd_slot_center_x, lcd_slot_center_y),
                                  rotate=degrees90, corner_radius=1.00, corner_count=3)
        camera_slot: Square = Square("Pi Hat Camera Cable Slot", camera_slot_dx, camera_slot_dy,
                                     center=P2D(camera_slot_center_x, camera_slot_center_y),
                                     rotate=degrees90, corner_radius=1.00, corner_count=3)
        pcb_polygon.append(lcd_slot)
        pcb_polygon.append(camera_slot)

        # We are done filling up *pcb_polygon*, create *pcb_polygon_module* and append
        # it *scad_program*:
        pcb_polygon.lock()
        pcb_polygon_module: Module2D = Module2D("Master PCB Module", [pcb_polygon])
        scad_program.append(pcb_polygon_module)
        scad_program.if2d.name_match_append("master_pcb", pcb_polygon_module, ["Master PCB"])

        # Do the same for the *bantam_polygon*:
        bantam_polygon.lock()
        bantam_polygon_module: Module2D = Module2D("Bantam PCB Module", [bantam_polygon])
        scad_program.append(bantam_polygon_module)
        scad_program.if2d.name_match_append("bantam_pcb", bantam_polygon_module, ["Bantam PCB"])

        # Write the *external_polygon* and *kicad_holes* out to *kicad_file_name*:
        kicad_file_name: str = "../electrical/master_board/rev_a/master_board.kicad_pcb"
        kicad_pcb: KicadPcb = KicadPcb(kicad_file_name, P2D(100.0, 100.0))
        edge_cuts: str = "Edge.Cuts"
        margin: str = "Margin"
        kicad_pcb.layer_remove(edge_cuts)
        kicad_pcb.layer_remove(margin)
        kicad_pcb.mounting_holes_update(kicad_mounting_holes)
        kicad_pcb.polygon_append(external_polygon, margin, 0.05)
        kicad_pcb.polygon_append(bantam_exterior, edge_cuts, 0.05)
        kicad_pcb.save()

        # Create *colored_pcb* from *pcb_polygon* by extrusion and translation:
        extruded_pcb: LinearExtrude = LinearExtrude("Extruded PCB", pcb_polygon, pcb_dz)
        translated_pcb: Translate3D = Translate3D("Translated PCB", extruded_pcb,
                                                  P3D(0.0, 0.0, pcb_bottom_z))
        colored_pcb: Color = Color("Green Color", translated_pcb, "SpringGreen")

        # Create *colored_bantam* from *bantam_polygon* by extrusion and translation:
        extruded_bantam: LinearExtrude = LinearExtrude("Extruded Bantam", bantam_polygon, pcb_dz)
        translated_bantam: Translate3D = Translate3D("Translated Bantam", extruded_bantam,
                                                     P3D(0.0, 0.0, pcb_bottom_z))
        colored_bantam: Color = Color("", translated_bantam, "Magenta")

        # Create *module*, append it to *scad_program* and stuff it into *master_pcb* (i.e. *self*):
        module: Module3D = Module3D("Master Board Module",
                                    (encoder_receptacles_use_modules +
                                     nucleo144_spacers +
                                     arm_spacers +
                                     sonars +
                                     st_link_connectors +
                                     [colored_pcb,
                                      colored_bantam,
                                      translated_st_link,
                                      receptacle_2x20.module.use_module_get(),
                                      north_morpho_connector.module.use_module_get(),
                                      south_morpho_connector.module.use_module_get()]))
        scad_program.append(module)
        scad_program.if3d.name_match_append("master_board", module, ["Master Board"])
        self.module: Module3D = module


# PiBoard:
class PiBoard:
    """Represents a Raspberry Pi or compatible SBC."""

    # PiBoard.__init__():
    def __init__(self, module: Module3D) -> None:
        """Initialize PiBoard parent class."""
        # Save *module* into *pi_board* (i.e. *self*):
        # pi_board: PiBoard = self
        self.module: Module3D = module


# OtherPi:
class OtherPi(PiBoard):
    """Represents a different SBC compatible with Raspberry Pi."""

    # OtherPi.__init__():
    def __init__(self, scad_program: ScadProgram) -> None:
        """Initialize OtherPi."""
        # Compute *other_pi_pcb_polygon* from *other_pi* (i.e. *self*):
        other_pi: OtherPi = self
        other_pi_pcb_polygon: Polygon = other_pi.pcb_polygon_get()

        # Create the Male 2x20 Header:
        male_2x20_header_center: P3D = P3D((7.37 + 57.67) / 2.0, (64.00 + 69.08) / 2.0, 0.0)
        # male_2x20_header_center = P3D(0.0, 0.0, 0.0)
        pi_total_header_height: float = 8.50  # From PCB bottom to top of pins
        pi_pcb_height: float = 1.00
        pi_header_insulation_height: float = 2.54
        pi_header_pin_height: float = (pi_total_header_height
                                       - pi_header_insulation_height - pi_pcb_height)
        male_2x20_header: RectangularConnector
        male_2x20_header = RectangularConnector(scad_program, "Other Pi", 2, 20,
                                                pi_header_insulation_height, 2.54,
                                                pi_header_pin_height,
                                                center=male_2x20_header_center,
                                                pcb_polygon=other_pi_pcb_polygon,
                                                insulation_color="DarkBlue")

        connectors: List[Scad3D] = []
        connectors.append(Color("Silver Ethernet Connector",
                                CornerCube("Ethernet Connector",
                                           P3D(80.00, 29.26, 0.00),
                                           P3D(105.00, 44.51, 13.08)),  # 80.00? 105?
                                "Silver"))
        connectors.append(Color("Silver USB2 Connector",
                                CornerCube("USB2 Connector",
                                           P3D(85.00, 12.82, 0.00),
                                           P3D(104.00, 25.57, 15.33)),  # 85.00? 104.00?
                                "Silver"))
        connectors.append(Color("Silver West Connector",
                                CornerCube("West Connector",
                                           P3D(98.00, 50.69, 0.00),
                                           P3D(105.00, 58.09, 3.00)),  # 98.00? 3.00? 105??
                                "Silver"))
        connectors.append(Color("Black North Connector",
                                CornerCube("North Connector",
                                           P3D(70.03, 60.92, 0.00),
                                           P3D(76.38, 66.00, 5.00)),  # 5.00? 66.00??
                                "Black"))
        connectors.append(male_2x20_header.module.use_module_get())
        # connectors.append(Translate3D("2x20 Male Header",
        #                               male_2x20_header.module.use_module_get(),
        #                               P3D((7.37 + 57.67) / 2.0, (64.00 + 69.08) / 2.0, 0.0)))
        connectors.append(Color("Black Audio Connector",
                                CornerCube("Audio Connector",
                                           P3D(66.31, -1.00, 0.00),
                                           P3D(72.31, 14.00, 5.12)),  # -1.00? 14.00?
                                "Black"))
        connectors.append(Color("Silver USB3A Connector",
                                CornerCube("USB3A Connector",
                                           P3D(9.54, 0.00, 0.00),
                                           P3D(18.52, 10.00, 2.95)),  # 10.00?
                                "Silver"))
        connectors.append(Color("Silver USB3B Connector",
                                CornerCube("USB3B Connector",
                                           P3D(22.73, 0.00, 0.00),
                                           P3D(31.71, 10.00, 2.95)),  # 10.00?
                                "Silver"))
        connectors.append(Color("Silver Power Connector",
                                CornerCube("Power Connector",
                                           P3D(-1.00, 22.50, 0.00),
                                           P3D(8.00, 29.00, 2.95)),  # -1.00? 8.00?
                                "Silver"))
        connectors.append(Color("White JST Connector",
                                CornerCube("JST Connector",
                                           P3D(0.00, 55.01, 0.00),
                                           P3D(3.00, 63.37, 2.95)),  # 0.00? 3.00?
                                "White"))
        connectors.append(Color("Black Buttons Area",
                                CornerCube("Buttons Area",
                                           P3D(85.51, 62.12, 0.00),
                                           P3D(98.23, 68.74, 2.95)),  # 10.00?
                                "Black"))
        self.connectors: List[Scad3D] = connectors

        # Now that we are done with *connectors* we can lock *other_pi_pcb_polygon*:
        other_pi_pcb_polygon.lock()
        other_pi_pcb_polygon_module: Module2D = Module2D("Other Pi PCB", [other_pi_pcb_polygon])
        scad_program.append(other_pi_pcb_polygon_module)
        scad_program.if2d.name_match_append("other_pi_pcb", other_pi_pcb_polygon_module,
                                            ["Other Pi PCB"])

        # No construct *green_other_pi_pcb* from *other_pi_polygon*:
        pcb_height: float = 0.990
        other_pi_pcb: LinearExtrude = LinearExtrude("OtherPi PCB", other_pi_pcb_polygon, pcb_height)
        translated_other_pi_pcb: Translate3D = Translate3D("Translated Other Pi",
                                                           other_pi_pcb,
                                                           P3D(0.0, 0.0, -pcb_height))
        orange_other_pi_pcb: Scad3D = Color("Green OtherPi PCB", translated_other_pi_pcb, "Orange")

        other_pi_union: Union3D = Union3D("Other Pi Union",
                                          [orange_other_pi_pcb] + connectors)

        # Make the origin of the other pi be the same as a regular RaspBerry Pi with regards
        # to the mounting holes and 2x20 connector:
        translate_other_pi: Scad3D = Translate3D("Translate Other Pi",
                                                 other_pi_union,
                                                 P3D(-(3.5 + 58/2.0),
                                                     -(70.0 - (3.5 + 49.0/2.0)),
                                                     pcb_height))

        # Create *module*, append it to *scad_program*, and save it into *other_pi* (i.e. *self*):
        module: Module3D = Module3D("Other Pi Module", [translate_other_pi])
        scad_program.append(module)
        scad_program.if3d.name_match_append("other_pi", module, ["OtherPI SBC"])
        super().__init__(module)

    # OtherPi.board_polygon_get():
    def pcb_polygon_get(self) -> Polygon:
        """Return the PCB Polygon that represents the PCB."""
        # Define the board dimensions:
        pcb_dx: float = 104.00
        pcb_dy: float = 70.00

        # Define the mounting hole centers:
        upper_left_hole_center: P2D = P2D(3.5, pcb_dy - 3.5)
        upper_right_hole_center: P2D = P2D(3.5 + 58.0, pcb_dy - 3.5)
        lower_left_hole_center: P2D = P2D(3.5, pcb_dy - 3.5 - 49.0)
        lower_right_hole_center: P2D = P2D(3.5 + 58.0, pcb_dy - 3.5 - 49.0)

        # Create the *pcb_outline* with the origin in the lower left corner:
        pcb_center: P2D = P2D(pcb_dx / 2.0, pcb_dy / 2.0)
        pcb_outline: Square = Square("PCB Outline", pcb_dx, pcb_dy, pcb_center)

        # Define the holes:
        hole_diameter: float = 2.75
        upper_left_hole: Circle = Circle("Upper Left Hole",
                                         hole_diameter, 16, upper_left_hole_center)
        upper_right_hole: Circle = Circle("Upper Right Hole",
                                          hole_diameter, 16, upper_right_hole_center)
        lower_left_hole: Circle = Circle("Lower Left Hole",
                                         hole_diameter, 16, lower_left_hole_center)
        lower_right_hole: Circle = Circle("Lower Right Hole",
                                          hole_diameter, 16, lower_right_hole_center)

        # Create *pcb_polygon* and append the *pcb_outline* and associated mounting holes:
        pcb_polygon: Polygon = Polygon("OtherPi PCB Poylgon", [pcb_outline], lock=False)
        pcb_polygon.extend([upper_left_hole, upper_right_hole, lower_left_hole, lower_right_hole])

        # Return *pcb_polygon* in an unlocked state:
        return pcb_polygon


# RaspberryPi3:
class RaspberryPi3(PiBoard):
    """Represents a Raspberry Pi 3B+."""

    # RaspberryPi3.__init__():
    def __init__(self, scad_program: ScadProgram) -> None:
        """Initialize RaspberryPi3 and append to ScadProgram."""
        # Create the R
        raspi3b: RaspberryPi3 = self
        raspi3b_pcb_polygon: Polygon = raspi3b.pcb_polygon_get()

        connectors: List[Scad3D] = []
        mating_length: float = 5.840
        male_pin_connector2x20: RectangularConnector
        male_pin_connector2x20 = RectangularConnector(scad_program,
                                                      "A", 2, 20, mating_length, 2.54, 2.00,
                                                      center=P3D((57.90 + 7.10) / 2.0,
                                                                 52.50, 0.0),
                                                      pcb_polygon=raspi3b_pcb_polygon)
        male_pin_connector2x2: RectangularConnector
        male_pin_connector2x2 = RectangularConnector(scad_program,
                                                     "B", 2, 2, mating_length, 2.54, 2.00,
                                                     center=P3D((58.918 + 64.087) / 2.0,
                                                                (44.005 + 48.727) / 2.0, 0.0),
                                                     pcb_polygon=raspi3b_pcb_polygon)
        male_pin_connector2x1: RectangularConnector
        male_pin_connector2x1 = RectangularConnector(scad_program,
                                                     "C", 2, 1, mating_length, 2.54, 2.00,
                                                     center=P3D((58.90 + 64.10) / 2.0,
                                                                (38.91 + 41.11) / 2.0, 0.0),
                                                     pcb_polygon=raspi3b_pcb_polygon)
        connectors.append(male_pin_connector2x20.module.use_module_get())
        connectors.append(male_pin_connector2x2.module.use_module_get())
        connectors.append(male_pin_connector2x1.module.use_module_get())
        connectors.append(Color("Silver RJ45 Connector",
                                CornerCube("RJ45 Connecttor",
                                           P3D(65.650, 2.495, 0.000),
                                           P3D(87.000, 18.005, 13.500)),
                                "Silver"))
        connectors.append(Color("Silver Lower USB2",
                                CornerCube("Lower USB2",
                                           P3D(69.30, 22.43, 0.00),
                                           P3D(87.00, 34.57, 16.00)),
                                "Silver"))
        connectors.append(Color("Silver Upper USB2",
                                CornerCube("Upper USB2",
                                           P3D(69.30, 40.43, 0.00),
                                           P3D(87.00, 53.57, 16.00)),
                                "Silver"))
        connectors.append(Color("Black Camera Connector",
                                CornerCube("Camera Connector",
                                           P3D(43.55, 0.30, 0.00),
                                           P3D(47.50, 22.70, 5.50)),
                                "Black"))
        connectors.append(Color("Silver HDMI Connector",
                                CornerCube("HDMI Connector",
                                           P3D(24.75, -1.50, 0.00),
                                           P3D(39.25, 10.65, 6.50)),
                                "Silver"))
        connectors.append(Color("Silver Power Connector",
                                CornerCube("Power Connector",
                                           P3D(6.58, -1.22, 0.00),
                                           P3D(14.62, 14.35, 2.00)),
                                "Silver"))
        connectors.append(Color("Black LCD Connector",
                                CornerCube("LCD Connector",
                                           P3D(2.65, 16.80, 0.00),
                                           P3D(5.45, 39.20, 5.50)),
                                "Black"))
        self.connectors: List[Scad3D] = connectors

        # Construct the green PCB:
        raspi3b_pcb_polygon.lock()
        raspi3b_pcb: Scad3D = LinearExtrude("Rasp3B PCB", raspi3b_pcb_polygon, height=1.0)
        translated_raspi3b_pcb: Translate3D = Translate3D("Translated Rasp3B PCB",
                                                          raspi3b_pcb,
                                                          P3D(0.0, 0.0, -1.0))
        raspi3b_green_pcb: Color = Color("Green Rasp3B PCB", translated_raspi3b_pcb, "Green")

        raspi3b_pcb_polygon_module: Module2D = Module2D("RasPi3B_PCB_Polygon_Module",
                                                        [raspi3b_pcb_polygon])
        scad_program.if2d.name_match_append("raspi3_pcb", raspi3b_pcb_polygon_module,
                                            ["Raspberry Pi 3B+ PCB"])
        scad_program.append(raspi3b_pcb_polygon_module)

        # Construct the final *raspi3b_union:
        raspi3b_union: Union3D = Union3D("Rasp3B Model", connectors + [raspi3b_green_pcb])
        translated_raspi3b: Translate3D = Translate3D("Translated RasPi3B Model",
                                                      raspi3b_union,
                                                      P3D(-(3.5 + 58.0/2.0),
                                                          -(3.5 + 49.0/2.0), 1.000))
        # Create *module*, append it to *scad_program* and save it into *raspi3b* (i.e. *self*):
        module: Module3D = Module3D("RasPi3B_Module", [translated_raspi3b])
        scad_program.if3d.name_match_append("raspi3", module, ["Raspberry Pi 3B+"])
        scad_program.append(module)

        # Initialize *PiBoard* parent class:
        super().__init__(module)

    # RaspberryPi3.pcb_polygon_get():
    def pcb_polygon_get(self) -> Polygon:
        """Return the RasPi 3B+ PCB Polygon."""
        # Origin is set to lower left so all dimensions are postive except for
        # one or two connectors along the bottom edge:
        pcb_dx: float = 85.0
        holes_dx: float = 58
        pcb_dy: float = 56.0
        pcb_center: P2D = P2D(pcb_dx / 2.0, pcb_dy / 2.0)
        pcb_board_outline: Square = Square("RasPi3B+ PCB Outiline",
                                           pcb_dx, pcb_dy, center=pcb_center)

        # Define the 4 holes:
        hole_diameter = 3.2
        lower_left_hole: Circle = Circle("Lower Left Hole", hole_diameter, 8,
                                         center=P2D(3.5, 3.5))
        lower_right_hole: Circle = Circle("Lower Right Hole", hole_diameter, 8,
                                          center=P2D(3.5 + holes_dx, 3.5))
        upper_left_hole: Circle = Circle("Upper Left Hole", hole_diameter, 8,
                                         center=P2D(3.5, pcb_dy - 3.5))
        upper_right_hole: Circle = Circle("Lower Left Hole", hole_diameter, 8,
                                          center=P2D(3.5 + holes_dx, pcb_dy - 3.5))

        # Create and return the *pcb_polygon*:
        pcb_polygon: Polygon = Polygon("Rasp3b+ PCB", [pcb_board_outline,
                                                       lower_left_hole,
                                                       lower_right_hole,
                                                       upper_left_hole,
                                                       upper_right_hole],
                                       lock=False)
        return pcb_polygon


# RectangularConnector:
class RectangularConnector:
    """Represents an NxM .1 inch male pin connector."""

    # RectangularConnector.__init__():
    def __init__(self, scad_program: ScadProgram, name: str, rows: int, columns: int,
                 insulation_height: float, pcb_pin_height: float, male_pin_height: float = 0.0,
                 center: P3D = P3D(0.0, 0.0, 0.0),
                 rows_pitch: float = 2.54, columns_pitch: float = 2.54,
                 is_top: bool = True,
                 vertical_rotate: float = 0.0,
                 pin_dx_dy: float = 0.640,
                 rows_extra_insulation: float = 0.000,
                 columns_extra_insulation: float = 0.000,
                 right_angle_length: float = 0.000,
                 cut_out: bool = False,
                 pcb_polygon: Optional[Polygon] = None, pcb_hole_diameter: float = 0.0,
                 insulation_color: str = "Black", pin_color: str = "Gold") -> None:
        """Initialize RectangularConnector and append to ScadProgram.

        Create a rectangular mail header with through hole PCB pins.
        The center of the resulting header is at the center of the
        header at the surface of the PCB.  All lengths are specified
        in millimeters.

        Args:
            *scad_program* (*ScadProgram*):
                The *ScadProgram* object to append the *Module3D*
                object for this connector to.
            *name* (*str*):
                A disambigutation name for the connector.  This name
                shows up in the generated `.scad` file.
            *insulation_height* (*float*):
                The insulation height from the PCB surface to the
                either the bottom of the male pin or the top of
                the female receptacle.
            *pcb_pin_height* (*float*):
                The length of the through hole pin from the bottom of
                the insulation to the end of the PCB pin.
            *male_pin_height* (*float*):
                (Optional: Defaults to 0.0)
                The pin distance above the insulation for male headers.
                Set to 0.0 to get a female receptacle.
            *center* (*P3D*):
                (Optional: Defaults to *P3D(0.0, 0.0, 0.0))  The final location to tranlsate the
                connector to.
            *rows_pitch* (*float*):
                (Optional: defaults to 2.54mm)
                The pin pitch between rows.
            *columns_pitch* (*float*):
                (Optional: defaults to 2.54mm)
                The pin pitch between columns.
            *pin_dx_dy* (*float*):
                (Optional: defaults to 0.640mm)
                The square pin edge length in X and Y.
            *rows_extra_insulation* (*float*):
                (Optional: defaults to 0.0mm)
                The amount the extra insulation in add to the rows.
            *columns_extra_insulation* (*float*):
                (Optional: defaults to 0.0mm.)
                The amount the extra insulation in add to the columns.
            *right_angle_length* (*float*):
                (Optional: defaults to 0.0)
                Set to positive to have a right angle pins pointig in
                positve row direction and negative in the negative
                row direction.
            *vertical_rotate* (*float*):
                (Optional: defaults to 0.0)
                The amount to rotate the connector around the vertical Z
                axis in radians.
            *cut_out* (*bool*):
                (Optional: Defaults to *False)
                If *True*, cut away some insulation to show how deep the
                pins will go down.
            *pcb_polygon* (Optional[*Polygon*]):
                (Optional: Defaults to *None*)
                Specifies a *Polygon* object to append pin holes to.
            *pcb_hole_diameter*:
                (Optional: Defaults to 0.0)
                When non-zero, specifies the hole diameter for the PCB
                holes.  When zero, a reasonable hole size is selected.
            *is_top* (*bool*):
                (Optional: defaults to *True*):
                When *True*, the mating pins point upwards and when
                *False*, the mainging pins point downwards.  This
                does not affect the origin.
            *insulation_color*: (*str*):
                (Optional: defaults to "Black")
                Sets the color of the insultation.
            *pin_color* (*str*): (Optional: defaults to "Gold")
                Sets the color of the pin.

        """
        # Stuff all of the values into *pin_header* (i.e. *self*):
        # pin_header: RectangularConnector = self
        self.name: str = name
        self.rows: int = rows
        self.columns: int = columns
        self.rows_pitch: float = rows_pitch
        self.columns_pitch: float = columns_pitch
        self.male_pin_height: float = male_pin_height
        self.insulation_height: float = insulation_height
        self.pcb_pin_height: float = pcb_pin_height
        self.pin_dx_dy: float = pin_dx_dy
        self.rows_extra_insulation: float = rows_extra_insulation
        self.columns_extra_insulation: float = columns_extra_insulation
        self.right_angle_length: float = right_angle_length
        self.pcb_polygon: Optional[Polygon] = pcb_polygon
        self.pcb_hole_diameter: float = pcb_hole_diameter
        self.vertical_rotate: float = vertical_rotate
        self.is_top: bool = is_top
        self.insulation_color: str = insulation_color
        self.pin_color: str = pin_color

        # Validate argument types:
        assert rows >= 1, f"Rows (={rows}) must be positive"
        assert columns >= 1, f"Columns (={columns}) must be positive"
        assert rows_pitch > 0.0, f"Rows Pitch (={rows_pitch}) must be positive"
        assert columns_pitch > 0.0, f"Columns Pitch (={columns_pitch}) must be positive"
        assert male_pin_height >= 0.0, f"Male Pin Height (={male_pin_height}) must be non-negative"
        assert pcb_hole_diameter >= 0.0, "PCB Hole Diameter (={pcb_hole_diameter}) must be positive"
        assert insulation_height > 0.0, f"Insulation Height (={insulation_height}) must be positive"
        assert pcb_pin_height > 0.0, f"PCB Pin Height (={pcb_pin_height}) must be positive"
        assert pin_dx_dy > 0.0, f"Pin DX/DY (={pin_dx_dy}) must be positive"
        assert right_angle_length >= 0.0, (f"Right Angle Length (={right_angle_length} "
                                           "must be non-negative)")

        # Define some local variables and constants:
        # X coordinate constants:
        center_x: float = center.x
        column_pins_dx: float = float(columns - 1) * columns_pitch
        insulation_dx: float = float(columns) * columns_pitch + columns_extra_insulation
        right_angle_dx: float = 0.0
        # Y coordinate constants:
        center_y: float = center.y
        half_pin_dx_dy: float = pin_dx_dy / 2.0
        insulation_dy: float = float(rows) * rows_pitch + rows_extra_insulation
        right_angle_dy: float = right_angle_length - pin_dx_dy / 2.0
        row_pins_dy: float = float(rows - 1) * rows_pitch
        # Z coordiante constants:
        pin_above_dz: float = (0.0 if male_pin_height == 0.0
                               else insulation_height + male_pin_height)
        right_angle_start_dz: float = pcb_pin_height - pin_dx_dy
        right_angle_stop_dz: float = pcb_pin_height
        # Miscellaneous constants:
        full_name: str = f"{name} {rows}x{columns}"
        have_right_angle: bool = right_angle_length > 0.0
        is_female: bool = male_pin_height <= 0.0

        # Create the *insulation_polygon* and append *insulation_exterior* to it:
        insulation_polygon: Polygon = Polygon(f"{full_name} Insulation Polygon", [], lock=False)
        insulation_exterior: Square = Square(f"{full_name} Insulation Exterior",
                                             insulation_dx, insulation_dy)
        insulation_polygon.append(insulation_exterior)

        # Create each *colored_pin* (and optional *colored_right_angle_pin*) and
        # append to *connector_pins*.
        if pcb_hole_diameter == 0:
            pcb_hole_diameter = sqrt(3.0) * pin_dx_dy
        row_start_y: float = -row_pins_dy / 2.0
        column_start_x: float = -column_pins_dx / 2.0
        connector_pins: List[Scad3D] = []
        row_index: int
        for row_index in range(rows):
            column_index: int
            y: float = row_start_y + row_index * rows_pitch
            for column_index in range(columns):
                # Create the *colored_vertical_pin* and append to *connector_pins*:
                x: float = column_start_x + column_index * columns_pitch
                pin_bsw: P3D = P3D(x - half_pin_dx_dy, y - half_pin_dx_dy, -pcb_pin_height)
                pin_tne: P3D = P3D(x + half_pin_dx_dy, y + half_pin_dx_dy, pin_above_dz)
                vertical_pin_corner_cube: CornerCube = CornerCube(f"Pin {full_name} "
                                                                  f"({column_index}:{row_index})"
                                                                  "Corner Cube",
                                                                  pin_bsw, pin_tne)
                colored_vertical_pin: Color = Color(f"{full_name} ({column_index},{row_index}) "
                                                    f"{pin_color} Pin",
                                                    vertical_pin_corner_cube, pin_color)
                connector_pins.append(colored_vertical_pin)

                # Create *colored_right_angle_pin* and append to *connector_pins* when we
                # *have_right_angle*:
                if have_right_angle:
                    minimum_right_angle_x: float = min(x, x + right_angle_dx)
                    maximum_right_angle_x: float = max(x, x + right_angle_dx)
                    minimum_right_angle_y: float = min(y, y + right_angle_dy)
                    maximum_right_angle_y: float = max(y, y + right_angle_dy)
                    right_angle_bsw: P3D = P3D(minimum_right_angle_x - half_pin_dx_dy,
                                               minimum_right_angle_y - half_pin_dx_dy,
                                               -right_angle_start_dz)
                    right_angle_tne: P3D = P3D(maximum_right_angle_x + half_pin_dx_dy,
                                               maximum_right_angle_y + half_pin_dx_dy,
                                               -right_angle_stop_dz)
                    right_angle_pin_name = f"Right Angle Pin {full_name} Corner Cube"
                    right_angle_pin_corner_cube: CornerCube = CornerCube(right_angle_pin_name,
                                                                         right_angle_bsw,
                                                                         right_angle_tne)
                    colored_right_angle_pin: Color = Color(f"{full_name} "
                                                           f"({column_index}:{row_index})"
                                                           f"{pin_color} Right Angle Pin",
                                                           right_angle_pin_corner_cube,
                                                           pin_color)
                    connector_pins.append(colored_right_angle_pin)

                # For female receptacles, append a *receptcale_hole* to *insulation_polyon*:
                if is_female:
                    receptacle_hole: Square = Square(f"{full_name} "
                                                     f"({column_index}:{row_index})"
                                                     "Receptacle Hole", pin_dx_dy, pin_dx_dy,
                                                     center=P2D(x, y))
                    insulation_polygon.append(receptacle_hole)

                # Do any any needed PCB holes preforming any *vertical_rotate*:
                if pcb_polygon is not None:
                    hole_rotate: float = vertical_rotate
                    # Rotation of a point around the origin:
                    # https://en.wikipedia.org/wiki/Rotation_(mathematics):
                    # x' = x * cos(theta) - y * sin(theta)
                    # y' = x * sin(theta) + y * cos(theta)
                    cos_hole_rotate: float = cos(hole_rotate)
                    sin_hole_rotate: float = sin(hole_rotate)
                    rotated_x: float = x * cos_hole_rotate - y * sin_hole_rotate
                    rotated_y: float = x * sin_hole_rotate + y * cos_hole_rotate
                    hole: Circle = Circle(f"{full_name} ({column_index}:{row_index}) PCB Hole",
                                          pcb_hole_diameter, 8,
                                          center=P2D(center_x + rotated_x,
                                                     center_y + rotated_y))
                    pcb_polygon.append(hole)
        insulation_polygon.lock()

        # Deal with *cut-out* that trims some insulation away from connector to see
        # the pins better.  This allows visual inspection of how deep male pins being
        # inserted into a female receptacle.
        insulation_scad2d: Scad2D = insulation_polygon
        if cut_out:
            # Create a couple of rectangles that trim away some insulation:
            cut_out_dy: float = rows_pitch + rows_extra_insulation
            sixth_insulation_dx: float = insulation_dx / 6.0
            forth_insulation_dx: float = insulation_dx / 4.0
            ne_cutout_square: Square = Square("NE Cutout Square",
                                              sixth_insulation_dx, cut_out_dy,
                                              center=P2D(forth_insulation_dx,
                                                         insulation_dy / 2.0))
            sw_cutout_square: Square = Square("NE Cutout Square",
                                              sixth_insulation_dx, cut_out_dy,
                                              center=P2D(-forth_insulation_dx,
                                                         -insulation_dy / 2.0))

            # Create a difference to subtract out the two cut-outs:
            insulation_scad2d = Difference2D(f"{full_name} Cutouts remove", insulation_polygon,
                                             [ne_cutout_square, sw_cutout_square])

        extruded_insulation: LinearExtrude = LinearExtrude(f"{full_name} Extruded Insulation",
                                                           insulation_scad2d,
                                                           abs(insulation_height))
        translated_insulation: Scad3D = (extruded_insulation if insulation_height > 0.0
                                         else Translate3D(f"{full_name} Translated Insulation",
                                                          extruded_insulation,
                                                          P3D(0.0, 0.0, insulation_height)))
        colored_insulation: Scad3D = Color(f"{insulation_color} Insulation",
                                           translated_insulation, insulation_color)

        # Now stuff everything in *union_connector* and then rotate and translate it into the
        # final position:
        union_connector: Union3D = Union3D(f"{full_name} Union",
                                           [colored_insulation] + connector_pins, lock=True)

        # For right angle pins, we need to translate the origin to the point between the bent pins,
        # but on the inside side surface of the insulation block.  Next, we need to rotate the
        # connector by 90 degrees in the correct direction so that the correct pins are sticking
        # into the PCB:
        x_axis: P3D = P3D(1.0, 0.0, 0.0)
        pre_centered_connector: Scad3D = union_connector
        if have_right_angle:
            # We need to translate to the inside surface of the insulator and between the bent pins:
            reorigin_x = 0.0
            reorigin_y: float = -insulation_dy / 2.0
            # reorigin_y = 0.0  # For debugging only
            reorigin_z: float = pcb_pin_height - half_pin_dx_dy
            # reorigin_z = 0.0  # for debugging only
            reorigin: P3D = P3D(reorigin_x, reorigin_y, reorigin_z)
            right_angle_reorigined_connector: Translate3D = Translate3D(f"Reorigined {full_name}",
                                                                        union_connector, reorigin)

            # Finally create *pre_centered_connector* where each
            degrees90: float = pi / 2.0
            # degrees90 = 0.0  # for debugging only
            right_angle_rotated_connector: Rotate3D = Rotate3D(f"Rotated {full_name}",
                                                               right_angle_reorigined_connector,
                                                               -degrees90, x_axis)
            pre_centered_connector = right_angle_rotated_connector

        # When *is_top* is *False*, we need to flip the connector 180 degrees over to the bottom:
        if not is_top:
            pre_centered_connector = Rotate3D(f"{full_name} Bottom Rotate",
                                              pre_centered_connector, pi, x_axis)

        # Now do any requested *vertical_rotate*:
        if vertical_rotate != 0.0:
            z_axis: P3D = P3D(0.0, 0.0, 1.0)
            pre_centered_connector = Rotate3D(f"{full_name} Vertical Rotate "
                                              f"{vertical_rotate*180.0/pi}deg",
                                              pre_centered_connector,
                                              vertical_rotate, z_axis)

        # Perform the final translation to *recentered_connector*:
        recentered_connector: Scad3D = Translate3D(f"Recentered {full_name}",
                                                   pre_centered_connector, center)

        # Construct *module*, append to *scad_program*:
        module: Module3D = Module3D(f"{full_name} Module", [recentered_connector])
        scad_program.append(module)
        self.module: Module3D = module


# RomiBase:
class RomiBase:
    """Represents the Romi Chasis Base."""

    # RomiBase.__init__():
    def __init__(self, scad_program: ScadProgram, base_dxf: BaseDXF) -> None:
        """Initialize Romi and append to ScadProgram."""
        # Set *debugging* to *True* to print out debugging messages:
        debugging: bool = False  # True

        # Load some values into *romi_base* (i.e. *self*) right now:
        romi_base: RomiBase = self
        self.debugging = debugging
        self.base_dxf: BaseDXF = base_dxf

        # if debugging:  # pragma: no cover
        #    print(f"origin_offset={origin_offset}")

        # Grab some Z values (X values are arbtrarily set to 0.0). We use the Y coordiante
        # which is actually the Z value:
        base_bottom_z: float = base_dxf.z_locate(-3.469098)
        battery_top_z: float = base_dxf.z_locate(-2.701374)
        base_top_z: float = base_dxf.z_locate(-3.095083)
        battery_dz: float = abs(battery_top_z - base_bottom_z)
        base_dz: float = abs(base_top_z - base_bottom_z)

        # Now construct the base:
        romi_battery_base_polygon: Polygon = romi_base.battery_base_polygon_get()
        romi_battery_base: Scad3D = LinearExtrude("Romi Battery Base",
                                                  romi_battery_base_polygon, battery_dz)
        romi_base_polygon: Polygon = romi_base.base_polygon_get()

        # Create *romi_base_polygon_module* and append it to *scad_program*:
        romi_base_polygon_module: Module2D = Module2D("Romi Base Polygon Module",
                                                      [romi_base_polygon])
        scad_program.append(romi_base_polygon_module)

        # Now expand the *romo_base_polygon* into a *romi_circular_base*:
        romi_circular_base: Scad3D = LinearExtrude("Romi Circular Base",
                                                   romi_base_polygon_module.use_module_get(),
                                                   base_dz)
        romi_base_union: Union3D = Union3D("Romi Base Union", [
            Translate3D("Battery Base Translate", romi_battery_base,
                        P3D(0.0, 0.0, base_bottom_z)),
            Translate3D("Circular Base Translate", romi_circular_base,
                        P3D(0.0, 0.0, base_bottom_z))])
        blue_romi_base: Color = Color("Blue Romi Base", romi_base_union, "Blue")

        # Create *module*, append to *scad_program* and save into *romi_base* (i.e. *self*):
        module: Module3D = Module3D("Romi Base Module", [blue_romi_base])
        scad_program.append(module)
        # romi_base: RomiBase = self
        self.module: Module3D = module

        # Mark *romi_base_polygon* for viewing:
        if2d: If2D = scad_program.if2d
        if2d.name_match_append("romi_base", romi_base_polygon_module, ["Romi Base Polygon"])

    # RomiBase.outline_polygon_get():
    def outline_polygon_get(self) -> SimplePolygon:
        """Return the outline of the Romi Base."""
        # Grab some values from *romi* (i.e. *self*):
        romi_base: RomiBase = self
        base_dxf: BaseDXF = romi_base.base_dxf
        debugging: bool = romi_base.debugging

        # These other dimensions are read off of the drawings in section 6 of the
        # the "Pololu Romi Chasis User's Guide":
        diameter: float = 163.0  # mm
        radius: float = diameter / 2.0
        overall_width: float = 149.0  # mm
        wheel_well_dx: float = 125.0  # mm
        wheel_well_dy: float = 72.0  # mm
        half_wheel_well_dx: float = wheel_well_dx / 2.0
        half_wheel_well_dy: float = wheel_well_dy / 2.0

        # Perform any requested *debugging*:
        if debugging:  # pragma: no cover
            print(f"diameter={diameter}mm radius={radius}mm")
            print(f"overall_width={overall_width}mm")
            print(f"wheel_well_dx={wheel_well_dx}mm")
            print(f"wheel_well_dy={wheel_well_dy}mm")

        # The outer edge of the wheel well points are on the circle of *radius*.
        # We need to compute the X/Y coordinates using trigonometry.  Using math:
        #
        #     (x, y) = (r * cos(angle), r * sin(angle)                   (1)
        #     x = r * cos(angle)                                         (2)
        #     y = r * sin(angle)                                         (3)
        #     y/r = sin(angle)                                           (4)
        #     asin(y/r) = angle                                          (5)
        #     x = r * sin(acos(y/r))                                     (6)
        #
        wheel_well_angle: float = asin(half_wheel_well_dy / radius)  # radians
        wheel_well_x: float = radius * cos(wheel_well_angle)  # mm (upper right)
        wheel_well_y: float = radius * sin(wheel_well_angle)  # mm (upper right)
        wheel_well_corner: P2D = P2D(wheel_well_x, wheel_well_y)

        # Compute the dimensions of the "neck" area of the *romi* platform:
        neck_west_x: float = base_dxf.x_locate(-5.704091)
        neck_east_x: float = base_dxf.x_locate(-2.031646)
        neck_dx: float = abs(neck_east_x - neck_west_x)
        half_neck_dx: float = neck_dx / 2.0
        neck_top_y: float = base_dxf.y_locate(3.410067)
        next_bottom_y: float = base_dxf.y_locate(2.465193)
        neck_dy: float = abs(neck_top_y - next_bottom_y)
        half_neck_dy: float = neck_dy / 2.0
        # print(f"neck_dx={neck_dx}mm")

        # Perform any requested *debugging*:
        if debugging:  # pragma: no cover
            print(f"wheel_well_angle={wheel_well_angle}={degrees(wheel_well_angle)}deg")
            print(f"wheel_well_corner={wheel_well_corner}")

        # Verify that the *distance* from the *origin* to the *wheel_well_corner* matches *radius*:
        origin: P2D = P2D(0.0, 0.0)
        wheel_well_radius: float = wheel_well_corner.length()
        assert abs(radius - wheel_well_radius) < .00001, "Something is not right"

        # Now we can draw the *outline_polygon* of the Romi platform.  It conists of two arcs
        # with some straight line segments to form the wheel well.  Start by creating
        # an empty *outline_polygon*:
        outline_polygon: SimplePolygon = SimplePolygon("Romi Base Exterior")

        # Create the upper arc:
        upper_start_angle: float = wheel_well_angle
        upper_end_angle: float = pi - wheel_well_angle
        arc_count = 21
        outline_polygon.arc_append(origin, radius, upper_start_angle, upper_end_angle, arc_count)

        # Create the west wheel well:
        outline_polygon.point_append(P2D(-half_wheel_well_dx, half_wheel_well_dy))
        outline_polygon.point_append(P2D(-half_wheel_well_dx, half_neck_dy))
        outline_polygon.point_append(P2D(-half_neck_dx, half_neck_dy))
        outline_polygon.point_append(P2D(-half_neck_dx, -half_neck_dy))
        outline_polygon.point_append(P2D(-half_wheel_well_dx, -half_neck_dy))
        outline_polygon.point_append(P2D(-half_wheel_well_dx, -half_wheel_well_dy))

        # Create the lower arc:
        lower_start_angle: float = wheel_well_angle + pi
        lower_end_angle: float = upper_end_angle + pi
        outline_polygon.arc_append(origin, radius, lower_start_angle, lower_end_angle, arc_count)

        # Create the right wheel well:
        outline_polygon.point_append(P2D(half_wheel_well_dx, -half_wheel_well_dy))
        outline_polygon.point_append(P2D(half_wheel_well_dx, -half_neck_dy))
        outline_polygon.point_append(P2D(half_neck_dx, -half_neck_dy))
        outline_polygon.point_append(P2D(half_neck_dx, half_neck_dy))
        outline_polygon.point_append(P2D(half_wheel_well_dx, half_neck_dy))
        outline_polygon.point_append(P2D(half_wheel_well_dx, half_wheel_well_dy))
        # assert len(outline_polygon) == 2 * arc_count + 4

        # Lock and return *outline_polygon*:
        outline_polygon.lock()
        return outline_polygon

    # RomiBase.base_polygon_get()
    def base_polygon_get(self) -> Polygon:
        """TODO."""
        # Grabe some values from *romi_base* (i.e. *self*):
        romi_base: RomiBase = self
        base_dxf: BaseDXF = romi_base.base_dxf
        debugging: bool = romi_base.debugging

        # Grab the *base_outline_polygon*:
        base_outline_polygon: SimplePolygon = romi_base.outline_polygon_get()

        # Grab the *battery_polygons*:
        battery_polygons: List[SimplePolygon] = romi_base.battery_polygons_get()

        # Grab the *upper_hex_polygons*:
        upper_hex_polygons: List[SimplePolygon] = romi_base.upper_hex_polygons_get()
        if debugging:  # pragma: no cover
            print("************************")
            print(f"len(upper_hex_polygons)={len(upper_hex_polygons)}")

        # Grab the *lower_hex_polygons* and *lower_hex_table*:
        lower_hex_polygons: List[SimplePolygon]
        lower_hex_table: Dict[str, P2D]
        lower_hex_polygons, lower_hex_table = romi_base.lower_hex_polygons_table_get()
        if debugging:  # pragma: no cover
            print(f"len(lower_hex_polygons)={len(lower_hex_polygons)}")

        line_hole_polygons: List[SimplePolygon] = romi_base.line_hole_polygons_get(lower_hex_table)
        lower_arc_holes_rectangles: List[SimplePolygon] = romi_base.lower_arc_holes_rectangles_get()
        upper_arc_holes_rectangles: List[SimplePolygon] = romi_base.upper_arc_holes_rectangles_get()
        miscellaneous_holes: List[Circle] = romi_base.miscellaneous_holes_get()
        vertical_rectangles: List[Square] = romi_base.vertical_rectangles_get()

        # Concatenate all of the polygons together into *all_polygons* with *base_outline_polygon*
        # being the required first *Polygon*:
        mirrorable_polygons: List[SimplePolygon] = []
        mirrorable_polygons.extend(upper_hex_polygons)
        mirrorable_polygons.extend(lower_hex_polygons)
        mirrorable_polygons.extend(line_hole_polygons)
        mirrorable_polygons.extend(lower_arc_holes_rectangles)
        mirrorable_polygons.extend(upper_arc_holes_rectangles)
        mirrorable_polygons.extend(miscellaneous_holes)
        mirrorable_polygons.extend(vertical_rectangles)

        # Do the random *center_rectangles_holes*:
        bottom_rectangle: Square = base_dxf.rectangle_locate("CENTER: Bottom Rectangle",
                                                             -3.942201, -0.169016,
                                                             -3.79352, -0.224126)
        bottom_castor_rectangle: Square = base_dxf.rectangle_locate(("CENTER: "
                                                                     "Bottom Castor Rectangle"),
                                                                    -3.930854, 1.164122,
                                                                    -3.804870, 1.226996)
        top_castor_hole: Circle = base_dxf.hole_locate("CENTER: Top Castor Hole",
                                                       -3.930756, 5.394929,
                                                       -3.805256, 5.520287)
        center_rectangles_holes: List[SimplePolygon] = [
            bottom_rectangle, bottom_castor_rectangle, top_castor_hole
        ]

        mirrorable_polygon: SimplePolygon
        mirrored_polygons: List[SimplePolygon] = [mirrorable_polygon.y_mirror("RIGHT:", "LEFT:")
                                                  for mirrorable_polygon in mirrorable_polygons]
        all_internal_polygons: List[SimplePolygon] = (battery_polygons + mirrorable_polygons +
                                                      mirrored_polygons + center_rectangles_holes)
        all_polygons: List[SimplePolygon] = ([base_outline_polygon] + all_internal_polygons)

        internal_polygon: SimplePolygon

        if debugging:  # pragma: no cover
            print(f"len(all_polygons)={len(all_polygons)}")

        # Create the final *base_scad_polygon*, write it out to disk and return it.
        base_scad_polygon: Polygon = Polygon("Romi Base ScadPolygon", all_polygons, lock=True)
        return base_scad_polygon

    # RomiBase.battery_base_polygon_get():
    def battery_base_polygon_get(self) -> Polygon:
        """Return the outline of the bas base polygon."""
        romi_base: RomiBase = self
        base_dxf: BaseDXF = romi_base.base_dxf
        outline: SimplePolygon = SimplePolygon("Battery Base Polygon", [], lock=False)
        # Start in the upper right corner an go counter clockwise:
        outline.point_append(base_dxf.point_locate(-1.658217, 4.748650))  # Upper Right Corner
        outline.point_append(base_dxf.point_locate(-2.499756, 4.748650))  # Upper Right Notch1UR
        outline.point_append(base_dxf.point_locate(-2.499756, 4.669913))  # Upper Right Notch1LR
        outline.point_append(base_dxf.point_locate(-2.873772, 4.669913))  # Upper Right Notch1LL
        outline.point_append(base_dxf.point_locate(-2.873772, 4.748650))  # Upper Right Notch1UL
        outline.point_append(base_dxf.point_locate(-4.861965, 4.748650))  # Upper Left Notch1UR
        outline.point_append(base_dxf.point_locate(-4.861965, 4.669913))  # Upper Left Notch1LR
        outline.point_append(base_dxf.point_locate(-5.235980, 4.669913))  # Upper Left Notch1LL
        outline.point_append(base_dxf.point_locate(-5.235980, 4.748650))  # Upper Left Notch1UL
        outline.point_append(base_dxf.point_locate(-6.077508, 4.748650))  # Upper Left Corner
        outline.point_append(base_dxf.point_locate(-6.077508, 3.410067))  # Left Motor Corner UL
        outline.point_append(base_dxf.point_locate(-5.704091, 3.410067))  # Left Motor Corner UR
        outline.point_append(base_dxf.point_locate(-5.704091, 2.189594))  # Lower Left Corner
        outline.point_append(base_dxf.point_locate(-2.031646, 2.189594))  # Lower Right Corner
        outline.point_append(base_dxf.point_locate(-2.031646, 3.410067))  # Right Motor Corner UL
        outline.point_append(base_dxf.point_locate(-1.658217, 3.410067))  # Right Motor Corner UR
        outline.lock()

        # Construct the final *battery_base_polygon* and return it:
        simple_polygons: List[SimplePolygon] = [outline]  # Outline comes first
        simple_polygons.extend(romi_base.battery_polygons_get())  # Then comes all of the holes
        battery_base_polygon: Polygon = Polygon("Battery Base Polygon", simple_polygons)
        return battery_base_polygon

    # RomiBase.battery_clips_get():
    def battery_clips_get(self) -> List[SimplePolygon]:
        """Return the 4 battery clip polygons."""
        # A battery clip is identified by two rectangles -- the one on the left
        # encloses the "dome* and the one the right that encloses the narrow slot.
        # They come in large and small varieties.
        def battery_clip_helper(name: str, is_large: bool, left_rectangle: Square,
                                right_rectangle: Square) -> SimplePolygon:
            """Help routine for generating battery clips."""
            # Extract the X/Y coordinates from lowest (i.e. *x0*/*y0*) to highest (i.e. *x3*/*y3*):
            x0: float = left_rectangle.center.x - left_rectangle.dx / 2.0
            x1: float = left_rectangle.center.x + left_rectangle.dx / 2.0
            x2: float = right_rectangle.center.x - right_rectangle.dx / 2.0
            x3: float = right_rectangle.center.x + right_rectangle.dx / 2.0
            y3: float = right_rectangle.center.y + right_rectangle.dy / 2.0
            y2: float = left_rectangle.center.y + left_rectangle.dy / 2.0
            y1: float = left_rectangle.center.y - left_rectangle.dy / 2.0
            y0: float = right_rectangle.center.y - right_rectangle.dy / 2.0

            # Do some value sanity checking:
            assert x0 < x1 <= x2 < x3, f"{x0} {x1} {x2} {x3}"
            assert y0 < y1 < y2 < y3, f"{y0} {y1} {y2} {y3}"

            # Compute interesting points staring in upper leftt corner and going counter clock-wise:
            p0: P2D = P2D(x3, y3)  # Slot Upper Right
            p1: P2D = P2D(x2, y3)  # Slot Upper Left
            p2: P2D = P2D(x2, y2)  # Slot Upper Corner (only for large clip)
            p3: P2D = P2D(x1, y2)  # Slot Upper Arc Start
            # p4: P2D = P2D(x0, (y1 + y2) / 2.0)  # Arc apex (not needed)
            p5: P2D = P2D(x1, y1)  # Slot Lower Arc End
            p6: P2D = P2D(x2, y1)  # Slot Lower Corner (only for large clip)
            p7: P2D = P2D(x2, y0)  # Slot Lower Left
            p8: P2D = P2D(x3, y0)  # Slot Lower Right

            # Start to fill in the battery clip:
            battery_clip: SimplePolygon = SimplePolygon(name, [], lock=False)
            battery_clip.points_extend([p0, p1])
            if is_large:
                battery_clip.point_append(p2)
            battery_clip.point_append(p3)

            # Draw the arc:
            left_dy: float = y2 - y1
            radius: float = left_dy / 2.0
            arc_center_x: float = x0 + radius
            arc_center_y: float = (y1 + y2) / 2.0
            arc_center: P2D = P2D(arc_center_x, arc_center_y)
            start_angle: float = atan2(p3.y - arc_center_y, p3.x - arc_center_x)
            end_angle: float = atan2(p5.y - arc_center_y, p5.x - arc_center_x) + 2.0 * pi
            battery_clip.arc_append(arc_center, radius, start_angle, end_angle, 7)

            # Wrap up the wrest of the *battery_clip*, lock it and return it:
            if is_large:
                battery_clip.point_append(p5)
            battery_clip.points_extend([p6, p7, p8])
            battery_clip.lock()
            return battery_clip

        # Grab some values from *romi_base* (i.e. *self*):
        romi_base: RomiBase = self
        base_dxf: BaseDXF = romi_base.base_dxf

        # The lowest battery is identified by two rectangles -- the one on the left
        # encloses the "dome* and the one the right is a narrow slot.  The batteries
        # are numbered from 0 (lower most) to 3 (upper most):
        battery0_left_rectangle: Square = base_dxf.rectangle_locate("Battery0 Left Rectangle",
                                                                    -3.060783, 2.425815,
                                                                    -2.922980, 2.701413)
        battery0_right_rectangle: Square = base_dxf.rectangle_locate("Battery0 Right Rectangle",
                                                                     -2.844244, 2.311650,
                                                                     -2.745811, 2.815583)
        battery_clip0: SimplePolygon = battery_clip_helper("Battery Clip 0", True,
                                                           battery0_left_rectangle,
                                                           battery0_right_rectangle)

        battery1_left_rectangle: Square = base_dxf.rectangle_locate("Battery1 Left Rectangle",
                                                                    -2.941728, 3.002429,
                                                                    -2.844244, 3.266539)
        battery1_right_rectangle: Square = base_dxf.rectangle_locate("Battery1 Right Rectangle",
                                                                     -2.844244, 2.882512,
                                                                     -2.745811, 3.386457)
        battery_clip1: SimplePolygon = battery_clip_helper("Battery Clip 1", False,
                                                           battery1_left_rectangle,
                                                           battery1_right_rectangle)

        battery2_left_rectangle: Square = base_dxf.rectangle_locate("Battery2 Left Rectangle",
                                                                    -2.051925, 3.665984,
                                                                    -1.9141182, 3.941567)
        battery2_right_rectangle: Square = base_dxf.rectangle_locate("Battery2 Right Rectangle",
                                                                     -1.835382, 3.551803,
                                                                     -1.736965, 4.055748)
        battery_clip2: SimplePolygon = battery_clip_helper("Battery Clip 2", True,
                                                           battery2_left_rectangle,
                                                           battery2_right_rectangle)

        battery3_left_rectangle: Square = base_dxf.rectangle_locate("Battery3 Left Rectangle",
                                                                    -1.932882, 4.242594,
                                                                    -1.835382, 4.506693)
        battery3_right_rectangle: Square = base_dxf.rectangle_locate("Battery3 Right Rectangle",
                                                                     -1.835382, 4.122677,
                                                                     -1.736965, 4.626610)
        battery_clip3: SimplePolygon = battery_clip_helper("Battery Clip 3", False,
                                                           battery3_left_rectangle,
                                                           battery3_right_rectangle)

        return [battery_clip0, battery_clip1, battery_clip2, battery_clip3]

    # RomiBase.battery_polygons_get():
    def battery_polygons_get(self) -> List[SimplePolygon]:
        """Return the holes for the Romi battery case."""
        # Grab some values from *romi* (i.e. *self*):
        romi_base: RomiBase = self
        base_dxf: BaseDXF = romi_base.base_dxf

        # All of the battery holes are done relative to the *battery_reference_hole*
        # indicated on the drawing of the dimensions and mounting holes seciont of the
        # "Pololu Romi Chassis User's Guide".
        reference_hole: Circle = base_dxf.hole_locate("ZILCH: Battery Hole",
                                                      -3.913146, 3.376610,
                                                      -3.822591, 3.286051)

        # The battery holes have an upper and lower group.  The lower group resides between
        # the motors and the upper group is above the motors.  The lower group is organized
        # in 3 rows by 9 columns and not all holes are poplulated.  We create a
        # *lower_battery_pattenrs* list to specify which of the holes need to be poputlated.
        # Remember, the `.dxf` orientation is being used and move down from the
        # *battery_reference_hole_center*:
        lower_battery_y_offsets: Tuple[float, ...] = (0.0, -12.3, -12.3 - 12.3)
        lower_battery_patterns: Tuple[str, ...] = (
            "*-**O**-*",  # Row with reference hole in the middle (at the 'O' location)
            "*-*****-*",  # Row below reference hole
            "*-*****-*")  # Two rows below referene hole
        simple_polygons: List[SimplePolygon] = list()
        reference_hole_center_y: float = reference_hole.center.y
        hole_dx_pitch: float = 10
        column0_x: float = -4.0 * hole_dx_pitch
        x_index: int
        for x_index in range(9):
            x: float = column0_x + x_index * hole_dx_pitch
            y_index: int
            lower_battery_pattern: str
            for y_index, lower_battery_pattern in enumerate(lower_battery_patterns):
                if lower_battery_pattern[x_index] != '-':
                    # We need a hole:
                    y: float = reference_hole_center_y + lower_battery_y_offsets[y_index]
                    lower_hole_center: P2D = P2D(x, y)
                    lower_hole: Circle = reference_hole.copy(("BATTERY: Lower Hole "
                                                             f"({4-x_index}, {y_index})"),
                                                             center=lower_hole_center)
                    simple_polygons.append(lower_hole)

        # The upper battery holes above the lower battery motor holes are organized as
        # 3 rows by 9 columns where all positions are populated:
        upper_battery_y_offsets: Tuple[float, ...] = (7.0, 7.0 + 12.3, 7.0 + 12.3 + 12.3)
        column0_x = -4.5 * hole_dx_pitch
        for x_index in range(10):
            x = column0_x + x_index * hole_dx_pitch
            for y_index in range(3):
                y = reference_hole_center_y + upper_battery_y_offsets[y_index]
                upper_hole_center: P2D = P2D(x, y)
                upper_hole: SimplePolygon
                if x_index in (2, 7) and y_index == 1:
                    slot_north_y: float = base_dxf.y_locate(4.142358)
                    slot_south_y: float = base_dxf.y_locate(4.036051)
                    slot_dy: float = abs(slot_north_y - slot_south_y)
                    slot_east_x: float
                    slot_west_x: float
                    if x_index == 2:  # -4.852118
                        slot_east_x = base_dxf.x_locate(-4.780091)
                        slot_west_x = base_dxf.x_locate(-4.904465)
                    else:
                        slot_east_x = base_dxf.x_locate(-2.831272)
                        slot_west_x = base_dxf.x_locate(-2.955646)
                    slot_dx: float = abs(slot_east_x - slot_west_x)
                    upper_hole = Square(f"BATTERY: Upper Slot ({x_index}, {y_index})",
                                        slot_dx, slot_dy, center=upper_hole_center,
                                        corner_radius=slot_dy/2.0, corner_count=2)
                else:
                    upper_hole = reference_hole.copy(("BATTERY: Upper Hole "
                                                      f"({x_index}, {y_index})"),
                                                     center=upper_hole_center)
                simple_polygons.append(upper_hole)

        # There are 6 rectangular slots that have a nub on the end to cleverly indicate battery
        # polarity direction.  We will model these as simple rectangular slots and skip the
        # cleverly battery nubs.  There are 4 batteries slots at the top called
        # *upper_left_battery_slot*, *upper_right_battery_slot*, *lower_left_battery_slot*, and
        # *lower_righ_battery_slot*.  Underneath these 4 batteries are 2 more centered battery
        # slots that are called *upper_center_battery_slot* and *lower_center_battery_slot*:
        upper_left_battery_slot: Square = base_dxf.rectangle_locate(("BATTERY: "
                                                                     "Upper Left Battery Slot"),
                                                                    -5.550937, 4.252594,
                                                                    -4.074563, 4.473067)
        upper_right_battery_slot: Square = base_dxf.rectangle_locate(("BATTERY: "
                                                                      "Upper Right Battery Slot"),
                                                                     -3.621799, 4.252594,
                                                                     -2.145425, 4.473067)
        lower_left_battery_slot: Square = base_dxf.rectangle_locate("BATTERY: "
                                                                    "Lower Left Battery Slot",
                                                                    -5.590311, 3.705346,
                                                                    -4.113925, 3.925815)
        lower_right_battery_slot: Square = base_dxf.rectangle_locate(("BATTERY: "
                                                                      "Lower Right Battery Slot"),
                                                                     -3.661173, 3.705346,
                                                                     -2.184799, 3.925815)
        upper_center_battery_slot: Square = base_dxf.rectangle_locate(("BATTERY: Upper Center "
                                                                       "Battery Slot"),
                                                                      -4.58637, 3.012429,
                                                                      -3.109992, 3.232913)
        lower_center_battery_slot: Square = base_dxf.rectangle_locate(("BATTERY: Lower Center "
                                                                       "Battery Slot"),
                                                                      -4.625744, 2.465193,
                                                                      -3.149354, 2.685665)

        # *battery_slot_polygons* list and append them to *polygons*:
        battery_slot_polygons: List[SimplePolygon] = [
            upper_left_battery_slot, upper_right_battery_slot,
            lower_left_battery_slot, lower_right_battery_slot,
            upper_center_battery_slot, lower_center_battery_slot
        ]
        simple_polygons.extend(battery_slot_polygons)

        # There 4 cutouts across the top of the battery case and 3 cutouts along the bottom.
        # The 4 upper cutouts are called *upper_outer_left_cutout*, *upper_inner_left_cutout*,
        # *upper_inner_right_cutout*, and *upper_outer_right_cutout*.
        upper_outer_left_cutout: Square = base_dxf.rectangle_locate(("BATTERY: "
                                                                     "Upper Outer Left Cutout"),
                                                                    -5.984008, 4.74865,
                                                                    -5.302909, 4.965193)
        upper_inner_left_cutout: Square = base_dxf.rectangle_locate(("BATTERY: "
                                                                     "Upper Inner Left Cutout"),
                                                                    -5.23598, 4.669913,
                                                                    -4.861965, 4.858886)
        upper_inner_right_cutout: Square = base_dxf.rectangle_locate(("BATTERY: "
                                                                      "Upper Inner Right Cutout"),
                                                                     -2.873772, 4.669913,
                                                                     -2.499756, 4.858886)
        upper_outer_right_cutout: Square = base_dxf.rectangle_locate(("BATTERY: "
                                                                      "Upper Outer Right Cutout"),
                                                                     -2.432827, 4.74865,
                                                                     -1.751728, 4.965193)

        # There are three cutouts across the bottom of the battery case and they are called
        # *lower_left_cutout*, *lower_center_cutout*, and *lower_right_cutout*:
        lower_left_cutout: Square = base_dxf.rectangle_locate("BATTERY: Lower Left Cutout",
                                                              -5.572591, 1.939594,
                                                              -4.655272, 2.189594)
        lower_center_cutout: Square = base_dxf.rectangle_locate("BATTERY: Lower Center Cutout",
                                                                -4.340311, 2.032122,
                                                                -3.395425, 2.189594)
        lower_right_cutout: Square = base_dxf.rectangle_locate("BATTERY: Lower Right Cutout",
                                                               -3.080465, 1.939594,
                                                               -2.163146, 2.189594)

        # Collect all of the cutouts into *cutout_polygons* and append to *polygons*:
        cutout_polygons: List[SimplePolygon] = [
            upper_outer_left_cutout, upper_inner_left_cutout,
            upper_inner_right_cutout, upper_outer_right_cutout,
            lower_left_cutout, lower_center_cutout, lower_right_cutout
        ]
        simple_polygons.extend(cutout_polygons)

        # There are 4 slots for where the encoder connectors through hole pins land.
        # There are only two of the 4 slots visible in the `.dxf`, so we find the
        # *inner_encoder_slot* corners using *base_dxf*:
        inner_encoder_corner_ne: P2D = base_dxf.point_locate(-2.552508, 3.288024)
        inner_encoder_corner_sw: P2D = base_dxf.point_locate(-2.652508, 2.587232)
        encoder_dx: float = abs(inner_encoder_corner_ne.x - inner_encoder_corner_sw.x)
        encoder_dy: float = abs(inner_encoder_corner_ne.y - inner_encoder_corner_sw.y)
        inner_encoder_center: P2D = (inner_encoder_corner_ne + inner_encoder_corner_sw) / 2.0

        # The distance between the closest edges of the inner and outer encoder slot
        # is measuered using calipers to be 1.65mm:
        outer_encoder_center: P2D = inner_encoder_center + P2D(1.65 + encoder_dx / 2.0, 0.0)

        # Now compute *outer_encoder_slot* and *inner_encoder_slot*:
        outer_encoder_slot: Square = Square("RIGHT: Outer Encoder Slot",
                                            encoder_dx, encoder_dy, outer_encoder_center)
        inner_encoder_slot: Square = Square("RIGHT: Inner Encoder Slot",
                                            encoder_dx, encoder_dy, inner_encoder_center)

        # Collect the encoder slots into *encoder_slots* and append to *polygons*:
        encoder_slots: List[SimplePolygon] = [
            outer_encoder_slot, outer_encoder_slot.y_mirror("RIGHT:", "LEFT:"),
            inner_encoder_slot, inner_encoder_slot.y_mirror("RIGHT:", "LEFT:")
        ]
        simple_polygons.extend(encoder_slots)

        battery_clips: List[SimplePolygon] = romi_base.battery_clips_get()
        simple_polygons.extend(battery_clips)

        return simple_polygons

    # RomiBase.hex_pattern_get():
    def hex_pattern_get(self, pattern_rows: Tuple[str, ...], slot_pairs: List[str],
                        hex_origin: P2D, hole_diameter: float, label: str,
                        include_short_slot: bool) -> Tuple[List[SimplePolygon], Dict[str, P2D]]:
        """Generate a hexagonal pattern of holes and slots.

        The Romi base and shelf have pattern of holes and slot that are
        hexagonal.  The patterns are quite irregular because that have
        fit within the circular confines of the base and miss other
        feature such as the castor ball assemblies, battery holder, etc.
        This method returns a list of *Polygon*'s that represent each
        hole and slot. It also returns a hole name to location *dict*.

        Args:
            *pattern_rows* (*Tuple*[*str*, ...]): As list of patterns
                that specify where the holes are to be located.  A '-'
                means "no hole", a lower case letter identifies a
                "virtual hole" that is the end-point of a slot, and
                and upper case letter corresponds to an actual hole.
            *slot_pairs* (*List*[*str*]): A list of two letter
                strings, where each letter specifies the end-point of
                a slot.
            *hex_origin (*P*): The known location of one of the
                in the hoes in the pattern.  It is required that
                one of the *pattern_rows* label this locatation with
                an 'O' letter.
            *hole_diameter: (*float*): The diameter of the holes in
                millimeters.
            *label* (*str*): The sub-label for the holes.
            *include_short_slot* (*bool*): If *True* include the one
                lower short slot that is only in the lower hex pattern;
                otherwise, do not include the slot.

        Returns:
            (*List*[*SimplePolygon*], *Dict*[*str*, *P"]): Returns a
                list of hole and slot *SimplePolygons*.  In addition,
                a *dict* that maps each hole letter name to a location
                is returned.

        """
        # Grab some values from *romi* (i.e. *self*):
        romi_base: RomiBase = self
        base_dxf: BaseDXF = romi_base.base_dxf
        debugging: bool = romi_base.debugging

        # The hexagonal slots and hole pattern is present on both the top and the
        # bottom of the platform.  The "User's Guide" implies that holes are spaced
        # by 7.5mm vertically.
        #
        # The math for equilateral triagngles is:
        #     b = equilateral triangle base width
        #     h = equalateral triangle height
        #     h = b*sqrt(3)/2
        #     b = 2*h/sqrt(3)
        hex_dy_pitch: float = 7.50
        half_hex_dx_pitch: float = hex_dy_pitch / sqrt(3.0)

        # We need to get the dimensions for one vertical slot and compute the *slot_dx*,
        # *slot_dy* and *slot_corner_radius*.  Note that we want *slot_dx* and to measure
        # the horizontal slot dimensions.  So, the dx/dy from the vertical slot is swapped:
        lower_right_vertical_slot: Square = base_dxf.rectangle_locate("Lower Right Hex Slot",
                                                                      -1.925717, 1.347083,
                                                                      -1.835161, 1.634484)
        slot_dx: float = lower_right_vertical_slot.dy
        slot_dy: float = lower_right_vertical_slot.dx
        slot_corner_radius: float = slot_dy / 2.0

        # Perform any requested debugging:
        if debugging:  # pragma: no cover
            print("-------------------")
            print(f"hex_origin={hex_origin}")
            print(f"hole_diameter={hole_diameter}")
            print(f"half_hex_dx_pitch={half_hex_dx_pitch}")
            print(f"hex_dy_pitch={hex_dy_pitch}")
            print(f"slot_dx={slot_dx}")
            print(f"slot_dy={slot_dy}")
            print(f"slot_corner_raidus={slot_corner_radius}")

        # Compute *upper_left_origin_x* and *upper_right_y* which is the X/Y location
        # the upper left location of the *pattern_rows*.  It is computed relative to
        # *hex_origin* base on finding the a hole labeled 'O' in the *pattern_rows*:
        upper_left_origin_x: float = nan
        upper_left_origin_y: float = nan
        y_index: int
        pattern_row: str
        for y_offset, pattern_row in enumerate(pattern_rows):
            x_offset: int = pattern_row.find('O')
            if x_offset >= 0:
                upper_left_origin_x = hex_origin.x - x_offset * half_hex_dx_pitch
                upper_left_origin_y = hex_origin.y + y_offset * hex_dy_pitch
                break
        else:
            assert False, "No origin hole found."  # pragma: no cover
        if debugging:  # pragma: no cover
            print("upper_left_origin_x={upper_left_origin_x}")
            print("upper_left_origin_y={upper_left_origin_y}")

        # The return values are *simple_polygons* and *locations*:
        simple_polygons: List[SimplePolygon] = list()
        locations: Dict[str, P2D] = dict()

        # *pattern_rows* contain the end-point locations for the hex pattern.
        # We iterate across *pattern_rows* in Y first and X second.
        points_count: int = 8
        pattern_index: int
        for y_index, pattern_row in enumerate(pattern_rows):
            y = upper_left_origin_y - (y_index * hex_dy_pitch)
            pattern_character: str
            x_index: float
            for x_index, pattern_character in enumerate(pattern_row):
                # Dispatch on *pattern_character*:
                if pattern_character != '-':
                    # Enter *left_hole_center* into *locations* keyed by *pattern_character*:
                    x = upper_left_origin_x + (x_index * half_hex_dx_pitch)
                    hole_center: P2D = P2D(x, y)
                    locations[pattern_character] = hole_center

                    # Only create holes when *pattern_character* is upper case:
                    if pattern_character.isupper():
                        hole: SimplePolygon
                        if pattern_character == 'A' and label == "LOWER":
                            # The 'A' location is actually a small vertical slot:
                            a_slot_east_x: float = base_dxf.x_locate(-2.850146)
                            a_slot_west_x: float = base_dxf.x_locate(-2.956453)
                            a_slot_dx: float = abs(a_slot_east_x - a_slot_west_x)
                            # a_slot_center_x: float = (a_slot_east_x + a_slot_west_x) / 2.0
                            a_slot_north_y: float = base_dxf.y_locate(1.858083)
                            a_slot_south_y: float = base_dxf.y_locate(1.733720)
                            # a_slot_center_y: float = (a_slot_north_y + a_slot_north_y) / 2.0
                            a_slot_dy: float = abs(a_slot_north_y - a_slot_south_y)
                            # a_slot_center: P2D = P2D(a_slot_center_x, a_slot_center_y)
                            a_corner_radius: float = min(a_slot_dx, a_slot_dy) / 2.0
                            a_name: str = f"RIGHT: {label} Small Hex Slot ({x_index}, {y_index})"
                            # print(f"a_slot_dx={a_slot_dx} a_slot_dy={a_slot_dy} "
                            #       f"a_corner_radius={a_corner_radius}")
                            # print(f"a_slot_center={a_slot_center} hole_center={hole_center}")
                            # print(f"a_name='{a_name}")
                            hole = Square(a_name,
                                          a_slot_dx, a_slot_dy,
                                          center=hole_center,
                                          corner_radius=a_corner_radius, corner_count=2)
                        else:
                            # Put in the *right_hole*:
                            hole = Circle(f"RIGHT: {label} Hex Hole ({x_index}, {y_index})",
                                          hole_diameter, points_count, hole_center)
                        simple_polygons.append(hole)

        # Now sweep through *slot_pairs* and install all of the slots:
        slot_pair: str
        for slot_pair in slot_pairs:
            # Do one slot for each *slot_pair*:
            hole1: P2D = locations[slot_pair[0]]
            hole2: P2D = locations[slot_pair[1]]
            center: P2D = (hole1 + hole2) / 2.0
            slot_angle: float = atan2(hole1.y - hole2.y, hole1.x - hole2.x)
            slot: Square = Square(f"RIGHT: {label} Slot '{slot_pair}'",
                                  slot_dx, slot_dy, center,
                                  rotate=slot_angle, corner_radius=slot_corner_radius,
                                  corner_count=points_count)
            simple_polygons.append(slot)

        # Deal with *short_slot*:
        if include_short_slot:
            short_slot_rectangle: Square = base_dxf.rectangle_locate("Short Slot Rectangle",
                                                                     -2.437146, 1.642358,
                                                                     -2.346591, 1.831331)
            short_slot_center: P2D = short_slot_rectangle.center
            short_slot_dx: float = short_slot_rectangle.dx
            short_slot_dy: float = short_slot_rectangle.dy
            short_slot_corner_radius = short_slot_dx / 2.0
            short_slot: Square = Square(f"RIGHT: {label} Short Slot",
                                        short_slot_dx, short_slot_dy, short_slot_center,
                                        corner_radius=short_slot_corner_radius,
                                        corner_count=points_count)
            simple_polygons.append(short_slot)

        # Return the *simple_polygons* and *locations*:
        return simple_polygons, locations

    # RomiBase.line_hole_polygons_get():
    def line_hole_polygons_get(self, lower_hex_table: Dict[str, P2D]) -> List[SimplePolygon]:
        """TODO."""
        # Grab some values from *romi* (i.e. *self*):
        romi_base: RomiBase = self
        debugging: bool = romi_base.debugging
        base_dxf: BaseDXF = romi_base.base_dxf

        # There is a line of holes along the bottom that have a smaller hole diameter.
        # We locate the smallest hole at the end of the line:
        small_hole: Circle = base_dxf.hole_locate("Small Hole",
                                                  -3.289535, 0.256524,
                                                  -3.198980, 0.165984)
        small_hole_diameter: float = small_hole.diameter
        small_hole_center: P2D = small_hole.center

        if debugging:  # pragma: no cover
            print(f"small_hole_diameter={small_hole_diameter}")
            print(f"small_hole_center={small_hole_center}")

        # Now using *s_center* and *q_center* we compute a "unit" vector along the line.
        # We enter holes that do not over lap with the larger holes.  We wind up skipping
        # one hole in 3:
        line_hole_polygons: List[SimplePolygon] = list()
        s_center: P2D = lower_hex_table["S"]
        q_center: P2D = lower_hex_table["Q"]
        hole_vector: P2D = q_center - s_center
        for vector_hole_index in range(9):
            if vector_hole_index % 3 != 1:
                # Do the hole on the right first:
                hole_center: P2D = (s_center + (vector_hole_index - 1) * hole_vector / 3.0)
                hole: Circle = Circle(f"RIGHT: Vector Hole {vector_hole_index}",
                                      small_hole_diameter, 8, hole_center)
                line_hole_polygons.append(hole)

        return line_hole_polygons

    # RomiBase.lower_arc_holes_rectangles_get():
    def lower_arc_holes_rectangles_get(self) -> List[SimplePolygon]:
        """TODO."""
        # Grab some values from *romi_base* (i.e. *self*):
        romi_base: RomiBase = self
        debugging: bool = romi_base.debugging
        base_dxf: BaseDXF = romi_base.base_dxf

        # The resulting *Polygon*'s are collected into *lower_arc_holes_rectangles*:
        lower_arc_holes_rectangles: List[SimplePolygon] = []

        # There are arcs of holes and and rectangular slots along the upper and lower rims.
        # Since they are mirrored across the Y axis, we only worry about the right side.
        # The hole closest to the wheel is the "start" hole and the one farthest from the
        # wheel is the "end" hole.  We have to locate each of these holes:
        lower_start: Circle = base_dxf.hole_locate("Lower Start Hole",
                                                   -1.483063, 1.348929,
                                                   -1.357508, 1.223803)
        lower_start_diameter: float = lower_start.diameter
        lower_start_center: P2D = lower_start.center
        lower_arc_start_angle: float = atan2(lower_start_center.y, lower_start_center.x)
        if debugging:  # pragma: no cover
            print(f"lower_start_diameter={lower_start_diameter}")
            print(f"lower_start_center={lower_start_center}")
            print(f"lower_start_angle={degrees(lower_arc_start_angle)}deg")

        # We pick the smallest hole that is next to the hole at the end to get the
        # *small_hole_diameter*:
        lower_end: Circle = base_dxf.hole_locate("Lower End Hole",
                                                 -3.144020, 0.125287,
                                                 -3.053465, 0.034732)
        lower_end_diameter: float = lower_end.diameter
        lower_end_center: P2D = lower_end.center
        small_hole_diameter: float = lower_end_diameter

        lower_arc_end_angle: float = atan2(lower_end_center.y, lower_end_center.x)
        if debugging:  # pragma: no cover
            print(f"lower_start_diameter={lower_start_diameter}")
            print(f"lower_start_center={lower_start_center}")

        # Compute the *lower_arc_radius*:
        lower_hole_radius: float = lower_start_center.length()

        # There are two sizes of rectangle -- small and large.  The width appears to
        # be the same for both, so we only need *rectangle_width*, *small_rectangle_length*
        # and *large_rectangle_length*.  Lastly, we need to find one *rectangle_center*
        # so we can determine the *rectangle_radius* from the *origin*:
        large_upper_left_corner: P2D = base_dxf.point_locate(-1.248201, 1.259484)
        large_lower_left_corner: P2D = base_dxf.point_locate(-1.331370, 1.136248)
        large_upper_right_corner: P2D = base_dxf.point_locate(-1.205772, 1.230858)
        large_rectangle_length: float = large_upper_left_corner.distance(large_lower_left_corner)
        rectangle_width: float = large_upper_left_corner.distance(large_upper_right_corner)
        rectangle_center: P2D = (large_upper_right_corner + large_lower_left_corner) / 2.0
        rectangle_radius: float = rectangle_center.length()
        small_upper_left_corner: P2D = base_dxf.point_locate(-1.368228, 1.081638)
        small_lower_left_corner: P2D = base_dxf.point_locate(-1.431575, 0.987760)
        small_rectangle_length: float = small_upper_left_corner.distance(small_lower_left_corner)
        if debugging:  # pragma: no cover
            print(f"lower_hole_radius={lower_hole_radius}")
            print(f"rectangle_radius={rectangle_radius}")
            print(f"rectangle_width={rectangle_width}")
            print(f"large_rectangle_length={large_rectangle_length}")
            print(f"rectangle_center={rectangle_center}")
            print(f"small_rectangle_length={small_rectangle_length}")

        # There are *lower_holes_count* + 1 holes to create along the arc. There are
        # *lower_holes_count* + 3 rectangles to create along the arc:
        lower_holes_count: int = 12
        delta_angle: float = (lower_arc_end_angle - lower_arc_start_angle) / (lower_holes_count - 1)
        lower_hole_index: int
        for lower_hole_index in range(lower_holes_count + 3):
            # The same *lower_arc_hole_diameter* is used for both the left and right arc holes:
            lower_arc_hole_diameter: float = (lower_start_diameter if lower_hole_index % 3 == 0
                                              else small_hole_diameter)

            # Likewise the *lower_rectangle_length* is used both the left and right rectangle arcs:
            lower_rectangle_length: float = (large_rectangle_length if lower_hole_index % 3 == 0
                                             else small_rectangle_length)

            # Do the *lower_right_hole* first:
            lower_hole_angle: float = lower_arc_start_angle + float(lower_hole_index) * delta_angle
            lower_hole_x: float = lower_hole_radius * cos(lower_hole_angle)
            lower_hole_y: float = lower_hole_radius * sin(lower_hole_angle)
            lower_hole_center: P2D = P2D(lower_hole_x, lower_hole_y)
            lower_hole: Circle = Circle(f"RIGHT: Lower hole {lower_hole_index}",
                                        lower_arc_hole_diameter, 8, lower_hole_center)
            if lower_hole_index < lower_holes_count + 1:
                lower_arc_holes_rectangles.append(lower_hole)

            # Next do the *lower_right_rectangle*:
            lower_rectangle_x: float = rectangle_radius * cos(lower_hole_angle)
            lower_rectangle_y: float = rectangle_radius * sin(lower_hole_angle)
            lower_rectangle_center: P2D = P2D(lower_rectangle_x, lower_rectangle_y)
            lower_rectangle: Square = Square(f"RIGHT: Lower left Rectangle {lower_hole_index}",
                                             rectangle_width, lower_rectangle_length,
                                             lower_rectangle_center, lower_hole_angle)
            lower_arc_holes_rectangles.append(lower_rectangle)

        # Return the resuting *arc_hole_rectangle_polygons*:
        return lower_arc_holes_rectangles

    # RomiBase.lower_hex_polygons_table_get():
    def lower_hex_polygons_table_get(self) -> Tuple[List[SimplePolygon], Dict[str, P2D]]:
        """TODO."""
        # The "User's Guide" identifies the lower hex whole used to reference the hex
        # pattern off of:
        # Grab some values from *romi* (i.e. *self*):
        romi_base: RomiBase = self
        debugging: bool = romi_base.debugging
        base_dxf: BaseDXF = romi_base.base_dxf

        # Extract the origin hole information for the lower hexagon pattern:
        lower_hex_hole: Circle = base_dxf.hole_locate("Lower Hex Hole",
                                                      -2.454646, 1.553776,
                                                      -2.329091, 1.428650)
        lower_hex_hole_center: P2D = lower_hex_hole.center
        lower_hex_hole_diameter: float = lower_hex_hole.diameter
        if debugging:  # pragma: no cover
            print(f"lower_hex_hole_center={lower_hex_hole_center}")
            print(f"lower_hex_hole_diameter={lower_hex_hole_diameter}")

        # Using the `.dxf` image, the pattern below represent the locations of the hex pattern
        # holes in the lower left quadrant.  'O' is at *lower_hex_hole_center*.  Upper case letters
        # indicate the location of a hole.  Lower case letters indicate  the end-point of a slot.
        # There is a weird little half slot above 'O' that is not currently modeled:
        lower_pattern_rows: Tuple[str, ...] = (
            "---A-B-C-D-",  # [-1]
            "----E-O-F-G",  # [0]
            "---a-H-I-J-",  # [1]
            "----K-L-M--",  # [2]
            "---N-Q-----",  # [3]
            "R-S---------"  # [4]
        )

        # *lower_slot_pairs* specifies the holes that bracket a slot.:
        lower_slot_pairs: List[str] = "AO:OD:Aa:aO:JO:DJ:OL:aL:LJ:aN:NL:RN".split(':')

        # Now we can invoke the *hex_pattern* method to fill in the hex pattern and
        # mirror it across the Y axis to the other sise:
        lower_hex_holes_table: Dict[str, P2D]

        lower_hex_polygons: List[SimplePolygon]
        lower_hex_polygons, lower_holes_table = romi_base.hex_pattern_get(lower_pattern_rows,
                                                                          lower_slot_pairs,
                                                                          lower_hex_hole_center,
                                                                          lower_hex_hole_diameter,
                                                                          "LOWER", True)
        return lower_hex_polygons, lower_holes_table

    # RomiBase.miscellaneous_holes_get():
    def miscellaneous_holes_get(self) -> List[Circle]:
        """Return the miscellaneous holes."""
        # Around the lower hex pattern there are a bunch of miscellaneous holes
        # around the periphery of the patten.  This code just picks them off:
        romi_base: RomiBase = self
        base_dxf: BaseDXF = romi_base.base_dxf
        lower_left_hole: Circle = base_dxf.hole_locate("RIGHT: Misc Small Lower Left",
                                                       -3.119047, 0.658110,
                                                       -3.041756, 0.74865)
        upper_left_hole: Circle = base_dxf.hole_locate("RIGHT: Misc Small Upper Left",
                                                       -3.028492, 1.642358,
                                                       -3.119047, 1.732913)
        upper_right_hole_minus30: Circle = base_dxf.hole_locate(("RIGHT: "
                                                                 "Misc Small Upper Right -30deg"),
                                                                -1.755228, 1.642358,
                                                                -1.664673, 1.732913)
        upper_right_hole_30: Circle = base_dxf.hole_locate("RIGHT: Misc Small Upper Right 30deg",
                                                           -1.755228, 1.839205,
                                                           -1.664673, 1.929760)
        upper_right_hole_90: Circle = base_dxf.hole_locate("RIGHT: Misc Small Upper Right 90deg",
                                                           -1.925717, 1.937638,
                                                           -1.835161, 2.028177)
        upper_right_hole_120: Circle = base_dxf.hole_locate("RIGHT: Misc Small Upper Right 120deg",
                                                            -2.096189, 1.839205,
                                                            -2.005634, 1.929760)
        upper_right_large: Circle = base_dxf.hole_locate("RIGHT: Misc Large Upper Right",
                                                         -1.84402, 2.252594,
                                                         -1.71848, 2.127469)

        # Store all of the located holes into a list of *miscellaneous_holes*:
        miscellaneous_holes: List[Circle] = [
            lower_left_hole,
            upper_left_hole,
            upper_right_hole_minus30,
            upper_right_hole_30,
            upper_right_hole_90,
            upper_right_hole_120,
            upper_right_large
        ]
        return miscellaneous_holes

    # RomiBase.upper_arc_holes_rectangles_get():
    def upper_arc_holes_rectangles_get(self) -> List[SimplePolygon]:
        """TODO."""
        # Grab some values from *romi*:
        romi_base: RomiBase = self
        base_dxf: BaseDXF = romi_base.base_dxf
        debugging: bool = romi_base.debugging

        # The resulting *Polygon*'s are collected into *upper_arc_holes_rectangles*:
        upper_arc_holes_rectangles: List[SimplePolygon] = []

        # There are arcs of holes and and rectangular slots along the upper and lower rims.
        # Since they are mirrored across the Y axis, we only worry about the right side.
        # The hole closest to the wheel is the "start" hole and the one farthest from the
        # wheel is the "end" hole.  We have to locate each of these holes:
        upper_start: Circle = base_dxf.hole_locate("Upper Start Hole",
                                                   -1.483063, 4.651469,
                                                   -1.357508, 4.526346)
        upper_start_diameter: float = upper_start.diameter
        upper_start_center: P2D = upper_start.center
        upper_arc_start_angle: float = atan2(upper_start_center.y, upper_start_center.x)
        if debugging:  # pragma: no cover
            print(f"upper_start_diameter={upper_start_diameter}")
            print(f"upper_start_center={upper_start_center}")
            print(f"upper_start_angle={degrees(upper_arc_start_angle)}deg")

        upper_end: Circle = base_dxf.hole_locate("Upper End Hole",
                                                 -3.14402, 5.840524,
                                                 -3.053465, 5.749969)
        upper_end_diameter: float = upper_end.diameter
        upper_end_center: P2D = upper_end.center
        upper_arc_end_angle: float = atan2(upper_end_center.y, upper_end_center.x)
        if debugging:  # pragma: no cover
            print(f"upper_end_diameter={upper_end_diameter}")
            print(f"upper_end_center={upper_end_center}")

        # Compute the *upper_hole_radius*:
        upper_hole_radius: float = upper_start_center.length()

        # There are two sizes of rectangle -- small and large.  The width appears to
        # be the same for both, so we only need *rectangle_width*, *small_rectangle_length*
        # and *large_rectangle_length*.  Lastly, we need to find one *rectangle_center*
        # so we can determine the *rectangle_radius* from the origin:
        large_upper_inner_corner: P2D = base_dxf.point_locate(-1.331370, 4.739012)
        large_lower_inner_corner: P2D = base_dxf.point_locate(-1.248201, 4.615776)
        large_lower_outer_corner: P2D = base_dxf.point_locate(-1.205772, 4.644402)
        large_rectangle_length: float = large_upper_inner_corner.distance(large_lower_inner_corner)
        rectangle_width: float = large_lower_inner_corner.distance(large_lower_outer_corner)
        rectangle_center: P2D = (large_upper_inner_corner + large_lower_outer_corner) / 2.0
        rectangle_radius: float = rectangle_center.length()
        small_upper_inner_corner: P2D = base_dxf.point_locate(-1.431575, 4.887512)
        small_lower_inner_corner: P2D = base_dxf.point_locate(-1.368228, 4.793638)
        small_rectangle_length: float = small_upper_inner_corner.distance(small_lower_inner_corner)
        if debugging:  # pragma: no cover
            print(f"upper_hole_radius={upper_hole_radius}")
            print(f"rectangle_radius={rectangle_radius}")
            print(f"rectangle_width={rectangle_width}")
            print(f"large_rectangle_length={large_rectangle_length}")
            print(f"rectangle_center={rectangle_center}")
            print(f"small_rectangle_length={small_rectangle_length}")

        # There *upper_holes_count* holes and rectangles to create along the arc. Not holes are
        # created.  There are *upper_holes_count* + 1 rectangles and all of these are poplulated:
        small_hole_diameter = upper_end_diameter
        upper_holes_count: int = 12
        delta_angle: float = (upper_arc_end_angle - upper_arc_start_angle) / (upper_holes_count - 1)
        upper_hole_index: int
        for upper_hole_index in range(upper_holes_count + 1):
            # The same *upper_arc_hole_diameter* is used for both the left and right arc holes:
            upper_arc_hole_diameter: float = (upper_start_diameter if upper_hole_index % 3 == 0
                                              else small_hole_diameter)

            # Likewise the *lower_rectangle_length* is used both the left and right rectangle arcs:
            upper_rectangle_length: float = (large_rectangle_length if upper_hole_index % 3 == 0
                                             else small_rectangle_length)

            # Do the *lower_right_hole* first:
            upper_hole_angle: float = upper_arc_start_angle + float(upper_hole_index) * delta_angle
            upper_hole_x: float = upper_hole_radius * cos(upper_hole_angle)
            upper_hole_y: float = upper_hole_radius * sin(upper_hole_angle)
            upper_hole_center = P2D(upper_hole_x, upper_hole_y)
            upper_hole: Circle = Circle(f"RIGHT: Upper hole {upper_hole_index}",
                                        upper_arc_hole_diameter, 8, upper_hole_center)

            # Skip 3 of the holes:
            if upper_hole_index not in (2, 3, 12, 13):
                upper_arc_holes_rectangles.append(upper_hole)

            # Next do the *upper_rectangle*:
            upper_rectangle_x: float = rectangle_radius * cos(upper_hole_angle)
            upper_rectangle_y: float = rectangle_radius * sin(upper_hole_angle)
            upper_rectangle_center: P2D = P2D(upper_rectangle_x, upper_rectangle_y)
            upper_rectangle: Square = Square(f"RIGHT: Upper Right Rectangle {upper_hole_index}",
                                             rectangle_width, upper_rectangle_length,
                                             upper_rectangle_center, upper_hole_angle)
            upper_arc_holes_rectangles.append(upper_rectangle)

        # Return the resulting *arc_holes_rectangles*:
        return upper_arc_holes_rectangles

    # RomiBase.upper_hex_polygons_get():
    def upper_hex_polygons_get(self) -> List[SimplePolygon]:
        """TODO."""
        # Grab some values from *romi* (i.e. *self*):
        romi_base: RomiBase = self
        base_dxf: BaseDXF = romi_base.base_dxf
        debugging: bool = romi_base.debugging

        # For the upper hex pattern, the hole that is at the end of the 4 slots is selected
        # as the upper hex hole:
        upper_hex_hole: Circle = base_dxf.hole_locate("Upper Hex Hole",
                                                      -2.749535, 5.441567,
                                                      -2.629075, 5.316441)
        upper_hex_hole_center: P2D = upper_hex_hole.center
        upper_hex_hole_diameter: float = upper_hex_hole.diameter
        if debugging:  # pragma: no cover
            print(f"upper_hex_hole_center={upper_hex_hole_center}")
            print(f"upper_hex_hole_diameter={upper_hex_hole_diameter}")

        # For the *upper_pattern_rows*, the 'O' marks the *upper_hex_hole_center*:
        upper_pattern_rows: Tuple[str, ...] = (
            "a------",
            "-A-O---",
            "b-B-C-c",
            "---d---",
        )

        # *upper_slot_pairs* specifies which slots to render.  Now we can invoke the *hex_pattern*
        # method to render the hex pattern and mirror it across to the other side:
        upper_slot_pairs: List[str] = "aO:bO:dO:cO".split(':')
        upper_holes_table: Dict[str, P2D]
        upper_hex_polygons: List[SimplePolygon]
        upper_hex_polygons, upper_holes_table = romi_base.hex_pattern_get(upper_pattern_rows,
                                                                          upper_slot_pairs,
                                                                          upper_hex_hole_center,
                                                                          upper_hex_hole_diameter,
                                                                          "UPPER", False)

        # The *upper_holes_table* is not needed, we just return *upper_hex_polygons*:
        return upper_hex_polygons

    # RomiBase.vertical_rectangles_get():
    def vertical_rectangles_get(self) -> List[Square]:
        """Return the vertical wheel well rectangles."""
        romi_base: RomiBase = self
        base_dxf: BaseDXF = romi_base.base_dxf
        upper_rectangle0: Square = base_dxf.rectangle_locate("RIGHT: UPPER: Rectangle 0",
                                                             -1.511965, 3.569984,
                                                             -1.460783, 3.683232)
        upper_rectangle1: Square = base_dxf.rectangle_locate("RIGHT: UPPER: Rectangle 1",
                                                             -1.511965, 3.749122,
                                                             -1.460783, 3.897803)
        upper_rectangle2: Square = base_dxf.rectangle_locate("RIGHT: UPPER: Rectangle 2",
                                                             -1.511965, 3.963677,
                                                             -1.460783, 4.076929)
        upper_rectangle3: Square = base_dxf.rectangle_locate("RIGHT: UPPER: Rectangle 3",
                                                             -1.511965, 4.142815,
                                                             -1.460783, 4.291496)
        upper_rectangles: List[Square] = [
            upper_rectangle0,
            upper_rectangle1,
            upper_rectangle2,
            upper_rectangle3
        ]
        upper_rectangle: Square
        lower_rectangles: List[Square] = []
        for upper_rectangle in upper_rectangles:
            lower_rectangle: SimplePolygon = upper_rectangle.x_mirror("UPPER:", "LOWER:")
            assert isinstance(lower_rectangle, Square)
            lower_rectangles.append(lower_rectangle)
        vertical_rectangles: List[Square] = upper_rectangles + lower_rectangles
        return vertical_rectangles

    # RomiBase.base_keys_get():
    def keys_get(self) -> List[Tuple[Any, ...]]:
        """Return the RomiBase hole/rectangle/slot keys."""
        romi_base: RomiBase = self
        romi_base_polygon: Polygon = romi_base.base_polygon_get()
        romi_base_simple_polygons: List[SimplePolygon] = romi_base_polygon.simple_polygons_get()
        romi_base_simple_polygon: SimplePolygon
        keys: List[Tuple[Any, ...]] = [romi_base_simple_polygon.key()
                                       for romi_base_simple_polygon
                                       in romi_base_simple_polygons[1:]]  # Note 1st one is skipped
        # Sort *keys* and return the result:
        keys.sort()
        return keys

    # RomiBase.holes_slots_rectangles_write():
    # def holes_slots_rectangles_write(self):
    #     """Write out the holes, slots, and rectangle lcoations."""
    #     # Obtain the hole, slot and rectangle locations:
    #     romi_base: RomiBase = self
    #     romi_base_keys: List[Tuple[Any, ...]] = romi_base.keys_get()
    #
    #     # Now output the hole/slot/rectangle `.csv` file:
    #     csv_file: IO[Any]
    #     with open("romi_base.csv", "w") as csv_file:
    #         Scad.keys_csv_file_write(romi_base_keys, csv_file)
    #
    #     # Output the hole/slot/recantangle `.html` file:
    #     html_file: IO[Any]
    #     with open("romi_base.html", "w") as html_file:
    #         Scad.keys_html_file_write(romi_base_keys, html_file, "Romi Base Holes and Rectangles")


# RomiExpansionPlate:
class RomiExpansionPlate:
    """Represents a Romi Expansion Plate."""

    # RomiExpansion.__init__():
    def __init__(self, scad_program: ScadProgram) -> None:
        """Initialize RomiExpansion and append to ScadProgram."""
        self.expansion_dxf: ExpansionDXF = ExpansionDXF()
        romi_expansion_plate: RomiExpansionPlate = self
        romi_expansion_polygon: Polygon = romi_expansion_plate.polygon_get()
        romi_expansion_polygon_module: Module2D = Module2D("Romi Expansion Flat",
                                                           [romi_expansion_polygon])
        scad_program.append(romi_expansion_polygon_module)
        scad_program.if2d.name_match_append("expansion_flat", romi_expansion_polygon_module,
                                            ["Flat Romi Expansion Plate"])

        plate_dz: float = 2.45  # Measured using calipers.
        extruded_expansion_plate: LinearExtrude = LinearExtrude("Extruded Expansion Plate",
                                                                romi_expansion_polygon, plate_dz)
        # Stuff *module* into *scad_program* and *romi_expansion_plate* (i.e. *self*):
        module: Module3D = Module3D("Romi Expansion Plate", [extruded_expansion_plate])
        scad_program.append(module)
        # romi_expansion_plate: RomiExpansionPlate = self
        self.polygon: Polygon = romi_expansion_polygon
        self.module: Module3D = module
        scad_program.if3d.name_match_append("expansion_plate", module, ["Romi Expansion Plate"])

    # RomiExpansionPlate.hex_holes_slot_get():
    def hex_holes_slots_get(self) -> List[SimplePolygon]:
        """Return the expansion chasis holes and slots."""
        # romi_expansion_plate: RomiExpansionPlate = self
        # Compute some constants
        large_hole_diameter: float = 3.2
        small_hole_diameter: float = 2.4
        hole_circle_diameter: float = 17.3
        hole_circle_radius: float = hole_circle_diameter / 2.0
        inner_slot_diameter: float = 10.0
        inner_slot_radius: float = inner_slot_diameter / 2.0  # Center of inner slot arc
        slot_center_to_center: float = 5.0
        slot_center_radius = inner_slot_radius + slot_center_to_center / 2.0
        slot_dy: float = small_hole_diameter
        slot_dx: float = small_hole_diameter + slot_center_to_center
        slot_corner_radius: float = small_hole_diameter / 2.0
        slot_corner_count: int = 3
        total_dy: float = 81.5
        hex_origin: P2D = P2D(0.0, -total_dy + 26.4)
        delta_angle: float = pi / 3.0
        hole_angle_offset: float = pi / 6.0
        horizontal_hole_pitch: float = 2 * inner_slot_radius + slot_center_to_center

        # Collect everything in the *holes* and *slots* lists:
        holes: List[Circle] = []
        slots: List[Square] = []

        # These tables are tediously defined:
        allowed_slots: Set[Tuple[int, int]] = {
            (0, 0), (0, 1), (0, 2),
            (1, 0), (1, 1), (1, 2), (1, 4), (1, 5),
            (2, 0), (2, 1), (2, 2), (2, 4), (2, 5),
            (3, 0), (3, 1), (3, 2), (3, 4), (3, 5),
            (4, 0), (4, 1), (4, 2), (4, 4), (4, 5),
            (5, 0), (5, 1), (5, 2), (5, 4), (5, 5),
            (6, 1), (6, 2),
        }
        allowed_holes: Set[Tuple[int, int]] = {
            (0, 0), (0, 1),
            (1, 0), (1, 1), (1, 4), (1, 5),
            (2, 0), (2, 1), (2, 4), (2, 5),
            (3, 0), (3, 1), (3, 4), (3, 5),
            (4, 0), (4, 1), (4, 4), (4, 5),
            (5, 0), (5, 1), (5, 4),
            (6, 1),
        }

        for hole_index in range(7):
            # Create the primary slot hole:
            center_hole: P2D = hex_origin + P2D(float(hole_index - 3) * horizontal_hole_pitch, 0.0)
            primary_hole: Circle = Circle(f"Primary Hole[{hole_index}]",
                                          large_hole_diameter, 8, center=center_hole)
            holes.append(primary_hole)

            # Sweep around *hole* in 60 degree increments putting ing slots and holes:
            for angle_index in range(6):
                slot_angle: float = float(angle_index) * delta_angle
                key: Tuple[int, int] = (hole_index, angle_index)
                if key in allowed_slots:
                    slot_x: float = slot_center_radius * cos(slot_angle)
                    slot_y: float = slot_center_radius * sin(slot_angle)
                    slot_center: P2D = center_hole + P2D(slot_x, slot_y)
                    slot: Square = Square(f"Slot[{hole_index},{angle_index}]", slot_dx, slot_dy,
                                          center=slot_center, rotate=slot_angle,
                                          corner_radius=slot_corner_radius,
                                          corner_count=slot_corner_count)
                    slots.append(slot)

                # Create the *hole* and append to *holes*:
                if key in allowed_holes:
                    hole_angle: float = slot_angle + hole_angle_offset
                    hole_x: float = hole_circle_radius * cos(hole_angle)
                    hole_y: float = hole_circle_radius * sin(hole_angle)
                    hole_center: P2D = center_hole + P2D(hole_x, hole_y)
                    hole = Circle(f"Angle Hole[{hole_index},{angle_index}]",
                                  large_hole_diameter, 8, center=hole_center)
                    holes.append(hole)

        bottom_index: int
        bottom_y: float = hex_origin.y - 1.5 * hole_circle_radius
        for bottom_index in range(5):
            bottom_x: float = float(2 - bottom_index) * hole_circle_radius
            bottom_center: P2D = P2D(bottom_x, bottom_y)
            if bottom_index % 2 == 0:
                # Bottom Slot:
                bottom_slot: Square = Square(f"Bottom Slot[{bottom_index}]", slot_dx, slot_dy,
                                             center=bottom_center, rotate=0.0,
                                             corner_radius=slot_corner_radius,
                                             corner_count=slot_corner_count)
                slots.append(bottom_slot)
            else:
                # Bottom Hole:
                bottom_hole: Circle = Circle(f"Bottom Hole[{bottom_index}]",
                                             large_hole_diameter, 8, center=bottom_center)
                holes.append(bottom_hole)

        # Fill *simple_polygons* with *holes* and *slots* and return it:
        simple_polygons: List[SimplePolygon] = []
        simple_polygons.extend(holes)
        simple_polygons.extend(slots)
        return simple_polygons

    # RomiExpansionPlate.large_holes_get():
    def large_holes_get(self) -> List[Circle]:
        """Return the expansion chasis small hole pattern."""
        # romi_expanson_plate: RomiExpansionPlate = self
        # Define some constants from dimentions `.pdf`:
        total_dy: float = 81.5
        large_hole_diameter: float = 3.2
        x_pitch: float = 10.0
        x_start: float = -4.0 * x_pitch
        y_start: float = -total_dy + 46.2
        y_pitch: float = 12.25
        y_pitches: List[float] = [0.0, y_pitch, 2.0 * y_pitch, 2.0 * y_pitch - 51.0]

        # Assemble all of the holes into *large_holes*:
        large_holes: List[Circle] = []
        x_index: int
        for x_index in range(9):
            x: float = x_start + float(x_index) * x_pitch
            y_index: int
            for y_index in range(4):
                # print(f"({x_index}, {y_index})")
                y: float = y_start + y_pitches[y_index]
                if y_index in (0, 1) and x_index == 4:  # Skip column 4
                    # print(f"  Skip column 4 Large Hole[{x_index}, {y_index}]")
                    pass
                elif y_index in (2, 3) and x_index in (1, 2, 3, 4, 5, 6, 7):  # Skip middle columns
                    # print(f"  Skip middle column Large Hole[{x_index}, {y_index}]")
                    pass
                else:
                    # print(f"  Large Hole[{x_index},{y_index}]")
                    center: P2D = P2D(x, y)
                    large_hole: Circle = Circle(f"Large Hole[{x_index}, {y_index}]",
                                                large_hole_diameter, 8, center)
                    large_holes.append(large_hole)
        return large_holes

    # RomiExpansionPlate.keys_get():
    def keys_get(self) -> List[Tuple[Any, ...]]:
        """Return the RomiExpansionPlate hole/rectangle/slot keys."""
        romi_expansion_plate: RomiExpansionPlate = self
        romi_expansion_plate_polygon: Polygon = romi_expansion_plate.polygon
        romi_expansion_plate_simple_polygons: List[SimplePolygon]
        romi_expansion_plate_simple_polygons = romi_expansion_plate_polygon.simple_polygons_get()
        romi_expansion_plate_simple_polygon: SimplePolygon
        keys: List[Tuple[Any, ...]] = [romi_expansion_plate_simple_polygon.key()
                                       for romi_expansion_plate_simple_polygon
                                       in romi_expansion_plate_simple_polygons[1:]]  # skip 1st one
        # Sort *keys* and return the result:
        keys.sort()
        return keys

    # RomiExpansionPlate.polygon_get():
    def polygon_get(self) -> Polygon:
        """Return outline of Romi Expansion Chasis."""
        romi_expansion_plate: RomiExpansionPlate = self
        # The various dimensions are from `mechanical/pdf/romi-chasis-expansion_plate.pdf`
        # We use the same orientation as in the `.pdf` file, where the rounded portion is
        # towards the bottom and the flat portion is on top.  The origin is set to be at the
        # center of the top edge.  All measurements are in millimeters:
        total_dx: float = 145.5
        half_total_dx: float = total_dx / 2.0
        wheel_well_dx: float = 124.0
        half_wheel_well_dx: float = wheel_well_dx / 2.0
        total_dy: float = 81.5   # The `.pdf` says 81.6, but 81.5 matches the Romi Chasis radius.
        wheel_well_dy: float = 19.6

        # We need to figure out the (*arc_x*, *arc_y) value for the arc-end point.  To make the math
        # a little simpler, we will do the math in the quandrant 1 so both *arc_x* and *arc_y*
        # are positive.  Minus signs are added as needed when using *arg_x* and *arc_y*.
        # We start with Pathagorean's Theorum:
        #
        #     (1)   r^2 = dx^2 + dy^2
        #
        # Both r and dx are known, so we can solve for dy:
        #
        #     (2)   dy^2 = r^2 - dx^2
        #     (3)   dy = sqrt(r^2 - dx^2)
        #
        # That was easy!
        arc_radius: float = total_dy  # From the Romi Chasis dimensions.
        arc_dx: float = half_total_dx
        arc_dy: float = sqrt(arc_radius * arc_radius - arc_dx * arc_dx)
        arc_angle: float = atan2(arc_dy, arc_dx)

        # Create *expanson_outer* and start filling in points:
        expansion_outer: SimplePolygon = SimplePolygon("Expansion_Outer", [])

        # Draw the top edge and left wheel well:
        expansion_outer.point_append(P2D(half_wheel_well_dx, 0.0))              # Upper right
        expansion_outer.point_append(P2D(-half_wheel_well_dx, 0.0))             # Upper left
        expansion_outer.point_append(P2D(-half_wheel_well_dx, -wheel_well_dy))  # Inner corner
        expansion_outer.point_append(P2D(-half_total_dx, -wheel_well_dy))       # Outer corner

        # # Now draw the arc along the bottom:
        origin: P2D = P2D(0.0, 0.0)
        start_angle: float = arc_angle + pi
        end_angle: float = 2.0 * pi - arc_angle
        # print(f"start_angle={degrees(start_angle)}")
        # print(f"end_angle={degrees(end_angle)}")
        expansion_outer.point_append(P2D(-half_total_dx, -arc_dy))                  # Arc start
        expansion_outer.arc_append(origin, arc_radius, start_angle, end_angle, 21)  # Arc

        # Draw the right wheel well and lock up *expansion_outer*:
        expansion_outer.point_append(P2D(half_total_dx, -wheel_well_dy))       # Outer corner
        expansion_outer.point_append(P2D(half_wheel_well_dx, -wheel_well_dy))  # Inner corner
        expansion_outer.lock()

        # Fill in *simple_polygons* starting with *expansion_outer* and the all of the
        # other holes, rectangles and slots:
        simple_polygons: List[SimplePolygon] = [expansion_outer]
        expansion_large_holes: List[Circle] = romi_expansion_plate.large_holes_get()
        simple_polygons.extend(expansion_large_holes)
        expansion_small_holes: List[Circle] = romi_expansion_plate.small_holes_get()
        simple_polygons.extend(expansion_small_holes)
        expansion_hex_holes_slots: List[SimplePolygon] = romi_expansion_plate.hex_holes_slots_get()
        simple_polygons.extend(expansion_hex_holes_slots)
        expansion_standoff_holes: List[Circle] = romi_expansion_plate.standoff_holes_get()
        simple_polygons.extend(expansion_standoff_holes)
        top_slots: List[SimplePolygon] = romi_expansion_plate.top_slots_get()
        simple_polygons.extend(top_slots)
        miscellaneous_polygons: List[SimplePolygon]
        miscellaneous_polygons = romi_expansion_plate.miscellaneous_polygons_get()
        assert len(miscellaneous_polygons) >= 1
        simple_polygons.extend(miscellaneous_polygons)

        # Create and return *expansion_polygon* in an unlocked state:
        expansion_polygon: Polygon = Polygon("Expansion", simple_polygons, lock=True)
        return expansion_polygon

    # RomiExpansionPlate.small_holes_get():
    def small_holes_get(self) -> List[Circle]:
        """Return the expansion chasis small hole pattern."""
        # romi_expansion_plate: RomiExpansionPlate = self
        # Define some constants from dimentions `.pdf`:
        total_dy: float = 81.5
        small_hole_diameter: float = 2.4
        x_pitch: float = 10.0
        x_start: float = -4.5 * x_pitch
        y_start: float = -total_dy + 40.1
        y_pitch: float = 12.25
        y_pitches: List[float] = [0.0, y_pitch, 2.0 * y_pitch, 2.0 * y_pitch + x_pitch]

        # Assemble all of the holes into *small_holes*:
        small_holes: List[Circle] = []
        y_index: int
        for y_index in range(4):
            y: float = y_start + y_pitches[y_index]
            x_index: int
            for x_index in range(10):
                x: float = x_start + float(x_index) * x_pitch
                center: P2D = P2D(x, y)
                small_hole: Circle = Circle(f"Small Hole[{x_index}, {y_index}]",
                                            small_hole_diameter, 8, center)
                small_holes.append(small_hole)
        return small_holes

    # RomiExpansionPlate.standoff_holes_get():
    def standoff_holes_get(self) -> List[Circle]:
        """Produce the 6 standoff holes."""
        # Define some constants:
        small_hole_diameter: float = 2.4
        # inner_upper_dx: float = 90.0
        outer_upper_dx: float = 106.0
        lower_dx: float = 48.2
        x0: float = -outer_upper_dx / 2.0
        # x1: float = -inner_upper_dx / 2.0
        x2: float = -lower_dx / 2.0
        x3: float = lower_dx / 2.0
        # x4: float = inner_upper_dx / 2.0
        x5: float = outer_upper_dx / 2.0
        # y2: float = -17.0
        y1: float = -19.0
        y0: float = -81.5 + 10.4

        standoff_centers: List[P2D] = [P2D(x0, y1), P2D(x2, y0), P2D(x3, y0), P2D(x5, y1)]
        standoff_holes: List[Circle] = []
        standoff_center: P2D
        standoff_index: int
        for standoff_index, standoff_center in enumerate(standoff_centers):
            standoff_hole: Circle = Circle(f"Standoff[{standoff_index}",
                                           small_hole_diameter, 8, center=standoff_center)
            standoff_holes.append(standoff_hole)
        return standoff_holes

    # RomiExpansionPlate.miscellaneous_polygons_get():
    def miscellaneous_polygons_get(self) -> List[SimplePolygon]:
        """Return the miscellaneous polygons."""
        # Grab the *expansion_dxf* from *romi_expansion_plate* (i.e. *self*):
        romi_expansion_plate: RomiExpansionPlate = self
        expansion_dxf: ExpansionDXF = romi_expansion_plate.expansion_dxf

        # Just crunch through varous random slots and holes:
        right_top_vertical_slot: SimplePolygon
        right_top_vertical_slot = expansion_dxf.slot_locate("RIGHT: Top Vertical Slot",
                                                            -0.043752, 3.487524,
                                                            0.050693, 3.701413)
        right_top_small_hole: Circle = expansion_dxf.hole_locate("RIGHT: Top small hole",
                                                                 -0.043752, 3.293079,
                                                                 0.050693, 3.386134)
        right_horizontal_slot: Square = expansion_dxf.slot_locate("RIGHT: Horizontal slot",
                                                                  0.202079, 2.866689,
                                                                  0.414579, 2.961134)
        right_small_vertical_slot: Square = expansion_dxf.slot_locate("RIGHT: Small Vertical Slot",
                                                                      0.261803, 2.470858,
                                                                      0.356248, 2.702803)
        right_small_rectangle: Square = expansion_dxf.rectangle_locate("Right: Small Rectangle",
                                                                       0.510413, 2.476413,
                                                                       0.561803, 2.625024)
        right_medium_rectangle: Square = expansion_dxf.rectangle_locate("RIGHT: Medium Rectangle",
                                                                        0.053469, 2.404189,
                                                                        0.161803, 2.718079)
        right_large_rectangle: Square = expansion_dxf.rectangle_locate("RIGHT: Large Rectangle",
                                                                       -0.339587, 2.206969,
                                                                       -0.231252, 2.718079)
        right_top_triple_hole: Circle = expansion_dxf.hole_locate("RIGHT: Top Triple Hole",
                                                                  -0.139587, 2.513913,
                                                                  -0.045142, 2.608358)
        right_middle_triple_hole: Circle = expansion_dxf.hole_locate("RIGHT: Middle Triple Hole",
                                                                     -0.154866, 2.301413,
                                                                     -0.029866, 2.422248)
        right_bottom_triple_hole: Circle = expansion_dxf.hole_locate("RIGHT: Bottom Triple Hole",
                                                                     -0.139587, 2.119469,
                                                                     -0.045142, 2.213913)

        # There are three small holes (center, right1, right2) just below the top row of holes.
        # The right two holes are reflected to get a total of five holes:
        center_top_small_hole: Circle = expansion_dxf.hole_locate("CENTER: Top Small Hole",
                                                                  -2.297921, 3.369469,
                                                                  -2.203476, 3.463913)
        right_top_middle_hole: Circle = expansion_dxf.hole_locate("RIGHT: Top Middle Small Hole",
                                                                  -1.510421, 3.369469,
                                                                  -1.415976, 3.463913)
        right_top_right_hole: Circle = expansion_dxf.hole_locate("RIGHT: Top Right Small Hole",
                                                                 -1.115976, 3.369469,
                                                                 -1.021531, 3.463913)
        # There is are two odd tower holes used to mount the arm tower:
        center_upper_tower_hole: Circle = expansion_dxf.hole_locate("CENTER: Upper Tower Hole",
                                                                    -2.313197, 3.034748,
                                                                    -2.186807, 3.161134)
        center_lower_tower_hole: Circle = expansion_dxf.hole_locate("CENTER: Upper Lower Hole",
                                                                    -2.313197, 2.248634,
                                                                    -2.186807, 2.373634)

        right_polygons: List[SimplePolygon] = [
            right_top_vertical_slot,
            right_top_small_hole,
            right_horizontal_slot,
            right_small_vertical_slot,
            right_small_rectangle,
            right_medium_rectangle,
            right_large_rectangle,
            right_top_triple_hole,
            right_middle_triple_hole,
            right_bottom_triple_hole,
            right_top_middle_hole,
            right_top_right_hole,
        ]
        center_polygons: List[SimplePolygon] = [
            center_top_small_hole,
            center_upper_tower_hole,
            center_lower_tower_hole,
        ]
        right_polygon: SimplePolygon
        left_polygons: List[SimplePolygon] = [right_polygon.y_mirror("RIGHT", "LEFT")
                                              for right_polygon in right_polygons]
        all_polygons: List[SimplePolygon] = left_polygons + center_polygons + right_polygons
        return all_polygons

    # RomiExpansionPlate.top_slots_get():
    def top_slots_get(self) -> List[SimplePolygon]:
        """Return the 9 top slots."""
        # Grab *expansion_dxf* from *romi_expansion_plate* (i.e. *self*):
        romi_expansion_plate: RomiExpansionPlate = self
        expansion_dxf: ExpansionDXF = romi_expansion_plate.expansion_dxf

        center_slot: SimplePolygon = expansion_dxf.rectangle_locate("CENTER: Top Slot",
                                                                    -2.324307, 3.758358,
                                                                    -2.175697, 3.706969)
        right_slot1: SimplePolygon = expansion_dxf.rectangle_locate("RIGHT: Top Slot 1",
                                                                    -2.111807, 3.758358,
                                                                    -1.995142, 3.706969)
        right_slot2: SimplePolygon = expansion_dxf.rectangle_locate("RIGHT: Top Slot 1",
                                                                    -1.915976, 3.758358,
                                                                    -1.797921, 3.706969)
        right_slot3: SimplePolygon = expansion_dxf.rectangle_locate("RIGHT: Top Slot 1",
                                                                    -1.734031, 3.758358,
                                                                    -1.585421, 3.706969)
        right_slot4: SimplePolygon = expansion_dxf.rectangle_locate("RIGHT: Top Slot 1",
                                                                    -1.521531, 3.758358,
                                                                    -1.404866, 3.706969)

        right_slots: List[SimplePolygon] = [right_slot1, right_slot2, right_slot3, right_slot4]
        left_slots: List[SimplePolygon] = [right_slot.y_mirror("RIGHT", "LEFT")
                                           for right_slot in right_slots]
        all_top_slots: List[SimplePolygon] = [center_slot] + right_slots + left_slots
        return all_top_slots


# RomiMagnet:
class RomiMagnet:
    """Represents the Romi Encoder Magnet."""

    # RomiMagnet.__init__():
    def __init__(self, scad_program: ScadProgram, base_dxf: BaseDXF) -> None:
        """Initialize RomiMagnet and append to ScadProgram."""
        # Find the east end of the west motor shaft:
        motor_shaft_east_x: float = base_dxf.x_locate(-5.08598)

        # The motor shaft is aligned with the Y-axis a 0.0:
        motor_shaft_north_y: float = base_dxf.y_locate(2.967165)
        motor_shaft_south_y: float = base_dxf.y_locate(2.908110)
        motor_shaft_diameter: float = abs(motor_shaft_north_y - motor_shaft_south_y)

        # Compute the top and bottom of the motor electrical connectors.  Use the center
        # of this to be *motor_shaft_z*.  The Z axis is mapped to the Y axis for the front view:
        electrical_top_z: float = base_dxf.z_locate(-1.842126)
        electrical_bottom_z: float = base_dxf.z_locate(-1.926776)
        # The center of the electrical connector is assumed to be aligned with the *motor_shaft_z*:
        motor_shaft_z: float = (electrical_top_z + electrical_bottom_z) / 2.0

        # A couple of values are not available from the `.dxf`, so they are measured using calipers:
        magnet_diameter: float = 9.75
        magnet_thickness: float = 2.05

        # Create a round *Polygon* with hole in the middle for the motor shaft:
        magnet_exterior: Circle = Circle("Magnet Exterior", magnet_diameter, 32)
        motor_shaft_hole: Circle = Circle("Magnet Shaft Hole", motor_shaft_diameter, 16)
        magnet_polygon: Polygon = Polygon("Magnet Polygon", [magnet_exterior, motor_shaft_hole])

        # Extrude, rotate and translate *magnet_polygon* into *translated_magnet*:
        flat_magnet: LinearExtrude = LinearExtrude("Flat Magnet", magnet_polygon, magnet_thickness)
        vertical_magnet: Rotate3D = Rotate3D("Vertical Magnet", flat_magnet,
                                             -pi/2.0, P3D(0.0, 1.0, 0.0))
        translated_magnet: Translate3D = Translate3D("Translated Magnet", vertical_magnet,
                                                     P3D(motor_shaft_east_x, 0.0, motor_shaft_z))
        colored_magnet: Color = Color("Colored Magnet", translated_magnet, "SaddleBrown")

        # Create *module*, append it to *scad_program*  and save into *RomiMagnet* (i.e. *self*):
        module: Module3D = Module3D("Magnet Module", [colored_magnet])
        scad_program.append(module)
        self.module: Module3D = module


# RomiMotor:
class RomiMotor:
    """Represents the RomiMotor."""

    # RomiMotor.__init__():
    def __init__(self, scad_program: ScadProgram, base_dxf: BaseDXF) -> None:
        """Initialize RomiMotor."""
        # Various values off of the Top view of the `.dxf` file.
        # Start with X coordinates:
        # Start grabbing some X values from west to east:
        gearbox_casing_west_x: float = base_dxf.x_locate(-6.326134)
        # Gearbox/motor surface X
        gearbox_motor_casing_x: float = base_dxf.x_locate(-5.782827)
        gearbox_casing_dx: float = abs(gearbox_motor_casing_x - gearbox_casing_west_x)
        motor_casing_west_x: float = base_dxf.x_locate(-5.253299)
        electrical_east_x: float = motor_casing_west_x
        electrical_west_x: float = base_dxf.x_locate(-5.074161)
        motor_shaft_west_x: float = motor_casing_west_x
        motor_shaft_east_x: float = base_dxf.x_locate(-5.085980)

        # Start grabbing some Y values:
        # Compute the *motor_shaft_diameter*:
        motor_shaft_north_y: float = base_dxf.y_locate(2.967165)
        motor_shaft_south_y: float = base_dxf.y_locate(2.908110)
        motor_shaft_diameter: float = abs(motor_shaft_north_y - motor_shaft_south_y)

        # Read off the *motor_casing_dy*:
        motor_casing_north_y: float = base_dxf.y_locate(3.382512)
        motor_casing_south_y: float = base_dxf.y_locate(2.492748)
        motor_casing_dy: float = abs(motor_casing_north_y - motor_casing_south_y)

        # Read off the *gearbox_casing_dy*:
        gearbox_casing_north_y: float = base_dxf.y_locate(3.331331)
        gearbox_casing_south_y: float = base_dxf.y_locate(2.543929)
        gearbox_casing_dy: float = abs(gearbox_casing_north_y - gearbox_casing_south_y)

        # Now capture the north electrical tabs:
        electrical_north_upper_y: float = base_dxf.y_locate(3.208303)
        electrical_north_lower_y: float = base_dxf.y_locate(3.188610)
        electrical_north_dy: float = abs(electrical_north_upper_y - electrical_north_lower_y)
        electrical_north_y: float = (electrical_north_upper_y + electrical_north_lower_y) / 2.0

        # Now capture the south electrical tab:
        electrical_south_upper_y: float = base_dxf.y_locate(2.686650)
        electrical_south_lower_y: float = base_dxf.y_locate(2.666957)
        electrical_south_dy: float = abs(electrical_south_upper_y - electrical_south_lower_y)
        electrical_south_y: float = (electrical_south_upper_y + electrical_south_lower_y) / 2.0

        # Compute *electrical_dy*:
        electrical_dy: float = (electrical_north_dy + electrical_south_dy) / 2.0

        # Read off off `.dxf` Front view:
        # Do some Z dimensions:
        wheel_shaft_diameter: float = base_dxf.wheel_shaft_diameter

        # Grab the electrical tab locations:
        electrical_top_z: float = base_dxf.z_locate(-1.842126)
        electrical_bottom_z: float = base_dxf.z_locate(-1.926776)
        electrical_dz: float = abs(electrical_top_z - electrical_bottom_z)
        electrical_z: float = (electrical_top_z + electrical_bottom_z) / 2.0
        # By inference, the *motor_shaft_z* is the same:
        motor_shaft_z: float = electrical_z

        # Compute *motor_casing_dz*, *top_motor_casing_z*, and *bottom_motor_casing_z*:
        motor_casing_top_z: float = base_dxf.z_locate(-1.658071)
        motor_casing_bottom_z: float = base_dxf.z_locate(-2.110835)
        # motor_casing_dz: float = (motor_casing_top_z + motor_casing_bottom_z) / 2.0
        motor_casing_top_z = motor_casing_top_z
        motor_casing_bottom_z = motor_casing_bottom_z

        # Compute *gearbox_casing_dz* realizing the top half goes from the wheel axle (i.e. Y==0)
        # and the bottom half is a half circle of diameter *gearbox_casing_dy*:
        gearbox_casing_top_z: float = motor_casing_top_z

        # Measure *wheel_shaft_dx* length using calipers:
        wheel_shaft_dx: float = 9.75

        # The *gearbox_casing* is pretty involved.  It is cube with a rounded bottom.
        # It is linear extruded from a rectangular polygon with a rounded bottom:
        gearbox_case_polygon: SimplePolygon = SimplePolygon("Gearbox Simple Polygon", [])
        # The polygon needs to be layed out in the X/Y plane, where as the final orientation
        # is in the Y/Z plane.  So we substitute Z values for X values in the *P2D* below.
        # We lay this polygon out with the wheel axis pointing upward from the origin:
        gearbox_case_polygon.point_append(P2D(gearbox_casing_top_z, gearbox_casing_dy/2.0))
        gearbox_case_polygon.arc_append(center=P2D(0.0, 0.0),  # Wheel axis is at (0.0, 0.0)
                                        radius=gearbox_casing_dy/2.0,
                                        start_angle=pi/2.0, end_angle=3.0*pi/2.0,
                                        points_count=16)
        gearbox_case_polygon.point_append(P2D(gearbox_casing_top_z, -gearbox_casing_dy/2.0))
        gearbox_case_polygon.lock()
        extruded_gearbox_case: LinearExtrude = LinearExtrude("Gearbox Casing Linear Extrude",
                                                             gearbox_case_polygon,
                                                             gearbox_casing_dx)
        upright_gearbox: Rotate3D = Rotate3D("Upright Gearbox", extruded_gearbox_case,
                                             -pi/2.0, P3D(0.0, 1.0, 0.0))
        gearbox_casing: Translate3D = Translate3D("Geabox Casing", upright_gearbox,
                                                  P3D(gearbox_motor_casing_x, 0.0, 0.0))

        # Construct everything else out of cubes of material:
        motor_casing: CornerCube = CornerCube("Motor Casing",
                                              P3D(gearbox_motor_casing_x,
                                                  -motor_casing_dy/2.0,
                                                  motor_casing_top_z),
                                              P3D(motor_casing_west_x,
                                                  motor_casing_dy/2.0,
                                                  motor_casing_bottom_z))
        motor_shaft: Cylinder = Cylinder("Motor Shaft", motor_shaft_diameter,
                                         P3D(motor_shaft_west_x, 0.0, motor_shaft_z),
                                         P3D(motor_shaft_east_x, 0.0, motor_shaft_z),
                                         16)
        wheel_shaft: Cylinder = Cylinder("Wheel Shaft", wheel_shaft_diameter,
                                         P3D(gearbox_casing_west_x - wheel_shaft_dx, 0.0, 0.0),
                                         P3D(gearbox_casing_west_x, 0.0, 0.0),
                                         16)
        electrical_north: CornerCube = CornerCube("Electrical North",
                                                  P3D(electrical_west_x,
                                                      electrical_north_y - electrical_dy/2.0,
                                                      electrical_z - electrical_dz/2.0),
                                                  P3D(electrical_east_x,
                                                      electrical_north_y + electrical_dy/2.0,
                                                      electrical_z + electrical_dz/2.0))
        electrical_south: CornerCube = CornerCube("Electrical South",
                                                  P3D(electrical_west_x,
                                                      electrical_south_y - electrical_dy/2.0,
                                                      electrical_z - electrical_dz/2.0),
                                                  P3D(electrical_east_x,
                                                      electrical_south_y + electrical_dy/2.0,
                                                      electrical_z + electrical_dz/2.0))

        # Create a *union* to store all of the parts into:
        union: Union3D = Union3D("Romi Motor Union 3D",
                                 [Color("Wheat Color", gearbox_casing, "Wheat"),
                                  Color("Wheat Color", motor_casing, "Wheat"),
                                  Color("Silver Color", motor_shaft, "Silver"),
                                  Color("Silver Color", wheel_shaft, "Silver"),
                                  Color("Gold Color", electrical_north, "Gold"),
                                  Color("Gold Color", electrical_south, "Gold")])

        # Create *module*, append to *scad_program,  and save into *romi_motor* (i.e. *self*):
        module: Module3D = Module3D("Romi Motor Module", [union])
        scad_program.append(module)
        # romi_motor: RomiMotor = self
        self.module: Module3D = module


# RomiMotorHolder:
class RomiMotorHolder:
    """Represents the Romi Chasis Motor Holder."""

    # RomiMotorHolder.__init__():
    def __init__(self, scad_program: ScadProgram, base_dxf: BaseDXF) -> None:
        """Initialize the RomiMotorHolder and append to ScadProgram."""
        # Start reading values off of *base_dxf*:
        # Start with X coordinates:
        # Now grab some X coordinates from west to east for the holder:
        holder_west_x: float = base_dxf.x_locate(-6.40487)
        motor_gearbox_west_x: float = base_dxf.x_locate(-6.326134)
        motor_clip_west_x: float = base_dxf.x_locate(-6.267075)
        motor_clip_east_x: float = base_dxf.x_locate(-5.991492)
        motor_gearbox_east_x: float = base_dxf.x_locate(-5.782827)
        base_clip_west_x: float = base_dxf.x_locate(-5.704091)
        base_clip_east_x: float = base_dxf.x_locate(-5.625350)

        # Lastly the bottom tabs are not really measurable from the `.dxf`, so use calipers:
        west_tab_dx: float = 2.60
        east_tab_dx: float = 2.10

        # Now grap some Y coodinates from south to north for the holder:
        holder_south_y: float = base_dxf.y_locate(2.465193)
        motor_gearbox_south_y: float = base_dxf.y_locate(2.543929)
        clip_lip_south_y: float = base_dxf.y_locate(2.642358)
        clip_lip_north_y: float = base_dxf.y_locate(3.232913)
        motor_gearbox_north_y: float = base_dxf.y_locate(3.331331)
        holder_north_y: float = base_dxf.y_locate(3.410067)
        base_clip_dy: float = 4.75  # Measured using calipers:
        # holder_dy: float = abs(holder_north_y - holder_south_y)
        motor_gearbox_dy: float = abs(motor_gearbox_north_y - motor_gearbox_south_y)

        # Grab some Z coordinates starting from top to bottom:
        clip_top_z: float = base_dxf.z_locate(-1.559654)
        motor_casing_top_z: float = base_dxf.z_locate(-1.658071)
        motor_casing_bottom_z: float = base_dxf.z_locate(-2.110835)
        lip_north_z: float = base_dxf.z_locate(-2.228945)
        lip_south_z: float = base_dxf.z_locate(-2.622638)
        base_clip_top_z: float = base_dxf.z_locate(-2.560638)
        battery_base_top_z: float = base_dxf.z_locate(-2.701374)
        base_top_z: float = base_dxf.z_locate(-3.095083)
        base_bottom_z: float = base_dxf.z_locate(-3.469098)

        # Start with the two east most "lips":
        base_clip: CornerCube = CornerCube("Base Clip",
                                           P3D(base_clip_west_x,
                                               -base_clip_dy/2.0, battery_base_top_z),
                                           P3D(base_clip_east_x,
                                               base_clip_dy/2.0, base_clip_top_z))
        east_side: CornerCube = CornerCube("East Side",
                                           P3D(motor_gearbox_east_x,
                                               holder_south_y, base_bottom_z),
                                           P3D(motor_gearbox_east_x + east_tab_dx,
                                               holder_north_y, motor_casing_bottom_z))
        north_clip: CornerCube = CornerCube("South Clip",
                                            P3D(motor_clip_west_x,
                                                motor_gearbox_north_y, motor_casing_bottom_z),
                                            P3D(motor_clip_east_x,
                                                holder_north_y, clip_top_z))
        north_latch: CornerCube = CornerCube("North Latch",
                                             P3D(motor_clip_west_x,
                                                 clip_lip_north_y, motor_casing_top_z),
                                             P3D(motor_clip_east_x,
                                                 holder_north_y, clip_top_z))
        north_lip: CornerCube = CornerCube("North Lip",
                                           P3D(holder_west_x,
                                               holder_north_y, lip_south_z),
                                           P3D(motor_gearbox_west_x,
                                               clip_lip_north_y, lip_north_z))
        north_side: CornerCube = CornerCube("North Side",
                                            P3D(motor_gearbox_west_x,
                                                motor_gearbox_north_y, base_top_z),
                                            P3D(motor_gearbox_east_x + east_tab_dx,
                                                holder_north_y, motor_casing_bottom_z))
        south_clip: CornerCube = CornerCube("North Clip",
                                            P3D(motor_clip_west_x,
                                                holder_south_y, motor_casing_bottom_z),
                                            P3D(motor_clip_east_x,
                                                motor_gearbox_south_y, clip_top_z))
        south_latch: CornerCube = CornerCube("South Latch",
                                             P3D(motor_clip_west_x,
                                                 holder_south_y, motor_casing_top_z),
                                             P3D(motor_clip_east_x,
                                                 clip_lip_south_y, clip_top_z))
        south_lip: CornerCube = CornerCube("South Lip",
                                           P3D(holder_west_x,
                                               clip_lip_south_y, lip_south_z),
                                           P3D(motor_gearbox_west_x,
                                               holder_south_y, lip_north_z))
        south_side: CornerCube = CornerCube("South Side",
                                            P3D(motor_gearbox_west_x,
                                                holder_south_y, base_top_z),
                                            P3D(motor_gearbox_east_x + east_tab_dx,
                                                motor_gearbox_south_y, motor_casing_bottom_z))

        # The *west_side* has a more complicated geometry.  It needs a half circle to make
        # room for the motor edge.  It is formed via a polygon that is subsequently  extruded,
        # rotated, and translated.  The west side final location is in the Y/Z plane, but the
        # polygon starts on the X/Y plane.  Thus, Z coordinates are show up in the X filed
        # of *P2D*'s:
        west_side_polygon: SimplePolygon = SimplePolygon("West Side Polygon", [], lock=False)
        west_side_polygon.point_append(P2D(0.0, holder_north_y))
        west_side_polygon.point_append(P2D(base_bottom_z, holder_north_y))
        west_side_polygon.point_append(P2D(base_bottom_z, holder_south_y))
        west_side_polygon.point_append(P2D(0.0, holder_south_y))
        west_side_polygon.arc_append(center=P2D(0.0, 0.0), radius=motor_gearbox_dy/2.0,
                                     start_angle=3.0*pi/2.0, end_angle=pi/2.0,
                                     points_count=16)
        west_side_polygon.lock()

        # Now convert *west_side_polygon* into *west_side*:
        extruded_west_side: LinearExtrude = LinearExtrude("Extruded West Side",
                                                          west_side_polygon, west_tab_dx)
        rotated_west_side: Rotate3D = Rotate3D("Rotated West Side", extruded_west_side,
                                               -pi/2.0, P3D(0.0, 1.0, 0.0))
        west_side: Translate3D = Translate3D("West Side", rotated_west_side,
                                             P3D(motor_gearbox_west_x + west_tab_dx, 0.0, 0.0))
        # west_side: CornerCube = CornerCube("West Side",
        #                                    P3D(motor_gearbox_west_x,
        #                                        holder_south_y, base_bottom_z),
        #                                    P3D(motor_gearbox_west_x + west_tab_dx,
        #                                        holder_north_y, motor_casing_bottom_z))

        # Create *holder_union* to contain all of the pieces:
        holder_union: Union3D = Union3D("Romi Motor Holder Union",
                                        [base_clip, east_side,
                                         north_clip, north_latch, north_lip, north_side,
                                         south_clip, south_latch, south_lip, south_side,
                                         west_side])

        # Create final *module*, append to *scad_program* and save into
        # *romi_motor_holder* (i.e. *self*):
        module: Module3D = Module3D("Romi Motor Holder Module",
                                    [Color("Blue Color", holder_union, "RoyalBlue")])
        scad_program.append(module)
        # romi_motor_holder: RomiMotorHolder = self
        self.module: Module3D = module


# RomiWheelAssembly:
class RomiWheelAssembly:
    """Represents a Romi motor/wheel/magnet/encoder board assembly."""

    # RomiWheelAssembly.__init__():
    def __init__(self, scad_program: ScadProgram, base_dxf: BaseDXF) -> None:
        """Initialize RomiWheelAssembly and append to ScadProgram."""
        # Create *romi_motor*, *romi_magnet*, and *encoder_board*:
        romi_motor: RomiMotor = RomiMotor(scad_program, base_dxf)
        romi_magnet: RomiMagnet = RomiMagnet(scad_program, base_dxf)
        encoder_board: EncoderBoard = EncoderBoard(scad_program, base_dxf)

        # Construct *module*, append to *scad_program*, and store into *rom_wheel_assembly*
        # (i.e. *self*):
        module = Module3D("Romi Wheel Assembly Module", [
            romi_motor.module.use_module_get(),
            romi_magnet.module.use_module_get(),
            encoder_board.module.use_module_get()
        ])

        scad_program.append(module)
        # romi_wheel_assembly: RomiWheelAssembly = self
        self.module: Module3D = module
        scad_program.if3d.name_match_append("wheel_assembly", module, ["Wheel Assembly"])

        # Save some values into *romi_wheel_assembly* (i.e. *self*).
        # romi_wheel_assembly: Romi_Wheel_Assembly = self
        # self.east_encoder_board: EncoderBoard = east_encoder_board
        self.romi_magnet: RomiMagnet = romi_magnet
        self.romi_motor: RomiMotor = romi_motor
        # self.west_encoder_board: EncoderBoard = west_encoder_board


# Spacer:
class Spacer:
    """Represents a Spacer."""

    # Spacer.__init__():
    def __init__(self, scad_program: ScadProgram, name: str, height: float, screw_class: str,
                 diameter: float = 0.0, is_hex: bool = False,
                 bottom_center: P3D = P3D(0.0, 0.0, 0.0),
                 bottom_height: float = 0.0, top_height: float = 0.0, color: str = "GoldenRod",
                 bottom_washers: List[Tuple[float, float, str]] = [],
                 top_washers: List[Tuple[float, float, str]] = []) -> None:
        """Generate a spacer with optional washers.

        All length measurements are in millimeters:

        Args:
            *scad_program* (*ScadProgram*):
                The *ScadProgram* to append the spacer module to.
            *name* (*str*):
                The name of the spacer.
            *height*: (*float*):
                The spacer height.
            *screw_class*: (*str*):
                The diameter of the screw to be supported, currently
                only "M2" and "M3" are supported.
            *diameter*: (*float*):
                (Optional: Defaults to 0.0 which causes the diameter
                to be looked up from the screw class.
            *is_threaded*: (*bool*):
                (Optional: Defaults to *True*.)
                If *True*, the spacer body is threaded; otherwise the
                spacer body is unthread.
            *is_hex*: (*bool*):
                (Optional: Defaults to *False*.):
                If *True*, the *Diameter* is the width of a hex spacer;
                otherwise the spacer is round.
            *bottom_center*: (*P3D*):
                (Optional: Defualts to *P3D*(0.0, 0.0, 0.0))
            *bottom_height* (*float*):
                (Optional: Defaults to 0.0 which means no male thread.)
                The bottom male thread extrusion.
            *top_height* (*float*):
                (Optional: Defaults to 0.0 which means no male thread.)
                The top male thread extrusion.
            *color*: (*str*):
                (Optional: Defaults to "GoldenRod" (i.e. Brass.))
            *bottom_washers* (*List*[*Tuple*[*float*, *float*, *str*]]):
                The list of bottom washers to put below the spacer.
                Each washer is specifed as a Tuple of the form:
                        (height, width, "color")
                where the "height" is the washer height in millimeters,
                "width" is the width in millimeters, and the "color" is
                a valid color.  If no *color* is "", "Silver" is used
                as a default color.
            *top_washers* (*List*[*Tuple*[*float*, *float*, *str*]]):
                Same as *bottom_washers*, but puts the washers on top.

        """
        # Verify argument sanity:
        screw_classes: Tuple[str, ...] = ("M2", "M2.5", "M3")
        assert name != "", "name is empty"
        assert height > 0.0, f"height (={height}) is not positive"
        assert screw_class in screw_classes, (f"screw_class (='{screw_class})' "
                                              "is not one of {screw_classes}")
        assert diameter >= 0.0, f"diameter (={diameter}) is negative"
        assert bottom_height >= 0.0, f"bottom_height (={bottom_height}) is negative)"
        assert top_height >= 0.0, f"top_height (={top_height}) is negative)"
        washer: Tuple[float, float, str]
        for washer in bottom_washers + top_washers:
            assert len(washer) == 3, "top/bottom washer (={washer}) does not have 3 entries"

        # https://littlemachineshop.com/images/gallery/PDF/TapDrillSizes.pdf
        # Determin some standard constants based on *screw_class*:
        screw_size: float
        standard_fit: float
        if screw_class == "M2":
            screw_size = 2.00
            standard_fit = 2.20
        elif screw_class == "M2.5":
            screw_size = 2.5
            standard_fit = 2.75
        elif screw_class == "M3":
            screw_size = 3.00
            standard_fit = 3.30
        else:
            assert False, "Bad screw class"  # pragma: no cover

        # Detemine the *hex_diameter* using math (http://mathworld.wolfram.com/RegularPolygon.html):
        #     (1) r = R * cos(pi / n)
        #     (2) R = r / cos(pi / n)
        # where:
        #     R is the circle diameter,
        #     r is the center to side distance, and
        #     n is the number of sides.
        hex_diameter: float = diameter / cos(pi / float(6))

        # Compute *bottom_washers_height* and *top_washers_height*:
        bottom_washers_height: float = 0.0
        for washer in bottom_washers:
            washer_height = washer[0]
            bottom_washers_height += washer_height
        top_washers_height: float = 0.0
        for washer in top_washers:
            washer_height = washer[0]
            top_washers_height += washer_height

        # Construct *body_stack* with *bottom_cylinder*, *extruded_body*, and *top_cylinder*:
        body_stack: List[Scad3D] = []
        if True:
            # If appropriate, construct the bottom threaded portion:
            spacer_stack: List[Scad3D] = []
            if bottom_height > 0.0:
                bottom_cylinder: Scad3D = Cylinder(f"Bottom Male {name}", screw_size,
                                                   P3D(0.0, 0.0, -bottom_height),
                                                   P3D(0.0, 0.0, 0.0), 8)
                body_stack.append(bottom_cylinder)

            # Now construct the body:
            body_diameter: float = hex_diameter if is_hex else diameter
            body_sides: int = 6 if is_hex else 16
            spacer_polygon: Polygon
            body_polygon: Polygon = Polygon(f"{name} Polygon", [
                Circle(f"{name} Body Exterior", body_diameter, body_sides),
                Circle(f"{name} Body Hole", screw_size, 8)])
            extruded_body: LinearExtrude = LinearExtrude(f"Extruded {name}",
                                                         body_polygon, height)
            body_stack.append(extruded_body)

            # If appropriate, construct the top threaded portion:
            if top_height > 0.0:
                top_cylinder: Scad3D = Cylinder(f"top Male {name}", screw_size,
                                                P3D(0.0, 0.0, height),
                                                P3D(0.0, 0.0, height + top_height), 8)
                body_stack.append(top_cylinder)

        # Weld the *body_stack* into *body_union* and color it, and append to *spacer_stack*:
        body_union: Union3D = Union3D(f"{name} Body Union", body_stack)
        colored_body: Color = Color(f"Colored {name} Body", body_union, color)
        spacer_stack.append(colored_body)

        # Stack up the *bottom_washers* into *spacer_stack* using *spacer* (i.e. *self*)
        # *washer_append* method.  Keep track of the to *z* height:
        z: float = -bottom_washers_height
        spacer: Spacer = self
        washer_index: int
        for washer_index, washer in enumerate(bottom_washers):
            z = spacer.washer_append(f"{name} Bottom[{washer_index}]",
                                     washer, standard_fit, z, spacer_stack)

        # Stack up the *top_washers*:
        z = height
        for washer_index, washer in enumerate(top_washers):
            z = spacer.washer_append(f"{name} Top['{washer_index}]",
                                     washer, standard_fit, z, spacer_stack)

        # Construct the *spacer_union*, the final *module* and append to *scad_program*:
        spacer_union: Union3D = Union3D(f"{name} Union", spacer_stack)
        translated_stack: Translate3D = Translate3D(f"Translated {name}",
                                                    spacer_union, bottom_center)
        module: Module3D = Module3D(f"{name} Module", [translated_stack])
        scad_program.append(module)
        self.module: Module3D = module

    def washer_append(self, name: str, washer: Tuple[float, float, str], inside_diameter: float,
                      start_z: float, spacer_stack: List[Scad3D]) -> float:
        """Construct one washer."""
        # Unpack *washer*:
        washer_height: float
        washer_diameter: float
        washer_color: str
        washer_height, washer_diameter, washer_color = washer
        if washer_color == "":
            washer_color = "Silver"

        # Create *washer_polygon*:
        washer_polygon: Polygon = Polygon(name, [
            Circle(f"{name} Exterior", washer_diameter, 16),
            Circle(f"{name} Hole", inside_diameter, 8)])
        extruded_washer: LinearExtrude = LinearExtrude(f"Extruded {name}",
                                                       washer_polygon, washer_height)
        translated_washer: Translate3D = Translate3D(f"Translated {name}",
                                                     extruded_washer, P3D(0.0, 0.0, start_z))
        colored_washer: Color = Color(f"{washer_color} {name}", translated_washer, washer_color)
        spacer_stack.append(colored_washer)
        return start_z + washer_height


# STLink:
class STLink:
    """Represents the STLink portion of a Nucleo144."""

    def __init__(self, scad_program: ScadProgram, center: P3D,
                 master_board_polygon: Polygon, master_board_connectors: List[Scad3D]) -> None:
        """Initialize the STLink object."""
        # The mechanical drawing in the Nucleo144 User Manual does not give very much dimensional
        # information.  So much of the numbers below are measured with calipers:
        # stlink: STLink = self
        nucleo_main_bottom_y: float = 0
        cn2_pin2_y: float = nucleo_main_bottom_y + 12.70 + 114.30
        south_y: float = cn2_pin2_y - 18.22
        north_y1: float = cn2_pin2_y + 2.54 + 3.80  # .1inch spacing between pins
        north_y2: float = nucleo_main_bottom_y + 133.34
        assert abs(north_y1 - north_y2) < .001, f"north_y1(={north_y1}) ~= north_y2(={north_y2})"
        pcb_dy: float = abs(north_y2 - south_y)
        pcb_dx: float = 70.0
        pcb_bottom_z: float = 0.0
        pcb_dz: float = 1.60  # Calipers
        pcb_top_z: float = pcb_bottom_z + pcb_dz

        # Create the exterior of the  PCB
        pcb_square: Square = Square("STLink Exterior Outline ",
                                    pcb_dx, pcb_dy, center=P2D(0.0, 0.0))
        st_polygon: Polygon = Polygon("STLink PCB Polygon", [pcb_square], lock=False)

        # Create all of the connectors:
        degrees90: float = pi / 2.0
        cn2_pin1_x = -pcb_dx / 2.0 + 3.25  # Calipers
        cn2_pin1_y = pcb_dy / 2.0 - 3.80  # Assume .1in spacing  (pin 1 on top)
        cn6_pin1_x = cn2_pin1_x + 5.00  # Calipers (pretty accurate)
        cn6_pin1_y = cn2_pin1_y - 2 * 2.54  # Pin 1 on on top
        jp2_pin1_x = cn2_pin1_x  # Visual inspection
        jp2_pin1_y = cn6_pin1_y - 5 * 2.54  # Visual inspection (pin 1 on bottom)
        cn4_pin1_x = cn2_pin1_x + 15.00  # Calipers (pretty accurate)
        cn4_pin1_y = cn2_pin1_y  # Visual inspection (pin 1 on top)
        cn5_pin1_x = cn4_pin1_x  # Visual inspecition
        cn5_pin1_y = jp2_pin1_y  # Visual inspection (pin 1 on bottom)
        jp1_pin1_x = -(2.54 / 2.0)  # It is pretty clear that it is centered along Y axis
        jp1_pin1_y = cn2_pin1_y  # Visual inspection
        cn3_pin1_x = -cn2_pin1_x
        cn3_pin1_y = cn2_pin1_y  # Visual inspection (pin 1 on top)
        print(f"jp1_pin1_x={jp1_pin1_x}")

        # Create the *st_connectors*  and *mb_connectors* list:
        # CN2:
        st_cn2_connector: RectangularConnector
        st_cn2_connector = RectangularConnector(scad_program, "ST CN2 GND Connector",
                                                1, 2, 2.54, 2.54, 5.84,
                                                center=P3D(cn2_pin1_x,
                                                           cn2_pin1_y - 2.54 / 2.54,
                                                           pcb_top_z),
                                                insulation_color="Lime",
                                                pcb_polygon=st_polygon,
                                                vertical_rotate=-degrees90)  # Pin 1 on top
        mb_cn2_connector: RectangularConnector
        mb_cn2_connector = RectangularConnector(scad_program, "MB CN2 GND Connector",
                                                1, 2, 8.50, 2.54, 0.0, is_top=False,
                                                center=(center + P3D(cn2_pin1_x,
                                                                     cn2_pin1_y - 2.54 / 2.54,
                                                                     0.0)),
                                                insulation_color="Gray",
                                                pcb_polygon=master_board_polygon,
                                                vertical_rotate=-degrees90)  # Pin 1 on top
        master_board_connectors.append(mb_cn2_connector.module.use_module_get())

        # CN6:
        st_cn6_connector: RectangularConnector
        st_cn6_connector = RectangularConnector(scad_program, "CN6 SWD Connector",
                                                1, 6, 2.54, 2.54, 5.84,
                                                center=P3D(cn6_pin1_x,
                                                           cn6_pin1_y - (5 * 2.54) / 2.0,
                                                           pcb_top_z),
                                                insulation_color="Lime",
                                                pcb_polygon=st_polygon,
                                                vertical_rotate=-degrees90)  # Pin 1 on top
        mb_cn6_connector: RectangularConnector
        mb_cn6_connector = RectangularConnector(scad_program, "MB CN6 SWD Connector",
                                                1, 6, 8.50, 2.54, 0.0, is_top=False,
                                                center=(center + P3D(cn6_pin1_x,
                                                                     cn6_pin1_y - (5 * 2.54) / 2.0,
                                                                     0.0)),
                                                insulation_color="Gray",
                                                pcb_polygon=master_board_polygon,
                                                vertical_rotate=-degrees90)  # Pin 1 on top
        master_board_connectors.append(mb_cn6_connector.module.use_module_get())

        # JP2:
        st_jp2_connector: RectangularConnector
        st_jp2_connector = RectangularConnector(scad_program, "JP2 Connector",
                                                1, 2, 2.54, 2.54, 5.84,
                                                center=P3D(jp2_pin1_x,
                                                           jp2_pin1_y + 2.54 / 2.0, pcb_top_z),
                                                insulation_color="Lime",
                                                pcb_polygon=st_polygon,
                                                vertical_rotate=degrees90)  # Pin 1 on bottom
        mb_jp2_connector: RectangularConnector
        mb_jp2_connector = RectangularConnector(scad_program, "MB JP2 Connector",
                                                1, 2, 8.50, 2.54, 5.84, is_top=False,
                                                center=(center + P3D(jp2_pin1_x,
                                                                     jp2_pin1_y + 2.54 / 2.0,
                                                                     0)),
                                                insulation_color="Gray",
                                                pcb_polygon=master_board_polygon,
                                                vertical_rotate=degrees90)  # Pin 1 on bottom
        master_board_connectors.append(mb_jp2_connector.module.use_module_get())

        # CN4:
        st_cn4_connector: RectangularConnector
        st_cn4_connector = RectangularConnector(scad_program, "CN4 Nucleo STLink Connector",
                                                1, 4, 2.54, 2.54, 0.0,
                                                center=P3D(cn4_pin1_x,
                                                           cn4_pin1_y - (3 * 2.54) / 2.0,
                                                           pcb_top_z),
                                                insulation_color="Lime",
                                                pcb_polygon=st_polygon,
                                                vertical_rotate=-degrees90)  # Pin 1 on top
        mb_cn4_connector: RectangularConnector
        mb_cn4_connector = RectangularConnector(scad_program, "MB CN4 Nucleo STLink Connector",
                                                1, 4, 8.50, 2.54, 5.84, is_top=False,
                                                center=(center + P3D(cn4_pin1_x,
                                                                     cn4_pin1_y - (3 * 2.54) / 2.0,
                                                                     0.0)),
                                                insulation_color="Gray",
                                                pcb_polygon=master_board_polygon,
                                                vertical_rotate=-degrees90)  # Pin 1 on top
        master_board_connectors.append(mb_cn4_connector.module.use_module_get())

        # CN5:
        st_cn5_connector: RectangularConnector
        st_cn5_connector = RectangularConnector(scad_program, "CN5 TX RX Connector",
                                                1, 2, 2.54, 2.54, 5.84,
                                                center=P3D(cn5_pin1_x,
                                                           cn5_pin1_y + 2.54 / 2.0, pcb_top_z),
                                                insulation_color="Lime",
                                                pcb_polygon=st_polygon,
                                                vertical_rotate=-degrees90)  # Pin 1 on top
        mb_cn5_connector: RectangularConnector
        mb_cn5_connector = RectangularConnector(scad_program, "MB CN5 TX RX Connector",
                                                1, 2, 8.50, 2.54, 0, is_top=False,
                                                center=(center + P3D(cn5_pin1_x,
                                                                     cn5_pin1_y + 2.54 / 2.0,
                                                                     0.0)),
                                                insulation_color="Gray",
                                                pcb_polygon=master_board_polygon,
                                                vertical_rotate=-degrees90)  # Pin 1 on top
        master_board_connectors.append(mb_cn5_connector.module.use_module_get())

        # JP1:
        st_jp1_connector: RectangularConnector
        st_jp1_connector = RectangularConnector(scad_program, "JP1 Connector",
                                                1, 2, 2.54, 2.54, 5.84,
                                                center=P3D(jp1_pin1_x + 2.54 / 2.0,
                                                           jp1_pin1_y, pcb_top_z),
                                                insulation_color="Lime",
                                                pcb_polygon=st_polygon)  # Pin1 to left
        mb_jp1_connector: RectangularConnector
        mb_jp1_connector = RectangularConnector(scad_program, "MB JP1 Connector",
                                                1, 2, 8.50, 2.54, 0.0, is_top=False,
                                                center=(center + P3D(jp1_pin1_x + 2.54 / 2.0,
                                                                     jp1_pin1_y,
                                                                     0.0)),
                                                insulation_color="Gray",
                                                pcb_polygon=master_board_polygon)  # Pin1 to left
        master_board_connectors.append(mb_jp1_connector.module.use_module_get())

        # CN3:
        st_cn3_connector: RectangularConnector
        st_cn3_connector = RectangularConnector(scad_program, "CN3 GND Connector",
                                                1, 2, 2.54, 2.54, 5.84,
                                                center=P3D(cn3_pin1_x,
                                                           cn3_pin1_y - 2.54 / 2.54, pcb_top_z),
                                                insulation_color="Lime",
                                                pcb_polygon=st_polygon,
                                                vertical_rotate=-degrees90)  # Pin 1 on top
        mb_cn3_connector: RectangularConnector
        mb_cn3_connector = RectangularConnector(scad_program, "MB CN3 GND Connector",
                                                1, 2, 8.50, 2.54, 0.0, is_top=False,
                                                center=(center + P3D(cn3_pin1_x,
                                                                     cn3_pin1_y - 2.54 / 2.54,
                                                                     0.0)),
                                                insulation_color="Gray",
                                                pcb_polygon=master_board_polygon,
                                                vertical_rotate=-degrees90)  # Pin 1 on top
        master_board_connectors.append(mb_cn3_connector.module.use_module_get())

        st_connectors: List[Scad3D] = [
            st_cn2_connector.module.use_module_get(),
            st_cn6_connector.module.use_module_get(),
            st_jp2_connector.module.use_module_get(),
            st_cn4_connector.module.use_module_get(),
            st_cn5_connector.module.use_module_get(),
            st_jp1_connector.module.use_module_get(),
            st_cn3_connector.module.use_module_get(),
        ]

        # Lockup *st_polygon* make it viewable in OpenScad:
        st_polygon.lock()
        stlink_polygon_module: Module2D = Module2D("STLink PCB Module", [st_polygon])
        scad_program.if2d.name_match_append("stlink_pcb", stlink_polygon_module, ["STLInk PCB"])
        scad_program.append(stlink_polygon_module)

        extruded_pcb: LinearExtrude = LinearExtrude("Extruded STLink PCB", pcb_square, pcb_dz)
        colored_pcb: Color = Color("While STLink PCB", extruded_pcb, "White")
        rotated_pcb: Scad3D = Rotate3D("Roated STLink", colored_pcb, pi, P3D(1.0, 0.0, 0.0))
        module: Module3D = Module3D("STLink", [rotated_pcb] + st_connectors)
        scad_program.append(module)
        scad_program.if3d.name_match_append("stlink", module, ["STLink"])
        self.module: Module3D = module


# SRF02:
class SRF02:
    """Represents an Devantech SRF02 sonar module."""

    # HCSR04.__init__():
    def __init__(self, scad_program: ScadProgram) -> None:  # pragma: no cover
        """Initialize anHCSR04."""
        # Create the PCB:
        pcb_dy: float = 24.0
        pcb_dx: float = 22.0
        pcb_dz: float = 1.20
        pcb_square: Square = Square("SRF02 PCB", pcb_dx, pcb_dy, center=P2D(0.0, pcb_dy / 2.0))
        extruded_pcb: LinearExtrude = LinearExtrude("Extruded SRF02 PCB", pcb_square, pcb_dz)
        colored_pcb: Color = Color("Colored HRCSR04", extruded_pcb, "Green")

        # Create the sonar transducer:
        transducer_diameter: float = 16.00
        transducer_dz: float = 12.00  # Pure guess, not in spec. sheet
        transducer_start_z: float = pcb_dz
        transducer_end_z: float = transducer_start_z + transducer_dz
        transducer_start: P3D = P3D(0.0, pcb_dy/2, transducer_start_z)
        transducer_end: P3D = P3D(0.0, pcb_dy/2, transducer_end_z)
        transducer: Cylinder = Cylinder("Left Sonar Transducer",
                                        transducer_diameter, transducer_start, transducer_end, 16)
        colored_transducer: Color = Color("Colored Sonar Transducer", transducer, "Gray")

        srf02_union: Union3D = Union3D("SRF02 Union", [
            colored_pcb,
            colored_transducer
        ])

        # Now tilt the sonar up followed by rotating it to vertical:
        x_axis: P3D = P3D(1.0, 0.0, 0.0)
        degrees90: float = pi / 2.0
        vertical_srf02: Rotate3D = Rotate3D("Veritcal SRF02", srf02_union, degrees90, x_axis)
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        east_facing_srf02: Rotate3D = Rotate3D("East Facing SRF02",
                                               vertical_srf02, degrees90, z_axis)

        # Create *module*, append to *scad_program*, and save into *srf02* (i.e. *self*):
        module: Module3D = Module3D("SRF02 Sonar Module", [east_facing_srf02])
        scad_program.append(module)
        scad_program.if3d.name_match_append("sonar", module, ["SRF02 Sonar"])
        # srf02: SRF02 = self
        self.module: Module3D = module


def main() -> int:  # pragma: no cover
    """Generate the openscand file."""
    print("hr2_models.main() called")
    # scad_file: IO[Any]
    # with open("romi_base_dxf.scad", "w") as scad_file:
    #     romi_base_polygon.scad_file_write(scad_file)

    # romi_base_extruded_polygon: LinearExtrude = LinearExtrude("Romi Base Extrude",
    #                                                           romi_base_polygon, 9.6)
    # union: Union = Union("Base Union", [romi_base_extruded_polygon])
    # with open("romi_base.scad", "w") as scad_file:
    #     union.scad_file_write(scad_file)

    # Create the top level *scad_program* program that we will stuff everything into:
    scad_program: ScadProgram = ScadProgram("Scad models")

    # Define the default value for the *name* OpenSCAD variable.  The *name* variable
    # can be set from the command line as shown below:
    #
    #     openscad -D 'name="VALID_NAME"' hr2_models.scad
    #
    # The list of *VALID_NAME*'s can be found near the bottom of `README.md`.
    scad_program.append(Variable2D("Name", "name", '"hr2_arm_assembly"'))

    # romi_base: RomiBase = RomiBase(scad_program, base_dxf)
    # romi_base.holes_slots_rectangles_write()
    # romi_base_keys: List[Tuple[Any, ...]] = romi_base.keys_get()

    # Now create *other_pi* and append it as well:
    # Distance between Pi board and Master board (bottom surface to bottom surface):
    # print(f"pi_offset={pi_offset}")
    #  other_pi: OtherPi = OtherPi(scad_program)

    # Create *raspi3b*:
    # raspi3b: RaspberryPi3 = RaspberryPi3(scad_program)

    # romi_expansion_plate: RomiExpansionPlate = RomiExpansionPlate(scad_program)
    # romi_expansion_plate = romi_expansion_plate

    # master_board: MasterBoard = MasterBoard(scad_program, base_dxf,
    #                                         master_board_z, pi_offset, romi_base_keys)

    # Now create *hr2*:
    # hr2: HR2 = HR2(scad_program, romi_base, east_romi_wheel_assembly, west_romi_wheel_assembly,
    #                master_board, other_pi, pi_offset)
    # hr2 = hr2

    # Update the `README.md` file:
    read_me_text: str = ""
    read_me_file: IO[Any]
    with open("README.md") as read_me_file:
        read_me_text = read_me_file.read()
    hr2_robot: HR2Robot = HR2Robot(scad_program)
    hr2_robot = hr2_robot

    scad_comment_lines: List[str]
    updated_read_me_text: str
    updated_read_me_text, scad_comment_lines = scad_program.read_me_update(read_me_text)
    if read_me_text != updated_read_me_text:
        with open("README.md", "w") as read_me_file:
            read_me_file.write(updated_read_me_text)
    # print(f"scad_comment_lines={scad_comment_lines}")

    # Generate `hr2_models.scad`:
    scad_lines: List[str] = []
    scad_program.scad_lines_append(scad_lines, "")
    # Funky slice syntax for inserting a list in the middle of another list:
    scad_lines[2:2] = scad_comment_lines
    scad_lines.append("")
    scad_text: str = '\n'.join(scad_lines)
    scad_file: IO[Any]
    with open("hr2_models.scad", "w") as scad_file:
        scad_file.write(scad_text)

    return 0


if __name__ == "__main__":    # pragma: no cover
    main()
