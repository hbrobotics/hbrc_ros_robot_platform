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
"""Code that generates an OpenSCAD model for HR2 (HBRC ROS Robot)."""

# Random useful URL:
# https://mechanicalc.com/reference/fastener-size-tables

from scad_models.scad import (
    Circle, Color, CornerCube, Cube, Cylinder, If2D, Difference2D, LinearExtrude,
    Module2D, Module3D, P2D, P3D, Polygon, Rotate3D, Scad2D, Scad3D, SimplePolygon, ScadProgram,
    Square, Translate3D, UseModule3D, Union3D)
import os
import time
from pathlib import Path
from typing import Any, Dict, IO, List, Optional, Set, Tuple
from math import asin, atan2, cos, degrees, nan, pi, radians, sin, sqrt

# Can not do this at the top level since it forms a circular import dependency loop:
# from scad_models.kicad import Footprint, KicadPCB


# Connectors:
class Connectors:
    """Represents a bunch of connectors."""

    # Connectors.__init__():
    def __init__(self, scad_program: ScadProgram) -> None:
        """Generate a bunch of common connectors."""
        # print("=>Connectors.__init__()")

        # Some constants:
        pins_dx_dy: float = 2.56  # = .1in
        pcb_pin_height: float = 2.79
        # female_insulation_height: float = 8.85
        female_insulation_height: float = 6.00
        male_insulation_height: float = pins_dx_dy

        f1x4lp: F1x4LP = F1x4LP(scad_program)

        # Common 1x2 connectors:
        m1x2: RectangularConnector = RectangularConnector(
            "M1x2", scad_program,
            1, 2, pins_dx_dy, pcb_pin_height,
            insulation_color="Maroon", male_pin_height=4.04,
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)
        f1x2: RectangularConnector = RectangularConnector(
            "F1x2", scad_program, 1, 2, female_insulation_height, pcb_pin_height,
            insulation_color="Fuchsia",
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)
        f1x2ra: RectangularConnector = RectangularConnector(
            "F1x2RA", scad_program, 1, 2, female_insulation_height, pcb_pin_height,
            insulation_color="Fuchsia",
            right_angle_length=4.00,  # <=calipers / schematic=>6.00 + 0.127
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)

        # Common 1x3 connectors:
        m1x3: RectangularConnector = RectangularConnector(
            "M1x3", scad_program, 1, 3, pins_dx_dy, pcb_pin_height,
            insulation_color="Maroon", male_pin_height=4.04,
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)
        # Digikey: 2057-PH1RB-03-UA-ND (Adam Tech):
        m1x3ra: RectangularConnector = RectangularConnector(
            "M1x3RA", scad_program, 1, 3, pins_dx_dy, pcb_pin_height,
            insulation_color="Cyan", male_pin_height=6.00,
            right_angle_length=(3.05 + 0.127),
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)
        f1x3: RectangularConnector = RectangularConnector(
            "F1x3", scad_program, 1, 3, female_insulation_height, pcb_pin_height,
            insulation_color="Fuchsia",
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)
        f1x3ra: RectangularConnector = RectangularConnector(
            "F1x3RA", scad_program, 1, 3, female_insulation_height, pcb_pin_height,
            insulation_color="Fuchsia",
            right_angle_length=4.00,  # <=calipers / schematic=>6.00 + 0.127
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)

        # Common 1x4 connectors:
        m1x4: RectangularConnector = RectangularConnector(
            "M1x4", scad_program, 1, 4, pins_dx_dy, pcb_pin_height,
            insulation_color="Maroon", male_pin_height=4.04,
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)
        # HCSR04 Right angle:
        m1x4ra: RectangularConnector = RectangularConnector(
            "M1x4RA", scad_program, 1, 4, pins_dx_dy, pcb_pin_height,
            insulation_color="Cyan", male_pin_height=6.00,
            right_angle_length=3.00 + 1.5 * 0.254,  # trial and error until it looks right.
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)
        f1x4: RectangularConnector = RectangularConnector(
            "F1x4", scad_program, 1, 4, female_insulation_height, pcb_pin_height,
            insulation_color="Fuchsia",
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)
        # High Insulation:
        f1x4h: RectangularConnector = RectangularConnector(
            "F1x4H", scad_program, 1, 4, female_insulation_height + 2.0,
            pcb_pin_height, insulation_color="Fuchsia",
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)
        f1x4ra: RectangularConnector = RectangularConnector(
            "F1x4RA", scad_program, 1, 4, female_insulation_height, pcb_pin_height,
            insulation_color="Fuchsia",
            right_angle_length=4.00,  # <=calipers / schematic=>6.00 + 0.127
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)

        # Common 1x5 connectors:
        m1x5: RectangularConnector = RectangularConnector(
            "M1x5", scad_program, 1, 5, pins_dx_dy, pcb_pin_height,
            insulation_color="Maroon", male_pin_height=4.04,
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)
        f1x5: RectangularConnector = RectangularConnector(
            "F1x5", scad_program, 1, 5, female_insulation_height, pcb_pin_height,
            insulation_color="Fuchsia",
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)

        # Common 1x6 connectors:
        m1x6: RectangularConnector = RectangularConnector(
            "M1x6", scad_program, 1, 6, pins_dx_dy, pcb_pin_height,
            insulation_color="Maroon", male_pin_height=4.04,
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)
        f1x6: RectangularConnector = RectangularConnector(
            "F1x6", scad_program, 1, 6, female_insulation_height, pcb_pin_height,
            insulation_color="Fuchsia",
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)

        # Common 1x8 connectors:
        m1x8: RectangularConnector = RectangularConnector(
            "M1x8", scad_program, 1, 8, pins_dx_dy, pcb_pin_height,
            insulation_color="Maroon", male_pin_height=4.04,
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)
        f1x8: RectangularConnector = RectangularConnector(
            "F1x8", scad_program, 1, 8, female_insulation_height, pcb_pin_height,
            insulation_color="Fuchsia",
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)

        # Common 2x2 male jumper:
        m2x2: RectangularConnector = RectangularConnector(
            "M2x2", scad_program, 2, 2, pins_dx_dy, pcb_pin_height,
            insulation_color="Fuchsia", male_pin_height=4.04,
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)

        # Standard 2x20 RaspberryPi connectors
        m2x20: RectangularConnector = RectangularConnector(
            "M2x20", scad_program, 2, 20, pins_dx_dy, male_insulation_height,
            insulation_color="Maroon", male_pin_height=4.04,
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)
        f2x20: RectangularConnector = RectangularConnector(
            "F2x20", scad_program, 2, 20, female_insulation_height, pcb_pin_height,
            insulation_color="Fuchsia", cut_out=True,
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)

        # Mating connector for Nucleo144 CN11 and CN12 connectors:
        # Nucleo144 Morpho CN11 and CN12 connectors:
        m2x35_long: RectangularConnector = RectangularConnector(
            "M2x35Long", scad_program, 2, 35, pins_dx_dy, pcb_pin_height,
            insulation_color="Maroon", male_pin_height=8.08,
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)
        f2x35: RectangularConnector = RectangularConnector(
            "F2x35", scad_program, 2, 35, female_insulation_height, pcb_pin_height,
            insulation_color="Maroon", cut_out=True,
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)

        # Nucleo144 CN7 Zio Connector:
        f2x10_long: RectangularConnector = RectangularConnector(
            "F2x10_Long", scad_program, 2, 10,
            female_insulation_height, female_insulation_height,
            insulation_color="SlateBlue", cut_out=True,
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)

        # Nucleo144 CN8 Zio Connector:
        f2x8_long: RectangularConnector = RectangularConnector(
            "F2x8_Long", scad_program, 2, 8,
            female_insulation_height, female_insulation_height,
            insulation_color="SlateBlue", cut_out=True,
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)

        # Nucleo144 CN9 Zio Connector:
        f2x15_long: RectangularConnector = RectangularConnector(
            "F2x15_Long", scad_program, 2, 15,
            female_insulation_height, female_insulation_height,
            insulation_color="SlateBlue", cut_out=True,
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)

        # Nucleo144 CN10 Zio Connector:
        f2x17_long:  RectangularConnector = RectangularConnector(
            "F2x17_Long", scad_program, 2, 17,
            female_insulation_height, female_insulation_height,
            insulation_color="SlateBlue", cut_out=True,
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)

        # Load up *connectors* (i.e. *self*)
        # connectors: Connectors = self
        self.m1x2: RectangularConnector = m1x2
        self.f1x2: RectangularConnector = f1x2
        self.f1x2ra: RectangularConnector = f1x2ra
        self.m1x3: RectangularConnector = m1x3
        self.m1x3ra: RectangularConnector = m1x3ra
        self.f1x3: RectangularConnector = f1x3
        self.f1x3ra: RectangularConnector = f1x3ra
        self.m1x4: RectangularConnector = m1x4
        self.f1x4lp: F1x4LP = f1x4lp
        self.f1x4ra: RectangularConnector = f1x4ra
        self.m1x4ra: RectangularConnector = m1x4ra
        self.f1x4: RectangularConnector = f1x4
        self.f1x4h: RectangularConnector = f1x4h
        self.m1x5: RectangularConnector = m1x5
        self.f1x5: RectangularConnector = f1x5
        self.m1x6: RectangularConnector = m1x6
        self.f1x6: RectangularConnector = f1x6
        self.m1x8: RectangularConnector = m1x8
        self.f1x8: RectangularConnector = f1x8
        self.m2x2: RectangularConnector = m2x2
        self.m2x20: RectangularConnector = m2x20
        self.f2x20: RectangularConnector = f2x20
        self.m2x35_long: RectangularConnector = m2x35_long
        self.f2x35: RectangularConnector = f2x35
        self.f2x10_long: RectangularConnector = f2x10_long
        self.f2x8_long: RectangularConnector = f2x8_long
        self.f2x15_long: RectangularConnector = f2x15_long
        self.f2x17_long:  RectangularConnector = f2x17_long

        # print("<=Connectors.__init__()")


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
    The *__init__* methods for these two sub-classes compute the correct
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


# Pad:
class Pad:
    """Represents a through hole PCB pad."""

    # Pad.__init__():
    def __init__(self, name: str, pad_dx: float, pad_dy: float, drill_diameter: float,
                 pad_center: "P2D", pad_rotate: float = 0.0, tracing: str = "") -> None:
        """Create a PAD."""
        # Perform any requested *tracing*:
        if tracing:
            print(f"{tracing}=>Pad.__init__('{name}', pdx:{pad_dx:.2f} pad_dy:{pad_dy:.2f} "
                  f"center:{pad_center} dd:{drill_diameter:.2f} pr:{degrees(pad_rotate)})deg")

        # Verify argument types:
        assert name != ""
        assert drill_diameter >= 0.0
        assert pad_dx >= 0.0
        assert pad_dy >= 0.0

        # Load up *pad* (i.e. *self*):
        # pad: Pad = self
        self.drill_diameter: float = drill_diameter
        self.name: str = name
        self.pad_center: P2D = pad_center
        self.pad_dx: float = pad_dx
        self.pad_dy: float = pad_dy
        self.pad_rotate: float = pad_rotate
        self.trace: str = tracing  # Temporary

        # Wrap-up any requested *tracing*:
        if tracing:
            print(f"{tracing}<=Pad.__init__('{name}', pdx:{pad_dx:.2f} pad_dy:{pad_dy:.2f} "
                  f"center:{pad_center} dd:{drill_diameter:.2f} pr:{degrees(pad_rotate)})deg")

    # Pad.__str__():
    def __str__(self) -> str:
        """Return a string representation of a Pad."""
        # Unpack some values from *pad* (i.e. *self*):
        pad: Pad = self
        drill_diameter: float = pad.drill_diameter
        name: str = pad.name
        pad_center: P2D = pad.pad_center
        pad_dx: float = pad.pad_dx
        pad_dy: float = pad.pad_dy
        pad_rotate: float = pad.pad_rotate

        # Create and return the string:
        text: str = (f"Pad('{name}', dd:{drill_diameter:.2f} c:{pad_center}, "
                     f"dx:{pad_dx:.2f} dy:{pad_dy:.2f} rotate:{degrees(pad_rotate):.2f})")
        return text

    # Pad.copy():
    def copy(self) -> "Pad":
        """Return a copy of a Pad."""
        # Unpack some values from *pad* (i.e. *self*):
        pad: Pad = self
        drill_diameter: float = pad.drill_diameter
        name: str = pad.name
        pad_center: P2D = pad.pad_center
        pad_dx: float = pad.pad_dx
        pad_dy: float = pad.pad_dy
        pad_rotate: float = pad.pad_rotate

        # Create *copied_pad* and return it:
        copied_pad: Pad = Pad(name, pad_dx, pad_dy, drill_diameter, pad_center, pad_rotate)
        return copied_pad

    # Pad.hole_append():
    def hole_append(self, polygon: Polygon, tracing: str = "") -> None:
        """Append the Pad drill holes to a Polygon."""
        # Unpack some values from *pad* (i.e. *self*):
        pad: Pad = self
        drill_diameter: float = pad.drill_diameter
        name: str = pad.name
        pad_center: P2D = pad.pad_center
        pad_dx: float = pad.pad_dx
        pad_dy: float = pad.pad_dy
        pad_rotate: float = pad.pad_rotate

        # Perform any requested *tracing*:
        tracing = tracing if tracing else pad.trace
        if tracing:
            print(f"{tracing}=>Pad.hole_append('{name}', *)")
            print(f"{tracing}name='{name}' drill_diameter={drill_diameter:.2f} "
                  f"pad_dx:{pad_dx:.2f} pad_dy:{pad_dy:.2f}")

        # Use a *Circle* for a round pad and a rounded *Square* for a "oval" pad:
        pad_dx_dy_minimum: float = min(pad_dx, pad_dy)
        small_distance: float = .0000001
        if pad_dx_dy_minimum < small_distance:
            # Round Mechanical hole only:
            polygon.append(Circle(f"{name} Hole", drill_diameter, 16, pad_center))
            if tracing:
                print(f"{tracing}Mechinical hole")
        else:
            # There is a pad as well.  The pad can be either circular or slotted.
            if abs(pad_dx - pad_dy) < small_distance:
                # The pad is circular, use a *Circle* for a round hole:
                polygon.append(Circle(f"{name} Hole", drill_diameter, 16, pad_center))
                if tracing:
                    print(f"{tracing}circular hole: {drill_diameter:.2f}")
            else:
                # Use a rounded *Sqaure* to generate a slot hole:
                pad_extra: float = pad_dx_dy_minimum - drill_diameter
                assert pad_extra > small_distance, "Really narrow slot pad"
                if pad_extra > small_distance:
                    corner_radius: float = (pad_dx_dy_minimum - pad_extra) / 2.0
                    polygon.append(Square(f"{name} Slot",
                                          pad_dx - pad_extra, pad_dy - pad_extra,
                                          pad_center, pad_rotate, corner_radius, 5))
                    if tracing:
                        print(f"{tracing}Oval hole: ...")
                else:
                    assert False, "Unhandled hole condition in Pad.hole_append()."

        # Wrap up any requested *tracing*:
        if tracing:
            print(f"{tracing}<=Pad.hole_append('{name}', *)")

    # Pad.rebase():
    def rebase(self, delta: int) -> "Pad":
        """Return a Pad with a rebased name."""
        # Unpack some values from *Pad* (i.e. *self*):
        pad: Pad = self
        name: str = pad.name

        # Compute *rebased_name* by adding *delta* to it:
        rebased_name: str = ""
        try:
            rebased_name = str(int(name) + delta)
        except ValueError:
            raise ValueError(f"Pad '{name}' is not a number and can not be rebased by {delta}.")

        # Create *rebased_pad*, update the name, and return it:
        rebased_pad: Pad = pad.copy()
        rebased_pad.name = rebased_name
        return rebased_pad

    # Pad.reposition():
    def reposition(self, flags: str, center: P2D, rotate: float,
                   translate: P2D, pads_base: int = 0) -> "Pad":
        """Return a repositioned PAD.

        Args:
            * *flags* (*str*):
               A list of one or more character flags that specify any
               mirroring:
               * 'x': Mirror along the X axis.
               * 'y': Mirror along the Y axis.
            * *center* (*P2D*):
               * The virtual center of the pads group.
            * *z_rotate* (*float*):
               * The amount to rotate around the *center* in radians.
            * *translate* (*P2D*):
               * The location to translate *center* to.
            * *pads_base* (*int*) (Optional: Defaults to 0):
               * A base number to add to each pin number.

        """
        # Unpack some values from *pad* (i.e. *self*):
        pad: Pad = self
        drill_diameter: float = pad.drill_diameter
        name: str = pad.name
        pad_center: P2D = pad.pad_center
        pad_dx: float = pad.pad_dx
        pad_dy: float = pad.pad_dy
        pad_rotate: float = pad.pad_rotate

        # Verify that the *flags* are OK:
        flag: str
        for flag in flags:
            if flag not in "xy":
                raise ValueError(f"Flag '{flag} in '{flags}' is not allowed.  Must be 'x' or 'y'.")

        # Return the *repositioned_pad*:
        reposition_pad_center: P2D = pad_center.reposition(flags, center, rotate, translate)

        new_pad_name: str = name if pads_base == 0 else str(int(name) + pads_base)
        repositioned_pad: Pad = Pad(new_pad_name, pad_dx, pad_dy, drill_diameter,
                                    reposition_pad_center, pad_rotate + rotate)
        return repositioned_pad

    # Pad.kicad_line():
    def kicad_line(self, prefix: str, pcb_origin: P2D, tracing: str = "") -> str:
        """Return a kicad pad S-experession as a line of text."""
        # Unpack some values from *pad* (i.e. *self*):
        pad: Pad = self
        name: str = pad.name
        pad_center: P2D = pad.pad_center
        drill_diameter: float = pad.drill_diameter
        pad_dx: float = pad.pad_dx
        pad_dy: float = pad.pad_dy
        pad_rotate: float = pad.pad_rotate

        # Perform any requested *tracing*:
        if tracing:
            print(f"{tracing}=>Pad.kicad_line(*, '{pad.name}', '{prefix}', {pcb_origin})")
            print(f"{tracing}pad={pad}")

        origin_x: float = pcb_origin.x
        origin_y: float = pcb_origin.y

        # Using *pad_dx*, *pad_dy* and *drill_diameter*, figure out the
        # *shape_kind* (i.e. one of "oval", "circle", or "rect") and the
        # hole kind (i.e. one of "np_thru_hole" or "thru_hole":
        is_oval: bool = abs(pad_dx - pad_dy) > .0000001
        drill_text: str = ""
        shape_kind: str
        hole_kind: str
        if drill_diameter > 0.0:
            drill_dx: float = drill_diameter
            drill_dy: float = drill_diameter
            if is_oval:
                # Compute *drill_dx* and *drill_dy*:
                min_pad_dx_dy: float = min(pad_dx, pad_dy)
                copper_minimum: float = min_pad_dx_dy - drill_diameter
                drill_dx = pad_dx - copper_minimum
                drill_dy = pad_dy - copper_minimum
                drill_text = (f"(drill oval {KicadPCB.number(drill_dx, 2)} "
                              f"{KicadPCB.number(drill_dy, 2)}) ")
                shape_kind = "oval"
            else:  # Circular
                shape_kind = "circle"
                drill_text = f"(drill {KicadPCB.number(drill_diameter, 2)}) "
            if pad_dx < drill_diameter or pad_dy < drill_diameter:
                # Mechancal non-plated through hole:
                hole_kind = "np_thru_hole"
                # The pad size in the `.kicad_mod` module matches the drill size,
                # even though there is no pad.  Go figure.
                pad_dx = drill_dx
                pad_dy = drill_dy
            else:
                # Plated through hole:a
                hole_kind = "thru_hole"
        else:
            # No drill => round pad or rectangular pad:
            shape_kind = "rect" if is_oval else "circle"
            assert False, "Surface pad hole_kind not defined yet"
            hole_kind = "???"  # FIXME: Surface Pad.

        # Circle pads do not need to be rotated:
        rotate_text: str = (f" {KicadPCB.number(degrees(pad_rotate), 1)}"
                            if pad_rotate != 0.0 and shape_kind != "circle" else "")
        line: str = (
            f"{prefix}  (pad {KicadPCB.string(name)} {hole_kind} {shape_kind} "
            f"(at {KicadPCB.number(origin_x + pad_center.x, 4)} "
            f"{KicadPCB.number(origin_y - pad_center.y, 4)}{rotate_text}) "
            f"(size {KicadPCB.number(pad_dx, 2)} {KicadPCB.number(pad_dy, 2)}) "
            f"{drill_text}(layers *.Cu *.Mask))")

        # Wrap up any requested *tracing* and return *line*:
        if tracing:
            print(f"{tracing}<=Pad.kicad_line(*, '{pad.name}', '{prefix}', {pcb_origin})=>'{line}'")
        return line

    # Pad.x_mirror():
    def x_mirror(self) -> "Pad":
        """Return a pad mirrored around X axis."""
        # Unpack some values from *pad* (i.e. *self*):
        pad: Pad = self
        name: str = pad.name
        pad_center: P2D = pad.pad_center
        drill_diameter: float = pad.drill_diameter
        pad_dx: float = pad.pad_dx
        pad_dy: float = pad.pad_dy
        pad_rotate: float = pad.pad_rotate

        # Normalize *pad_rotate* to be between 180 and -180 degerees:
        degrees180: float = pi
        degrees360: float = 2 * degrees180
        while pad_rotate > degrees360:
            pad_rotate -= degrees360
        while pad_rotate <= -degrees180:
            pad_rotate += degrees360

        # Compute the *x_mirrored_pad* and return it.
        x_mirrored_pad_rotate: float = -pad_rotate
        x_mirrored_pad_center: P2D = P2D(pad_center.x, -pad_center.y)
        x_mirrored_pad: Pad = Pad(name, pad_dx, pad_dy, drill_diameter,
                                  x_mirrored_pad_center, x_mirrored_pad_rotate)
        return x_mirrored_pad

    # Pad.y_mirror():
    def y_mirror(self) -> "Pad":
        """Return a pad mirrored around Y axis."""
        # Unpack some values from *pad* (i.e. *self*):
        pad: Pad = self
        name: str = pad.name
        pad_center: P2D = pad.pad_center
        drill_diameter: float = pad.drill_diameter
        pad_dx: float = pad.pad_dx
        pad_dy: float = pad.pad_dy
        pad_rotate: float = pad.pad_rotate

        # The math rotating about the Y axis is angle' = -(angle - d90) + d90 = d180 - angle:
        degrees180: float = pi
        degrees360: float = 2 * degrees180
        y_mirrored_pad_rotate: float = degrees360 - pad_rotate

        # Normalize *y_mirrored_pad_rotate* to be between 180 and -180 degerees:
        while y_mirrored_pad_rotate > degrees360:
            y_mirrored_pad_rotate -= degrees360
        while y_mirrored_pad_rotate <= -degrees180:
            y_mirrored_pad_rotate += degrees360

        # Compute the *y_mirrored_pad* and return it.
        y_mirrored_pad_center: P2D = P2D(-pad_center.x, pad_center.y)
        y_mirrored_pad: Pad = Pad(name, pad_dx, pad_dy, drill_diameter,
                                  y_mirrored_pad_center, y_mirrored_pad_rotate)
        return y_mirrored_pad


# PadsGroup:
class PadsGroup:
    """Represents a list of Pads."""

    # PadsGroup.__init__():
    def __init__(self) -> None:
        """Return an empty list of Pads."""
        # Load up *pads_group* (i.e. *self*):
        # pads_group: PadsGroup = self
        self.pads: List[Pad] = []

    # PadsGroup.__len__():
    def __len__(self) -> int:
        """Return the number of pads currently in a PadsGroup."""
        pads_group: PadsGroup = self
        return len(pads_group.pads)

    # PadsGroup.__str__():
    def __str__(self) -> str:
        """Return short string about PadsGroup."""
        return f"PadsGroup(size={len(self.pads)})"

    # PadsGroup.holes_append():
    def holes_append(self, polygon: Polygon, tracing: str = "") -> None:
        """Append all of the drill holes to a Polygon."""
        # Use *pads_group* instead of *self*:
        if tracing:
            print(f"{tracing}=>PadsGroup.holes_append(*, polygon:{len(polygon)})")
        pads_group: PadsGroup = self
        pads: List[Pad] = pads_group.pads
        pad: Pad
        for pad in pads:
            pad.hole_append(polygon)
        if tracing:
            print(f"{tracing}<=PadsGroup.holes_append(*, polygon:{len(polygon)})")

    # PadsGroup.insert():
    def insert(self, pad: Pad) -> None:
        """Append a Pad to a list of Pads."""
        # Use *pads_group* instead of *self*:
        pads_group: PadsGroup = self
        pads_group.pads.append(pad)

    # PadsGroup.reposition():
    def reposition(self, flags: str, center: P2D, rotate: float, translate: P2D,
                   pads_base: int = 0) -> "PadsGroup":
        """Return a repositioned list of Pad's.

        Args:
            * *flags* (*str*):
               A list of one or more character flags that specify any
               mirroring:
               * 'x': Mirror along the X axis.
               * 'y': Mirror along the Y axis.
            * *center* (*P2D*):
               * The virtual center of the pads group.
            * *z_rotate* (*float*):
               * The amount to rotate around the *center* in radians.
            * *translate* (*P2D*):
               * The location to translate *center* to:
            * *pads_base* (*int*) (Optional: Defaults to 0):
               * An amount to add to each each pin numbr.

        """
        # Unpacksome values from *pads_group*:
        pads_group: PadsGroup = self
        pads: List[Pad] = pads_group.pads

        # Create the *new_pads* fill it with the respositioned pads:
        new_pads_group: PadsGroup = PadsGroup()
        for pad in pads:
            new_pads_group.insert(pad.reposition(flags, center, rotate, translate, pads_base))
        return new_pads_group


# LED:
class LED:
    """Represents a right angle LED."""

    # LED.__init__():
    def __init__(self, name: str, scad_program: "ScadProgram", color: str = "LightGreen") -> None:
        """Initialize a right angle LED."""
        # Mfg Part #: TLPRG5600  Digi-Key part #: TLPG5600-ND
        # Points forward along the +X axis direction.

        # Create the LED *base*:
        base_dx: float = 1.7  # mm
        base_dy: float = 4.8  # mm
        base_dz: float = 4.5  # mm
        base: Scad3D = Cube(
            "LED Base", base_dx, base_dy, base_dz, center=P3D(0.0, 0.0, base_dz / 2.0))

        # Create the LED *optic*:
        optic_dx: float = 4.0  # mm
        optic_diameter: float = 2.0 * 1.75  # mm
        optic_start: P3D = P3D(base_dx / 2.0,           0.0, base_dz / 2.0)
        optic_end: P3D = P3D(-base_dx / 2.0 + optic_dx, 0.0, base_dz / 2.0)
        optic: Scad3D = Cylinder("LED Optic", optic_diameter, optic_start, optic_end, 16)

        # Create *led_union* and *led_color*:
        led_union: Union3D = Union3D("LED Union", [base, optic])
        led_color: Color = Color("LED Color", led_union, color)

        # Create the pins:
        pin_dx: float = 0.4  # mm
        pin_dy: float = 0.45  # mm
        pin_dz: float = 1.6 + .24  # mm
        pin_pitch: float = 2.54  # mm
        pin: Cube = Cube("LED Pin", pin_dx, pin_dy, pin_dz, center=P3D(0.0, 0.0, -pin_dz / 2.0))
        colored_pin: Color = Color("Colored LED Pin", pin, "Gold")
        positive_pin_position2d: P2D = P2D(0.0, -pin_pitch / 2.0)
        positive_pin_position3d: P3D = P3D(0.0, -pin_pitch / 2.0, 0.0)
        negative_pin_position2d: P2D = P2D(0.0, pin_pitch / 2.0)
        negative_pin_position3d: P3D = P3D(0.0, pin_pitch / 2.0, 0.0)
        pin1: Translate3D = Translate3D("LED +", colored_pin, positive_pin_position3d)
        pin2: Translate3D = Translate3D("LED -", colored_pin, negative_pin_position3d)

        # Create the final *led_module* and stuff it into *scad_program*:
        led_module: Module3D = Module3D("LED", [led_color, pin1, pin2])
        scad_program.append(led_module)
        scad_program.if3d.name_match_append("LED", led_module, ["LED"])
        led_use_module: UseModule3D = led_module.use_module_get()

        # Create the *led_pads*:
        pad_diameter: float = 1.80  # mm   (Guess for now)
        pad_drill: float = max(pin_dx, pin_dy) + 0.20  # mm (a little extra pin clearance)
        positive_pad: Pad = Pad("+", pad_diameter, pad_diameter, pad_drill, positive_pin_position2d)
        negative_pad: Pad = Pad("-", pad_diameter, pad_diameter, pad_drill, negative_pin_position2d)
        led_pads: List[Pad] = [positive_pad, negative_pad]

        # Create the *led_artwork*:
        led_outline: SimplePolygon = Square("LED Outline", base_dx, base_dy)
        plus_diameter: float = 2.54  # mm
        plus_x_offset: P2D = P2D(plus_diameter / 2.0, 0)
        plus_y_offset: P2D = P2D(0, plus_diameter / 2.0)
        plus_center: P2D = P2D(0.0, base_dy / 2.0 - 1.5 * plus_diameter)  # 1.5 = trial & error
        plus_horizontal: SimplePolygon = SimplePolygon(
            "Led Plus Horizontal", [plus_center - plus_x_offset, plus_center + plus_x_offset])
        plus_vertical: SimplePolygon = SimplePolygon(
            "Led Plus vertical", [plus_center - plus_y_offset, plus_center + plus_y_offset])
        led_artwork: List[SimplePolygon] = [led_outline, plus_horizontal, plus_vertical]

        # Create the *led_pcb_chunk*:
        led_pcb_chunk: PCBChunk = PCBChunk(
            f"{name} LED", led_pads, [led_use_module], front_artworks=led_artwork)

        # Stuff everything into *led* (i.el *self*).
        # led: LED = self
        self.module: Module3D = led_module
        self.pcb_chunk: PCBChunk = led_pcb_chunk


# Reference:
class Reference:
    """Represents a reference from a schematic to a PCB footprint."""

    # Reference.__init__():
    def __init__(self, name: str, is_front: bool, rotate: float, position: P2D,
                 pcb_chunk: "PCBChunk", value: str) -> None:
        """Initialize a Reference."""
        # Load values into *reference* (i.e. *self*):
        # reference: Reference = self
        self.is_front: bool = is_front
        self.name: str = name
        self.pcb_chunk: PCBChunk = pcb_chunk
        self.position: P2D = position
        self.rotate: float = rotate
        self.value: str = value

    # Reference.__str__():
    def __str__(self) -> str:
        """Return a string representation of a Reference."""
        # Unload values from *reference* (i.e. *self*):
        reference: Reference = self
        name: str = reference.name
        is_front: bool = reference.is_front
        position: P2D = reference.position
        rotate: float = reference.rotate
        value: str = reference.value
        return f"Reference('{name}', {is_front}, {position}, {degrees(rotate)}deg, '{value}')"

    # Reference.reposition():
    def reposition(self, rotate_adjust: float, translate: P2D) -> "Reference":
        """Reposition Reference to a new location and orientation."""
        # Unload values from *reference* (i.e. *self*):
        reference: Reference = self
        name: str = reference.name
        is_front: bool = reference.is_front
        position: P2D = reference.position
        rotate: float = reference.rotate
        pcb_chunk: PCBChunk = reference.pcb_chunk
        value: str = reference.value

        # Create *repositioned_reference* and return it:
        repostioned_reference: Reference = Reference(
            name, is_front, rotate + rotate_adjust, position + translate, pcb_chunk, value)
        return repostioned_reference


# PCBChunk
class PCBChunk:
    """Represents a chunk of stuff that can be put on a PCB."""

    # PCBChunk.__init__():
    def __init__(self, name: str, pads: List[Pad], front_scads: List[Scad3D],
                 front_artworks: List[SimplePolygon] = [], cuts: List[SimplePolygon] = [],
                 back_artworks: List[SimplePolygon] = [], back_scads: List[Scad3D] = [],
                 references: List[Reference] = [], value: str = "",
                 parent: Optional["PCBChunk"] = None) -> None:
        """Return an initialized PCB Chunk.

        Args:
            * *name* (*str*):
              The name of the *PCBChunk*.
            * *pads* (*List*[*Pad*]*):
              A list of *Pad* to placed.  Besides the landing that is
              goes into a foot print, the associated hole is generated
              as well.
            * *front_scads*: (*List*[*Scad3D*]):
              A list of *Scad3d*'s to render in the visual model.
            * *front_artworks*: (*List[*SimplePolygon*])
              (Optional: default is []):
              A list of front artwork polygons to draw.
            * *cuts*: (*List*[*SimplePolygon*])
              (Optional: default is []):
              A list of holes to render in the visual model.
            * *back_artworks*: (*List[*SimplePolygon*])
              (Optional: default is []):
              A list of front artwork polygons to draw.
            * *back_scads*: (*List*[*Scad3D*])
              (Optional: default is []):
              A list of *Scad3D*'s to render on back side of the PCB.
            * *references*: (*List*[*Reference*]):
              A list of *Reference*'s  associated with the PCBChunk.
            * *value*: (*str*)
              (Optional: default is ""):
              A value string for a PCB footprint.

        """
        # Load up *pcb_chunk* (i.e. *self*):
        # pcb_chunk: PCBChunk = self
        self.name: str = name
        self.back_artworks: List[SimplePolygon] = back_artworks[:]
        self.back_scads: List[Scad3D] = back_scads[:]
        self.cuts: List[SimplePolygon] = cuts[:]
        self.front_artworks: List[SimplePolygon] = front_artworks[:]
        self.front_scads: List[Scad3D] = front_scads[:]
        self.pads: List[Pad] = pads[:]
        self.parent: Optional[PCBChunk] = parent
        self.references: List[Reference] = references[:]
        self.value: str = value

    # PCBChunk.__str__():
    def __str__(self) -> str:
        """Return the a summary of PCBChunk."""
        # Unpack some values from *PCBChunk* (i.e. *self*):
        pcb_chunk: PCBChunk = self
        back_artworks: List[SimplePolygon] = pcb_chunk.back_artworks
        back_scads: List[Scad3D] = pcb_chunk.back_scads
        cuts: List[SimplePolygon] = pcb_chunk.cuts
        front_artworks: List[SimplePolygon] = pcb_chunk.front_artworks
        front_scads: List[Scad3D] = pcb_chunk.front_scads
        name: str = pcb_chunk.name
        pads: List[Pad] = pcb_chunk.pads
        parent: Optional[PCBChunk] = pcb_chunk.parent
        references: List[Reference] = pcb_chunk.references
        value: str = pcb_chunk.value

        parent_text: str = f"'{parent.name}'" if isinstance(parent, PCBChunk) else "None"
        return (f"PCBChunk('{name}', C:{len(cuts)}, "
                f"BA::{len(back_artworks)}, BS:{len(back_scads)}, C:{len(cuts)}, "
                f"FA:{len(front_artworks)}, FS:{len(front_scads)} P:{len(pads)} "
                f"R:{len(references)} P:{parent_text} V:'{value}')")
        return ""

    # PCBChunk.artworks_x_mirror():
    def artworks_x_mirror(self) -> "PCBChunk":
        """Return a PCBChunk with pads mirrored around the X axis."""
        # Unpack some values from *pcb_chunk* (i.e. *self*):
        pcb_chunk: PCBChunk = self
        back_artworks: List[SimplePolygon] = pcb_chunk.back_artworks
        front_artworks: List[SimplePolygon] = pcb_chunk.front_artworks

        # Create *x_mirrored_pcb_chunk*, update the pads with X axis mirrored ones, and return it:
        x_mirrored_pcb_chunk: PCBChunk = pcb_chunk.copy()
        back_artwork: SimplePolygon
        x_mirrored_pcb_chunk.back_artworks = [back_artwork.x_mirror("X Mirrored Back Artwork")
                                              for back_artwork in back_artworks]
        front_artwork: SimplePolygon
        x_mirrored_pcb_chunk.front_artworks = [front_artwork.x_mirror("X Mirrored Front Artwork")
                                               for front_artwork in front_artworks]
        return x_mirrored_pcb_chunk

    # PCBChunk.artworks_y_mirror():
    def artworks_y_mirror(self) -> "PCBChunk":
        """Return a PCBChunk with pads mirrored around the Y axis."""
        # Unpack some values from *pcb_chunk* (i.e. *self*):
        pcb_chunk: PCBChunk = self
        back_artworks: List[SimplePolygon] = pcb_chunk.back_artworks
        front_artworks: List[SimplePolygon] = pcb_chunk.front_artworks

        # Create *y_mirrored_pcb_chunk*, update the pads with Y axis mirrored ones, and return it:
        y_mirrored_pcb_chunk: PCBChunk = pcb_chunk.copy()
        back_artwork: SimplePolygon
        y_mirrored_pcb_chunk.back_artworks = [back_artwork.y_mirror("Y Mirrored Back Artwork")
                                              for back_artwork in back_artworks]
        front_artwork: SimplePolygon
        y_mirrored_pcb_chunk.front_artworks = [front_artwork.y_mirror("Y Mirrored Front Artwork")
                                               for front_artwork in front_artworks]
        return y_mirrored_pcb_chunk

    # PCBChunk.copy():
    def copy(self) -> "PCBChunk":
        """Return a copy of a PCBChunk."""
        # Unpack some values from *PCBChunk* (i.e. *self*):
        pcb_chunk: PCBChunk = self
        back_artworks: List[SimplePolygon] = pcb_chunk.back_artworks
        back_scads: List[Scad3D] = pcb_chunk.back_scads
        cuts: List[SimplePolygon] = pcb_chunk.cuts
        front_artworks: List[SimplePolygon] = pcb_chunk.front_artworks
        front_scads: List[Scad3D] = pcb_chunk.front_scads
        name: str = pcb_chunk.name
        pads: List[Pad] = pcb_chunk.pads
        parent: Optional[PCBChunk] = pcb_chunk.parent
        references: List[Reference] = pcb_chunk.references
        value: str = pcb_chunk.value

        # Create the *copied_pcb_chunk* and return it.
        # (Note, PCBChunk always makes shallow copies of the lists):
        copied_pcb_chunk: PCBChunk = PCBChunk(name, pads, front_scads,
                                              front_artworks=front_artworks, cuts=cuts,
                                              back_artworks=back_artworks, back_scads=back_scads,
                                              references=references, parent=parent, value=value)
        return copied_pcb_chunk

    # PBChunk.header_line_generate():
    def footprint_header_line_generate(self, prefix: str, is_front: bool,
                                       library_name: str, footprint_name: str,
                                       edit_timestamp: int, create_timestamp: int,
                                       tracing: str = "") -> str:
        """Return the first line of KiCad footprint file."""
        # Extratt some values from *pcb_chunk* (i.e. *self*):
        pcb_chunk: PCBChunk = self
        name: str = pcb_chunk.name

        # Perform any requested *tracing*:
        if tracing:
            print(f"{tracing}=>PCBChunk.footprint_header_line_generate('{name}', {is_front} "
                  f"'{library_name}', '{footprint_name}', "
                  f"{edit_timestamp:08X}, {create_timestamp:0X})")

        # Generate *header_line*:
        layer: str = "F.Cu" if is_front else "B.Cu"
        full_footprint_name: str = f"{library_name}:{footprint_name}"
        header_line: str = f"{prefix}(module {KicadPCB.string(full_footprint_name)} (layer {layer})"
        if edit_timestamp != 0:
            header_line += f" (tedit {edit_timestamp:08X})"
        if create_timestamp != 0:
            header_line += f" (tstamp {create_timestamp:08X})"

        # Wrap up any requested *tracing* and return *header_line*:
        if tracing:
            print(f"{tracing}<=PCBChunk.footprint_header_line_generate('{name}', {is_front} "
                  f"'{library_name}', '{footprint_name}', "
                  f"{edit_timestamp:08X}, {create_timestamp:0X})=>'{header_line}'")
        return header_line

    # PCBChunk.footprint_body_lines_generate():
    def footprint_body_lines_generate(self, prefix: str, footprint_mode: bool, is_front: bool,
                                      pcb_origin: P2D, footprint_name: str, reference_name: str,
                                      value_name: str, path_id: int, rotate: float, position: P2D,
                                      pads_reverse: bool, tracing: str = "") -> List[str]:
        """Generate a list of corresponding to a footprint."""
        def polygon_lines(pcb_origin: P2D, polygon: SimplePolygon, layer: str) -> List[str]:
            lines: List[str] = []
            size = len(polygon)
            if size == 1:
                lines.append(KicadPCB.fp_line(prefix, pcb_origin, polygon[0], polygon[0], layer))
            elif size == 2:
                lines.append(KicadPCB.fp_line(prefix, pcb_origin, polygon[0], polygon[1], layer))
            else:
                for index in range(size):
                    start_point: P2D = polygon[index]
                    end_point: P2D = polygon[(index + 1) % size]
                    lines.append(
                        KicadPCB.fp_line(prefix, pcb_origin, start_point, end_point, layer))
                    # print(f"[{index}]: {start_point} {end_point}")
            return lines

        # Do any side swapping if the *PCBChunk* (i.e. *self*) is actually flipped to the back.:
        pcb_chunk: PCBChunk = self
        if not is_front:
            pcb_chunk = pcb_chunk.sides_swap().artworks_x_mirror().pads_x_mirror()

        # Unpack some values from *PCBChunk* (i.e. *self*):
        back_artworks: List[SimplePolygon] = pcb_chunk.back_artworks
        # back_scads: List[Scad3D] = pcb_chunk.back_scads
        cuts: List[SimplePolygon] = pcb_chunk.cuts
        front_artworks: List[SimplePolygon] = pcb_chunk.front_artworks
        # front_scads: List[Scad3D] = pcb_chunk.front_scads
        name: str = pcb_chunk.name
        pads: List[Pad] = pcb_chunk.pads
        # parent: Optional[PCBChunk] = pcb_chunk.parent
        # references: List[Reference] = pcb_chunk.references

        # *footprint_mode* may not be needed:
        footprint_mode = False

        # Perform any requested *tracing*:
        next_tracing: str = tracing + " " if tracing else ""
        if tracing:
            print(f"{tracing}=>PCBChunk.body_lines_generate('{name}', '{prefix}', {footprint_mode},"
                  f" {is_front}, {pcb_origin}, '{footprint_name}', '{reference_name}', "
                  f"'{value_name}' {path_id:08X}, {degrees(rotate)}deg, {position})")

        # Extract *origin_x* and *origin_y* from *pcb_origin*.
        # Note that KiCad has the Y axis inverted, so there are minus signs on coversion.
        pcb_origin_x: float = pcb_origin.x
        pcb_origin_y: float = pcb_origin.y
        footprint_origin: P2D = P2D(0.0, 0.0)
        origin_x: float = footprint_origin.x
        origin_y: float = footprint_origin.y
        if tracing:
            print(f"{tracing}pcb_origin={pcb_origin} footprint_origin={footprint_origin}")

        lines: List[str] = []
        rotate_text: str = "" if rotate == 0.0 else f" {KicadPCB.number(degrees(rotate), 1)}"
        if tracing:
            print(f"{tracing}rotate_text: '{rotate_text}'")

        # If appropriate add a ...(at ...) line:
        if position.x != 0.0 or position.y != 0.0 or rotate != 0.0:
            lines.append(f"{prefix}  (at {KicadPCB.number(pcb_origin_x + position.x, 4)} "
                         f"{KicadPCB.number(pcb_origin_y - position.y, 4)}{rotate_text})")

        # If appropriate add a ...(path /XXXXX) line:
        if path_id != 0:
            lines.append(f"{prefix}  (path /{path_id:08X})")

        # Do a ...(fp_text reference ...) line:
        justify_mirror: str = "" if is_front else " (justify mirror)"
        tail_parenthesis: str = ")" if footprint_mode else ""
        reference_position: P2D = P2D(0.0, 2.0)
        reference_layer: str = "F.SilkS" if is_front else "B.SilkS"
        line = f"{prefix}  (fp_text reference {KicadPCB.string(reference_name)}"
        line += f" (at {KicadPCB.number(origin_x + reference_position.x, 4)} "
        line += f"{KicadPCB.number(origin_y - reference_position.y, 4)}{rotate_text})"
        line += f" (layer {reference_layer})"
        lines.append(line)
        lines.append(f"{prefix}    (effects (font (size 1 1) (thickness 0.2))"
                     f"{justify_mirror}){tail_parenthesis}")
        if not footprint_mode:
            lines.append(f"{prefix}  )")

        # Do a ...(fp_text value ...) line:
        fabrication_layer: str = "F.Fab" if is_front else "B.Fab"
        value_position: P2D = P2D(0.0, 0.0)
        if value_name == "":
            value_name = "NO_VALUE"
        line = f"{prefix}  (fp_text value {KicadPCB.string(value_name)}"
        line += f" (at {KicadPCB.number(origin_x + value_position.x, 4)} "
        line += f"{KicadPCB.number(origin_y - value_position.y, 4)}{rotate_text})"
        line += f" (layer {fabrication_layer})"
        lines.append(line)
        lines.append(f"{prefix}    (effects (font (size 1 1) (thickness 0.2))"
                     f"{justify_mirror}){tail_parenthesis}")
        if not footprint_mode:
            lines.append(f"{prefix}  )")

        # Add all of the artwork to *lines*:
        polygon: SimplePolygon
        if pads_reverse:
            for polygon in reversed(front_artworks):
                lines.extend(polygon_lines(footprint_origin, polygon, "F.SilkS"))
                for polygon in back_artworks:
                    lines.extend(polygon_lines(footprint_origin, polygon, "B.SilkS"))
        else:
            for polygon in reversed(front_artworks):
                lines.extend(polygon_lines(footprint_origin, polygon, "F.SilkS"))
                for polygon in back_artworks:
                    lines.extend(polygon_lines(footprint_origin, polygon, "B.SilkS"))

        # Add all of the *pads* to *lines*.  The pads seem to get reversed when they copied from
        # the `.kicad_mod` file over to the `.kicad_pcb file`; hence the *pads_reverse_flag*:
        pad: Pad
        if pads_reverse:
            for pad in reversed(pads):
                lines.append(pad.kicad_line(prefix, footprint_origin, tracing=next_tracing))
        else:
            for pad in pads:
                lines.append(pad.kicad_line(prefix, footprint_origin, tracing=next_tracing))

        # Add any cut lines:
        offset_x: int = 0
        offset_y: int = 0
        layer: str = "Edge.Cuts"
        width: float = .05
        cut: SimplePolygon
        for cut in cuts:
            cut_size: int = len(cut)
            if tracing:
                print(f"{tracing}cut_size:{cut_size}")
            cut_index1: int
            cut_index2: int
            for cut_index1 in range(cut_size):
                cut_index2 = (cut_index1 + 1) % cut_size
                point1: P2D = cut[cut_index1]
                point2: P2D = cut[cut_index2]
                cut_line: str = ("  (fp_line "
                                 f"(start {KicadPCB.number(offset_x + point1.x, 2)} "
                                 f"{KicadPCB.number(offset_y - point1.y, 2)}) "
                                 f"(end {KicadPCB.number(offset_x + point2.x, 2)} "
                                 f"{KicadPCB.number(offset_y - point2.y, 2)}) "
                                 f"(layer {layer}) "
                                 f"(width {KicadPCB.number(width, 2)}))")
                lines.append(cut_line)

        # Add the closing parenthesis and return the resulting *lines*:
        lines.append(f"{prefix})")

        if tracing:
            print(f"{tracing}<=PCBChunk.body_lines_generate('{name}', '{prefix}', {footprint_mode},"
                  f" {is_front}, {pcb_origin}, '{footprint_name}', '{reference_name}', "
                  f"'{value_name}' {path_id:08X}, {degrees(rotate)}deg, {position})")
        return lines

    # PCBChunk.footprint_generate():
    def footprint_generate(self, library_name: str, directory: Path, tracing: str = "") -> None:
        """Generate a footprint from PCBChunk.

        Args:
            * *directory* (*Path*):
              The directory to store the footprint into.

        """
        # Perform any reqested *tracing*:
        next_tracing: str = tracing + " " if tracing else ""
        if tracing:
            print(f"{tracing}=>PCBChunk.footprint_generate('library_name', '{directory.name}')")

        # Unpack some values from *PCBChunk* (i.e. *self*):
        pcb_chunk: PCBChunk = self
        # back_artworks: List[SimplePolygon] = pcb_chunk.back_artworks
        # front_artworks: List[SimplePolygon] = pcb_chunk.front_artworks
        name: str = pcb_chunk.name
        # pads: List[Pad] = pcb_chunk.pads
        # parent: Optional[PCBChunk] = pcb_chunk.parent
        value: str = pcb_chunk.value
        if tracing:
            print(f"{tracing}value:'{value}'")

        # Create the *footprint_path* file name path:
        footprint_path: Path = directory / (name + ".kicad_mod")
        footprint_mode: bool = True

        # Preread any previous version of *footprint_path* to find any previous *create_timestamp*,
        # *edit_timestamp*, and *previous_body_lines*:
        create_timestamp: int = 0
        edit_timestamp: int = 0
        previous_body_lines: List[str] = []
        footprint_file: IO[Any]
        if footprint_path.is_file():
            # Read in *previous_footprint_lines*:
            with open(footprint_path, "r") as footprint_file:
                previous_footprint_lines = footprint_file.read().splitlines()
            if len(previous_footprint_lines) >= 1:
                # There is a *first_line*, extract the two timestamps if possible:
                first_line: str = previous_footprint_lines[0]
                tedit_index: int = first_line.find("(tedit ")
                if tedit_index >= 0:
                    edit_timestamp_text: str = first_line[tedit_index+7:tedit_index+7+8]
                    try:
                        edit_timestamp = int(edit_timestamp_text, 16)
                    except ValueError:
                        assert False, f"Edit '{edit_timestamp_text}' from '{first_line}'"
                tstamp_index: int = first_line.find("(tstamp ")
                if tstamp_index >= 0:
                    create_timestamp_text: str = first_line[tstamp_index+8:tstamp_index+8+8]
                    try:
                        create_timestamp = int(create_timestamp_text, 16)
                    except ValueError:
                        assert False, f"Create '{create_timestamp_text}' from '{first_line}'"
                previous_body_lines = previous_footprint_lines[1:]

        # Create the *new_body_lines*:
        origin2d: P2D = P2D(0.0, 0.0)
        prefix: str = ""
        path_id: int = 0  # No *path_id*'s in a footprint.
        is_front: bool = True
        new_body_lines: List[str] = pcb_chunk.footprint_body_lines_generate(
            prefix, footprint_mode, is_front, origin2d, name, "REF**", value, path_id,
            0.0, origin2d, False, tracing=next_tracing)

        # It is really important to create the *create_timestamp* only once and always resuse it.
        # The *edit_timestamp* needs to be changed anytime the file content changes.
        if create_timestamp == 0:
            create_timestamp = int(time.time())
        if edit_timestamp < create_timestamp:
            edit_timestamp = create_timestamp
        if previous_body_lines != new_body_lines:
            # Footprint content changed:
            edit_timestamp = int(time.time())

        # Always write out the file, even if the content has not changed so that the modification
        # timestamp of the file is updated for make dependency checking:
        new_header_line: str = pcb_chunk.footprint_header_line_generate(
            prefix, is_front, library_name, name, edit_timestamp, create_timestamp)
        new_footprint_lines: List[str] = [new_header_line] + new_body_lines
        if tracing:
            print(f"{tracing}Writing footprint out to {footprint_path.name}")
        with open(footprint_path, "w") as footprint_file:
            footprint_text: str = '\n'.join(new_footprint_lines + [""])
            footprint_file.write(footprint_text)

        # Wrap up any reqested *tracing*:
        if tracing:
            print(f"{tracing}<=PCBChunk.footprint_generate('library_name', '{directory.name}')")

    @staticmethod
    # PCBChunk.join():
    def join(name: str, pcb_chunks: List["PCBChunk"], value: str = "") -> "PCBChunk":
        """Return a merged set of PCBChunk's."""
        # Create *merged_pcb_chunk* with empty values and unpack them all:
        merged_pcb_chunk: PCBChunk = PCBChunk(name, [], [], front_artworks=[], cuts=[],
                                              back_artworks=[], back_scads=[], value=value)
        merged_back_artworks: List[SimplePolygon] = merged_pcb_chunk.back_artworks
        merged_back_scads: List[Scad3D] = merged_pcb_chunk.back_scads
        merged_cuts: List[SimplePolygon] = merged_pcb_chunk.cuts
        merged_front_artworks: List[SimplePolygon] = merged_pcb_chunk.front_artworks
        merged_front_scads: List[Scad3D] = merged_pcb_chunk.front_scads
        merged_pads: List[Pad] = merged_pcb_chunk.pads
        merged_references: List[Reference] = merged_pcb_chunk.references

        # Sweep through each *pcb_chunk* in *pcb_chunks* and merge the values in:
        merged_parent: Optional[PCBChunk] = None
        index: int
        pcb_chunk: PCBChunk
        for index, pcb_chunk in enumerate(pcb_chunks):
            merged_back_artworks.extend(pcb_chunk.back_artworks)
            merged_back_scads.extend(pcb_chunk.back_scads)
            merged_cuts.extend(pcb_chunk.cuts)
            merged_front_artworks.extend(pcb_chunk.front_artworks)
            merged_front_scads.extend(pcb_chunk.front_scads)
            merged_pads.extend(pcb_chunk.pads)
            merged_references.extend(pcb_chunk.references)
            parent: Optional[PCBChunk] = pcb_chunk.parent
            if isinstance(parent, PCBChunk):
                # Complain if differing parents are found in *pcb_chunk*:
                assert merged_parent is None or merged_parent is parent, "Incompatible parents"
                merged_parent = parent
        merged_pcb_chunk.parent = merged_parent

        # Wrap up and and return *merged_pcb_chunk*:
        return merged_pcb_chunk

    # PCBChunk.pads_rebase():
    def pads_rebase(self, delta: int) -> "PCBChunk":
        """Return a PadsChunk with rebased Pad names."""
        # Unpack some values PCBChunk (i.e. *self*):
        pcb_chunk: PCBChunk = self
        pads: List[Pad] = pcb_chunk.pads

        # Create *rebased_pads*:
        pad: Pad
        rebased_pads: List[Pad] = [pad.rebase(delta) for pad in pads]

        # Create *rebased_pcb_chunk*, stuff in the updated *rebased_pads* and return it:
        rebased_pcb_chunk: PCBChunk = pcb_chunk.copy()
        rebased_pcb_chunk.pads = rebased_pads
        return rebased_pcb_chunk

    # PCBChunk.pads_show():
    def pads_show(self, prefix: str = "") -> None:
        """Show the pads."""
        # Unpack some values from *pcb_chunk* (i.e. *self*):
        pcb_chunk: PCBChunk = self
        pads: List[Pad] = pcb_chunk.pads

        index: int
        pad: Pad
        for index, pad in enumerate(pads):
            print(f"{prefix}Pad[{index}]: {pad}")

    # PCBChunk.pads_x_mirror():
    def pads_x_mirror(self) -> "PCBChunk":
        """Return a PCBChunk with pads mirrored around the X axis."""
        # Unpack some values from *pcb_chunk* (i.e. *self*):
        pcb_chunk: PCBChunk = self
        pads: List[Pad] = pcb_chunk.pads

        # Create *x_mirrored_pcb_chunk*, update the pads with X axis mirrored ones, and return it:
        x_mirrored_pcb_chunk: PCBChunk = pcb_chunk.copy()
        pad: Pad
        x_mirrored_pcb_chunk.pads = [pad.x_mirror() for pad in pads]
        return x_mirrored_pcb_chunk

    # PCBChunk.pads_y_mirror():
    def pads_y_mirror(self) -> "PCBChunk":
        """Return a PCBChunk with pads mirrored around the X axis."""
        # Unpack some values from *pcb_chunk* (i.e. *self*):
        pcb_chunk: PCBChunk = self
        pads: List[Pad] = pcb_chunk.pads

        # Create *y_mirrored_pcb_chunk*, update the pads with Y axis  mirrored ones, and return it:
        y_mirrored_pcb_chunk: PCBChunk = pcb_chunk.copy()
        pad: Pad
        y_mirrored_pcb_chunk.pads = [pad.y_mirror() for pad in pads]
        return y_mirrored_pcb_chunk

    # PCBChunk.parent_set():
    def parent_set(self, parent: "PCBChunk") -> None:
        """Assign a parent PCBChunk to mirror operations to."""
        # Use *pcb_chunk* instead of *self*:
        pcb_chunk: PCBChunk = self
        assert pcb_chunk.parent is None, "Parent already set"
        pcb_chunk.parent = parent

    # PCBChunk.pcb_update():
    def pcb_update(self, scad_program: "ScadProgram", pcb_origin: P2D, dz: float,
                   pcb_exterior: SimplePolygon, color: str,
                   kicad_pcb_path: Optional[Path], more_references: List[Reference],
                   tracing: str = "") -> Module3D:
        """Generate a PCB."""
        # Perform any requested *tracing*:
        path_name: str = kicad_pcb_path.name if isinstance(kicad_pcb_path, Path) else "None"
        next_tracing: str = tracing + " " if tracing else ""
        if tracing:
            print(f"{tracing}=>PCBChunk.pcb_update(*, {dz:.1f}, '{color}', '{path_name}')")

        # Note: this operation does *NOT* get mirrored to a parent.

        # Unpack some values from *pcb_chunk* (i.e. *self*):
        pcb_chunk: PCBChunk = self
        # back_artworks: List[SimplePolygon] = pcb_chunk.back_artworks
        back_scads: List[Scad3D] = pcb_chunk.back_scads
        cuts: List[SimplePolygon] = pcb_chunk.cuts
        # front_artworks: List[SimplePolygon] = pcb_chunk.front_artworks
        front_scads: List[Scad3D] = pcb_chunk.front_scads
        name: str = pcb_chunk.name
        # print(f"PCBChunk.pcb_update(): name:'{name}'")
        pads: List[Pad] = pcb_chunk.pads
        references: List[Reference] = pcb_chunk.references

        # all_references: List[Reference] = references + more_references
        all_cuts: List[SimplePolygon] = cuts + [pcb_exterior]

        # Create the *board_polygon* fill it with *pads* and *cuts*:
        board_polygon: Polygon = Polygon(f"{name} Board Polygon", [pcb_exterior], lock=False)
        pad: Pad
        for pad in pads:
            pad.hole_append(board_polygon)
        cut: SimplePolygon
        for cut in cuts:
            board_polygon.append(cut)
        board_polygon.lock()

        # Extrude and *color* the *board_polygon*:
        extruded_pcb: LinearExtrude = LinearExtrude(f"Extruded {name} PCB", board_polygon, dz)
        colored_pcb: Color = Color(f"Colored {name} PCB", extruded_pcb, color)

        # Group all of the *front_scads* into a *front_union* and construct *module*:
        front_scads_union: Union3D = Union3D(f"{name} Front Union", front_scads)
        translated_front_scads_union: Translate3D = Translate3D(
            f"{name} Translated Front Scads Union", front_scads_union, P3D(0.0, 0.0, dz))
        back_scads_union: Union3D = Union3D(f"{name} Back Scads Union", back_scads)
        translated_back_scads_union: Translate3D = Translate3D(
            f"{name} Translated Back Scads Union", back_scads_union, P3D(0.0, 0.0, 0.0))
        module3d_scads: List[Scad3D] = [
            colored_pcb, translated_front_scads_union, translated_back_scads_union]
        module3d: Module3D = Module3D(f"{name}_pcb", module3d_scads)
        scad_program.append(module3d)
        scad_program.if3d.name_match_append(f"{name.lower()}_board", module3d, [f"{name} Board"])

        if isinstance(kicad_pcb_path, Path):
            pcb_chunk.pcb_cuts_refereneces_update(kicad_pcb_path, pcb_exterior, pcb_origin,
                                                  more_references, tracing=next_tracing)

        # Read in the *previous_pcb_lines* associated with *kicad_pcb_path*:
        if False and isinstance(kicad_pcb_path, Path):
            assert kicad_pcb_path.is_file(), f"'{kicad_pcb_path}' does not exist."
            previous_pcb_text: str
            kicad_pcb_file: IO[Any]
            with open(kicad_pcb_path, "r") as kicad_pcb_file:
                previous_pcb_text = kicad_pcb_file.read()
            previous_pcb_lines: List[str] = previous_pcb_text.splitlines()

            # Extract the *pcb_modules_table* and *ordered_pcb_modules* from *previous_pcb_lines*:
            pcb_modules_table: Dict[str, PCBModule]
            ordered_pcb_modules: List[PCBModule]
            pcb_modules_table, ordered_pcb_modules = PCBModule.modules_extract(previous_pcb_lines)
            reassembled_lines: List[str] = PCBModule.generate_lines(ordered_pcb_modules)
            assert reassembled_lines == previous_pcb_lines

            # Remove the cut lines:
            for pcb_module in ordered_pcb_modules:
                pcb_module.cut_lines_strip()

            # Sweep through all of *references*:
            reference: Reference
            for reference in references:
                # Unpack some values from *reference*:
                reference_is_front: bool = reference.is_front
                reference_name: str = reference.name
                reference_position: P2D = reference.position
                reference_rotate: float = reference.rotate
                reference_value: str = reference.value
                reference_rotate = 0.0  # Kludge for now.
                reference_pcb_chunk: PCBChunk = reference.pcb_chunk
                if tracing:
                    print(f"{tracing}Ref: name:'{reference_name}' postion:{reference_position} "
                          f"rotate:{degrees(reference_rotate)} is_front:{reference_is_front}")

                # Make sure that there is *reference_pcb_module* that match *reference_name*:
                if reference_name in pcb_modules_table:
                    # We have a match; now unpack some values form *reference_pcb_module*:
                    reference_pcb_module: PCBModule = pcb_modules_table[reference_name]
                    create_timestamp: int = reference_pcb_module.create_timestamp
                    edit_timestamp: int = reference_pcb_module.edit_timestamp
                    footprint_name: str = reference.pcb_chunk.name
                    path_id: int = reference_pcb_module.path_id
                    module_prefix: str = reference_pcb_module.module_prefix

                    # Create the *footprint_header_line*:
                    library_name: str = "HR2"  # Kludge!
                    if tracing:
                        print(f"{tracing}Ref '{reference_name}' for '{path_name}'")
                    footprint_header_line: str = (
                        reference_pcb_chunk.footprint_header_line_generate(
                            module_prefix, reference_is_front, library_name, footprint_name,
                            edit_timestamp, create_timestamp, tracing=next_tracing))

                    # Create the *footprint_body_lines*:
                    footprint_mode: bool = False  # False implies PCB mode.
                    footprint_body_lines: List[str] = (
                        reference_pcb_chunk.footprint_body_lines_generate(
                            module_prefix, footprint_mode, reference_is_front, pcb_origin,
                            f"{library_name}:{footprint_name}", reference_name,
                            reference_value, path_id, reference_rotate,
                            reference_position, True, tracing=next_tracing))
                    footprint_lines: List[str] = [footprint_header_line] + footprint_body_lines
                    reference_pcb_module.module_lines = footprint_lines
                else:
                    # Obviously something is wrong:
                    if tracing:
                        print(f"{tracing}Reference '{reference_name}' not for '{kicad_pcb_path}'")

            # Insert the cuts into the *final_pcb_module*:
            final_pcb_module: PCBModule = ordered_pcb_modules[-1]
            final_pcb_module.cut_lines_insert(all_cuts, pcb_origin)

            # Create *final_assembled_lines* and write them out:
            final_reassembled_lines = PCBModule.generate_lines(ordered_pcb_modules)
            with open(kicad_pcb_path, "w") as kicad_pcb_file:
                kicad_pcb_file.write('\n'.join(final_reassembled_lines + [""]))

        # Wrap up any requested *tracing*:
        if tracing:
            print(f"{tracing}<=PCBChunk.pcb_update(*, {dz:.1f}, '{color}', '{path_name}')")

        # Stuff *module* into *pcb_chunk* (i.e. *self*) and return it:
        self.module: Module3D = module3d
        return module3d

    # PCBChunk.pcb_cuts_references_update():
    def pcb_cuts_refereneces_update(self,
                                    kicad_pcb_path: Optional[Path],
                                    pcb_exterior: SimplePolygon,
                                    pcb_origin: P2D,
                                    more_references: List[Reference] = [],
                                    tracing: str = "") -> None:
        """Update reference positions and cut lines for a PCBChunk."""
        # Unpack some values from *pcb_chunk* (i.e. *self*):
        pcb_chunk: PCBChunk = self
        cuts: List[SimplePolygon] = pcb_chunk.cuts
        name: str = pcb_chunk.name
        references: List[Reference] = pcb_chunk.references

        # Perform any requested *tracing*:
        path_text: str = kicad_pcb_path.name if isinstance(kicad_pcb_path, Path) else "None"
        next_tracing: str = tracing + " " if tracing else ""
        if tracing:
            print(f"{tracing}=>PCBChunk.pcb_cuts_references_update('{name}', '{path_text}', *, "
                  f"{pcb_origin}, |more_references|={len(references)}")

        all_references: List[Reference] = references + more_references
        all_cuts: List[SimplePolygon] = cuts + [pcb_exterior]
        if tracing:
            print(f"{tracing}|all_references|={len(all_references)} |all_cuts|={len(all_cuts)}")

        # Read in the *previous_pcb_lines* associated with *kicad_pcb_path*:
        if isinstance(kicad_pcb_path, Path):
            assert kicad_pcb_path.is_file(), f"'{kicad_pcb_path}' does not exist."
            previous_pcb_text: str
            kicad_pcb_file: IO[Any]
            with open(kicad_pcb_path, "r") as kicad_pcb_file:
                previous_pcb_text = kicad_pcb_file.read()
            previous_pcb_lines: List[str] = previous_pcb_text.splitlines()

            # Extract the *pcb_modules_table* and *ordered_pcb_modules* from *previous_pcb_lines*:
            pcb_modules_table: Dict[str, PCBModule]
            ordered_pcb_modules: List[PCBModule]
            pcb_modules_table, ordered_pcb_modules = PCBModule.modules_extract(previous_pcb_lines,
                                                                               kicad_pcb_path)
            reassembled_lines: List[str] = PCBModule.generate_lines(ordered_pcb_modules)
            assert reassembled_lines == previous_pcb_lines

            # Remove the cut lines:
            for pcb_module in ordered_pcb_modules:
                pcb_module.cut_lines_strip()

            # Sweep through all of *references*:
            reference: Reference
            for reference in references:
                # Unpack some values from *reference*:
                reference_name: str = reference.name
                reference_position: P2D = reference.position

                # Make sure that there is *reference_pcb_module* that match *reference_name*:
                if reference_name in pcb_modules_table:
                    # We have a match; now unpack some values form *reference_pcb_module*:
                    reference_pcb_module: PCBModule = pcb_modules_table[reference_name]
                    reference_pcb_module.reference_position_update(
                        reference_position, pcb_origin, tracing=next_tracing)
                else:
                    # Obviously something is wrong:
                    if tracing:
                        print(f"{tracing}Reference '{reference_name}' not for '{kicad_pcb_path}'")

            # Insert the cuts into the *final_pcb_module*:
            final_pcb_module: PCBModule = ordered_pcb_modules[-1]
            final_pcb_module.cut_lines_insert(all_cuts, pcb_origin)

            # Create *final_assembled_lines* and write them out:
            final_reassembled_lines = PCBModule.generate_lines(ordered_pcb_modules)
            with open(kicad_pcb_path, "w") as kicad_pcb_file:
                kicad_pcb_file.write('\n'.join(final_reassembled_lines + [""]))

        # Wrap up any requested *tracing*:
        if tracing:
            print(f"{tracing}<=PCBChunk.pcb_cuts_references_update('{name}', '{path_text}', *, "
                  f"{pcb_origin}, |more_references|={len(more_references)}")

    # PCBChunk.references_show():
    def references_show(self, prefix: str = "") -> None:
        """Show the pads."""
        # Unpack some values from *pcb_chunk* (i.e. *self*):
        pcb_chunk: PCBChunk = self
        references: List[Reference] = pcb_chunk.references

        index: int
        reference: Reference
        for index, reference in enumerate(references):
            name: str = reference.name
            is_front: bool = reference.is_front
            position: P2D = reference.position
            rotate: float = reference.rotate
            print(f"{prefix}Reference[{index}]: "
                  f"Reference('{name}' {is_front} {position} {degrees(rotate)})")

    # PCBChunk.scads_x_flip():
    def scads_x_flip(self) -> "PCBChunk":
        """Return a PCBChunk with scads fliped around the X axis."""
        # Unpack some values from *pcb_chunk* (i.e. *self*):
        pcb_chunk: PCBChunk = self
        back_scads: List[Scad3D] = pcb_chunk.back_scads
        front_scads: List[Scad3D] = pcb_chunk.front_scads
        name: str = pcb_chunk.name
        parent: Optional[PCBChunk] = pcb_chunk.parent

        # Mirror operation to any parent:
        if isinstance(parent, PCBChunk):
            parent.scads_x_flip()

        # Create *x_mirrored_pcb_chunk*, update the back/front scads with X axis mirrored ones,
        # and return it:
        x_flipped_pcb_chunk: PCBChunk = pcb_chunk.copy()
        scad: Scad3D
        x_flipped_pcb_chunk.back_scads = [scad.x_flip(f"X Flipped {name}") for scad in back_scads]
        x_flipped_pcb_chunk.front_scads = [scad.x_flip(f"X Flipped {name}") for scad in front_scads]
        return x_flipped_pcb_chunk

    # PCBChunk.scads_y_flip():
    def scads_y_flip(self) -> "PCBChunk":
        """Return a PCBChunk with scads fliped around the Y axis."""
        # Unpack some values from *pcb_chunk* (i.e. *self*):
        pcb_chunk: PCBChunk = self
        back_scads: List[Scad3D] = pcb_chunk.back_scads
        front_scads: List[Scad3D] = pcb_chunk.front_scads
        name: str = pcb_chunk.name
        parent: Optional[PCBChunk] = pcb_chunk.parent

        # Mirror operation to any parent:
        if isinstance(parent, PCBChunk):
            parent.scads_y_flip()

        # Create *y_mirrored_pcb_chunk*, update the back/front with Y axis mirrored ones,
        # and return it:
        y_flipped_pcb_chunk: PCBChunk = pcb_chunk.copy()
        scad: Scad3D
        y_flipped_pcb_chunk.back_scads = [scad.y_flip(f"Y Flipped {name}") for scad in back_scads]
        y_flipped_pcb_chunk.front_scads = [scad.y_flip(f"Y Flipped {name}") for scad in front_scads]
        return y_flipped_pcb_chunk

    # PCBChunk.sides_swap():
    def sides_swap(self) -> "PCBChunk":
        """Swap from the artworks and scads from front to back."""
        # Unpack some values from *PCBChunk* (i.e. *self*):
        pcb_chunk: PCBChunk = self
        back_artworks: List[SimplePolygon] = pcb_chunk.back_artworks
        back_scads: List[Scad3D] = pcb_chunk.back_scads
        front_artworks: List[SimplePolygon] = pcb_chunk.front_artworks
        front_scads: List[Scad3D] = pcb_chunk.front_scads
        parent: Optional[PCBChunk] = pcb_chunk.parent

        # Mirror operation to any parent:
        if isinstance(parent, PCBChunk):
            parent.sides_swap()

        # Create *side_swapped_pcb_chunk* with front/back scad/artworks swapped and return it:
        side_swapped_pcb_chunk: PCBChunk = pcb_chunk.copy()
        side_swapped_pcb_chunk.back_artworks = front_artworks
        side_swapped_pcb_chunk.back_scads = front_scads
        side_swapped_pcb_chunk.front_artworks = back_artworks
        side_swapped_pcb_chunk.front_scads = back_scads
        return side_swapped_pcb_chunk

    # PCBChunk.reposition():
    def reposition(self, center: P2D, rotate: float, translate: P2D) -> "PCBChunk":
        """Return a repositioned PCBChunk."""
        # Unpack some values *pcb_chunk* (i.e. *self*):
        pcb_chunk: PCBChunk = self
        back_artworks: List[SimplePolygon] = pcb_chunk.back_artworks
        back_scads: List[Scad3D] = pcb_chunk.back_scads
        cuts: List[SimplePolygon] = pcb_chunk.cuts
        front_scads: List[Scad3D] = pcb_chunk.front_scads
        front_artworks: List[SimplePolygon] = pcb_chunk.front_artworks
        name: str = pcb_chunk.name
        pads: List[Pad] = pcb_chunk.pads
        parent: Optional[PCBChunk] = pcb_chunk.parent
        references: List[Reference] = pcb_chunk.references
        assert len(references) == 0, "References should never be repositioned."

        # Mirror any operation to the *parent*:
        if isinstance(parent, PCBChunk):
            parent.reposition(center, rotate, translate)

        # Create some 3D values:
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        center3d: P3D = P3D(center.x, center.y, 0.0)
        translate3d: P3D = P3D(translate.x, translate.y, 0.0)

        # Create all of the repositioned values:
        back_artwork: SimplePolygon
        repositioned_back_artworks: List[SimplePolygon] = [
            back_artwork.reposition(center, rotate, translate)
            for back_artwork in back_artworks]

        back_scad: Scad3D
        repositioned_back_scads: List[Scad3D] = [
            back_scad.reposition(name, center3d, z_axis, rotate, translate3d)
            for back_scad in back_scads]

        cut: SimplePolygon
        repositioned_cuts: List[SimplePolygon] = [
            cut.reposition(center, rotate, translate)
            for cut in cuts]

        front_artwork: SimplePolygon
        repositioned_front_artworks: List[SimplePolygon] = [
            front_artwork.reposition(center, rotate, translate)
            for front_artwork in front_artworks]

        front_scad: Scad3D
        repositioned_front_scads: List[Scad3D] = [
            front_scad.reposition(name, center3d, z_axis, rotate, translate3d)
            for front_scad in front_scads]

        pad: Pad
        repositioned_pads: List[Pad] = [
            pad.reposition("", center, rotate, translate)
            for pad_index, pad in enumerate(pads)]

        repositioned_references: List[Reference] = [
            reference.reposition(rotate, translate)
            for reference in references]

        # Create *repositioned_pcb_chunk* and return it:
        repositioned_pcb_chunk: PCBChunk = PCBChunk(
            name, repositioned_pads, repositioned_front_scads,
            front_artworks=repositioned_front_artworks, cuts=repositioned_cuts,
            back_artworks=repositioned_back_artworks, back_scads=repositioned_back_scads,
            references=repositioned_references)
        return repositioned_pcb_chunk


# PCBGroup:
class PCBGroup:
    """A helper class to catagorize PCB stuff into sub-groups."""

    # PCBGroup.__init__():
    def __init__(self, name: str) -> None:
        """Initilize an empty PCBGroup."""
        # Load values into *pcb_group* (i.e. *self*):
        # pcb_group: PCBGroup = self
        self.name: str = name
        self.pads_groups: List[PadsGroup] = []
        self.scad3ds: List[Scad3D] = []


# PCBModule:
class PCBModule:
    """Represents a module in `.kicad_pcb` file."""

    # PCBModule.__init__():
    def __init__(self, file_path: Path, line_number: int,
                 reference_name: str, preceeding_lines: List[str],
                 module_lines: List[str], edit_timestamp: int, create_timestamp: int,
                 path_id: int, module_prefix: str) -> None:
        """Initialize a Module."""
        # Stuff values into *module* (i.e *self*):
        # module: Module = self
        self.create_timestamp: int = create_timestamp
        self.edit_timestamp: int = edit_timestamp
        self.file_path: Path = file_path
        self.line_number: int = line_number
        self.module_lines: List[str] = module_lines
        self.module_prefix: str = module_prefix
        self.path_id: int = path_id
        self.preceeding_lines: List[str] = preceeding_lines
        self.reference_name: str = reference_name

    # PCBModule.modules_extract():
    @staticmethod
    def modules_extract(pcb_lines: List[str],
                        file_path: Path) -> Tuple[Dict[str, "PCBModule"], List["PCBModule"]]:
        """Extract a table and list of Module's from some PCB lines."""
        # Create *modules_table* and *ordered_modules*:
        pcb_modules_table: Dict[str, PCBModule] = {}
        ordered_pcb_modules: List[PCBModule] = []

        # State machine to extract *Modules*'s:
        create_timestamp: int = 0
        edit_timestamp: int = 0
        in_module: bool = False
        index: int
        module_end: str = ""
        module_start_line_index: int = 0
        module_lines: List[str] = []
        module_prefix: str = ""
        path_id: int = 0
        path_pattern: str = "(path /"
        pcb_line: str
        pcb_module: PCBModule
        preceeding_lines: List[str] = []
        reference_name: str = ""
        reference_pattern: str = "(fp_text reference "
        for pcb_line_index, pcb_line in enumerate(pcb_lines):
            if in_module:
                module_lines.append(pcb_line)
                path_index: int = pcb_line.find(path_pattern)
                reference_index: int = pcb_line.find(reference_pattern)
                if reference_index >= 0:
                    # We found the reference:
                    assert reference_name == "", "Duplicate reference?"
                    reference_start_index: int = reference_index + len(reference_pattern)
                    reference_end_index: int = pcb_line[reference_start_index:].find(' ')
                    assert reference_end_index >= 0, "Bad reference"
                    reference_end_index += reference_start_index
                    reference_name = pcb_line[reference_start_index:reference_end_index]
                elif path_index >= 0:
                    # We found "...(path /XXXXXXXX)"
                    path_str: str = pcb_line[path_index+len(path_pattern):-1]
                    try:
                        path_id = int(path_str, 16)
                    except ValueError:
                        assert False, f"'{path_str}' in 'pcb_line' is not a hex number"
                elif pcb_line == module_end:
                    # We found the end of the module:
                    assert reference_name != "", "No reference in module"
                    pcb_module = PCBModule(file_path, module_start_line_index,
                                           reference_name, preceeding_lines, module_lines,
                                           edit_timestamp, create_timestamp, path_id, module_prefix)
                    pcb_modules_table[reference_name] = pcb_module
                    ordered_pcb_modules.append(pcb_module)

                    # Reset everything to search for another module:
                    in_module = False
                    module_lines = []
                    preceeding_lines = []
                    reference_name = ""
                    module_prefix = ""
                    module_end = ""
                    path_id = 0
                    edit_timestamp = 0
                    create_timestamp = 0
            else:
                # Search for "...(module..."):
                module_index: int = pcb_line.find("(module ")
                module_start_line_index = pcb_line_index
                if module_index >= 0:
                    # Found it; extract the timestamps:
                    assert not in_module, "Nested module?"
                    tedit_index: int = pcb_line.find("(tedit ")
                    if tedit_index >= 0:
                        edit_timestamp_text: str = pcb_line[tedit_index+7:tedit_index+7+8]
                        try:
                            edit_timestamp = int(edit_timestamp_text, 16)
                        except ValueError:
                            assert False, f"'{edit_timestamp_text}' from '{pcb_line}' is not hex"
                    tstamp_index: int = pcb_line.find("(tstamp ")
                    if tstamp_index >= 0:
                        create_timestamp_text: str = pcb_line[tstamp_index+8:tstamp_index+8+8]
                        try:
                            create_timestamp = int(create_timestamp_text, 16)
                        except ValueError:
                            assert False, f"'{create_timestamp_text}' from '{pcb_line}' is not hex"
                    module_prefix = pcb_line[:module_index]
                    module_end = module_prefix + ")"
                    in_module = True
                    module_lines = [pcb_line]
                else:
                    preceeding_lines.append(pcb_line)

        # Create one last *final_module* to contain the remaining lines and return everything:
        final_pcb_module: PCBModule = PCBModule(file_path, -1, "",
                                                preceeding_lines, module_lines, 0, 0, 0, "")
        ordered_pcb_modules.append(final_pcb_module)
        return pcb_modules_table, ordered_pcb_modules

    # PCBModule.generate_lines():
    @staticmethod
    def generate_lines(ordered_modules: List["PCBModule"]) -> List[str]:
        """Return the lines associated with a list of PCBModule's."""
        # Sweep through *ordered_modules*, create *reassembled_lines* and return them:
        reassembled_lines: List[str] = []
        for module in ordered_modules:
            reassembled_lines.extend(module.preceeding_lines)
            reassembled_lines.extend(module.module_lines)
        return reassembled_lines

    # PCBModule.cut_lines_strip():
    def cut_lines_strip(self) -> None:
        """Remove cut-lines from a PCBModule."""
        # Grab some values from *pcb_module* (i.e. *self*):
        pcb_module: PCBModule = self
        preceeding_lines: List[str] = pcb_module.preceeding_lines
        stripped_pcb_lines: List[str] = []
        pcb_line: str
        for pcb_line in preceeding_lines:
            if pcb_line.find("(gr_line ") < 0 or pcb_line.find("(layer Edge.Cuts)") < 0:
                stripped_pcb_lines.append(pcb_line)
        pcb_module.preceeding_lines = stripped_pcb_lines

    # PCBModule.cuts_insert():
    def cut_lines_insert(self, cuts: List[SimplePolygon], offset: P2D):
        """Insert some cut lines at the end of a ModulePCB."""
        # Grab some values from *pcb_module* (i.e. *self*):
        pcb_module: PCBModule = self
        reference_name: str = pcb_module.reference_name
        preceeding_lines: List[str] = pcb_module.preceeding_lines

        # Require that this be the last *PCBModule" (i.e. empty reference name):
        assert reference_name == ""

        # Since this is the last *PCBModule* *preceeding_lines* is actually *final_lines*:
        final_lines: List[str] = preceeding_lines

        # Remove the last two lines (probably a blank line followed by the closing parenthesis):
        last_line: str = final_lines.pop()
        next_to_last_line: str = final_lines.pop()

        # Append the cut lines from *cuts* to *final_lines*:
        offset_x = offset.x
        offset_y = offset.y
        width: float = 0.05
        layer: str = "Edge.Cuts"
        cut: SimplePolygon
        for cut in cuts:
            cut_size: int = len(cut)
            index: int
            for index in range(cut_size):
                point1: P2D = cut[index]
                point2: P2D = cut[(index + 1) % cut_size]
                cut_line: str = ("  (gr_line "
                                 f"(start {KicadPCB.number(offset_x + point1.x, 2)} "
                                 f"{KicadPCB.number(offset_y - point1.y, 2)}) "
                                 f"(end {KicadPCB.number(offset_x + point2.x, 2)} "
                                 f"{KicadPCB.number(offset_y - point2.y, 2)}) "
                                 f"(layer {layer}) "
                                 f"(width {KicadPCB.number(width, 2)}))")
                final_lines.append(cut_line)

        # Restore the final two lines:
        final_lines.append(next_to_last_line)
        final_lines.append(last_line)

    # PCBModule.reference_position_update():
    def reference_position_update(self, reference_position: P2D, pcb_origin: P2D,
                                  tracing: str = "") -> bool:
        """Update the reference position in a PCBModule."""
        # Unpack some values from *PCBModule* (i.e *self*):
        pcb_module: PCBModule = self
        file_path: Path = pcb_module.file_path
        line_number: int = pcb_module.line_number
        # create_timestamp: int = pcb_module.create_timestamp
        # edit_timestamp: int = pcb_module.edit_timestamp
        module_lines: List[str] = pcb_module.module_lines
        # module_prefix: str = pcb_module.module_prefix
        # path_id: int = pcb_module.path_id
        # preceeding_lines: List[str] = pcb_module.preceeding_lines
        reference_name: str = pcb_module.reference_name

        # Perform an requested *tracing*:
        if tracing:
            print(f"{tracing}=>reference_position_update(*, "
                  f"'{reference_name}', {reference_position})")

        # Find *at_line* and split out the locations [x, y, optional_rotation]:
        at_line: str = module_lines[1]
        at_pattern: str = "(at "
        at_index: int = at_line.find(at_pattern)
        assert at_index >= 0, (f"Pattern '{at_pattern} ' found in '{at_line}' "
                               f"{file_path}:{line_number}")
        at_end_index = at_index + len(at_pattern)
        close_index: int = at_line.find(")")
        assert close_index >= 0, "No closing parenthesis ')'."
        # at_xyz: str = at_line[at_end_index:close_index]
        locations: List[str] = at_line[at_end_index:close_index].split(' ')
        assert 2 <= len(locations) <= 3, f"Wrong number of values. {file_path}:{line_number}"

        # Update the X/Y locations:
        locations[0] = KicadPCB.number(pcb_origin.x + reference_position.x, 4)
        locations[1] = KicadPCB.number(pcb_origin.y - reference_position.y, 4)

        # Create the *new_at_line*:
        at_front: str = at_line[:at_end_index]
        at_middle: str = ' '.join(locations)
        at_end: str = at_line[close_index:]
        new_at_line = at_front + at_middle + at_end
        changed: bool = at_line != new_at_line
        module_lines[1] = new_at_line
        # print(f"{file_path}:{line_number}:")
        # print(f"    '{at_line}' => '{new_at_line}' '{at_xyz}'")
        # print(f"    '{at_front}' {locations} '{at_middle}' '{at_end}'")

        # Wrap up any requested *tracing*
        if tracing:
            print(f"{tracing}=>reference_position_update(*, "
                  f"'{reference_name}', {reference_position})")
        return changed


# Encoder:
class Encoder:
    """Represents a motor encoder board."""

    # Encoder.__init__():
    def __init__(self, scad_program: ScadProgram, base_dxf: BaseDXF,
                 connectors: Connectors, pcb_origin: P2D, tracing: str = "") -> None:
        """Initialize the Encoder and append to ScadProgram."""
        # Perform any requested *tracing*:
        # next_tracing: str = tracing + " " if tracing else ""
        if tracing:
            print(f"{tracing}=>Encoder.__init__(...)")

        # Grab some X/Y/Z coordinates from *base_dxf*:
        motor_casing_east_x: float = base_dxf.x_locate(-5.253299)
        motor_casing_north_y: float = base_dxf.y_locate(3.382512)
        north_electrical_north_y: float = base_dxf.y_locate(3.208303)
        north_electrical_south_y: float = base_dxf.y_locate(3.188610)
        motor_shaft_north_y: float = base_dxf.y_locate(2.967165)
        motor_shaft_south_y: float = base_dxf.y_locate(2.908110)
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
        motor_shaft_diameter: float = (motor_shaft_north_y + motor_shaft_south_y) / 2.0
        motor_shaft_z: float = motor_casing_bottom_z + motor_casing_dz / 2.0

        # Some random constants:
        degrees90: float = pi / 2.0
        origin2d: P2D = P2D(0.0, 0.0)

        # The PCB is designed in a flat orientation with the motor shaft in the center aligned
        # with the Z-axis and then rotated on end and translated into position.  This tends to
        # swap X and Z coordinates.  Thus the constants below are *VERY* confusing:
        pcb_west_x: float = -9.0  # Trail and error
        pcb_connector_x: float = -motor_casing_dz / 2.0 - 3.0   # Note the X/Z coordinate swap
        pcb_east_x: float = motor_casing_dz / 2.0
        pcb_header_dx: float = 6.5  # Trial and error
        pcb_corner_x: float = -2.5  # Trial and error
        pcb_dy_extra: float = 7.0 * 2.54
        pcb_dz: float = 1.6  # mm
        pcb_north_y: float = (motor_casing_dy + pcb_dy_extra) / 2.0
        pcb_north_corner_y: float = motor_casing_dy / 2.0
        pcb_south_corner_y: float = -pcb_north_corner_y
        pcb_south_y: float = -pcb_north_y
        pcb_shaft_diameter_extra: float = 3.0
        pcb_shaft_hole_diameter: float = motor_shaft_diameter + pcb_shaft_diameter_extra
        pcb_slot_extra: float = 0.400
        pcb_slot_dx: float = electrical_dz + pcb_slot_extra
        pcb_slot_dy: float = electrical_dy + pcb_slot_extra
        pcb_north_slot_center: P2D = P2D(0.0, north_electrical_y)
        pcb_south_slot_center: P2D = P2D(0.0, south_electrical_y)
        pcb_pad_extra: float = 0.75
        pcb_pad_dx: float = pcb_slot_dx + pcb_pad_extra
        pcb_pad_dy: float = pcb_slot_dy + pcb_pad_extra
        pcb_drill_diameter: float = min(pcb_slot_dx, pcb_slot_dy)

        # Create the *encoder_exterior* to allow enough space for the *MasterBoard* to slide up
        # to approximately the motor shaft Z:
        #
        #    A---H
        #    |   |
        #    |   |
        #    |   G--F
        #    |      |
        #    |    O |  O = shaft hole at origin
        #    |      |
        #    |   D--E
        #    |   |
        #    |   |
        #    B---C
        #
        encoder_exterior: SimplePolygon = SimplePolygon(
            "Encoder PCB Exterior Polygon", [], lock=False)
        corner_radius: float = 1.5
        encoder_exterior.corner_arc_append(
            P2D(pcb_west_x, pcb_north_y), corner_radius, "ES")           # A
        encoder_exterior.corner_arc_append(
            P2D(pcb_west_x, pcb_south_y), corner_radius, "NE")           # B
        encoder_exterior.corner_arc_append(
            P2D(pcb_corner_x, pcb_south_y), corner_radius, "WN")         # C
        encoder_exterior.corner_arc_append(
            P2D(pcb_corner_x, pcb_south_corner_y), corner_radius, "SE")  # D
        encoder_exterior.corner_arc_append(
            P2D(pcb_east_x, pcb_south_corner_y), corner_radius, "WN")    # E
        encoder_exterior.corner_arc_append(
            P2D(pcb_east_x, pcb_north_corner_y), corner_radius, "SW")    # F
        encoder_exterior.corner_arc_append(
            P2D(pcb_corner_x, pcb_north_corner_y), corner_radius, "EN")  # G
        encoder_exterior.corner_arc_append(
            P2D(pcb_corner_x, pcb_north_y), corner_radius, "SW")         # H
        encoder_exterior.lock()

        # Create *north_header* and *south_header* for (Digikey: 2057-PH1RB-03-UA-ND (Adam Tech))
        # and make sure the header pin holes are appended to *pcb_polygon*:
        header_offset: float = 1.5 * 2.54  # Trail and error
        north_header_center2d: P2D = P2D(pcb_connector_x + 0.5 * 2.54, pcb_north_y - header_offset)
        south_header_center2d: P2D = P2D(pcb_connector_x + 0.5 * 2.54, pcb_south_y + header_offset)

        # Create *encoder_pcb_chunk* and *master_pcb_chunk*" (for master board):
        m1x3ra_pcb_chunk: PCBChunk = connectors.m1x3ra.pcb_chunk
        # m1x3ra_pcb_chunk.pads_show("m1x3ra_pcb:")
        # print(f"north_header_center2d:{north_header_center2d}")
        m1x3ra_pcb_chunk_north: PCBChunk = (m1x3ra_pcb_chunk.
                                            sides_swap().
                                            pads_rebase(3).
                                            pads_y_mirror().
                                            scads_y_flip().
                                            reposition(origin2d, -degrees90, north_header_center2d))
        # m1x3ra_pcb_chunk_north.pads_show("m1x3ra_chunks_north:")
        m1x3ra_pcb_chunk_south: PCBChunk = (m1x3ra_pcb_chunk.
                                            sides_swap().
                                            pads_y_mirror().
                                            scads_y_flip().
                                            reposition(origin2d, -degrees90, south_header_center2d))

        # Create *motor_slots_pcb_chunk* (put "8" before "7" because the footprint looks better):
        north_motor_pad: Pad = Pad("8", pcb_pad_dx, pcb_pad_dy,
                                   pcb_drill_diameter, pcb_north_slot_center)
        south_motor_pad: Pad = Pad("7", pcb_pad_dx,
                                   pcb_pad_dy, pcb_drill_diameter, pcb_south_slot_center)
        motor_slots_pcb_chunk: PCBChunk = PCBChunk("motor_slots",
                                                   [north_motor_pad, south_motor_pad], [])

        # Create the *shaft_hole_pcb_chunk*:
        shaft_hole_pad: Pad = Pad("Encoder Shaft Hole", 0.0, 0.0, pcb_shaft_hole_diameter, origin2d)
        shaft_hole_pcb_chunk: PCBChunk = PCBChunk("Encoder Shaft Hole", [shaft_hole_pad], [])

        # Figure out *encoder_pcb_directory* and *encoder_pcb_pretty_directory*:
        assert "HR2_DIRECTORY" in os.environ, "HR2_DIRECTORY environement variable not set"
        hr2_directory: Path = Path(os.environ["HR2_DIRECTORY"])
        encoder_pcb_directory: Path = hr2_directory / "electrical" / "encoder" / "rev_a"
        encoder_pcb_pretty_directory: Path = encoder_pcb_directory / "pretty"
        encoder_pcb_path: Path = encoder_pcb_directory / "encoder.kicad_pcb"

        # Create *encoder_pcb_chunk* that is used on the *encoder_pcb* (see below):
        encoder_pcb_chunk: PCBChunk = PCBChunk.join("Encoder", [
            m1x3ra_pcb_chunk_north,
            m1x3ra_pcb_chunk_south,
            motor_slots_pcb_chunk,
            shaft_hole_pcb_chunk,
        ])
        encoder_pcb_chunk.footprint_generate("HR2", encoder_pcb_pretty_directory)
        encoder_module: Module3D = encoder_pcb_chunk.pcb_update(
            scad_program, pcb_origin, pcb_dz, encoder_exterior, "Purple", encoder_pcb_path, [])
        encoder_use_module: UseModule3D = encoder_module.use_module_get()

        # Create the *encoder_pcb_mate_chunk* next.  This has the 2 F1x3 connectors and
        # the actual encoder in a vertical orientation in it.  Start with *f1x3_pcb_chunk*:
        f1x3: RectangularConnector = connectors.f1x3
        f1x3_pcb_chunk: PCBChunk = f1x3.pcb_chunk

        # Rotate the F1x3 90 degrees and make two copies along the Y axis for the encoder PCB
        # to plug into:
        north_translate2d: P2D = P2D(0.0, pcb_north_y - header_offset)
        south_translate2d: P2D = P2D(0.0, pcb_south_y + header_offset)
        north_f1x3_pcb_chunk: PCBChunk = f1x3_pcb_chunk.reposition(
            origin2d, degrees90, north_translate2d).pads_rebase(3)
        south_f1x3_pcb_chunk: PCBChunk = f1x3_pcb_chunk.reposition(
            origin2d, degrees90, south_translate2d)

        # Now generate *repositioned_use_module* which is vertical with connectors centered
        # along the Y axis at a Z altitude slightly above the *f1x3* connectors:
        pin_exposed: float = 0.2  # mm
        y_axis: P3D = P3D(0.0, 1.0, 0.0)
        reposition_center3d: P3D = P3D(pcb_corner_x + pin_exposed, 0.0, -0.5 * 2.54)
        translate3d: P3D = P3D(0.0, 0.0, f1x3.insulation_height)
        repositioned_use_module: Scad3D = encoder_use_module.reposition(
            "Vertical Repositioned Encoder", reposition_center3d, y_axis, degrees90, translate3d)
        repositioned_encoder_pcb_chunk: PCBChunk = PCBChunk("Repositioned Encoder", [],
                                                            [repositioned_use_module])
        encoder_pcb_chunk_mate: PCBChunk = PCBChunk.join(
            "ENCODER_MATE", [north_f1x3_pcb_chunk, south_f1x3_pcb_chunk,
                             repositioned_encoder_pcb_chunk])
        encoder_pcb_chunk_mate.footprint_generate("HR2", encoder_pcb_pretty_directory)

        # encoder_module = encoder_pcb.scad_program_append(scad_program, "Purple")

        # *translate* is the point on the east motor where the shaft comes out:
        # No need to worry about the west motor because the entire east wheel assembly is
        # is rotated and placed on the west side of the robot.
        translate: P3D = P3D(motor_casing_east_x, 0.0, motor_shaft_z)

        # Digikey: 2057-PH1RB-03-UA-ND (Adam Tech):

        # *offset* is the point the east side of the master board to place the mating connectors
        # footprint.  This is complicated because the center of the footprint is the motor shaft.
        # So an additional *adjustment* and *tweak* is needed to get *offset* correct.
        # *tweak* is determined by trial and error:
        tweak: float = 0.290  # mm
        adjustment: float = pcb_header_dx
        offset: P2D = P2D(motor_casing_east_x + adjustment - tweak, 0.0)

        # *bottom_z* is the lower insulation lip on the encoder connectors, measured in
        # the robot frame (i.e. origin at the center of the base.)
        male_inuslation_bottom_z: float = -pcb_corner_x  # Usual X/Z coordinate swap (see above.)
        mating_female_insulation_length: float = 7.00
        bottom_z: float = motor_shaft_z + male_inuslation_bottom_z - mating_female_insulation_length

        # Stuff some values into *encoder* (i.e. *self*):
        # encoder: Encoder = self
        self.bottom_z: float = bottom_z
        self.module: Module3D = encoder_module
        self.offset: P2D = offset
        # self.pcb: PCB = encoder_pcb
        self.translate: P3D = translate
        self.encoder_pcb_chunk: PCBChunk = encoder_pcb_chunk
        self.encoder_mate_pcb_chunk: PCBChunk = encoder_pcb_chunk_mate

        # Wrap up any requested *tracing*:
        if tracing:
            print(f"{tracing}<=Encoder.__init__(...)")


# Grove:
class Grove:
    """Represents Seeedstudio Grove module."""

    def __init__(self, scad_program: ScadProgram, dx_mm: int, dy_mm: int, color: str) -> None:
        """Initialize a Seeedstudo Grove module."""
        # Verify argument types:
        assert dx_mm % 20 == 0, f"dx_mm={dx_mm} must be a multiple of 20"
        assert dy_mm % 20 == 0, f"dx_mm={dy_mm} must be a multiple of 20"
        assert color != ""

        # Define some constants:
        dx: float = float(dx_mm)
        dy: float = float(dy_mm)
        dz: float = 1.6  # mm
        pitch: float = 20.0  # mm
        corner_radius: float = 0.80
        nub_radius: float = 2.00
        hole_diameter: float = 2.20  # For M2 hardware
        columns: int = int(dx_mm / 20)
        rows: int = int(dy_mm / 20)
        degrees0: float = 0.0
        degrees180: float = pi
        degrees90: float = degrees180 / 2.0
        degrees270: float = degrees180 + degrees90
        degrees10: float = degrees90 / 9.0
        east_x: float = dx / 2.0
        west_x: float = -east_x
        north_y: float = dy / 2.0
        south_y: float = -north_y
        spacer_height: float = 2.0  # mm

        # Create a Spacer:
        full_name: str = f"{color} {dx_mm}x{dy_mm} Grove"
        grove_spacer: Spacer = Spacer(scad_program, f"{full_name} Spacer", spacer_height, "M2")
        spacer_use_module: UseModule3D = grove_spacer.module.use_module_get()

        # Start the *grove_exterior* by drawing the 4 corners along with the various nubs,
        # and indentations:
        grove_exterior: SimplePolygon = SimplePolygon(f"{full_name} Exterior", [])

        # Start with the NW rounded corner:
        corner_center: P2D = P2D(west_x + corner_radius, north_y - corner_radius)
        grove_exterior.arc_append(corner_center, corner_radius, degrees180, degrees90, degrees10)

        # Draw the North nubs and holes:
        holes: List[SimplePolygon] = []
        pads: List[Pad] = []
        pad: Pad
        spacer_scads: List[Scad3D] = []
        hole_center: P2D
        translated_spacer: Scad3D
        column: int
        for column in range(columns):
            hole_center = P2D(west_x + (float(column) + 0.5) * pitch, north_y)
            grove_exterior.arc_append(hole_center, nub_radius, degrees180, degrees0, degrees10)
            holes.append(Circle(f"N[{column}] Hole", hole_diameter, 16, center=hole_center))
            translated_spacer = Translate3D(
                f"N[{column}] Spacer", spacer_use_module, P3D(hole_center.x, hole_center.y, 0.0))
            spacer_scads.append(translated_spacer)
            pad = Pad(f"N[{column}] {full_name}", 0.0, 0.0, hole_diameter, hole_center)
            pads.append(pad)

        # End with the NE rounded corner:
        corner_center = P2D(east_x - corner_radius, north_y - corner_radius)
        grove_exterior.arc_append(corner_center, corner_radius, degrees90, degrees0, degrees10)

        # Draw the East nub indents:
        row: int
        for row in range(rows):
            hole_center = P2D(east_x, north_y - (float(row) + 0.5) * pitch)
            grove_exterior.arc_append(hole_center, nub_radius, degrees90, degrees270, degrees10)

        # Draw the SE corner:
        corner_center = P2D(east_x - corner_radius, south_y + corner_radius)
        grove_exterior.arc_append(corner_center, corner_radius, degrees0, -degrees90, degrees10)

        # Draw the South nubs, holes, and pads:
        for column in range(columns):
            hole_center = P2D(east_x - (float(column) + 0.5) * pitch, south_y)
            grove_exterior.arc_append(hole_center, nub_radius, degrees0, -degrees180, degrees10)
            holes.append(Circle(f"S[{column}] Hole", hole_diameter, 16, center=hole_center))
            translated_spacer = Translate3D(
                f"S[{column}] Spacer", spacer_use_module, P3D(hole_center.x, hole_center.y, 0.0))
            spacer_scads.append(translated_spacer)
            pad = Pad(f"S[{column}] {full_name}", 0.0, 0.0, hole_diameter, hole_center)
            pads.append(pad)

        # Draw the SW corner:
        corner_center = P2D(west_x + corner_radius, south_y + corner_radius)
        grove_exterior.arc_append(corner_center, corner_radius, -degrees90, -degrees180, degrees10)

        # Draw the West nub indents:
        for row in range(rows):
            hole_center = P2D(west_x, south_y + (float(row) + 0.5) * pitch)
            grove_exterior.arc_append(hole_center, nub_radius, -degrees90, degrees90, degrees10)
        grove_exterior.lock()

        # Create the *grove_connector*:
        connector_dx: float = 5.0  # mm
        connector_dy: float = 10.0  # mm
        connector_dz: float = 7.7  # mm
        connector_center_x: float = west_x + 1.4 + connector_dx / 2.0
        connector_center_y: float = 0.0
        connector_center_z: float = spacer_height + dz + connector_dz / 2.0
        connector_center: P3D = P3D(connector_center_x, connector_center_y, connector_center_z)
        connector_cube: Cube = Cube(
            f"{full_name} Cube", connector_dx, connector_dy, connector_dz, center=connector_center)
        colored_connector: Scad3D = Color(f"{full_name} Connector", connector_cube, "Beige")

        # Create the extruded PCB:
        grove_polygon: Polygon = Polygon(f"{dx_mm}x{dy_mm} Polygon", [grove_exterior] + holes)
        extruded_grove: LinearExtrude = LinearExtrude("Extruded Grove", grove_polygon, dz)
        translated_grove: Translate3D = Translate3D(
            "Translated Grove", extruded_grove, P3D(0.0, 0.0, spacer_height))
        colored_grove: Scad3D = Color(f"{color} Colored Grove", translated_grove, color)

        # Now create the *grove_module*:
        grove_module: Module3D = Module3D(f"{full_name} Module",
                                          [colored_grove, colored_connector] + spacer_scads)
        scad_program.append(grove_module)
        scad_program.if3d.name_match_append(full_name, grove_module, [full_name])

        # Create the *grove_pcb_chunk*:
        footprint_name: str = f"GROVE{int(dx)}x{int(dy)}"
        grove_pcb_chunk: PCBChunk = PCBChunk(
            footprint_name, pads, [grove_module.use_module_get()], front_artworks=[grove_exterior])
        assert "HR2_DIRECTORY" in os.environ, "HR2_DIRECTORY environement variable not set"
        hr2_directory: Path = Path(os.environ["HR2_DIRECTORY"])
        hr2_pretty_directory: Path = (hr2_directory /
                                      "electrical" / "master_board" / "rev_a" / "pretty")
        grove_pcb_chunk.footprint_generate("HR2", hr2_pretty_directory)

        # Stuff some values into *grove* (i.e. *self*):
        # grove: Grove = self
        self.module: Module3D = grove_module
        self.pcb_chunk: PCBChunk = grove_pcb_chunk


# HCSR04:
class HCSR04:
    """Represents an HC-SR04 sonar module."""

    # HCSR04.__init__():
    def __init__(self, scad_program: ScadProgram, connectors: Connectors, pcb_origin: P2D) -> None:
        """Initialize an HCSR04."""
        # Define some constants:
        #   https://www.makerguides.com/wp-content/uploads/2019/02/HC-SR04-Dimensions-964x1024.jpg
        mount_hole_pitch_dx: float = 41.00  # mm
        mount_hole_pitch_dy: float = 16.50  # mm
        mount_hole_diameter: float = 2.00   # mm
        mount_hole_ne: P2D = P2D(mount_hole_pitch_dx / 2.0, mount_hole_pitch_dy / 2.0)
        mount_hole_sw: P2D = -mount_hole_ne
        pcb_dy: float = 20.00  # mm
        pcb_dx: float = 45.00  # mm
        pcb_dz: float = 1.60   # mm
        transducer_pitch: float = 26.00  # mm
        transducer_diameter: float = 16.00  # mm
        transducer_dz: float = 12.00  # mm
        transducer_left_x: float = transducer_pitch / 2.0
        transducer_right_x: float = -transducer_left_x
        center_y: float = 0.0  # *center_y* is Y center line value:

        # Do all of the sonar *PCBChunk* stuff now:

        # Compute the tie down hole *Pad*'s, which are mechanical only (i.e. no copper pads.):
        # The "miniture" nylon ties are 2.5mm wide.  The hole should be a little larger:
        # There are two tie down hole locations.  For the normal (F1X4) and low profile (F1X4LP)
        # locations and the a high (F1X4H) locations.  We use *lp* and *h* to differentiate,
        # *lp* is also used for the normal (F1X4) case.
        tie_down_hole_diameter: float = 2.5 + 0.05  # mm
        tie_down_hole_radius: float = tie_down_hole_diameter / 2.0
        tie_down_hole_x_extra: float = 0.2  # mm
        tie_down_hole_lp_e_x: float = 2.2 * 2.54 + tie_down_hole_radius + tie_down_hole_x_extra
        tie_down_hole_lp_w_x: float = -tie_down_hole_lp_e_x
        tie_down_hole_h_e_x: float = 1.25 * 2.54 + tie_down_hole_radius + tie_down_hole_x_extra
        tie_down_hole_h_w_x: float = -tie_down_hole_h_e_x
        tie_down_hole_lp_y: float = -0.5 * 2.54  # mm
        tie_down_hole_h_y: float = -1.15 * 2.54  # mm
        tie_down_hole_center_lp_e: P2D = P2D(tie_down_hole_lp_e_x, tie_down_hole_lp_y)
        tie_down_hole_center_lp_w: P2D = P2D(tie_down_hole_lp_w_x, tie_down_hole_lp_y)
        tie_down_hole_center_h_e: P2D = P2D(tie_down_hole_h_e_x, tie_down_hole_h_y)
        tie_down_hole_center_h_w: P2D = P2D(tie_down_hole_h_w_x, tie_down_hole_h_y)
        tie_down_hole_pad_lp_e: Pad = Pad("HCSR04 East Tie Down Hole (Normal/LP)",
                                          0.0, 0.0, tie_down_hole_diameter,
                                          tie_down_hole_center_lp_e)
        tie_down_hole_pad_lp_w: Pad = Pad("HCSR04 West Tie Down Hole (Normal/LP)",
                                          0.0, 0.0, tie_down_hole_diameter,
                                          tie_down_hole_center_lp_w)
        tie_down_hole_pad_h_e: Pad = Pad("HCSR04 East Tie Down Hole (High)",
                                         0.0, 0.0, tie_down_hole_diameter,
                                         tie_down_hole_center_h_e)
        tie_down_hole_pad_h_w: Pad = Pad("HCSR04 West Tie Down Hole (High)",
                                         0.0, 0.0, tie_down_hole_diameter,
                                         tie_down_hole_center_h_w)
        tie_down_hole_lp_pads: List[Pad] = [tie_down_hole_pad_lp_e, tie_down_hole_pad_lp_w]
        tie_down_hole_h_pads: List[Pad] = [tie_down_hole_pad_h_e, tie_down_hole_pad_h_w]
        tie_down_hole_pads_lp_pcb_chunk: PCBChunk = PCBChunk("HCSR4 Tie Down Holes (Normal/LP)",
                                                             tie_down_hole_lp_pads, [])
        tie_down_hole_pads_h_pcb_chunk: PCBChunk = PCBChunk("HCSR4 Tie Down Holes (High)",
                                                            tie_down_hole_h_pads, [])

        # Construct the 3 female connector *PCBChunk*s:
        y_mirrored_f1x4_pcb_chunk: PCBChunk = (
            connectors.f1x4.pcb_chunk.pads_y_mirror().artworks_y_mirror())
        y_mirrored_f1x4h_pcb_chunk: PCBChunk = (
            connectors.f1x4h.pcb_chunk.pads_y_mirror().artworks_y_mirror())
        y_mirrored_f1x4lp_pcb_chunk: PCBChunk = (
            connectors.f1x4lp.pcb_chunk.pads_y_mirror().artworks_y_mirror())
        f1x4_mate: PCBChunk = PCBChunk.join(
            "HCSR04_F1x4",
            [y_mirrored_f1x4_pcb_chunk, tie_down_hole_pads_lp_pcb_chunk], value="HCSR04;F1x4")
        f1x4h_mate: PCBChunk = PCBChunk.join(
            "HCSR04_F1x4H",
            [y_mirrored_f1x4h_pcb_chunk, tie_down_hole_pads_h_pcb_chunk], value="HCSR04;F1x4H")
        f1x4lp_mate: PCBChunk = PCBChunk.join(
            "HCSR04_F1x4LP",
            [y_mirrored_f1x4lp_pcb_chunk, tie_down_hole_pads_lp_pcb_chunk], value="HCSR04;F1x4LP")

        # Generate the 3 female mate connector footprints:
        assert "HR2_DIRECTORY" in os.environ, "HR2_DIRECTORY environement variable not set"
        hr2_directory: Path = Path(os.environ["HR2_DIRECTORY"])
        hr2_pretty_directory: Path = (hr2_directory /
                                      "electrical" / "master_board" / "rev_a" / "pretty")
        f1x4_mate.footprint_generate("HR2", hr2_pretty_directory)
        f1x4h_mate.footprint_generate("HR2", hr2_pretty_directory)
        f1x4lp_mate.footprint_generate("HR2", hr2_pretty_directory)

        # Construct the *hcsr04_cpcb_chunk*:
        # Create the *sonar_pcb*:
        sonar_exterior: Square = Square("HCSR04 PCB", pcb_dx, pcb_dy, center=P2D(0.0, center_y))
        # sonar_pcb: PCB = PCB("HCSR04", scad_program, pcb_dz, sonar_exterior)

        # Create the *colored_transducer* and position one on the left and the other on the right:
        transducer_start: P3D = P3D(0.0, center_y, 0.0)
        transducer_end: P3D = P3D(0.0, center_y, transducer_dz)
        transducer_cylinder: Cylinder = Cylinder("Sonar Transducer", transducer_diameter,
                                                 transducer_start, transducer_end, 64)
        colored_transducer: Color = Color("Colored Transducer", transducer_cylinder, "Silver")

        # Create the both the *transducers_pcb_chunk*:
        origin3d: P3D = P3D(0.0, 0.0, 0.0)
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        transducer_left_position: P3D = P3D(transducer_left_x, pcb_dz)  # Centered on bottom
        transducer_right_position: P3D = P3D(transducer_right_x, pcb_dz)  # Centered on bottom
        left_transducer: Scad3D = colored_transducer.reposition(
            "HCSR04 Left Transducer", origin3d, z_axis, 0.0, transducer_left_position)
        right_transducer: Scad3D = colored_transducer.reposition(
            "HCSR04 Right Transducer", origin3d, z_axis, 0.0, transducer_right_position)
        transducers_pcb_chunk: PCBChunk = PCBChunk(
            "HCSR04 Transducers", [], [left_transducer, right_transducer])

        # Create the *mount_holes_pcb_chunk*:
        ne_pad: Pad = Pad("NE Sonar Mount", 0.0, 0.0, mount_hole_diameter, mount_hole_ne)
        sw_pad: Pad = Pad("SW Sonar Mount", 0.0, 0.0, mount_hole_diameter, mount_hole_sw)
        mount_holes_pcb_chunk: PCBChunk = PCBChunk("HCSR04 Mount Holes", [ne_pad, sw_pad], [])

        # The standard pin number is with pin 1 on the left when viewed from the top.
        # The right-angle connector is installed on the back side so we swap sides
        # without swapping pad numbering:
        m1x4ra_connector: RectangularConnector = connectors.m1x4ra
        m1x4ra_offset: float = (
             m1x4ra_connector.pcb_pin_height
             - m1x4ra_connector.pin_dx_dy / 2.0
             + m1x4ra_connector.insulation_height)
        m1x4ra_pcb_chunk: PCBChunk = m1x4ra_connector.pcb_chunk
        origin2d = P2D(0.0, 0.0)
        degrees180: float = pi
        repositioned_m1x4: PCBChunk = (
            m1x4ra_pcb_chunk.scads_x_flip().sides_swap().  # Want pin 1 on left, no pads mirroring!
            reposition(origin2d, 0.0, P2D(0.0, - pcb_dy / 2.0 + m1x4ra_offset))
        )
        m1x4ra_offset = m1x4ra_offset  # Delete this line.
        centered_hcsr04: PCBChunk = PCBChunk.join("Sonar Centered", [
            mount_holes_pcb_chunk,
            transducers_pcb_chunk,
            repositioned_m1x4,
        ])
        edge_center: P2D = P2D(0.0, pcb_dy / 2.0)
        hcsr04_pcb_chunk: PCBChunk = centered_hcsr04.reposition(origin2d, 0.0, edge_center)
        edge_sonar_exterior: SimplePolygon = sonar_exterior.reposition(origin2d, 0.0, edge_center)

        # Generate the *hrsr04_module* but there is no associated `.kicad_pcb`):
        hcsr04_module: Module3D = hcsr04_pcb_chunk.pcb_update(
            scad_program, pcb_origin, pcb_dz, edge_sonar_exterior, "LightGreen", None, [])

        # Now create a *vertical_sonar_module* which is centered over the connector and points
        # along +X axis:
        sonar_use_module: UseModule3D = hcsr04_module.use_module_get()
        # Put center into middle of right angle connector:
        recentered_sonar: Translate3D = Translate3D(
            "Recentered HCSR04", sonar_use_module, P3D(0.0, 0.0, 0.5 * 2.54))
        degrees90: float = degrees180 / 2.0

        # Make sensors sonar connector pins point along the +X axis:

        # Make the *upright_sonar*, but the sensor are pointing down the -X axis:
        x_axis: P3D = P3D(1.0, 0.0, 0.0)
        upright_sonar: Rotate3D = Rotate3D(
            "Vertical Rotated HCS404", recentered_sonar, degrees90, x_axis)
        z_rotated_sonar: Rotate3D = Rotate3D(
            "+X Aligned HCSR04", upright_sonar, 0.0, z_axis)
        vertical_sonar_module: Module3D = Module3D("Veritcal HCSR04", [z_rotated_sonar])
        scad_program.append(vertical_sonar_module)
        scad_program.if3d.name_match_append(
            "sonar_vertical", vertical_sonar_module, ["Vertical Sonar"])

        # Stuff some values into *hrcsr04* (i.e. *self*):
        # hcsr04: HCSR04 = self
        self.module: Module3D = hcsr04_module
        # self.pcb: PCB = sonar_pcb
        self.pcb_dx: float = pcb_dx
        self.pcb_dy: float = pcb_dy
        self.pcb_dz: float = pcb_dz
        self.right_angle_pin_dy: float = 3.00 - 0.127  # Pin tips to PCB edge; caliper measurement
        self.f1x4_mate_pcb_chunk: PCBChunk = f1x4_mate
        self.f1x4h_mate_pcb_chunk: PCBChunk = f1x4h_mate
        self.f1x4lp_mate_pcb_chunk: PCBChunk = f1x4lp_mate
        self.vertical_sonar_module: Module3D = vertical_sonar_module


# HeatSink:
class HeatSink:
    """Represents a finned heat sink."""

    def __init__(self, scad_program: ScadProgram, name: str, dx: float, dy: float, dz: float,
                 base_dz: float, fin_dx: float, fin_count: int, rotate90: bool, color: str,
                 tracing: str = "") -> None:
        """Initialize a HeatSink 3D model."""
        # Perform any requested *tracing*:
        if tracing:
            print(f"{tracing}=>HeatSink(*, '{name}', {dx:.2f}, {dy:.2f}, {dz:.2f}, "
                  f"{base_dz:.2f} {fin_dx:.2f}, {fin_count}, '{color}')")

        # fin_spacing: float = float(fin_count - 1) * fin_dx
        fin_pitch: float = (dx - fin_dx) / float(fin_count - 1)
        start_x: float = -dx / 2.0 + fin_dx / 2.0
        # Note that outline is in X/Y plane, so Z is mapped to Y:
        top_y: float = dz
        middle_y: float = base_dz
        bottom_y: float = 0

        # Crude ASCII art of what is desired:
        #
        #     B---C   B---C   B---C   B---C   B---C
        #     |   |   |   |   |   |   |   |   |   |
        #     |   |   |   |   |   |   |   |   |   |
        #     |   |   |   |   |   |   |   |   |   |
        #     A   D---A   D---A   D---A   D---A   D
        #     |                                   |
        #     a-----------------O-----------------d =====> +X

        # Create the end *profile* as show immediately above:
        heat_sink_points: List[P2D] = []
        index: int
        x: float
        for index in range(fin_count):
            x = start_x + float(index) * fin_pitch
            x1: float = x - fin_dx / 2.0
            x2: float = x + fin_dx / 2.0
            start_y: float = bottom_y if index == 0 else middle_y  # a or A
            end_y: float = bottom_y if index == fin_count - 1 else middle_y  # d or D
            heat_sink_points.append(P2D(x1, start_y))  # A or a (pick "a" for index == first)
            heat_sink_points.append(P2D(x1, top_y))    # B
            heat_sink_points.append(P2D(x2, top_y))    # C
            heat_sink_points.append(P2D(x2, end_y))    # D or d (pick "d" for index == last)
        heat_sink_profile: SimplePolygon = SimplePolygon(f"{name} Outline",
                                                         heat_sink_points, lock=True)
        if tracing:
            point: P2D
            heat_sink_point_texts: List[str] = [f"{point}" for point in heat_sink_points]
            print(f"{tracing}heat_sink_points:{heat_sink_point_texts}")

        # Now make *extruded_heat_sink*:
        extruded_heat_sink: LinearExtrude = LinearExtrude(
            f"Extruded {name}", heat_sink_profile, dy, center=True)

        # Now rotate and translate to get a new *repositioned_heat_sink* that is origin centered:
        x_axis: P3D = P3D(1.0, 0.0, 0.0)
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        bottom_center: P3D = P3D(0.0, 0.0, dy / 2.0)
        origin3d: P3D = P3D(0.0, 0.0, 0.0)
        degrees90: float = pi / 2.0
        if tracing:
            print(f"origin3d:{origin3d} bottom_center:{bottom_center}")
        flat_heat_sink: Scad3D = extruded_heat_sink.reposition(
            "Flat {name}", origin3d, x_axis, degrees90, origin3d)
        rotated_heat_sink: Scad3D = (Rotate3D(f"Rotated {name}", flat_heat_sink, degrees90, z_axis)
                                     if rotate90 else flat_heat_sink)
        # repositioned_heat_sink: Scad3D = rotated_heat_sink.reposition(
        #      f"Repositioned {name}", origin3d, x_axis, degrees90, origin3d)
        colored_heat_sink: Color = Color(f"Colored {name} Heat Sink",
                                         rotated_heat_sink, color)
        heat_sink_module: Module3D = Module3D(f"{name} Module", [colored_heat_sink])
        scad_program.append(heat_sink_module)
        scad_program.if3d.name_match_append(name.lower(), heat_sink_module, [f"{name}"])

        # Load values into *heat_sink* (i.e. *self*):
        # heat_sink: HeatSink = self
        self.module: Module3D = heat_sink_module

        # Perform any requested *tracing*:
        if tracing:
            print(f"{tracing}<=HeatSink(*, '{name}', {dx:.2f}, {dy:.2f}, {dz:.2f}, "
                  f"{base_dz:.2f} {fin_dx:.2f}, {fin_count}, '{color}')")


# F11x4LP:
class F1x4LP:
    """Represents a Mill-Max F1x4LP low profile connector."""

    def __init__(self, scad_program: ScadProgram) -> None:
        """Initilialize the F1x4LP."""
        # Define some constants for the Mill Max 315-87-164-41-003101 (low profile 64-pins).
        # Digikey: 1212-1125-ND
        inch2mm: float = 25.4
        insulation_dx: float = 2.54
        insulation_dy: float = 2.54
        pin_diameter: float = 0.78
        landing_diameter: float = 2 * pin_diameter  # Estimate.
        drill_diameter: float = pin_diameter + .2
        pad_diameter: float = drill_diameter + 1.00  # Guess
        pin_pitch: float = 0.100 * inch2mm
        top_z: float = 2.41
        insulation_z: float = top_z - 1.9
        insulation_dz: float = abs(top_z - insulation_z)
        landing_z: float = 0.0
        landing_dz: float = top_z - landing_z
        pin_bottom_z: float = top_z - 0.155 * inch2mm
        bottom_z: float = landing_z - 2.42

        # First, create the *landing_tube* which contacts the top of the PCB and has an
        # opening that the pin fits into:
        landing_inner_circle: Circle = Circle("F1x4LP Landing Inner Circle", pin_diameter, 16)
        landing_outer_circle: Circle = Circle("F1x4LP Landing Outer Circle", landing_diameter, 16)
        landing_polygon: Polygon = Polygon("F1x4LP Landing Polygon",
                                           [landing_outer_circle, landing_inner_circle])
        landing_tube: LinearExtrude = LinearExtrude("F1x4LP Landing Tube",
                                                    landing_polygon, landing_dz)

        # Next, create the *pin_cylinder* which is the pin that pushes through PCB to the
        # other side.  It sticks out the bottom a little in order to be low profile:
        pin_cylinder: Cylinder = Cylinder("F1x4LP Pin Cylinder", pin_diameter,
                                          P3D(0.0, 0.0, bottom_z), P3D(0.0, 0.0, landing_z), 16)

        # Join the two *landing_tube* and *pin_cylinder* together into *colored_pin*:
        pin_union: Union3D = Union3D("F1x4LP Pin Union", [landing_tube, pin_cylinder])
        colored_pin: Color = Color("F1x4LP Colored Pin", pin_union, "Gold")

        # Create the *pin_module* and associated *pin_use_module*:
        pin_module: Module3D = Module3D("F1x4LP Pin Module", [colored_pin])
        scad_program.append(pin_module)
        pin_use_module: UseModule3D = pin_module.use_module3d

        # Now create the 1x4 insulation:
        insulation_square: Square = Square("F1x4LP Insulation Square",
                                           4 * insulation_dx, insulation_dy)
        insulation_polygon: Polygon = Polygon("F1x4LP Insulation Polygon",
                                              [insulation_square], lock=False)

        # Start place in the *translate_pin*'s into the *f1x4lp_union*:
        pads: List[Pad] = []
        pads_group: PadsGroup = PadsGroup()
        f1x4lp_union: Union3D = Union3D("F1x4LP Union", [], lock=False)
        index: int
        x_fraction: float
        for index, x_pitch_fraction in enumerate([-1.5, -0.5, 0.5, 1.5]):
            # Create a *pad* and stuff it into *pads_group*:
            x: float = x_pitch_fraction * pin_pitch
            pad: Pad = Pad(f"{index + 1}", pad_diameter, pad_diameter, drill_diameter, P2D(x, 0.0))
            pads_group.insert(pad)
            pads.append(pad)

            # Place each *translated_pin* into the *f1x4lp_union*:
            pin_position: P3D = P3D(x, 0.0, 0.0)
            translated_pin: Translate3D = Translate3D(f"F1x4LP Translated Pin {index}",
                                                      pin_use_module, pin_position)
            f1x4lp_union.append(translated_pin)

            # Add a *pin_hole* to the *insulation_polygon*:
            pin_hole: Circle = Circle(f"F1x4LP Pin Circle {index}",
                                      landing_diameter, 16, P2D(x, 0.0))
            insulation_polygon.append(pin_hole)
        insulation_polygon.lock()

        # Now extrude and color the *colored_insulation*:
        insulation_polygon.lock()
        extruded_insulation: LinearExtrude = LinearExtrude("F1x4LP Extruded Insulation",
                                                           insulation_polygon, insulation_dz)
        translated_insulation: Translate3D = Translate3D("`F1x4LP Translated Insulation",
                                                         extruded_insulation,
                                                         P3D(0.0, 0.0, insulation_z))
        colored_insulation: Color = Color("F1x4LP Colored Insulation",
                                          translated_insulation, "Blue")
        f1x4lp_union.append(colored_insulation)
        f1x4lp_union.lock()

        # Now create the *f1x41lp_module*:
        f1x4lp_module: Module3D = Module3D("F1x4LP", [f1x4lp_union])
        scad_program.append(f1x4lp_module)
        scad_program.if3d.name_match_append("f1x4lp", f1x4lp_module,
                                            ["Female 1x4 Low Profile Cnnector"])

        # Create some *front_artworks*:
        front_artwork_rectangle: Square = Square("F1x4LP Rectangle", 4.2 * 2.54, 1.2 * 2.54)
        front_artwork_pin1_circle: Circle = Circle("F1x4LP Pin 1 Circle", 0.2, 8,
                                                   P2D(-2.25 * 2.54, + 0.75 * 2.54))
        front_artworks: List[SimplePolygon] = [front_artwork_rectangle, front_artwork_pin1_circle]

        # Now create the *pcb_chunk*:
        pcb_chunk: PCBChunk = PCBChunk("F1x4lp", pads, [f1x4lp_union],
                                       front_artworks=front_artworks)

        # Now insert *pads_group* into *f1x4lp_module*:
        f1x4lp_module.tag_insert("PadsGroup", pads_group)

        # Stuff some values into *f1x4lp* (i.e. *self*):
        # f1x4lp: F1x4LP = self
        self.module: Module3D = f1x4lp_module
        self.insulation_height: float = top_z
        self.pcb_chunk: PCBChunk = pcb_chunk
        self.pin_bottom_z: float = pin_bottom_z
        self.top_z: float = top_z


# HR2BaseAssembly:
class HR2BaseAssembly:
    """Represents the HR2 base with motor holders and spacers."""

    # HR2BaseAssembly.__init__():
    def __init__(self, scad_program: ScadProgram, romi_base: "RomiBase", romi_motor_holder,
                 connectors: Connectors, base_battery_top_z: float, base_top_z: float,
                 pi_board_z: float, master_board_z: float, arm_z: float) -> None:
        """Initialize a HR2BaseAssembly."""
        west_romi_motor_holder: UseModule3D = romi_motor_holder.module.use_module_get()
        degrees180: float = pi
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        east_romi_motor_holder: Rotate3D = Rotate3D("East Romi Motor Holder",
                                                    west_romi_motor_holder, degrees180, z_axis)

        lidar: Lidar = Lidar(scad_program, "yplidar_x2", 96.0, 60.5, 50.3, 60.5, 2 * 7.9,
                             35.3 + 5.0, 30.0, 10.0,
                             [P2D(-35.3, 0.0), P2D(36.0, 18.0), P2D(36.0, -18.0)])
        lidar = lidar

        # Create the *base_master_spacer* and the *battery_pi_spacer*:
        # base_dz: float = romi_base.base_dz
        battery_dz: float = romi_base.battery_dz

        # Create the 4 different spacer heights:
        base_master_height: float = master_board_z - base_top_z
        base_pi_height: float = pi_board_z - base_top_z
        battery_master_height: float = master_board_z - base_top_z + battery_dz
        battery_pi_height: float = pi_board_z - base_battery_top_z

        # Create the 4 different *Spacer*'s:
        base_master_spacer: Spacer = Spacer(
            scad_program, "Base Master Spacer", base_master_height, "M2.5")
        base_pi_spacer: Spacer = Spacer(
            scad_program, "Base Pi Spacer", base_pi_height, "M2.5")
        battery_master_spacer: Spacer = Spacer(
            scad_program, "Battery Master Spacer", battery_master_height, "M2.5")
        battery_pi_spacer: Spacer = Spacer(
            scad_program, "Battery Pi Spacer", battery_pi_height, "M2.5")

        spacer_positions: Dict[str, Tuple[P2D, str, float]] = romi_base.spacer_positions_get()
        spacers: List[Scad3D] = []
        spacer_name: str
        spacer_tuple: Tuple[P2D, str, float]
        for spacer_name, spacer_tuple in spacer_positions.items():
            # Upnpack *spacer_tuple*:
            position: P2D
            reference_name: str
            dz: float
            position, reference_name, dz = spacer_tuple

            # Select the correct *spacer* and *spacer_z*:
            spacer: Spacer
            spacer_z: float
            if spacer_name.startswith("Pi"):
                if dz > 0.0:
                    spacer = battery_pi_spacer
                    spacer_z = base_battery_top_z
                else:
                    spacer = base_pi_spacer
                    spacer_z = base_top_z
            else:
                if dz > 0.0:
                    spacer = battery_master_spacer
                    spacer_z = base_battery_top_z
                else:
                    spacer = base_master_spacer
                    spacer_z = base_top_z

            # Translate *spacer_scad* into postion and append to *spacer*:
            spacer_scad: UseModule3D = spacer.module.use_module_get()
            translated_spacer_scad: Scad3D = Translate3D(
                f"Translated {spacer_name}", spacer_scad, P3D(position.x, position.y, spacer_z))
            spacers.append(translated_spacer_scad)

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
        # self.spacer_tuples: List[Tuple[str, str, float, float, float]] = spacer_tuples
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
    def __init__(self,
                 scad_program: ScadProgram, pcb_origin: P2D, connectors: Connectors,
                 hr2_pi_assembly: "HR2PiAssembly", base_dxf: BaseDXF,
                 pi_board_z, master_board_z: float, nucleo_board_z: float, arm_z: float,
                 pi_offset2d: P2D, nucleo_offset2d: P2D, nucleo_rotate: float,
                 st_link_offset2d: P2D, encoder: "Encoder", raspi4b: "RaspberryPi4",
                 nucleo144: "Nucleo144", st_link: "STLink",
                 base_spacer_positions: Dict[str, Tuple[P2D, str, float]],
                 plate_spacer_positions: Dict[str, Tuple[P2D, str, str]],
                 romi_base_keys: List[Tuple[Any, ...]],
                 romi_expansion_plate_keys: List[Tuple[Any, ...]]) -> None:
        """Initialize the HR2MasterAssembly."""
        # print(f"HR2MasterAssembly: nucleo_offset2d:{nucleo_offset2d}")
        master_board: MasterBoard = MasterBoard(
            scad_program, pcb_origin, base_dxf, connectors, encoder, raspi4b, nucleo144, st_link,
            pi_offset2d, nucleo_offset2d, nucleo_rotate, st_link_offset2d, master_board_z,
            nucleo_board_z, arm_z, base_spacer_positions, plate_spacer_positions,
            romi_base_keys, romi_expansion_plate_keys)

        # Create *translated_master_without_nucleo* and *translated_master_with_nucleol*:
        # (i.e. *self*):
        master_module_without_nucleo: Module3D = master_board.master_module_without_nucleo
        translated_master_without_nucleo_board: Translate3D = Translate3D(
            "MasterBoard Without Nucleo Translated", master_module_without_nucleo.use_module_get(),
            P3D(0.0, 0.0, master_board_z))
        master_module_with_nucleo: Module3D = master_board.master_module_with_nucleo
        translated_master_with_nucleo_board: Translate3D = Translate3D(
            "MasterBoard With Nucleo Translated", master_module_with_nucleo.use_module_get(),
            P3D(0.0, 0.0, master_board_z))

        # Create *module_with_nucleo* and *module_without_nucleo*:
        module_without_nucleo: Module3D = Module3D("HR2 Master Assembly", [
            hr2_pi_assembly.module.use_module_get(),
            translated_master_without_nucleo_board])
        module_with_nucleo: Module3D = Module3D("HR2 Master Assembly", [
            hr2_pi_assembly.module.use_module_get(),
            translated_master_with_nucleo_board])

        # The master assembly uses the one without the Nucleo:
        scad_program.append(module_without_nucleo)
        scad_program.if3d.name_match_append(
            "hr2_master_assembly", module_without_nucleo, ["HR2 Base Assembly"])

        # Stuff some values into *hr2_master_assembly* (i.e. *self*):
        # hr2_master_assembly: HR2MasterAssembly = self
        self.module_without_nucleo: Module3D = module_without_nucleo
        self.module_with_nucleo: Module3D = module_with_nucleo
        self.hr2_pi_assembly: HR2PiAssembly = hr2_pi_assembly


# HR2NucleoAssembly:
class HR2NucleoAssembly:
    """Represents the HR2 up to the Nucleo144."""

    # HR2NucleoAssembly.__init__():
    def __init__(self, scad_program: ScadProgram, connectors: Connectors,
                 hr2_wheel_assembly: "HR2WheelAssembly", nucleo144: "Nucleo144",
                 nucleo_board_z: float, nucleo_offset2d: P2D, nucleo_rotate: float) -> None:
        """Initialize the HR2NucleoAssembly."""
        # Create *module*, append to *scad_program* and save into *hr2_nucleo_assembly*
        # (i.e. *self*):

        nucleo144_module: Module3D = nucleo144.nucleo_module
        nucleo144_use_module: UseModule3D = nucleo144_module.use_module_get()
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        rotated_nucleo144: Rotate3D = Rotate3D("Rotated Nucleo144 PCB",
                                               nucleo144_use_module, nucleo_rotate, z_axis)
        nucleo_translate: P3D = P3D(nucleo_offset2d.x, nucleo_offset2d.y, nucleo_board_z)
        translated_nucleo144: Translate3D = Translate3D("Translated Nucleo 144",
                                                        rotated_nucleo144, nucleo_translate)

        hr2_nucleo_assembly: Module3D = Module3D("HR2 Nucleo Assembly", [
             hr2_wheel_assembly.module.use_module_get(),
             translated_nucleo144,
        ])
        scad_program.append(hr2_nucleo_assembly)
        self.module = hr2_nucleo_assembly
        scad_program.if3d.name_match_append("hr2_nucleo_assembly",
                                            hr2_nucleo_assembly, ["HR2 Nucleo144 Assembly"])


# HR2PiAssembly:
class HR2PiAssembly:
    """Represents the HR2 Base with Pi mounted."""

    # HR2PiAssembly.__init__():
    def __init__(self, scad_program: ScadProgram, hr2_base_assembly: HR2BaseAssembly,
                 connectors: Connectors, raspi4b: "RaspberryPi4", other_pi: "OtherPi",
                 st_link: "STLink",
                 pi_board_z: float, pi_offset2d: P2D, st_link_offset: P2D) -> None:
        """Initialize the HR2BaseAssembly."""
        # Define some constants
        degrees90: float = pi / 2.0
        origin3d: P3D = P3D(0.0, 0.0, 0.0)
        z_axis: P3D = P3D(0.0, 0.0, 1.0)

        # Create the *other_pi* and rotate and translate to the correct location:
        other_pi_board: Module3D = scad_program.module3d_get("SBC_Other_Pi_pcb")
        other_pi_board_use_module: UseModule3D = other_pi_board.use_module_get()
        pi_reposition: P3D = P3D(pi_offset2d.x, pi_offset2d.y, pi_board_z)
        repositioned_other_pi: Scad3D = other_pi_board_use_module.reposition(
            "Reposition Other Pi", origin3d, z_axis, degrees90, pi_reposition)

        # Create the *raspberry_pi3* and rotate and translate to the correct location.:
        raspi4b_board_module: Module3D = scad_program.module3d_get("SBC_RasPi4_pcb")
        raspi4b_board_use_module: UseModule3D = raspi4b_board_module.use_module_get()
        repositioned_raspi4b: Scad3D = raspi4b_board_use_module.reposition(
            "Reposition Raspberry Pi 3B", origin3d, z_axis, degrees90, pi_reposition)

        # Create *module*, append to *scad_program* and save into *hr2_base_assembly* (i.e. *self*):
        module: Module3D = Module3D("HR2 Pi Assembly", [
            hr2_base_assembly.module.use_module_get(),
            repositioned_other_pi, repositioned_raspi4b])
        scad_program.append(module)

        # hr2_pi_assembly: HR2PiAssembly = self
        self.module = module
        self.hr2_base_assembly: HR2BaseAssembly = hr2_base_assembly
        scad_program.if3d.name_match_append("hr2_pi_assembly", module, ["HR2 Base Assembly"])


# HR2Robot:
class HR2Robot:
    """Represents the entire HR2 robot."""

    def __init__(self, scad_program: ScadProgram, pcb_origin: P2D) -> None:
        """Initialize an HR2Robot."""
        # Create the various shared *connectors* first:
        connectors: Connectors = Connectors(scad_program)

        # Build the *encoder8:
        base_dxf: BaseDXF = BaseDXF()
        encoder: Encoder = Encoder(scad_program, base_dxf, connectors, pcb_origin)

        # The various critical board heights are based off of the *encoder_bottom_z*:
        encoder_bottom_z: float = encoder.bottom_z
        master_board_z: float = encoder_bottom_z - 0.2  # Leave .2mm slop between encoder connectors
        pi_board_z: float = master_board_z - 10.250

        # It turns out that the code above computes the origin, so we'll just *pi_x* and
        # *pi_y* to 0.0.
        # *pi_dz* is selected to make all the master pcb pins fit:
        pi_x: float = 0.0
        pi_y: float = 0.0
        pi_offset2d: P2D = P2D(pi_x, pi_y)
        st_link_offset2d: P2D = P2D(17.5, -47.5)  # Trial and error
        # master_board_z: float = pi_z + 8.000

        # *nucleo_board_z* is selected such that the bottom lib of the Ethernet connector
        # on the Nucleo144 board is just below the top of the motor assembly latch.  This
        # allows us to connect to the RJ45 with a right-angle Ethernet cable.
        # Monoprice SlimRun Cat6A 90 Degree 36AWG S/STP Ethernet Network Cable .5ft, 1ft, etc.
        # The right angle connector is required to clear the wheel.
        nucleo_board_z: float = master_board_z + 15.00
        arm_z: float = master_board_z + 30.00

        # Now create the *raspi4b* and the *st_link*
        other_pi: OtherPi = OtherPi(scad_program, connectors)
        raspi4b: RaspberryPi4 = RaspberryPi4(scad_program, connectors, other_pi, pcb_origin)
        st_link: STLink = STLink(scad_program, connectors, pcb_origin)

        # Create *romi_base* and *romi_motor_holder*:
        romi_base: RomiBase = RomiBase(scad_program, base_dxf)
        romi_motor_holder: RomiMotorHolder = RomiMotorHolder(scad_program, base_dxf)

        # Create the *nucleo144* before *master_board* so it can be passed in:
        degrees90: float = pi / 2.0
        nucleo_offset2d: P2D = P2D(pi_x + 8.5, pi_y - 0.75)  # Trial and error
        nucleo_rotate: float = -degrees90
        # nucleo_offset2d: P2D = P2D(pi_x - 8.5, pi_y - 1.0)
        # nucleo_rotate: float = degrees90
        # print(f"HR2PiAssembly:nucleo_offset2d:{nucleo_offset2d}")
        nucleo144: Nucleo144 = Nucleo144(scad_program, connectors)

        # Create the *romi_expansion_plate* before *master_board* so it can be passed in:
        plate: RomiExpansionPlate = RomiExpansionPlate(scad_program)
        plate_keys: List[Tuple[Any, ...]] = plate.keys_get()

        # Create the *hr2_base_assembly* object that can accept the various PCB's and assemblies
        # that go on top of it:
        # Grab some Z values via *base_dxf*:
        base_battery_top_z: float = base_dxf.z_locate(-2.701374)
        base_top_z: float = base_dxf.z_locate(-3.095083)
        hr2_base_assembly: HR2BaseAssembly = HR2BaseAssembly(
            scad_program, romi_base, romi_motor_holder, connectors, base_battery_top_z, base_top_z,
            pi_board_z, master_board_z, arm_z)
        hr2_pi_assembly: HR2PiAssembly = HR2PiAssembly(scad_program, hr2_base_assembly,
                                                       connectors, raspi4b, other_pi, st_link,
                                                       pi_board_z, pi_offset2d, st_link_offset2d)
        romi_base_keys: List[Tuple[Any, ...]] = hr2_base_assembly.romi_base_keys_get()
        hr2_master_assembly: HR2MasterAssembly = HR2MasterAssembly(
            scad_program, pcb_origin, connectors, hr2_pi_assembly, base_dxf,
            pi_board_z, master_board_z, nucleo_board_z, arm_z,
            pi_offset2d, nucleo_offset2d, nucleo_rotate, st_link_offset2d,
            encoder, raspi4b, nucleo144, st_link,
            romi_base.spacer_positions_get(),
            plate.spacer_positions_get(),
            romi_base_keys, plate_keys)
        hr2_wheel_assembly: HR2WheelAssembly = HR2WheelAssembly(scad_program, hr2_master_assembly,
                                                                connectors,
                                                                base_dxf, encoder)

        hr2_nucleo_assembly: HR2NucleoAssembly = HR2NucleoAssembly(
            scad_program, connectors, hr2_wheel_assembly, nucleo144,
            nucleo_board_z, nucleo_offset2d, nucleo_rotate)
        hr2_arm_assembly: HR2ArmAssembly = HR2ArmAssembly(
            scad_program, hr2_nucleo_assembly, plate, arm_z)
        hr2_arm_assembly = hr2_arm_assembly


# HR2WheelAssembly:
class HR2WheelAssembly:
    """Represents HR2 with wheels, motors, and encoders installed."""

    # HR2WheelAssembly.__init__():
    def __init__(self, scad_program: ScadProgram,
                 hr2_master_assembly: HR2MasterAssembly,
                 connectors: Connectors,
                 base_dxf: BaseDXF, encoder: Encoder) -> None:
        """Initialzie HR2WheelAssembly."""
        # Create the *west_romi_wheel_assembly* and associated *UseModule3D*:
        west_romi_wheel_assembly: RomiWheelAssembly = RomiWheelAssembly(scad_program,
                                                                        base_dxf, encoder)
        west_romi_wheel_assembly_use_module: UseModule3D
        west_romi_wheel_assembly_use_module = west_romi_wheel_assembly.module.use_module_get()

        # Create *east_romi_wheel_assembly*:
        degrees180: float = pi
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        east_romi_wheel_assembly: Rotate3D = Rotate3D("East RomiWheel Assembly",
                                                      west_romi_wheel_assembly_use_module,
                                                      degrees180, z_axis)

        hr2_pi_assembly: HR2PiAssembly = hr2_master_assembly.hr2_pi_assembly

        # Create the *module* and append it to *scad_program*:
        module: Module3D = Module3D("HR2 Wheel Assembly", [
            hr2_master_assembly.module_without_nucleo.use_module_get(),
            west_romi_wheel_assembly_use_module,
            east_romi_wheel_assembly,
            hr2_pi_assembly.module.use_module_get(),
        ])
        scad_program.append(module)
        self.module: Module3D = module
        self.hr2_master_assembly: HR2MasterAssembly = hr2_master_assembly
        scad_program.if3d.name_match_append("hr2_wheel_assembly", module, ["HR2 Wheel Assembly"])


# Lidar:
class Lidar:
    """Represents an inexpensive Lidar."""

    # Lidar.__init__():
    def __init__(self, scad_program: ScadProgram, name: str, dx: float, dy: float, dz: float,
                 scanner_diameter: float, scanner_dz: float, scanner_center_dx: float,
                 motor_diameter: float, base_dz: float, spacer_positions: List[P2D]) -> None:
        """Initialize a Lidar."""
        motor_radius: float = motor_diameter / 2.0

        def arc_append(start_angle: float, end_angle: float,
                       start_radius: float, end_radius: float, arc_points: List[P2D]) -> None:
            """Append a curved arc of points to a list."""
            degrees180: float = pi
            degrees90: float = degrees180 / 2.0
            segments_count: int = int(90 / 5)
            delta_angle: float = degrees90 / float(segments_count)
            # *delta_radius* can be postive or negative:
            delta_radius: float = (end_radius - start_radius) / float(segments_count)
            index: int
            for index in range(segments_count):
                angle: float = start_angle + float(index) * delta_angle
                radius: float = start_radius + float(index) * delta_radius
                x: float = radius * cos(angle)
                y: float = radius * sin(angle)
                arc_point: P2D = P2D(x, y)
                arc_points.append(arc_point)

        # The origin is centered under the Lidar center, with the motor along the +X axis:
        degrees0: float = 0
        degrees180: float = pi
        degrees90: float = degrees180 / 2.0
        degrees270: float = degrees180 + degrees90
        degrees360: float = 2.0 * degrees180

        # Create the *base_exterior* out of 4 blended arcs:
        base_lidar_radius: float = scanner_center_dx
        base_points: List[P2D] = []
        arc_append(degrees0, degrees90, dx - scanner_center_dx, dy / 2.0, base_points)
        arc_append(degrees90, degrees180, dy / 2.0, base_lidar_radius, base_points)
        arc_append(degrees180, degrees270, base_lidar_radius, dy / 2.0, base_points)
        arc_append(degrees270, degrees360, dy / 2.0, dx - scanner_center_dx, base_points)
        base_exterior: SimplePolygon = SimplePolygon(f"{name} Base Outline", base_points, lock=True)

        # Compute various dz values:
        motor_dz: float = dz - scanner_dz - base_dz
        assert abs(motor_dz + base_dz + scanner_dz - dz) < 0.0001

        # Create the *motor_scad*:
        motor_x: float = -scanner_center_dx + dx - motor_radius
        motor_cylinder: Cylinder = Cylinder("Lidar Motor", motor_diameter,
                                            P3D(motor_x, 0.0, 0.0), P3D(motor_x, 0.0, motor_dz), 32)
        motor_scad: Scad3D = Color("Colored Lidar Motor", motor_cylinder, "Silver")

        # Create the *base_scad*:
        base_color: str = "Blue"
        extruded_base: LinearExtrude = LinearExtrude("Extruded Lidar Base", base_exterior, base_dz)
        translated_base: Translate3D = Translate3D("Translated Lidar Base", extruded_base,
                                                   P3D(0.0, 0.0, motor_dz))
        base_scad: Scad3D = Color("Colored Lidar Base", translated_base, base_color)

        # Create the *scanner_scad*:
        scanner_cylinder: Cylinder = Cylinder("Lidar Scanner", scanner_diameter,
                                              P3D(0.0, 0.0, motor_dz + base_dz),
                                              P3D(0.0, 0.0, dz), 64)
        scanner_scad: Scad3D = Color("Colored Lidar Scaner", scanner_cylinder, base_color)

        # Create the *lidar_module*:
        lidar_scads: List[Scad3D] = [motor_scad, base_scad, scanner_scad]
        lidar_module: Module3D = Module3D(f"{name} Lidar", lidar_scads)
        scad_program.append(lidar_module)
        scad_program.if3d.name_match_append(f"{name}_lidar", lidar_module, [f"{name} Lidar"])

        # Stuff everything into *lidar* (i.e. *self*):
        # lidar: Lidar = self
        self.module = lidar_module


# Nucleo144:
class Nucleo144:
    """Represents a STM32 Nucleo-144 development board."""

    # Nucleo144.__init__():
    def __init__(self, scad_program: ScadProgram,
                 connectors: Connectors, tracing: str = "") -> None:
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
        # The origin for this board is in the exact center, not the offset origin in files
        # above.

        # All units are in thousandths of an inch and need to be converted to millimeters:
        def mil(mil: float) -> float:
            return mil * 0.0254

        # Perform any requested *tracing*:
        # next_tracing: str = ""
        if tracing:
            print(f"{tracing}=>Nucleo144.__init()")
            # next_tracing = tracing + " "

        # Misc. constants:
        degrees90: float = pi / 2.0
        # origin: P2D = P2D(0, 0)
        # morpho_pin_columns: int = 35
        # morpho_pin_rows: int = 2
        mount_hole_diameter: float = 3.00  # Calipers

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
        # zio_pin_dz: float = 7.60 + pcb_dz
        # zio_insulation_dz: float = 8.85

        # Stuff some values into *nucleo144* (i.e. *self*):
        # nucleo144: Nucleo144 = self
        cn11_morpho_center: P2D = P2D(cn11_morpho_center_x, cn11_morpho_center_y)
        cn12_morpho_center: P2D = P2D(cn12_morpho_center_x, cn12_morpho_center_y)
        self.cn11_morpho_center: P2D = cn11_morpho_center
        self.cn12_morpho_center: P2D = cn12_morpho_center

        # Create the *colored_ethernet* connector and add it to *nucleo_pcb*:
        nucleo_exterior: Square = Square("Nucleo144 Exterior", pcb_dx, pcb_dy, corner_radius=1.0)
        # nucleo_pcb: PCB = PCB("Nucleo144", scad_program, pcb_dz, nucleo_exterior)
        ethernet_bsw: P3D = P3D(ethernet_west_x, ethernet_south_y, ethernet_bottom_z)
        ethernet_tne: P3D = P3D(ethernet_east_x, ethernet_north_y, ethernet_top_z)
        ethernet_connector_cube: CornerCube = CornerCube(
            "Ethernet Connector Corner Cube", ethernet_tne, ethernet_bsw)
        colored_ethernet: Color = Color(
            "Colored Ethenet Connector", ethernet_connector_cube, "Silver")
        ethernet_pcb_chunk: PCBChunk = PCBChunk(
            "Nucleo144 Ethernet Connector", [], [colored_ethernet])

        # Now do the *PCBChunk*'s:
        origin2d: P2D = P2D(0.0, 0.0)

        # Define mount hole centers:
        ne_hole_center: P2D = P2D(ne_mount_hole_x, ne_mount_hole_y)
        nw_hole_center: P2D = P2D(nw_mount_hole_x, nw_mount_hole_y)
        se_hole_center: P2D = P2D(se_mount_hole_x, se_mount_hole_y)
        sw_hole_center: P2D = P2D(sw_mount_hole_x, sw_mount_hole_y)
        center_hole_center: P2D = P2D(center_mount_hole_x, center_mount_hole_y)

        # Create the *mount_holes_pcb_chunk*:
        mount_holes_pads: List[Pad] = [
            Pad("NE Mount Hole", 0.0, 0.0, mount_hole_diameter, ne_hole_center, 0.0),
            Pad("NW Mount Hole", 0.0, 0.0, mount_hole_diameter, nw_hole_center, 0.0),
            Pad("SE Mount Hole", 0.0, 0.0, mount_hole_diameter, se_hole_center, 0.0),
            Pad("SW Mount Hole", 0.0, 0.0, mount_hole_diameter, sw_hole_center, 0.0),
            Pad("Center Mount Hole", 0.0, 0.0, mount_hole_diameter, center_hole_center, 0.0),
        ]
        mount_holes_pcb_chunk: PCBChunk = PCBChunk("Nucleo Mount Holes", mount_holes_pads, [])
        spacer_dz: float = 13.0  # mm
        spacer_hole_diameter: float = 2.90  # mm
        spacer_hex_diameter: float = 4.76  # mm
        spacer_polygon: Polygon = Polygon("Spacer Polygon", [
            Circle("Nucleo Spacer Hex Exterior", spacer_hex_diameter, 16),
            Circle("Nucleo Spacer Hole", spacer_hole_diameter, 16),
        ])
        extruded_spacer: LinearExtrude = LinearExtrude("Extruded Nucleo Spacer",
                                                       spacer_polygon, spacer_dz)
        origin3d: P3D = P3D(0.0, 0.0, 0.0)
        alignment: Cylinder = Cylinder("Alignment Screw", 2.45,
                                       P3D(0.0, 0.0, -2.0 * pcb_dz),
                                       P3D(0.0, 0.0, spacer_dz + 2.0 * pcb_dz), 16)
        colored_alignment: Color = Color("Colored Alignment Screw", alignment, "Silver")

        silver_spacer: Color = Color("Silver Nucleo Spacer", extruded_spacer, "Silver")
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        spacers_chunk: PCBChunk = PCBChunk("Repositioned Nucleo Spacers", [], [
            silver_spacer.reposition("NE Nucleo spacer", origin3d, z_axis, 0.0,
                                     P3D(ne_hole_center.x, ne_hole_center.y, 0.0)),
            silver_spacer.reposition("NW Nucleo spacer", origin3d, z_axis, 0.0,
                                     P3D(nw_hole_center.x, nw_hole_center.y, 0.0)),
            silver_spacer.reposition("SE Nucleo spacer", origin3d, z_axis, 0.0,
                                     P3D(se_hole_center.x, se_hole_center.y, 0.0)),
            silver_spacer.reposition("SW Nucleo spacer", origin3d, z_axis, 0.0,
                                     P3D(sw_hole_center.x, sw_hole_center.y, 0.0)),
            colored_alignment.reposition("Center Nucleo spacer", origin3d, z_axis, 0.0,
                                         P3D(center_hole_center.x, center_hole_center.y, 0.0)),
        ])

        # Create some cut-outs to improve visibility:
        cutout_dx: float = 40.0  # mm
        cutout_e: float = cutout_dx / 2.0
        cutout_w: float = - cutout_e
        extra_dy: float = 1.75  # mm

        # Create the *upper_cut_out*:
        upper_cutout_ne: P2D = P2D(cutout_e, ne_mount_hole_y - extra_dy)
        upper_cutout_sw: P2D = P2D(cutout_w, center_mount_hole_y + extra_dy)
        upper_cutout_center: P2D = (upper_cutout_ne + upper_cutout_sw) / 2.0
        upper_cutout_dx: float = abs(upper_cutout_ne.x - upper_cutout_sw.x)
        upper_cutout_dy: float = abs(upper_cutout_ne.y - upper_cutout_sw.y)
        upper_cutout: Square = Square(
            "Upper Cutout", upper_cutout_dx, upper_cutout_dy, center=upper_cutout_center)

        # Create the *middle_cut_out*:
        middle_cutout_ne: P2D = P2D(cutout_e, center_mount_hole_y - extra_dy)
        middle_cutout_sw: P2D = P2D(cutout_w, sw_mount_hole_y + extra_dy)
        middle_cutout_center: P2D = (middle_cutout_ne + middle_cutout_sw) / 2.0
        middle_cutout_dx: float = abs(middle_cutout_ne.x - middle_cutout_sw.x)
        middle_cutout_dy: float = abs(middle_cutout_ne.y - middle_cutout_sw.y)
        middle_cutout: Square = Square(
            "Middle Cutout", middle_cutout_dx, middle_cutout_dy, center=middle_cutout_center)

        # Create the *lower_cut_out*:
        lower_cutout_ne: P2D = P2D(cutout_e, sw_mount_hole_y - extra_dy)
        lower_cutout_sw: P2D = P2D(cutout_w, -pcb_dy / 2.0 + extra_dy)
        lower_cutout_center: P2D = (lower_cutout_ne + lower_cutout_sw) / 2.0
        lower_cutout_dx: float = abs(lower_cutout_ne.x - lower_cutout_sw.x)
        lower_cutout_dy: float = abs(lower_cutout_ne.y - lower_cutout_sw.y)
        lower_cutout: Square = Square(
            "Lower Cutout", lower_cutout_dx, lower_cutout_dy, center=lower_cutout_center)

        # # Digikey: SAM1066-40-ND; pcb_pin=2.79  insulation=2.54  mating_length=15.75
        # # Digikey: S2212EC-40-ND; pcb_pin=3.05  insulation=2.50  mating_length=8.08  price=$1.15/1
        # Create the *cutouts_pcb_chunk*:
        cutouts_pcb_chunk: PCBChunk = PCBChunk(
            "Nucleo Cutouts", [], [], cuts=[upper_cutout, middle_cutout, lower_cutout])

        # Create the 2 morpho connectors and 2 associated ground connectors:
        cn11_pcb_chunk: PCBChunk = (connectors.m2x35_long.pcb_chunk.scads_x_flip().sides_swap().
                                    reposition(origin2d, -degrees90, cn11_morpho_center).
                                    pads_rebase(1100))
        cn11_mate_pcb_chunk: PCBChunk = (connectors.f2x35.pcb_chunk.
                                         reposition(origin2d, -degrees90, cn11_morpho_center).
                                         pads_rebase(1100))
        cn11_ground_pcb_chunk: PCBChunk = (connectors.m1x2.pcb_chunk.scads_y_flip().sides_swap().
                                           reposition(origin2d, 0.0,
                                                      P2D(cn11_morpho_center_x, ground_south_y)))
        cn12_pcb_chunk: PCBChunk = (connectors.m2x35_long.pcb_chunk.scads_x_flip().sides_swap().
                                    reposition(origin2d, -degrees90, cn12_morpho_center).
                                    pads_rebase(1200))
        cn12_mate_pcb_chunk: PCBChunk = (connectors.f2x35.pcb_chunk.
                                         reposition(origin2d, -degrees90, cn12_morpho_center).
                                         pads_rebase(1200))
        cn12_ground_pcb_chunk: PCBChunk = (connectors.m1x2.pcb_chunk.scads_y_flip().sides_swap().
                                           reposition(origin2d, 0.0,
                                                      P2D(cn12_morpho_center_x, ground_south_y)))
        # Insert the 4 ZIO connectors:
        cn7_pcb_chunk: PCBChunk = (connectors.f2x10_long.pcb_chunk.
                                   reposition(origin2d, degrees90,
                                              P2D(cn7_zio_center_x, cn7_zio_center_y)))
        cn8_pcb_chunk: PCBChunk = (connectors.f2x8_long.pcb_chunk.
                                   reposition(origin2d, degrees90,
                                              P2D(cn8_zio_center_x, cn8_zio_center_y)))
        cn9_pcb_chunk: PCBChunk = (connectors.f2x15_long.pcb_chunk.
                                   reposition(origin2d, degrees90,
                                              P2D(cn9_zio_center_x, cn9_zio_center_y)))
        cn10_pcb_chunk: PCBChunk = (connectors.f2x17_long.pcb_chunk.
                                    reposition(origin2d, degrees90,
                                               P2D(cn10_zio_center_x, cn10_zio_center_y)))

        # Create the *nucleo_pcb_chunk*:
        nucleo_pcb_chunk: PCBChunk = PCBChunk.join("nucleo_144", [
            ethernet_pcb_chunk,
            cn7_pcb_chunk,
            cn8_pcb_chunk,
            cn9_pcb_chunk,
            cn10_pcb_chunk,
            cn11_pcb_chunk,
            cn11_ground_pcb_chunk,
            cn12_pcb_chunk,
            cn12_ground_pcb_chunk,
            mount_holes_pcb_chunk,
            cutouts_pcb_chunk,
        ])

        # Define the directories to use to write out the footprint to:
        hr2_directory: Path = Path(os.environ["HR2_DIRECTORY"])
        master_board_directory: Path = hr2_directory / "electrical" / "master_board" / "rev_a"
        master_board_pretty_directory: Path = master_board_directory / "pretty"
        nucleo_module: Module3D = nucleo_pcb_chunk.pcb_update(
            scad_program, origin2d, 1.6, nucleo_exterior, "White", None, [])
        nucleo_module = nucleo_module

        raised_nucleo_pcb_chunk: PCBChunk = PCBChunk(
            "Raised Nucleo 144", [], [nucleo_module.use_module_get().reposition(
                "Raised Nucleo", origin3d, z_axis, 0.0, P3D(0.0, 0.0, spacer_dz)),
            ])

        # Create *nucleo_mate_without_pcb_chunk*:
        nucleo_mate_without_nucleo_pcb_chunk: PCBChunk = PCBChunk.join("NUCLEO144_MATE", [
            mount_holes_pcb_chunk,
            cn11_mate_pcb_chunk,
            cn12_mate_pcb_chunk,
            spacers_chunk,
        ])

        # Create *nucleo_mate_with_nucleo_pcb_chunk*:
        nucleo_mate_with_nucleo_pcb_chunk: PCBChunk = PCBChunk.join("NUCLEO144_MATE", [
            raised_nucleo_pcb_chunk,
            nucleo_mate_without_nucleo_pcb_chunk,
        ])

        # Create *nucleo_mate_with_nucleo_module:
        nucleo_mate_without_nucleo_pcb_chunk.footprint_generate(
            "HR2", master_board_pretty_directory)

        # Stuff a some values into *nucleo144* (i.e. *self*):
        # nucleo144: Nucleo144 = self
        # self.module: Module3D = nucleo_board
        # self.pcb: PCB = nucleo_pcb
        self.nucleo_module: Module3D = nucleo_module
        self.nucleo_pcb_chunk: PCBChunk = nucleo_pcb_chunk
        self.nucleo_mate_with_nucleo_pcb_chunk: PCBChunk = nucleo_mate_with_nucleo_pcb_chunk
        self.nucleo_mate_without_nucleo_pcb_chunk: PCBChunk = nucleo_mate_without_nucleo_pcb_chunk

        # Wrap up any requested *tracing*:
        if tracing:
            # nucleo_pcb.show("", next_tracing)
            print(f"{tracing}<=Nucleo144.__init()")


# MasterBoard:
class MasterBoard:
    """Represents Master PCB that the various Pi boards mount to."""

    # MasterBoard.__init__():
    def __init__(self, scad_program: ScadProgram, pcb_origin: P2D, base_dxf: BaseDXF,
                 connectors: Connectors, encoder: "Encoder", raspi4b: "RaspberryPi4",
                 nucleo144: "Nucleo144", st_link: "STLink", pi_offset: P2D, nucleo_offset2d: P2D,
                 nucleo_rotate: float, st_link_offset: P2D, master_board_bottom_z: float,
                 nucleo_board_z: float, arm_z: float,
                 base_spacer_positions: Dict[str, Tuple[P2D, str, float]],
                 plate_spacer_positions: Dict[str, Tuple[P2D, str, str]],
                 romi_base_keys: List[Tuple[Any, ...]],
                 romi_expansion_plate_keys: List[Tuple[Any, ...]], tracing: str = "") -> None:
        """Initialize the MasterBoard."""
        master_board: MasterBoard = self

        next_tracing: str = tracing + " " if tracing else ""
        if tracing:
            next_tracing = tracing + " "
            print(f"{tracing}=>MasterBoard.__init__(...)")

        # Define some PCB constants
        pcb_dz: float = 1.6

        # Create the various exterior *SimplePolygon*'s:
        master_exterior: SimplePolygon
        center_exterior: SimplePolygon
        ne_exterior: SimplePolygon
        nw_exterior: SimplePolygon
        se_exterior: SimplePolygon
        sw_exterior: SimplePolygon
        (master_exterior, center_exterior, ne_exterior, nw_exterior, se_exterior,
         sw_exterior) = master_board.exteriors_create(scad_program, pcb_dz)

        # Install various cut out slots, mount_holes and spacers:
        ne_spacer_pcb_chunk: PCBChunk
        nw_spacer_pcb_chunk: PCBChunk
        center_spacer_pcb_chunk: PCBChunk
        arm_spacer_dz: float = arm_z - master_board_bottom_z - pcb_dz

        # Create the spacer *PCBChunk*'s:
        ne_spacer_pcb_chunk, nw_spacer_pcb_chunk, center_spacer_pcb_chunk = (
            master_board.spacer_mounts_create(
                scad_program, base_spacer_positions, plate_spacer_positions,
                romi_base_keys, romi_expansion_plate_keys, 0.0, arm_spacer_dz))

        # Install the nucleo144 and Raspberry Pi connectors and mounting holes:
        origin2d: P2D = P2D(0.0, 0.0)
        degrees180: float = pi
        degrees90: float = degrees180 / 2.0

        # Install the Grove modules:
        center_grove_pcb_chunk: PCBChunk
        ne_grove_pcb_chunk: PCBChunk
        nw_grove_pcb_chunk: PCBChunk
        center_grove_pcb_chunk, ne_grove_pcb_chunk, nw_grove_pcb_chunk = (
            master_board.groves_install(scad_program))

        # Create and install all of the sonars:
        hcsr04: HCSR04 = HCSR04(scad_program, connectors, pcb_origin)
        master_board.sonar_modules_create(scad_program, hcsr04, connectors)
        center_sonars_pcb_chunk: PCBChunk
        ne_sonars_pcb_chunk: PCBChunk
        nw_sonars_pcb_chunk: PCBChunk
        center_sonars_pcb_chunk, ne_sonars_pcb_chunk, nw_sonars_pcb_chunk = (
            master_board.sonars_install(hcsr04, connectors, tracing=next_tracing))

        # Grab the LED *PCBChunks*'s:
        ne_leds_pcb_chunk: PCBChunk
        nw_leds_pcb_chunk: PCBChunk
        se_leds_pcb_chunk: PCBChunk
        sw_leds_pcb_chunk: PCBChunk
        ne_leds_pcb_chunk, nw_leds_pcb_chunk, se_leds_pcb_chunk, sw_leds_pcb_chunk = (
            master_board.leds_install(scad_program))

        # Create the base MicroBus *PCBChunk*'s:
        mikrobus_small: MikroBus = MikroBus(scad_program, connectors, "S")
        mikrobus_small_pcb_chunk: PCBChunk = mikrobus_small.pcb_chunk
        mikrobus_small_pcb_chunk = mikrobus_small_pcb_chunk
        mikrobus_medium: MikroBus = MikroBus(scad_program, connectors, "M")
        mikrobus_medium_pcb_chunk: PCBChunk = mikrobus_medium.pcb_chunk
        mikrobus_medium_pcb_chunk = mikrobus_medium_pcb_chunk
        mikrobus_large: MikroBus = MikroBus(scad_program, connectors, "L")
        mikrobus_large_pcb_chunk: PCBChunk = mikrobus_large.pcb_chunk
        mikrobus_large_pcb_chunk = mikrobus_large_pcb_chunk

        # Create the MikroBus *PCBChunk*'s:
        center_mikro_bus_chunk: PCBChunk
        ne_mikrobus_pcb_chunk: PCBChunk
        nw_mikrobus_pcb_chunk: PCBChunk
        se_mikrobus_pcb_chunk: PCBChunk
        sw_mikrobus_pcb_chunk: PCBChunk
        center_mikrobus_pcb_chunk, ne_mikrobus_pcb_chunk, nw_mikrobus_pcb_chunk, \
            se_mikrobus_pcb_chunk, sw_mikrobus_pcb_chunk = master_board.mikrobus_install(
                mikrobus_small, mikrobus_medium, mikrobus_large)

        # Create the *u3v70_9v_pcb_chunk*:
        u3v70x: U3V70x = U3V70x(scad_program, 9.0, connectors)
        u3v70x_9v_pcb_chunk: PCBChunk = (
            u3v70x.mate_pcb_chunk.reposition(origin2d, 0.0, P2D(22.0, -8.5)))  # Trial and error

        # Squirt everything into the associated KiCad PCB's:
        assert "HR2_DIRECTORY" in os.environ, "HR2_DIRECTORY environement variable not set"
        hr2_directory: Path = Path(os.environ["HR2_DIRECTORY"])
        master_board_directory: Path = hr2_directory / "electrical" / "master_board" / "rev_a"
        master_kicad_pcb_path: Path = master_board_directory / "master.kicad_pcb"

        # Create the east and west encoder mating *PCBChunk*'s:
        encoder_mate_pcb_chunk: PCBChunk = encoder.encoder_mate_pcb_chunk
        encoder_mate_translate2d: P2D = P2D(36.3, 0.0)  # Trial and Error
        east_encoder_mate_pcb_chunk: PCBChunk = encoder_mate_pcb_chunk.reposition(
            origin2d, degrees180, encoder_mate_translate2d)
        east_encoder_reference: Reference = Reference("CN55", True, 0.0, encoder_mate_translate2d,
                                                      east_encoder_mate_pcb_chunk,
                                                      "HR2_EncoderMate;2XF1X3")
        west_encoder_mate_pcb_chunk: PCBChunk = encoder_mate_pcb_chunk.reposition(
            origin2d, 0.0, -encoder_mate_translate2d)
        west_encoder_reference: Reference = Reference("CN56", True, 0.0, -encoder_mate_translate2d,
                                                      west_encoder_mate_pcb_chunk,
                                                      "HR2_EncoderMate;2XF1X3")
        encoder_references_pcb_chunk: PCBChunk = PCBChunk("Encoder References", [], [],
                                                          references=[east_encoder_reference,
                                                                      west_encoder_reference])
        raspi4b_mate_pcb_chunk: PCBChunk = raspi4b.mate_pcb_chunk.reposition(
            origin2d, degrees90, origin2d)
        raspi4b_mate_reference: Reference = Reference(
            "CN57", True, 0.0, origin2d, raspi4b_mate_pcb_chunk, "RASPI;F2X20")
        raspi4b_mate_references_pcb_chunk: PCBChunk = PCBChunk(
            "Raspi4b References", [], [], references=[raspi4b_mate_reference])

        nucleo144_mate_with_nucleo_pcb_chunk: PCBChunk = (
            nucleo144.nucleo_mate_with_nucleo_pcb_chunk.reposition(
                origin2d, nucleo_rotate, nucleo_offset2d))
        nucleo144_mate_without_nucleo_pcb_chunk: PCBChunk = (
            nucleo144.nucleo_mate_without_nucleo_pcb_chunk.reposition(
                origin2d, nucleo_rotate, nucleo_offset2d))

        # Create *nucleo144_mate_refererence* and associated *PCBChunk*:
        nucleo144_mate_reference: Reference = Reference(
            "CN59", True, 0.0, origin2d,
            nucleo144_mate_with_nucleo_pcb_chunk, "NUCLEO144;MORPHO144")
        nucleo144_mate_references_pcb_chunk: PCBChunk = PCBChunk(
            "Nucleo144 References", [], [], references=[nucleo144_mate_reference])

        # Place the ST Link assembly (ST adaptor and ST-Link):
        st_mate_location: P2D = P2D(7.0 * 2.54, -47.0)
        st_mate_pcb_chunk: PCBChunk = (
            st_link.st_mate_pcb_chunk.reposition(origin2d, 0.0, st_mate_location))
        st_mate_reference: Reference = Reference(
            "CN58", True, 0.0, st_mate_location, st_mate_pcb_chunk, "ST_MATE;ST_MATE")
        st_mate_references_pcb_chunk: PCBChunk = PCBChunk(
            "ST_Mate References", [], [], references=[st_mate_reference])

        # Create *no_nucleo_chunk* which contains all the *PCBChunk*'s except the
        # Nucleo-144:
        no_nucleo_pcb_chunk: PCBChunk = PCBChunk.join("No Nucelo Center", [
            encoder_references_pcb_chunk,
            east_encoder_mate_pcb_chunk,
            center_grove_pcb_chunk,
            west_encoder_mate_pcb_chunk,
            center_sonars_pcb_chunk,
            st_mate_pcb_chunk,
            st_mate_references_pcb_chunk,
            raspi4b_mate_pcb_chunk,
            raspi4b_mate_references_pcb_chunk,
        ])

        # Create *center_without_nucleo_pcb_chunk* that does not contain the the Nucleo-14:
        center_without_nucleo_pcb_chunk: PCBChunk = PCBChunk.join("Master Center Without Nucleo", [
            u3v70x_9v_pcb_chunk,
            center_mikrobus_pcb_chunk,
            center_spacer_pcb_chunk,
            no_nucleo_pcb_chunk,
            nucleo144_mate_without_nucleo_pcb_chunk,
            nucleo144_mate_references_pcb_chunk,
        ])
        center_kicad_pcb_path: Path = master_board_directory / "center.kicad_pcb"
        center_module_without_nucleo: Module3D = center_without_nucleo_pcb_chunk.pcb_update(
            scad_program, pcb_origin, pcb_dz,
            center_exterior, "Tan", center_kicad_pcb_path, [])

        # Create *center_with_nucleo_pcb_chunk* that does contain the the Nucleo-14:
        center_with_nucleo_pcb_chunk: PCBChunk = PCBChunk.join("Master Center With Nucleo", [
            center_mikrobus_pcb_chunk,
            center_without_nucleo_pcb_chunk,
            nucleo144_mate_with_nucleo_pcb_chunk,
        ])
        center_module_with_nucleo: Module3D = center_with_nucleo_pcb_chunk.pcb_update(
            scad_program, pcb_origin, pcb_dz, center_exterior, "Tan", None, [])

        # Create *ne_pcb_chunk* and update its associated PCB:
        ne_pcb_chunk: PCBChunk = PCBChunk.join("Master NE", [
            ne_grove_pcb_chunk,
            ne_mikrobus_pcb_chunk,
            ne_leds_pcb_chunk,
            ne_spacer_pcb_chunk,
            ne_sonars_pcb_chunk,
        ])
        ne_kicad_pcb_path: Path = master_board_directory / "ne.kicad_pcb"
        ne_module: Module3D = ne_pcb_chunk.pcb_update(
            scad_program, pcb_origin, pcb_dz, ne_exterior, "YellowGreen", ne_kicad_pcb_path, [])

        nw_pcb_chunk: PCBChunk = PCBChunk.join("Master NW", [
            nw_grove_pcb_chunk,
            nw_mikrobus_pcb_chunk,
            nw_leds_pcb_chunk,
            nw_spacer_pcb_chunk,
            nw_sonars_pcb_chunk,
        ])
        nw_kicad_pcb_path: Path = master_board_directory / "nw.kicad_pcb"
        nw_module: Module3D = nw_pcb_chunk.pcb_update(
            scad_program, pcb_origin, pcb_dz, nw_exterior, "Orange", nw_kicad_pcb_path, [])

        se_pcb_chunk: PCBChunk = PCBChunk.join("Master SE", [
            se_mikrobus_pcb_chunk,
            se_leds_pcb_chunk,
        ])
        se_kicad_pcb_path: Path = master_board_directory / "se.kicad_pcb"
        se_module: Module3D = se_pcb_chunk.pcb_update(
            scad_program, pcb_origin, pcb_dz, se_exterior, "Purple", se_kicad_pcb_path, [])

        sw_pcb_chunk: PCBChunk = PCBChunk.join("Master SW", [
            sw_mikrobus_pcb_chunk,
            sw_leds_pcb_chunk,
        ])
        sw_kicad_pcb_path: Path = master_board_directory / "sw.kicad_pcb"
        sw_module: Module3D = sw_pcb_chunk.pcb_update(
            scad_program, pcb_origin, pcb_dz, sw_exterior, "Red", sw_kicad_pcb_path, [])

        # Create *master_with_nucleo_pcb_chunk* and *master_module_with_nucleo*
        master_with_nucleo_pcb_chunk: PCBChunk = PCBChunk.join("Master Without Nucleo", [
            center_with_nucleo_pcb_chunk,
            ne_pcb_chunk,
            nw_pcb_chunk,
            se_pcb_chunk,
            sw_pcb_chunk
        ])
        master_without_nucleo_pcb_chunk: PCBChunk = PCBChunk.join("Master", [
            center_without_nucleo_pcb_chunk,
            ne_pcb_chunk,
            nw_pcb_chunk,
            se_pcb_chunk,
            sw_pcb_chunk
        ])
        master_module_with_nucleo: Module3D = master_with_nucleo_pcb_chunk.pcb_update(
            scad_program, pcb_origin, pcb_dz, master_exterior, "Orange", master_kicad_pcb_path, [])
        master_module_without_nucleo: Module3D = master_without_nucleo_pcb_chunk.pcb_update(
            scad_program, pcb_origin, pcb_dz, master_exterior, "Orange", None, [])

        # Create a union of the 5 sub-boards:
        union_with_nucleo_module: Module3D = Module3D("Master With Nucleo Union", [
            center_module_with_nucleo.use_module_get(),
            ne_module.use_module_get(),
            nw_module.use_module_get(),
            se_module.use_module_get(),
            sw_module.use_module_get(),
        ])
        union_without_nucleo_module: Module3D = Module3D("Master Without Nucleo Union", [
            center_module_without_nucleo.use_module_get(),
            ne_module.use_module_get(),
            nw_module.use_module_get(),
            se_module.use_module_get(),
            sw_module.use_module_get(),
        ])

        # Create an entry for the `scad_show` program:
        scad_program.append(union_with_nucleo_module)
        scad_program.if3d.name_match_append(
            "master_union_board", union_with_nucleo_module, ["Master Union"])

        # Stuff some values into *master_board* (i.e. *self*):
        # master_board: MasterBoard = self
        self.master_module_with_nucleo: Module3D = master_module_with_nucleo
        self.master_module_without_nucleo: Module3D = master_module_without_nucleo
        self.master_with_nucleo_pcb_chunk: PCBChunk = master_with_nucleo_pcb_chunk
        self.master_without_nucleo_pcb_chunk: PCBChunk = master_without_nucleo_pcb_chunk
        self.center_module_with_nucleo: Module3D = center_module_with_nucleo
        self.center_module_without__nucleo: Module3D = center_module_without_nucleo
        self.center_with_nucleo_pcb_chunk: PCBChunk = center_with_nucleo_pcb_chunk
        self.center_without_nucleo_pcb_chunk: PCBChunk = center_without_nucleo_pcb_chunk
        self.ne_module: Module3D = ne_module
        self.ne_pcb_chunk: PCBChunk = ne_pcb_chunk
        self.nw_module: Module3D = nw_module
        self.nw_pcb_chunk: PCBChunk = nw_pcb_chunk
        self.pcb_dz: float = pcb_dz
        self.se_module: Module3D = se_module
        self.se_pcb_chunk: PCBChunk = se_pcb_chunk
        self.sw_module: Module3D = sw_module
        self.sw_pcb_chunk: PCBChunk = sw_pcb_chunk
        self.union_with_nucleo_module: Module3D = union_with_nucleo_module
        self.union_without_nucleo_module: Module3D = union_without_nucleo_module

        # Wrap up any requested *tracing*:
        if tracing:
            next_tracing = tracing + " "
            print(f"{tracing}<=MasterBoard.__init__(...)")

    # MasterBoard.exteriors_create():
    def exteriors_create(self, scad_program: "ScadProgram",
                         pcb_dz: float) -> Tuple[SimplePolygon, SimplePolygon,
                                                 SimplePolygon, SimplePolygon, SimplePolygon,
                                                 SimplePolygon]:
        """Create the master board PCB's with exterior contours."""
        # This routine creates 6 *SimmplePolygon*'s named *master_exterior*, *center_exterior*,
        # *ne_exterior*, *nw_exterior*, *se_exteior*, *sw_exterior*, *master_exterior*.
        # When the first 5 boards are attached to one another, they form the final Master PCB.
        # The first 5 boards are designed to be manufatured using the Bantam Labs PCB milling
        # machine and associated software to allow rapid prototyping.  The Bantum Labs PCB milling
        # machine has a maximum size contraint of 5in by 4in.  The 5 individual boards will
        # eventually into the merged one final "Master" *PCB*.)  The "Master" PCB is designed
        # to be manufactured using a standard PCB manfucturing service.
        #
        # The outline of the PCB shown in cruddy ASCII art below.  The PCB is broken into 6 boards
        # where '-' means a straight line and and '@' means an arc:
        #
        # * *master_pcb*: The master PCB is A-a@c-D-E-f#@h-H-I-J-K-L-M-m@p-Q-R-s@v-V-W-X-Y-Z-A
        # * *center_pcb*: The center PCB is squarish A-B-C-D-E-F-G-H-I-J-K-L-M-N-O-P-Q-R-X-Y-Z-A
        # * *ne_pcb*: The north east PCB is A-a@c-C-B-A
        # * *nw_pcb*: The north west PCB is F-f@h-H-G-F
        # * *se_pcb*: The south east PCB is S-s@v-V-U-T-S
        # * *sw_pcb*: The south west PCB is M-m@p-P-O-N-M
        #
        #
        #         *-------f             c-------*
        #        /        |             |        \
        #       /         |             |         \
        #      / G--------F             C--------B \
        #     h--H        E------+------D        A--a
        #        |               |               |
        #        |               |               |
        #        |               |               |
        #        I--------J      |      Y--------Z
        #                 |      |      |
        #                 +------+------+
        #                 |      |      |
        #        L--------K      |      X--------W
        #        |               |               |
        #        |               |               |
        #        |            Q--+--R            |
        #     m--M            |     |            V--v
        #      \ N            |     |            U /
        #       \ \           |     |           / /
        #        \ O----------P     S----------T /
        #         \           |     |           /
        #          \          |     |          /
        #           \         |     |         /
        #            *--------p     s--------*
        #
        # The dimensions of interest in alphabetical order are:
        # * *arm_well_dx*: The distance between the connector arm well edges in the X direction
        #   (|QR|=|PS|=|ps|).
        # * *arm_well_north: The north edge of the arm well.
        #   (Qy=Ry).
        # * *center_pcb_north*: The north coordinate of the master pcb.
        #   (Gy=Fy=Cy=By).
        # * *center_pcb_south*: The south coordinate of the master pcb.
        # * (Oy=Py=Sy=Ty).
        # * *connctor_well_dx*: The distance between the connector well edges in the X direction
        #   (|fc|=|FC|=|ED|).
        # * *connctor_well_south*: The south edge of the connector well.
        #   (Ey=Dy).
        # * *inner_radius*: The distance between the board origin and N, O, T, and U.
        # * *motor_well_dx*: The distance between the motor well edges in the X direction
        #   (|JY|=|KX|).
        # * *motor_well_dy*: The distance between the motor well edges in the X direction
        #   (|IL|=|JK|=|YX|=|ZW|).
        # * *diameter*: The diameter of the PCB.
        #   (|hv|/2=|ma|/2).
        # * *wheel_well_dx*: The distaace between the wheel well edges in the X direction
        #   (|GB|=|HA|=|IZ|=|LW|=|MV|=|NU|).
        # * *wheel_well_dy*: The distance between the wheel well edges in the Y direction
        #   (|hm|=|HM|=|CX|=|BZ|).

        # These constants are defined first since other constants depend upon them:
        wheel_well_dx: float = 125.0  # mm (from user's guide)
        wheel_well_dy: float = 72.0  # mm (from user's guide)
        radius: float = 163.0 / 2.0  # mm (from user's guide)

        # The remaining variable are defined in alphabetical order:
        arm_well_dx: float = 13.0  # mm (trail and error)
        arm_well_north: float = -53.0  # mm (trail and error)
        # bantam_dx: float = 5 * 25.4  # mm (size of Bantam Labs PCB blank)
        bantam_dy: float = 4 * 25.4  # mm (size of Bantam Labs PCB blank)
        center_pcb_north: float = wheel_well_dy / 2.0 + 2.5  # mm (trail and error)
        center_pcb_south: float = center_pcb_north - (bantam_dy - 4.0)  # mm (trail and error)
        center_pcb_dy: float = abs(center_pcb_north - center_pcb_south)
        inch2mm: float = 25.40
        # wheel_well_dx_inch: float = wheel_well_dx / inch2mm
        center_pcb_dy_inch: float = center_pcb_dy / inch2mm
        # print(f"wheel_well_dx:{wheel_well_dx_inch:.2f}in")
        # print(f"center_pcb_dx:{center_pcb_dy_inch:.2f}in")
        assert center_pcb_dy_inch <= 4.0, (f"Center PCB height(={center_pcb_dy_inch}in) "
                                           "is too wide for Bantum PCB Mill")
        corner_radius: float = 1.5  # mm
        # print(f"center_pcb_south:{center_pcb_south}")
        # print(f"center_pcb_north:{center_pcb_north}")
        connector_well_dx: float = 60.0  # mm (trail and error)
        connector_well_south: float = 32.5  # mm (trail and error)
        degrees5: float = 5.0 * pi / 180.0  # radians
        inner_radius: float = radius - 7.00  # mm (trail and error)
        origin: P2D = P2D(0.0, 0.0)
        # pcb_corner: float = 1.0  # mm
        motor_well_dx: float = 60.0  # mm (trail and error)
        motor_well_dy: float = 25.0  # mm (trail and error)

        # When you have a right triangle and you have the length of the hypotenuse (H) and
        # the one of the sides (S), the other side is computed via the Pythagorean Theorem.
        # The sign of O is determined independently.
        # (1) H^2 = S^2 + O^2
        # (2) O^2 = H^2 - S^2
        # (3) O = sqrt(H^2 - S^2)
        def pythagorean(h: float, s: float) -> float:
            return sqrt(h * h - s * s)

        # Well start with the central board locations A-Z:
        A: P2D = P2D(wheel_well_dx/2.0, wheel_well_dy/2.0)  # On center board edge, not a corner
        B: P2D = P2D(wheel_well_dx/2.0, center_pcb_north)
        C: P2D = P2D(connector_well_dx/2.0, center_pcb_north)
        D: P2D = P2D(connector_well_dx/2.0, connector_well_south)
        E: P2D = P2D(-connector_well_dx/2.0, connector_well_south)
        F: P2D = P2D(-connector_well_dx/2.0, center_pcb_north)
        G: P2D = P2D(-wheel_well_dx/2.0, center_pcb_north)
        H: P2D = P2D(-wheel_well_dx/2.0, wheel_well_dy/2.0)  # On center board edge, not a corner
        I: P2D = P2D(-wheel_well_dx/2.0, motor_well_dy/2.0)
        J: P2D = P2D(-motor_well_dx/2.0, motor_well_dy/2.0)
        K: P2D = P2D(-motor_well_dx/2.0, -motor_well_dy/2.0)
        L: P2D = P2D(-wheel_well_dx/2.0, -motor_well_dy/2.0)
        M: P2D = P2D(-wheel_well_dx/2.0, -wheel_well_dy/2.0)  # On center board edge, not a corner
        N: P2D = P2D(-wheel_well_dx/2.0, -pythagorean(inner_radius, -wheel_well_dx/2.0))
        O: P2D = P2D(-pythagorean(inner_radius, center_pcb_south), center_pcb_south)
        P: P2D = P2D(-arm_well_dx/2.0, center_pcb_south)
        Q: P2D = P2D(-arm_well_dx/2.0, arm_well_north)
        R: P2D = P2D(arm_well_dx/2.0, arm_well_north)
        S: P2D = P2D(arm_well_dx/2.0, center_pcb_south)
        T: P2D = P2D(pythagorean(inner_radius, center_pcb_south), center_pcb_south)
        U: P2D = P2D(wheel_well_dx/2.0, -pythagorean(inner_radius, wheel_well_dx/2.0))
        V: P2D = P2D(wheel_well_dx/2.0, -wheel_well_dy/2.0)  # On center board edge, not a corner
        W: P2D = P2D(wheel_well_dx/2.0, -motor_well_dy/2.0)
        X: P2D = P2D(motor_well_dx/2.0, -motor_well_dy/2.0)
        Y: P2D = P2D(motor_well_dx/2.0, motor_well_dy/2.0)
        Z: P2D = P2D(wheel_well_dx/2.0, motor_well_dy/2.0)

        # Now compute the points of the NE board:
        a: P2D = P2D(pythagorean(radius, wheel_well_dy/2.0), wheel_well_dy/2.0)
        c: P2D = P2D(connector_well_dx/2.0, pythagorean(radius, connector_well_dx/2.0))
        a_angle: float = a.atan2()
        # a_angle = a_angle
        c_angle: float = c.atan2()
        # c_angle = c_angle
        # print(f"a_angle:{a_angle * 180.0 / pi}")
        # print(f"c_angle:{c_angle * 180.0 / pi}")

        # Now compute the arc points of the NW board:
        f: P2D = P2D(-connector_well_dx/2.0, pythagorean(radius, -connector_well_dx/2.0))
        h: P2D = P2D(-pythagorean(radius, wheel_well_dy/2.0), wheel_well_dy/2.0)
        f_angle: float = f.atan2()
        # f_angle = f_angle
        h_angle: float = h.atan2()
        # h_angle = h_angle
        # print(f"f_angle:{f_angle * 180.0 / pi}")
        # print(f"h_angle:{h_angle * 180.0 / pi}")

        # Now compute the arc points of the SW board:
        m: P2D = P2D(-pythagorean(radius, -wheel_well_dy/2.0), -wheel_well_dy/2.0)
        p: P2D = P2D(-arm_well_dx/2.0, -pythagorean(radius, -arm_well_dx/2.0))
        m_angle: float = m.atan2()
        # m_angle = m_angle
        N_angle: float = N.atan2()
        # N_angle = N_angle
        O_angle: float = O.atan2()
        # O_angle = O_angle
        p_angle: float = p.atan2()
        # p_angle = p_angle

        # Now compute the arc points of the SE board:
        s: P2D = P2D(arm_well_dx/2.0, -pythagorean(radius, arm_well_dx/2.0))
        v: P2D = P2D(pythagorean(radius, -wheel_well_dy/2.0), -wheel_well_dy/2.0)
        s_angle: float = s.atan2()
        # s_angle = s_angle
        T_angle: float = T.atan2()
        # T_angle = T_angle
        U_angle: float = U.atan2()
        # U_angle = U_angle
        v_angle: float = v.atan2()
        # v_angle = v_angle

        # Create *master_exterior*:
        master_exterior: SimplePolygon = SimplePolygon("Master PCB Exterior Simple Polygon",
                                                       [], lock=False)
        master_exterior.corner_arc_append(A, corner_radius, "SE")
        master_exterior.rounded_arc_append("BH++EV++", origin, radius,
                                           a_angle, c_angle, corner_radius)
        master_exterior.corner_arc_append(D, corner_radius, "NW")
        master_exterior.corner_arc_append(E, corner_radius, "EN")
        master_exterior.rounded_arc_append("BV-+EH++", origin, radius,
                                           f_angle, h_angle, corner_radius)
        master_exterior.corner_arc_append(H, corner_radius, "WS")
        master_exterior.corner_arc_append(I, corner_radius, "NE")
        master_exterior.corner_arc_append(J, corner_radius, "WS")
        master_exterior.corner_arc_append(K, corner_radius, "NW")
        master_exterior.corner_arc_append(L, corner_radius, "ES")
        master_exterior.corner_arc_append(M, corner_radius, "NW")
        master_exterior.rounded_arc_append("BH-+EV-+", origin, radius,
                                           m_angle, p_angle, corner_radius)
        master_exterior.corner_arc_append(Q, corner_radius, "SE")
        master_exterior.corner_arc_append(R, corner_radius, "WS")
        # master_exterior.point_append(Q)
        # master_exterior.point_append(R)
        master_exterior.rounded_arc_append("BV++EH-+", origin, radius,
                                           s_angle, v_angle, corner_radius)
        master_exterior.corner_arc_append(V, corner_radius, "EN")
        master_exterior.corner_arc_append(W, corner_radius, "SW")
        master_exterior.corner_arc_append(X, corner_radius, "EN")
        master_exterior.corner_arc_append(Y, corner_radius, "SE")
        master_exterior.corner_arc_append(Z, corner_radius, "WN")
        # For some reason there is an extra segment genterated that covers the rounded corner.
        # Eventually this needs to be debugged or kludged around, but not for now.
        master_exterior.corner_arc_append(A, corner_radius, "SE")  # Strange! no rounded corner!
        master_exterior.lock()

        # Create *center_exterior*:
        center_exterior: SimplePolygon = SimplePolygon("Center Exterior", [], lock=False)
        center_exterior.corner_arc_append(B, corner_radius, "SW")
        center_exterior.point_append(C)
        center_exterior.corner_arc_append(D, corner_radius, "NW")
        center_exterior.corner_arc_append(E, corner_radius, "EN")
        center_exterior.point_append(F)
        center_exterior.corner_arc_append(G, corner_radius, "ES")
        # Skip H
        center_exterior.corner_arc_append(I, corner_radius, "NE")
        center_exterior.corner_arc_append(J, corner_radius, "WS")
        center_exterior.corner_arc_append(K, corner_radius, "NW")
        center_exterior.corner_arc_append(L, corner_radius, "ES")
        # Skip M
        center_exterior.arc_append(origin, inner_radius, N_angle, O_angle, degrees5)
        center_exterior.point_append(P)
        center_exterior.corner_arc_append(Q, corner_radius, "SE")
        center_exterior.corner_arc_append(R, corner_radius, "WS")
        center_exterior.point_append(S)
        center_exterior.arc_append(origin, inner_radius, T_angle, U_angle, degrees5)
        # Skip V
        center_exterior.corner_arc_append(W, corner_radius, "SW")
        center_exterior.corner_arc_append(X, corner_radius, "EN")
        center_exterior.corner_arc_append(Y, corner_radius, "SE")
        center_exterior.corner_arc_append(Z, corner_radius, "WN")
        # Skip A
        center_exterior.lock()

        # Create *ne_exterior*:
        ne_exterior: SimplePolygon = SimplePolygon("NE Exterior ", [], lock=False)
        ne_exterior.point_append(A)
        ne_exterior.rounded_arc_append("BH++EV++", origin, radius,
                                       a_angle, c_angle, corner_radius)
        ne_exterior.point_append(C)
        ne_exterior.corner_arc_append(B, corner_radius, "WS")
        ne_exterior.lock()

        # Create *nw_exterior*:
        nw_exterior: SimplePolygon = SimplePolygon("NW Exterior", [], lock=False)
        nw_exterior.point_append(F)
        nw_exterior.rounded_arc_append("BV-+EH++", origin, radius,
                                       f_angle, h_angle, corner_radius)
        nw_exterior.point_append(H)
        nw_exterior.corner_arc_append(G, corner_radius, "SE")
        nw_exterior.lock()

        # Create the *se_exterior*:
        se_exterior: SimplePolygon = SimplePolygon(
            "SE PCB Exterior SimplePolygon", [], lock=False)
        se_exterior.point_append(S)
        se_exterior.rounded_arc_append("BV++EH-+", origin, radius,
                                       s_angle, v_angle, corner_radius)
        se_exterior.point_append(V)
        # se_exterior.point_append(U)
        se_exterior.arc_append(origin, inner_radius, U_angle, T_angle, 0.0)
        # se_exterior.point_append(T)
        se_exterior.lock()

        # Create *sw_exterior*:
        sw_exterior: SimplePolygon = SimplePolygon("SW Exterior", [], lock=False)
        sw_exterior.point_append(M)
        sw_exterior.rounded_arc_append("BH-+EV-+", origin, radius,
                                       m_angle, p_angle, corner_radius)
        sw_exterior.point_append(P)
        # sw_exterior.point_append(O)
        sw_exterior.arc_append(origin, inner_radius, O_angle, N_angle, 0.0)
        # sw_exterior.point_append(N)
        sw_exterior.lock()

        # Return the resulting exterior *SimplePolygon*'s:
        return master_exterior, center_exterior, ne_exterior, nw_exterior, se_exterior, sw_exterior

    # MasterBoard.groves_install():
    def groves_install(self, scad_program: ScadProgram) -> Tuple[PCBChunk, PCBChunk, PCBChunk]:
        """Install the Grove mounting holes."""
        center_grove_pcb_chunks: List[PCBChunk] = []
        ne_grove_pcb_chunks: List[PCBChunk] = []
        nw_grove_pcb_chunks: List[PCBChunk] = []

        grove20x20: Grove = Grove(scad_program, 20, 20, "Blue")
        grove20x20_pcb_chunk: PCBChunk = grove20x20.pcb_chunk

        # Create a list grove placements:
        placements: List[Tuple[str, bool, P2D, float, PCBChunk, List[PCBChunk]]] = [
            ("NW Outer Bottom", False, P2D(-42.0, 53.0), radians(-90),
             grove20x20_pcb_chunk, nw_grove_pcb_chunks),
            ("NE Outer Bottom", False, P2D(47.0, 45.0), radians(-90),
             grove20x20_pcb_chunk, ne_grove_pcb_chunks),
            ("Center NE Inner Bottom", False, P2D(47.0, 25.0), radians(180),
             grove20x20_pcb_chunk, center_grove_pcb_chunks),
            ("Center NW Inner Top", True, P2D(-48.5, 31.0), radians(180 + 50),
             grove20x20_pcb_chunk, center_grove_pcb_chunks),
            ("Center SW Top", True, P2D(-48.0, -28.0), radians(180 - 22.5),
             grove20x20_pcb_chunk, center_grove_pcb_chunks),
            ("Center SE Bottom", False, P2D(47.5, -24.25), radians(180),
             grove20x20_pcb_chunk, center_grove_pcb_chunks),
        ]
        origin2d: P2D = P2D(0.0, 0.0)
        name: str
        is_front: bool
        position: P2D
        rotate: float
        grove: Grove
        pcb_chunks: List[PCBChunk]
        for name, is_front, position, rotate, grove_pcb_chunk, pcb_chunks in placements:
            pcb_chunk: PCBChunk = grove_pcb_chunk
            if not is_front:
                pcb_chunk = pcb_chunk.scads_x_flip().sides_swap()
            pcb_chunk = pcb_chunk.reposition(origin2d, rotate, position)
            pcb_chunks.append(pcb_chunk)

        # Assemble the final PCB Chunk:
        center_grove_pcb_chunk: PCBChunk = PCBChunk.join(
            "Center Groves", center_grove_pcb_chunks)
        ne_grove_pcb_chunk: PCBChunk = PCBChunk.join("NE Groves", ne_grove_pcb_chunks)
        nw_grove_pcb_chunk: PCBChunk = PCBChunk.join("NW Groves", nw_grove_pcb_chunks)
        return center_grove_pcb_chunk, ne_grove_pcb_chunk, nw_grove_pcb_chunk

    # MasterBoard.leds_install():
    def leds_install(self,
                     scad_program: ScadProgram) -> Tuple[PCBChunk, PCBChunk, PCBChunk, PCBChunk]:
        """Install the LEDs."""
        led: LED = LED("Led", scad_program)
        led_pcb_chunk: PCBChunk = led.pcb_chunk

        # Define *led_positions* which specifies the locations of all of the LED's:
        ne_led_pcb_chunks: List[PCBChunk] = []
        nw_led_pcb_chunks: List[PCBChunk] = []
        se_led_pcb_chunks: List[PCBChunk] = []
        sw_led_pcb_chunks: List[PCBChunk] = []
        n_pitch_angle: float = 16.0  # degrees
        n_center_offset: float = 43.0  # degrees
        ne_center_angle: float = 90.0 - n_center_offset  # degrees
        nw_center_angle: float = 90.0 + n_center_offset  # degrees
        s_pitch_angle: float = 13.0  # degrees
        s_center_offset: float = 34.0  # degrees
        sw_center_angle: float = 270.0 - s_center_offset  # degrees
        se_center_angle: float = 270.0 + s_center_offset  # degrees
        led_positions: List[Tuple[str, float, float, List[PCBChunk]]] = [
            ("NE LED1", radians(ne_center_angle - n_pitch_angle), 0.0, ne_led_pcb_chunks),
            ("NE LED2", radians(ne_center_angle), 0.0, ne_led_pcb_chunks),
            ("NE LED3", radians(ne_center_angle + n_pitch_angle), 0.0, ne_led_pcb_chunks),
            ("NW LED3", radians(nw_center_angle - n_pitch_angle), 0.0, nw_led_pcb_chunks),
            ("NW LED2", radians(nw_center_angle), 0.0, nw_led_pcb_chunks),
            ("NW LED1", radians(nw_center_angle + n_pitch_angle), 0.0, nw_led_pcb_chunks),
            ("SW LED1", radians(sw_center_angle - 2.0 * s_pitch_angle), 0.0, sw_led_pcb_chunks),
            ("SW LED2", radians(sw_center_angle - 1.0 * s_pitch_angle), 0.0, sw_led_pcb_chunks),
            ("SW LED3", radians(sw_center_angle), 0.0, sw_led_pcb_chunks),
            ("SW LED4", radians(sw_center_angle + 1.0 * s_pitch_angle), 0.0, sw_led_pcb_chunks),
            ("SW LED5", radians(sw_center_angle + 2.0 * s_pitch_angle), 0.0, sw_led_pcb_chunks),
            ("SE LED1", radians(se_center_angle - 2.0 * s_pitch_angle), 0.0, se_led_pcb_chunks),
            ("SE LED2", radians(se_center_angle - 1.0 * s_pitch_angle), 0.0, se_led_pcb_chunks),
            ("SE LED3", radians(se_center_angle), 0.0, se_led_pcb_chunks),
            ("SE LED4", radians(se_center_angle + 1.0 * s_pitch_angle), 0.0, se_led_pcb_chunks),
            ("SE LED5", radians(se_center_angle + 2.0 * s_pitch_angle), 0.0, se_led_pcb_chunks),
        ]
        origin2d: P2D = P2D(0.0, 0.0)
        led_radius: float = 78.0  # mm  Trial and error
        led_name: str
        led_angle: float  # radians
        led_offset: float  # mm
        pcb_chunks: List[PCBChunk] = []
        for led_name, led_angle, led_offset, pcb_chunks in led_positions:
            x: float = (led_radius - led_offset) * cos(led_angle)
            y: float = (led_radius - led_offset) * sin(led_angle)
            led_position: P2D = P2D(x, y)
            repositioned_led_pcb_chunk: PCBChunk = led_pcb_chunk.reposition(
                origin2d, led_angle, led_position)
            pcb_chunks.append(repositioned_led_pcb_chunk)

        # Create the final *PCBChunk*'s and return.
        ne_led_pcb_chunk: PCBChunk = PCBChunk.join("NE LED's", ne_led_pcb_chunks)
        nw_led_pcb_chunk: PCBChunk = PCBChunk.join("NW LED's", nw_led_pcb_chunks)
        se_led_pcb_chunk: PCBChunk = PCBChunk.join("SE LED's", se_led_pcb_chunks)
        sw_led_pcb_chunk: PCBChunk = PCBChunk.join("SW LED's", sw_led_pcb_chunks)
        return ne_led_pcb_chunk, nw_led_pcb_chunk, se_led_pcb_chunk, sw_led_pcb_chunk

    # MasterBoard.mikrobus_install():
    def mikrobus_install(
            self, mikrobus_small: "MikroBus", mikrobus_medium: "MikroBus",
            mikrobus_large: "MikroBus") -> Tuple[PCBChunk, PCBChunk, PCBChunk, PCBChunk, PCBChunk]:
        """Install the MikroBus boards."""
        center_pcb_chunks: List[PCBChunk] = []
        ne_pcb_chunks: List[PCBChunk] = []
        nw_pcb_chunks: List[PCBChunk] = []
        se_pcb_chunks: List[PCBChunk] = []
        sw_pcb_chunks: List[PCBChunk] = []

        # Create a placement table:
        degrees180: float = pi
        # degrees90: float = degrees180 / 2.0
        placements: List[Tuple[str, MikroBus, bool, P2D, float, List[PCBChunk]]] = [
            ("SW MikroBus", mikrobus_small,
             False, P2D(-45.0, -25.25), degrees180, center_pcb_chunks),
            ("NW MikroBus", mikrobus_medium, False, P2D(-44.5, 25.25), 0.0, center_pcb_chunks),
        ]

        origin2d: P2D = P2D(0.0, 0.0)
        name: str
        mikrobus: MikroBus
        position: P2D
        rotate: float
        pcb_chunks: List[PCBChunk]
        in_front: bool
        for name, mikrobus, in_front, position, rotate, pcb_chunks in placements:
            pcb_chunk: PCBChunk = mikrobus.mate_pcb_chunk
            if not in_front:
                pcb_chunk = pcb_chunk.scads_y_flip().pads_y_mirror().sides_swap()
            pcb_chunk = pcb_chunk.reposition(origin2d, rotate, position)
            pcb_chunks.append(pcb_chunk)

        center_pcb_chunk: PCBChunk = PCBChunk.join("Center MikroBus modules", center_pcb_chunks)
        ne_pcb_chunk: PCBChunk = PCBChunk.join("NE MikroBus modules", ne_pcb_chunks)
        nw_pcb_chunk: PCBChunk = PCBChunk.join("NW MikroBus modules", nw_pcb_chunks)
        se_pcb_chunk: PCBChunk = PCBChunk.join("SE MikroBus modules", se_pcb_chunks)
        sw_pcb_chunk: PCBChunk = PCBChunk.join("SW MikroBus modules", sw_pcb_chunks)
        return center_pcb_chunk, ne_pcb_chunk, nw_pcb_chunk, se_pcb_chunk, sw_pcb_chunk

    # MasterBoard.sonar_modules_create():
    def sonar_modules_create(self, scad_program: ScadProgram,
                             hcsr04: HCSR04, connectors: Connectors) -> None:
        """Create all of the sonar modules."""
        # Create "F1x4LP" low profile 1x4 .1in female connector:
        f1x4lp: F1x4LP = connectors.f1x4lp
        f1x4lp_top_z: float = f1x4lp.top_z

        # Create the *hcsr04_module* and grab some values out of it:
        hcsr04_pcb_dy: float = hcsr04.pcb_dy
        hcsr04_vertical_use_module: UseModule3D = hcsr04.vertical_sonar_module.use_module_get()

        # Create an *upright_sonar_module* that has repositioned *hcs04_use_module* so that
        # fits properly into the *f1x41p*:
        degrees180: float = pi
        degrees90: float = degrees180 / 2.0
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        z_rotated_hcsr04: Rotate3D = Rotate3D("Z-Axis Rotated HC-SR04",
                                              hcsr04_vertical_use_module, degrees180, z_axis)
        low_dy: float = (hcsr04_pcb_dy / 2.0     # Move PCB bottom edge to origin
                         + f1x4lp_top_z)         # Move PCB to top of F1x4LP (LP=>Low Profile)
        medium_dy: float = (hcsr04_pcb_dy / 2.0  # Move PCB bottom edge to origin
                            + 6.00)              # Move PBB to top of F1x4 (M=>Medium Profile)
        high_dy: float = (hcsr04_pcb_dy / 2.0    # Move PCB bottom edge to origin
                          + 8.00)                # Move PCB to top of F1x4H (H=>High Connector)
        recenter_dz: float = 1.27                # Move from PCB bottom to center of pins

        # There are three different sonar heights -- low, medium and high:
        # * *low*: Use low profile (and more expensive) connector to keep Sonar close to PCB.
        # * *medium*: Use nominal (and inexpensive) height connector where sonar height unimportant.
        # * *high*: Use high profile (and more expensive) connecto to boost Sonar above rear
        #   connectors on SBC (in particular the RaspBerry Pi.
        low_recentered_hcsr04: Translate3D = Translate3D(
            "Low Recentered HC-SR04", z_rotated_hcsr04, P3D(0.0, low_dy, recenter_dz))
        medium_recentered_hcsr04: Translate3D = Translate3D(
            "Medium Recentered HC-SR04", z_rotated_hcsr04, P3D(0.0, medium_dy, recenter_dz))
        high_recentered_hcsr04: Translate3D = Translate3D(
            "High Recentered HC-SR04", z_rotated_hcsr04, P3D(0.0, high_dy, recenter_dz))

        # Rotate both by 90 degrees so they point along the X axis:
        x_axis: P3D = P3D(1.0, 0.0, 0.0)
        low_x_rotated_hcsr04: Rotate3D = Rotate3D("Low X-Axis Rotated HC-SR04",
                                                  low_recentered_hcsr04, degrees90, x_axis)
        medium_x_rotated_hcsr04: Rotate3D = Rotate3D("Medium X-Axis Rotated HC-SR04",
                                                     medium_recentered_hcsr04, degrees90, x_axis)
        high_x_rotated_hcsr04: Rotate3D = Rotate3D("High X-Axis Rotated HC-SR04",
                                                   high_recentered_hcsr04, degrees90, x_axis)
        # Create both low and high modules:
        hcsr04low_module: Module3D = Module3D("HCSR04Low", [low_x_rotated_hcsr04])
        hcsr04medium_module: Module3D = Module3D("HCSR04Medium", [medium_x_rotated_hcsr04])
        hcsr04high_module: Module3D = Module3D("HCSR04High", [high_x_rotated_hcsr04])
        scad_program.append(hcsr04low_module)
        scad_program.append(hcsr04medium_module)
        scad_program.append(hcsr04high_module)
        # scad_program.if3d.name_match_append("hcsr04rp", hcsr04rp_module,
        #                                     ["Repositioned HC-SR04 sonar"])

    # MasterBoard.sonars_install():
    def sonars_install(self, hcsr04: HCSR04, connectors: Connectors,
                       tracing: str = "") -> Tuple[PCBChunk, PCBChunk, PCBChunk]:
        """Install all of the sonars."""
        # Perform any requestd *tracing*:
        # next_tracing: str = ""
        if tracing:
            # next_tracing = tracing + " "
            print(f"{tracing}=>MasterBoard.sonars_install(*, *, *)")

        # Create *sonar_poses* list, which has 1 4-tuple for each sonar.  The format of
        # the 7-tuple is:
        #      (*name*, *height, *board*,*rim_angle*, *beam_angle*, *sonar_offset*, *on_bottom*)
        # where:
        # * *name*: Is the name of the sonar:
        # * *board*: The sub-PCB to install sonars on:
        # * *height*: One of "high", "normal", "low" for the sonar height.
        # * *rim_angle*: The polar coordinate angle from the orgin of the sonar.
        # * *beam_angle*: The angle of the sonar direction.
        # * *sonar_offset*: The offset from the nominal *sonar_radius*.
        # * *on_bottom*: A flag that is *True* if the sonar is on the PCB bottom.
        sonars_radius: float = 62.0
        degrees2radians: float = pi / 180.0
        center_pcb_chunks: List[PCBChunk] = []
        center_references: List[Reference] = []
        ne_pcb_chunks: List[PCBChunk] = []
        ne_references: List[Reference] = []
        nw_pcb_chunks: List[PCBChunk] = []
        nw_references: List[Reference] = []
        sonar_poses: List[Tuple[str, str, List[PCBChunk], List[Reference], str,
                                float, float, float, bool]] = [
            ("Rear Left Sonar", "high", ne_pcb_chunks, ne_references, "CN60",
             (90.0 - 2 * 22.5 + 1.0) * degrees2radians,    # Trial and error to fit on PCB
             (90.0 + 0.5 * 22.5) * degrees2radians,
             2.8 * 2.54, True),  # robot perimeter and miss spacer.
            ("Rear Right Sonar", "high", nw_pcb_chunks, nw_references, "CN70",
             (90.0 + 2 * 22.5 - 1.0) * degrees2radians,    # Same Trial and error as above
             (90.0 - 0.5 * 22.5) * degrees2radians,
             2.8 * 2.540, True),
            ("Front Right Top Sonar", "normal",
             center_pcb_chunks, center_references, "CN50",
             (270.0 - 2 * 22.5 + 10.5) * degrees2radians,  # Trail and error
             (270.0 - 2 * 22.5) * degrees2radians,
             0.0 * 2.54, True),
            ("Front Right Bottom Sonar", "low",
             center_pcb_chunks, center_references, "CN51",
             (270.0 - 1 * 22.5 - 5.5) * degrees2radians,   # Trail and error
             (270.0 - 1 * 22.5) * degrees2radians,
             2.5 * 2.54, False),
            ("Front Center Sonar", "low",
             center_pcb_chunks, center_references, "CN52",
             (270.0) * degrees2radians,                    # Rim Angle
             (270.0) * degrees2radians,                    # Beam Angle
             24.0, True),
            ("Front Left Bottom Sonar", "low",
             center_pcb_chunks, center_references, "CN53",
             (270.0 + 1 * 22.5 + 5.5) * degrees2radians,   # Trail and error
             (270.0 + 1 * 22.5) * degrees2radians,
             2.5 * 2.54, False),
            ("Front Left Top Sonar", "normal",
             center_pcb_chunks, center_references, "CN54",
             (270.0 + 2 * 22.5 - 10.5) * degrees2radians,  # Trail and error
             (270.0 + 2 * 22.5) * degrees2radians,
             0.0 * 2.54, True),
        ]

        # Now create *translated_hcsr04_f1x1lp_pcb_chunk*:
        f1x4_mate_pcb_chunk: PCBChunk = hcsr04.f1x4_mate_pcb_chunk
        vertical_hcsr04_use_module: UseModule3D = hcsr04.vertical_sonar_module.use_module_get()
        translated_hcsr04_for_f1x4: Translate3D = Translate3D(
            "Translated HCSR04 for F1x4", vertical_hcsr04_use_module,
            P3D(0.0, 0.0, connectors.f1x4.insulation_height))
        translated_hcsr04_f1x4_pcb_chunk: PCBChunk = PCBChunk("Translated HCSR04 F1x4", [],
                                                              [translated_hcsr04_for_f1x4])
        f1x4_hcsr04_pcb_chunk: PCBChunk = PCBChunk.join(
            "F1X4 and Sonar", [f1x4_mate_pcb_chunk, translated_hcsr04_f1x4_pcb_chunk])

        # Now create both *top_surface_hcsr04_f1x4lp_pcb_chunk* for the PCB top surface:
        f1x4lp_mate_pcb_chunk: PCBChunk = hcsr04.f1x4lp_mate_pcb_chunk
        translated_hcsr04_for_f1x4lp: Translate3D = Translate3D(
            "Translated HCSR04 for F1x4LP", vertical_hcsr04_use_module,
            P3D(0.0, 0.0, connectors.f1x4lp.insulation_height))
        translated_hcsr04_pcb_chunk: PCBChunk = PCBChunk("Translated HRCS04 for F1X4LP", [],
                                                         [translated_hcsr04_for_f1x4lp])
        top_surface_hcsr04_f1x4lp_pcb_chunk: PCBChunk = PCBChunk.join(
            "F1X4LP and HCSR04 (Top Surface)",
            [f1x4lp_mate_pcb_chunk, translated_hcsr04_pcb_chunk])

        # Now create both *bottom_surface_hcsr04_f1x4lp_pcb_chunk* for the PCB bottom surface:
        degrees180: float = pi
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        z_rotated_hcsr04_f1x4lp: Rotate3D = Rotate3D(
            "Z Rotated HCSR04 for F1x4LPB", translated_hcsr04_for_f1x4lp, degrees180, z_axis)
        z_rotated_hcsr04_pcb_chunk: PCBChunk = PCBChunk(
            "Z Rotated HCSR04", [], [z_rotated_hcsr04_f1x4lp])
        bottom_surface_hcsr04_f1x4lp_pcb_chunk: PCBChunk = PCBChunk.join(
            "Bottom Surface F1X4 and HCSR04",
            [f1x4lp_mate_pcb_chunk, z_rotated_hcsr04_pcb_chunk])

        # Now create *translated_hcsr04_f1x4h_pcb_chunk*:
        f1x4h_mate_pcb_chunk: PCBChunk = hcsr04.f1x4h_mate_pcb_chunk
        translated_hcsr04_for_f1x4h: Translate3D = Translate3D(
            "Translated HCSR04 for F1x4H", vertical_hcsr04_use_module,
            P3D(0.0, 0.0, connectors.f1x4h.insulation_height))
        translated_hcsr04_f1x4h_pcb_chunk: PCBChunk = PCBChunk("Translated HCSR04 F1x4H", [],
                                                               [translated_hcsr04_for_f1x4h])
        f1x4h_hcsr04_pcb_chunk: PCBChunk = PCBChunk.join(
            "F1X4_Sonar", [f1x4h_mate_pcb_chunk, translated_hcsr04_f1x4h_pcb_chunk])

        # Iterate across all of the sonars in *sonar_poses*:
        origin2d: P2D = P2D(0.0, 0.0)
        degrees90: float = pi / 2.0
        sonar_name: str
        height: str
        pcb_chunks: List[PCBChunk]
        foot_print_name: str
        references: List[Reference]
        rim_angle: float
        beam_angle: float
        radius_offset: float
        is_front: bool
        placements_file: IO[Any] = open("/tmp/placments.csv", "w")
        for (sonar_name, height, pcb_chunks, references, reference_name,
             rim_angle, beam_angle, sonar_offset, is_front) in sonar_poses:
            sonar_radius: float = sonars_radius - sonar_offset
            sonar_x: float = sonar_radius * cos(rim_angle)
            sonar_y: float = sonar_radius * sin(rim_angle)
            # sonar_flags: str = "" if is_front else "byY"
            sonar_translate: P2D = P2D(sonar_x, sonar_y)
            connector_pcb_chunk: PCBChunk
            rotate: float = beam_angle + degrees90
            if height == "low":
                connector_pcb_chunk = (top_surface_hcsr04_f1x4lp_pcb_chunk
                                       if is_front else bottom_surface_hcsr04_f1x4lp_pcb_chunk)
                footprint_name = "HCSR04;F1x4LP"
            elif height == "normal":
                connector_pcb_chunk = f1x4_hcsr04_pcb_chunk
                footprint_name = "HCSR04;F1x4"
            elif height == "high":
                connector_pcb_chunk = f1x4h_hcsr04_pcb_chunk
                footprint_name = "HCSR04;F1x4H"
            else:
                assert False, f"'{height}' is not one of 'high', 'medium', or 'low'."
            reference: Reference = Reference(reference_name, is_front, rotate, sonar_translate,
                                             connector_pcb_chunk, footprint_name)
            placement_rotate: float = degrees(rotate)
            if not is_front:
                placement_rotate += 180.0
            while placement_rotate > 360.0:
                placement_rotate -= 360.0
            while placement_rotate < 0.0:
                placement_rotate += 360.0
            placements_file.write(f"{reference_name}\t"
                                  f"{origin2d.x + sonar_x:.3f}\t{origin2d.y - sonar_y:.3f}\t"
                                  f"{is_front}\t{placement_rotate:.1f}\n")

            references.append(reference)
            if tracing:
                print(f"{tracing}'{sonar_name}': reference:{reference}")
                print(f"{tracing}'{sonar_name}Pad[0]': reference:{reference.pcb_chunk.pads[0]}")

            if not is_front:
                connector_pcb_chunk = connector_pcb_chunk.scads_x_flip().sides_swap()
            repostioned_connector_pcb_chunk: PCBChunk = connector_pcb_chunk.reposition(
                origin2d, beam_angle + degrees90, sonar_translate)
            pcb_chunks.append(repostioned_connector_pcb_chunk)
        placements_file.close()

        center_references_pcb_chunk: PCBChunk = PCBChunk("Center References", [], [],
                                                         references=center_references)
        center_sonar_pcb_chunk: PCBChunk = PCBChunk.join("", center_pcb_chunks +
                                                         [center_references_pcb_chunk])

        if tracing:
            print(f"{tracing}f1x4h_mate_pcb_chunk.pads[0]={f1x4h_mate_pcb_chunk.pads[0]}")
        ne_references_pcb_chunk: PCBChunk = PCBChunk("NE References", [], [],
                                                     references=ne_references)
        ne_sonar_pcb_chunk: PCBChunk = PCBChunk.join("", ne_pcb_chunks + [ne_references_pcb_chunk])

        nw_references_pcb_chunk: PCBChunk = PCBChunk("NW References", [], [],
                                                     references=nw_references)
        nw_sonar_pcb_chunk: PCBChunk = PCBChunk.join("", nw_pcb_chunks + [nw_references_pcb_chunk])

        # Wrap up any requested *tracing*:
        if tracing:
            print(f"{tracing}<=MasterBoard.sonars_install(*, *, *)")
        return center_sonar_pcb_chunk, ne_sonar_pcb_chunk, nw_sonar_pcb_chunk

    # MasterBoard.spacer_mounts_create():
    def spacer_mounts_create(
            self, scad_program: ScadProgram,
            base_spacer_positions: Dict[str, Tuple[P2D, str, float]],
            plate_spacer_positions: Dict[str, Tuple[P2D, str, str]],
            base_keys: List[Tuple[Any, ...]], expansion_keys: List[Tuple[Any, ...]],
            master_board_top_z: float, arm_spacer_dz: float) -> Tuple[PCBChunk, PCBChunk, PCBChunk]:
        """Append some spacers mounts to MasterBoard."""
        # Use *base_keys* to build *base_keys_table*:
        base_keys_table: Dict[str, Tuple[Any, ...]] = {}
        base_key: Tuple[Any, ...]
        base_key_name: str
        for base_key in base_keys:
            base_key_name = base_key[1]
            base_keys_table[base_key_name] = base_key

        # Use *expansion_keys* to build *expansion_keys_table*:
        expansion_keys_table: Dict[str, Tuple[Any, ...]] = {}
        expansion_key: Tuple[Any, ...]
        expansion_key_name: str
        index: int
        for index, expansion_key in enumerate(expansion_keys):
            expansion_key_name = expansion_key[1]
            expansion_keys_table[expansion_key_name] = expansion_key
            # print(f"[{index}]: '{expansion_key_name}', {expansion_key}")

        # Collect the *Pad*s and *Scad3D*'s in various lists:
        center_pcb_chunks: List[PCBChunk] = []
        center_references: List[Reference] = []
        ne_pcb_chunks: List[PCBChunk] = []
        ne_references: List[Reference] = []
        nw_pcb_chunks: List[PCBChunk] = []
        nw_references: List[Reference] = []

        # Iterate through *base_spacer_positions* and install the mounting holes:
        hole_diameter: float = 2.7  # M2.5 spacer hole diameter
        base_spacer_name: str
        base_spacer_tuple: Tuple[P2D, str, float]
        for base_spacer_name, base_spacer_tuple in base_spacer_positions.items():
            # Unpack *spacer_tuple*:
            # FIXME: Add a hole reference:
            base_position: P2D
            base_reference_name: str
            base_dz: float
            base_position, base_reference_name, base_dz = base_spacer_tuple
            if not base_spacer_name.startswith("Pi"):
                # Construct the *Pad*, *PCBChunk*, and *Reference*:
                base_spacer_pad: Pad = Pad(
                    f"{base_spacer_name} Mount Hole", 0.0, 0.0, hole_diameter, base_position)
                base_spacer_pcb_chunk: PCBChunk = PCBChunk(
                    f"{base_spacer_name} Mount", [base_spacer_pad], [])
                base_spacer_reference: Reference = Reference(
                    base_reference_name, True, 0.0, base_position, base_spacer_pcb_chunk,
                    "HOLE_FOOTPRINT")

                # Use *base_position* to determine which board to put the *PCBChunk*/*Reference* on:
                if base_position.y < 0.0:
                    center_pcb_chunks.append(base_spacer_pcb_chunk)
                    center_references.append(base_spacer_reference)
                elif base_position.x < 0.0:
                    nw_pcb_chunks.append(base_spacer_pcb_chunk)
                    nw_references.append(base_spacer_reference)
                else:
                    ne_pcb_chunks.append(base_spacer_pcb_chunk)
                    ne_references.append(base_spacer_reference)

        # Create the *master_arm_spacer*:
        master_arm_spacer: Spacer = Spacer(
            scad_program, "Mater Arm Spacer", arm_spacer_dz, "M2.5")

        # Iterate through *plate_spacer_positions*:
        plate_spacer_name: str
        plate_spacer_tuple: Tuple[P2D, str, str]
        arm_hole_diameter: float = 2.70  # M2.5 standard fit
        for plate_spacer_name, plate_spacer_tuple in plate_spacer_positions.items():
            # Unpack *plate_spacer_tuple*:
            plate_position: P2D
            plate_reference: str
            plate_board_name: str
            plate_position, plate_reference, plate_board_name = plate_spacer_tuple
            # The plate is rotate 180 degrees from the documentation, hence the minus sign:
            plate_position = -plate_position

            # *arm_spacer_scad* and *arm_spacer_pad* and stuff them onto the appropiate PCB:
            arm_spacer_scad: Scad3D = Translate3D(
                "Translated Arm Spacer", master_arm_spacer.module.use_module_get(),
                P3D(plate_position.x, plate_position.y, 0.0))
            arm_spacer_pad: Pad = Pad(
                f"{plate_spacer_name} Mount", 0.0, 0.0, arm_hole_diameter, plate_position)
            arm_spacer_pads: List[Pad] = ([] if plate_spacer_name.endswith("NO_HOLE")
                                          else [arm_spacer_pad])
            arm_spacer_pcb_chunk: PCBChunk = PCBChunk(
                f"{plate_spacer_name}", arm_spacer_pads, [arm_spacer_scad])
            arm_spacer_reference: Reference = Reference(
                plate_reference, False, 0.0, plate_position, arm_spacer_pcb_chunk, "HOLE_FOOTPRINT")

            # Install *PCBChunk* and *Reference* onto appropriate board:
            if plate_board_name == "center":
                center_pcb_chunks.append(arm_spacer_pcb_chunk)
                center_references.append(arm_spacer_reference)
            elif plate_board_name == "ne":
                ne_pcb_chunks.append(arm_spacer_pcb_chunk)
                ne_references.append(arm_spacer_reference)
            elif plate_board_name == "nw":
                nw_pcb_chunks.append(arm_spacer_pcb_chunk)
                nw_references.append(arm_spacer_reference)
            else:
                assert False, f"'{plate_board_name}' is not a valid board name"

        # Create various *PCBChunk*'s that contains
        center_references_pcb_chunk: PCBChunk = PCBChunk(
            "Center Mount References", [], [], references=center_references)
        ne_references_pcb_chunk: PCBChunk = PCBChunk(
            "NE Mount References", [], [], references=ne_references)
        nw_references_pcb_chunk: PCBChunk = PCBChunk(
            "NW Mount References", [], [], references=nw_references)

        # Create and return the final *PCBChunk*'s:
        final_center_pcb_chunk: PCBChunk = PCBChunk.join(
            "Center Spacers", center_pcb_chunks + [center_references_pcb_chunk])
        final_ne_pcb_chunk: PCBChunk = PCBChunk.join(
            "NE Spacers", ne_pcb_chunks + [ne_references_pcb_chunk])
        final_nw_pcb_chunk: PCBChunk = PCBChunk.join(
            "NW Spacers", nw_pcb_chunks + [nw_references_pcb_chunk])
        return final_ne_pcb_chunk, final_nw_pcb_chunk, final_center_pcb_chunk


# MikroBus:
class MikroBus:
    """Represents a Mikrobus Module."""

    # Mikrobus.__init__():
    def __init__(self, scad_program: ScadProgram, connectors: Connectors, size: str) -> None:
        """Initialize a Mikrobus module."""
        # Dimensions from spec. sheet:
        notch_dx_dy: float = 2.54  # mm
        pcb_dx: float = 25.4  # mm
        pcb_dy_short: float = 28.6  # mm
        pcb_dy_medium: float = 42.9  # mm
        pcb_dy_long: float = 57.15  # mm
        pcb_dz: float = 1.6  # mm
        connector_pitch_dx: float = 22.86  # mm

        # Determine *pcb_dy*:
        pcb_dy_table: Dict[str, float] = {
            "S": pcb_dy_short,
            "M": pcb_dy_medium,
            "L": pcb_dy_long,
        }
        assert size in pcb_dy_table, f"Mikro bus {size} must be one of 'S', 'M', 'L'"
        pcb_dy: float = pcb_dy_table[size]

        # Create the *mikrobus_exterior*:
        pcb_east_x: float = pcb_dx / 2.0
        pcb_west_x: float = -pcb_east_x
        pcb_south_y: float = -4.0 * 2.54 - notch_dx_dy
        pcb_north_y: float = pcb_south_y + pcb_dy
        # Origin is in the center of the connectors:
        mikrobus_exterior: SimplePolygon = SimplePolygon(f"Mirkobus {size} Exterior", [
            P2D(pcb_east_x, pcb_north_y),
            P2D(pcb_west_x, pcb_north_y),
            P2D(pcb_west_x, pcb_south_y),
            P2D(pcb_east_x - notch_dx_dy, pcb_south_y),
            P2D(pcb_east_x, pcb_south_y + notch_dx_dy),
        ], lock=True)

        # Position the east/west connector chunks and create *microbus_pcb_chunk*:
        origin2d: P2D = P2D(0.0, 0.0)
        degrees90: float = pi / 2.0
        m1x8_west_pcb_chunk: PCBChunk = (
            connectors.m1x8.pcb_chunk.scads_x_flip().sides_swap().
            reposition(origin2d, degrees90, P2D(-connector_pitch_dx / 2.0, 0.0))
        )
        m1x8_east_pcb_chunk: PCBChunk = (
            connectors.m1x8.pcb_chunk.scads_x_flip().sides_swap().pads_rebase(8).
            reposition(origin2d, -degrees90, P2D(connector_pitch_dx / 2.0, 0.0))
        )
        mikrobus_pcb_chunk: PCBChunk = PCBChunk.join(f"MikroBus {size}", [
            m1x8_west_pcb_chunk,
            m1x8_east_pcb_chunk,
        ])

        # Generate *mikobus_module* for use by *microbus_mate_pcb_chunk*:
        mikrobus_module: Module3D = mikrobus_pcb_chunk.pcb_update(
            scad_program, origin2d, pcb_dz, mikrobus_exterior, "LightGreen", None, [])

        # Create the east/west female *PCBChunk*'s:
        f1x8_west_pcb_chunk: PCBChunk = (
            connectors.f1x8.pcb_chunk.
            reposition(origin2d, degrees90, P2D(-connector_pitch_dx / 2.0, 0.0))
        )
        f1x8_east_pcb_chunk: PCBChunk = (
            connectors.f1x8.pcb_chunk.pads_rebase(8).
            reposition(origin2d, -degrees90, P2D(connector_pitch_dx / 2.0, 0.0))
        )

        # Create *tranalsted_mikrobus_chunk* that is positioned just above the female connectors:
        translate_position: P3D = P3D(0.0, 0.0, 8.70)  # Trail and error
        translated_mikrobus_scad: Scad3D = Translate3D(
            "Translated MikroBus Scad", mikrobus_module.use_module_get(), translate_position)
        translated_mikrobus_pcb_chunk: PCBChunk = PCBChunk(
            f"Translated MikroBus {size}", [], [translated_mikrobus_scad])

        # Create the *tie_downs_pcb_chunk* that has the two tie down holes:
        tie_down_diameter: float = 2.5 + 0.05  # mm
        tie_down_pitch_dx: float = connector_pitch_dx + 2.54 + tie_down_diameter
        tie_down_y: float = -1.0 * 2.54
        east_tie_down_center: P2D = P2D(tie_down_pitch_dx / 2.0, tie_down_y)
        west_tie_down_center: P2D = P2D(-tie_down_pitch_dx / 2.0, tie_down_y)
        east_tie_down_pad: Pad = Pad(
            "East Tie Down", 0.0, 0.0, tie_down_diameter, east_tie_down_center)
        west_tie_down_pad: Pad = Pad(
            "West Tie Down", 0.0, 0.0, tie_down_diameter, west_tie_down_center)
        tie_down_pads: List[Pad] = [east_tie_down_pad, west_tie_down_pad]
        tie_downs_pcb_chunk: PCBChunk = PCBChunk("Tie Down Holes", tie_down_pads, [])

        # Create *mikrobus_mate_pcb_chunk*:
        mikrobus_mate_pcb_chunk: PCBChunk = PCBChunk.join(f"MikroBus {size} Mate", [
            f1x8_west_pcb_chunk,
            f1x8_east_pcb_chunk,
            tie_downs_pcb_chunk,
            translated_mikrobus_pcb_chunk,
        ])

        # Create the *mikrobus_mate_exterior* for the example board:
        pcb_dx_extra: float = 5.0
        mate_pcb_east_x: float = pcb_dx / 2.0 + pcb_dx_extra
        mate_pcb_west_x: float = -mate_pcb_east_x
        mate_pcb_south_y: float = -4.0 * 2.54 - notch_dx_dy
        mate_pcb_north_y: float = mate_pcb_south_y + pcb_dy
        # Origin is in the center of the connectors:
        mikrobus_mate_exterior: SimplePolygon = SimplePolygon(f"Mirkobus {size} Exterior", [
            P2D(mate_pcb_east_x, mate_pcb_north_y),
            P2D(mate_pcb_west_x, mate_pcb_north_y),
            P2D(mate_pcb_west_x, mate_pcb_south_y),
            P2D(mate_pcb_east_x - notch_dx_dy, mate_pcb_south_y),
            P2D(mate_pcb_east_x, mate_pcb_south_y + notch_dx_dy),
        ], lock=True)
        mikrobus_mate_module: Module3D = mikrobus_mate_pcb_chunk.pcb_update(
            scad_program, origin2d, pcb_dz, mikrobus_mate_exterior, "Violet", None, [])

        # Stuff some value into the mikrobus object (i.e. *self*):
        # mikrobus: Mikrobus = self
        self.mate_module: Module3D = mikrobus_mate_module
        self.mate_pcb_chunk: PCBChunk = mikrobus_mate_pcb_chunk
        self.module: Module3D = mikrobus_module
        self.pcb_chunk: PCBChunk = mikrobus_pcb_chunk


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
    def __init__(self, scad_program: ScadProgram, connectors: Connectors) -> None:
        """Initialize OtherPi."""
        # Define the board dimensions:
        pcb_dx: float = 104.00
        pcb_center_x: float = pcb_dx / 2
        pcb_dy: float = 70.00
        pcb_center_y: float = pcb_dy / 2
        # pcb_dz: float = 1.0
        pcb_center: P2D = P2D(pcb_center_x, pcb_center_y)
        pcb_corner_radius: float = 3.0

        # The dimensions read off the drawing assume that the origin is at the lower left corner.
        # When we are done we want *pcb_center* to point to the exact middle of the 4 mounting
        # holes.  Define all of the hole locations first:
        hole_east_x: float = 3.5
        hole_pitch_dx: float = 58.0
        hole_west_x: float = hole_east_x + hole_pitch_dx
        holes_center_x: float = (hole_east_x + hole_west_x) / 2.0
        hole_north_y: float = pcb_dy - 3.5
        hole_pitch_dy: float = 49.0
        hole_south_y: float = hole_north_y - hole_pitch_dy
        holes_center_y: float = (hole_south_y + hole_north_y) / 2.0
        holes_center: P2D = P2D(holes_center_x, holes_center_y)
        # The code below offsets by *holes_center* so that origin aligns with *holes_center*.

        # Define the 4 hole center locations:
        hole_ne: P2D = P2D(hole_east_x, hole_north_y)
        hole_nw: P2D = P2D(hole_west_x, hole_north_y)
        hole_se: P2D = P2D(hole_east_x, hole_south_y)
        hole_sw: P2D = P2D(hole_west_x, hole_south_y)

        # male_2x20_header_center = P3D(0.0, 0.0, 0.0)
        origin2d: P2D = P2D(0.0, 0.0)

        # Assemble all of the various connector *Scad*'s:
        connectors_origin: P3D = P3D(-holes_center.x, -holes_center.y, 0.0)
        connector_scads: List[Scad3D] = [
            Color("Silver Ethernet Connector",
                  Translate3D("Translated Ethernet Connector",
                              CornerCube("Ethernet Connector",
                                         P3D(80.00, 29.26, 0.0),
                                         P3D(105.00, 44.51, 13.08)),
                              connectors_origin),
                  "Silver"),
            Color("Silver USB2 Connector",
                  Translate3D("Translated USB2 Connector",
                              CornerCube("USB2 Connector",
                                         P3D(85.00, 12.82, 0.0),
                                         P3D(104.00, 25.57, 15.33)),
                              connectors_origin),
                  "Silver"),
            Color("Silver West Connector",
                  Translate3D("Translated West Connector",
                              CornerCube("West Connector",
                                         P3D(98.00, 50.69, 0.0),
                                         P3D(105.00, 58.09, 3.00)),
                              connectors_origin),
                  "Silver"),
            Color("Black North Connector",
                  Translate3D("Translated North Connector",
                              CornerCube("North Connector",
                                         P3D(70.03, 60.92, 0.0),
                                         P3D(76.38, 66.00, 5.00)),
                              connectors_origin),
                  "Black"),
            Color("Black Audio Connector",
                  Translate3D("Translated Audio Connector",
                              CornerCube("Audio Connector",
                                         P3D(66.31, -1.00, 0.0),
                                         P3D(72.31, 14.00, 5.12)),
                              connectors_origin),
                  "Black"),
            Color("Silver USB3A Connector",
                  Translate3D("Translated USB3A Connector",
                              CornerCube("USB3A Connector",
                                         P3D(9.54, 0.00, 0.0),
                                         P3D(18.52, 10.00, 2.95)),
                              connectors_origin),
                  "Silver"),
            Color("Silver USB3B Connector",
                  Translate3D("Translated USB3B Connector",
                              CornerCube("USB3B Connector",
                                         P3D(22.73, 0.00, 0.0),
                                         P3D(31.71, 10.00, 2.95)),
                              connectors_origin),
                  "Silver"),
            Color("Silver Power Connector",
                  Translate3D("Translated Power Connector",
                              CornerCube("Power Connector",
                                         P3D(-1.00, 22.50, 0.0),
                                         P3D(8.00, 29.00, 2.95)),
                              connectors_origin),
                  "Silver"),
            Color("White JST Connector",
                  Translate3D("Translated JST Connector",
                              CornerCube("JST Connector",
                                         P3D(0.00, 55.01, 0.0),
                                         P3D(3.00, 63.37, 2.95)),  # 0.00? 3.00?
                              connectors_origin),
                  "White"),
            Color("Black Buttons Area",
                  Translate3D("Translated Buttons Area",
                              CornerCube("Buttons Area",
                                         P3D(85.51, 62.12, 0.0),
                                         P3D(98.23, 68.74, 2.95)),  # 10.00?
                              connectors_origin),
                  "Black"),
        ]  # End *connector_scads*

        # other_pi_scads_mounts_pcb_chunk
        pads: List[Pad] = [
            Pad("NE Mount Hole", 0.0, 0.0, 2.7, hole_ne - holes_center, 0.0),
            Pad("NW Mount Hole", 0.0, 0.0, 2.7, hole_nw - holes_center, 0.0),
            Pad("SE Mount Hole", 0.0, 0.0, 2.7, hole_se - holes_center, 0.0),
            Pad("SW Mount Hole", 0.0, 0.0, 2.7, hole_sw - holes_center, 0.0),
        ]
        other_pi_mount_holes_scads_pcb_chunk = PCBChunk("", pads, connector_scads)
        other_pi_m2x20_pcb_chunk: PCBChunk = connectors.m2x20.pcb_chunk.reposition(
            origin2d, 0.0, P2D(0.0, 24.5))

        # Do the *PCBChunk* model.
        other_pi_pcb_chunk: PCBChunk = PCBChunk.join("SBC Other Pi", [
            other_pi_m2x20_pcb_chunk,
            other_pi_mount_holes_scads_pcb_chunk,
        ])
        square_center: P2D = pcb_center - holes_center
        other_pi_exterior: Square = Square("PCB Exterior", pcb_dx, pcb_dy,
                                           center=square_center, corner_radius=pcb_corner_radius)
        other_pi_pcb_xmodule: Module3D = other_pi_pcb_chunk.pcb_update(
            scad_program, origin2d, 1.6, other_pi_exterior, "Green", None, [])
        other_pi_pcb_xmodule = other_pi_pcb_xmodule

        # Stuff some values into *other_pi* (i.e. *self*):
        # other_pi: OtherPi = self
        self.module: Module3D = other_pi_pcb_xmodule
        # self.pcb: PCB = other_pi_pcb
        self.pcb_chunk: PCBChunk = other_pi_pcb_chunk


# RaspberryPi4:
class RaspberryPi4:
    """Represents a Raspberry Pi 4B+."""

    # RaspberryPi4.__init__():
    def __init__(self, scad_program: ScadProgram, connectors: Connectors, other_pi: OtherPi,
                 pcb_origin: P2D) -> None:
        """Initialize Raspberrypi4 and append to ScadProgram."""
        # Define the board dimensions:
        pcb_dx: float = 85.00
        pcb_center_x: float = pcb_dx / 2
        pcb_dy: float = 56.00
        pcb_center_y: float = pcb_dy / 2
        pcb_dz: float = 1.6
        pcb_center: P2D = P2D(pcb_center_x, pcb_center_y)
        pcb_corner_radius: float = 3.0

        # The dimensions read off the drawing assume that the origin is at the lower left corner.
        # When we are done we want *pcb_center* to point to the exact middle of the 4 mounting
        # holes.  Define all of the hole locations first:
        hole_east_x: float = 3.5
        hole_pitch_dx: float = 58.0
        hole_west_x: float = hole_east_x + hole_pitch_dx
        holes_center_x: float = (hole_east_x + hole_west_x) / 2.0
        hole_south_y: float = 3.5
        hole_pitch_dy: float = 49.0  # Note: Is equal to *pcb_dy* - 2 * 3.5 = 56 - 7 = 49
        hole_north_y: float = hole_south_y + hole_pitch_dy
        holes_center_y: float = (hole_south_y + hole_north_y) / 2.0
        # Note that *pcb_center_y* is equal to *holes_center_y*:
        holes_center: P2D = P2D(holes_center_x, holes_center_y)
        # All of the code below offsets by *holes_center* so that origin aligns with *holes_center*.

        # Define the 4 hole center locations:
        hole_ne: P2D = P2D(hole_east_x, hole_north_y)
        hole_nw: P2D = P2D(hole_west_x, hole_north_y)
        hole_se: P2D = P2D(hole_east_x, hole_south_y)
        hole_sw: P2D = P2D(hole_west_x, hole_south_y)

        # The *Sqaure* class takes a *center* argument that specifies where the center
        # of the square is to be located in the X/Y plane.  We need to compute *square_center*
        # such that square is offset so that the holes center is at the origin (0, 0):
        # Create the *pcb_outline* with the origin in the lower left corner:
        square_center: P2D = pcb_center - holes_center
        raspi4b_exterior: Square = Square("PCB Exterior", pcb_dx, pcb_dy,
                                          center=square_center, corner_radius=pcb_corner_radius)

        # Create the *raspib_pcb*:
        # raspi4b_pcb: PCB = PCB("Raspi4B", scad_program, pcb_dz, raspi4b_exterior)

        # Create the 4 mounting holes (i.e *mount_pads*):
        hole_diameter: float = 2.75
        mount_pads: List[Pad] = []
        # raspi4b_pcb.mount_hole_append("NE Mount Hole", {"mounts"},
        #                              hole_diameter, hole_ne - holes_center)
        mount_pads.append(Pad("NE Mount_hole", 0.0, 0.0, hole_diameter, hole_ne - holes_center))
        # raspi4b_pcb.mount_hole_append("NW Mount Hole", {"mounts"},
        #                               hole_diameter, hole_nw - holes_center)
        mount_pads.append(Pad("NW Mount_hole", 0.0, 0.0, hole_diameter, hole_nw - holes_center))
        # raspi4b_pcb.mount_hole_append("SE Mount Hole", {"mounts"},
        #                               hole_diameter, hole_se - holes_center)
        mount_pads.append(Pad("SE Mount_hole", 0.0, 0.0, hole_diameter, hole_se - holes_center))
        # raspi4b_pcb.mount_hole_append("SW Mount Hole", {"mounts"},
        #                               hole_diameter, hole_sw - holes_center)
        mount_pads.append(Pad("SW Mount_hole", 0.0, 0.0, hole_diameter, hole_sw - holes_center))

        # Install the header connectors:
        origin2d: P2D = P2D(0.0, 0.0)
        # The main Pi 2x20 header:
        connector2x20_center: P2D = P2D((57.90 + 7.10) / 2.0, 52.50)
        connector2x2_center: P2D = P2D((58.918 + 64.087) / 2.0, (44.005 + 48.727) / 2.0)

        # The nominal selected heat sinks are the Seeed Studio 110991327 which contains
        # 4 heat sinks for raspberry pi 4.  The Digi-Key Part number is 1597-110991327-ND.
        # The numbers below are all origined in the lower left corner of the board.
        #
        # The following values were read off the Raspberry Pi 4B `.dxf` file.
        #
        # Heat sinks X x Y (fins) Rasp .dxf min/max X/Y
        # 14.30 x 14.10 (7 fins)  X:21.75,36.75     Y:24.9,40.1  Processor
        # 14.30 x 9.90 (7 fins)   X:38.95,50.65     Y:24.9,40.1  SDRAM
        # 8.80 x 8.90 (5 fins)    X:55.5, 61.5      Y:35,41      Ethernet
        # 8.80 x 8.90 (f fins)    X:55.1, 63.5      Y:20,28      USB

        # Find the heat sink centers with the origin in lower left corner of Raspberry Pi 4 board:
        holes_center3d: P3D = P3D(holes_center.x, holes_center.y, 0)
        processor_ne2d: P2D = P2D(36.75, 40.1)
        processor_sw2d: P2D = P2D(21.75, 24.9)
        processor_center2d: P2D = (processor_sw2d + processor_ne2d) / 2.0
        processor_center3d: P3D = (P3D(processor_center2d.x, processor_center2d.y, 0.0) -
                                   holes_center3d)
        sdram_ne2d: P2D = P2D(50.65, 40.1)
        sdram_sw2d: P2D = P2D(38.95, 24.9)
        sdram_center2d: P2D = (sdram_sw2d + sdram_ne2d) / 2.0
        sdram_center3d: P3D = P3D(sdram_center2d.x, sdram_center2d.y, 0.0) - holes_center3d
        ethernet_ne2d: P2D = P2D(61.5, 41.0)
        ethernet_sw2d: P2D = P2D(55.5, 35.0)
        ethernet_center2d: P2D = (ethernet_sw2d + ethernet_ne2d) / 2.0
        ethernet_center3d: P3D = P3D(ethernet_center2d.x, ethernet_center2d.y, 0.0) - holes_center3d
        usb_ne2d: P2D = P2D(63.5, 28.0)
        usb_sw2d: P2D = P2D(55.5, 20.0)
        usb_center2d: P2D = (usb_sw2d + usb_ne2d) / 2.0
        usb_center3d: P3D = P3D(usb_center2d.x, usb_center2d.y, 0.0) - holes_center3d

        # Construct the heat sinks:
        heat_sink_dz: float = 8.0
        heat_sink_base_dz: float = 2.5
        fin_dx: float = 1.0
        processor_heat_sink: HeatSink = HeatSink(
            scad_program, "RPi4 Processor Heat Sink",
            14.30, 14.10, heat_sink_dz, heat_sink_base_dz, fin_dx, 7, False, "Silver")
        sdram_heat_sink: HeatSink = HeatSink(
            scad_program, "RPi4 SDRam Heat Sink",
            14.30, 9.90, heat_sink_dz, heat_sink_base_dz, fin_dx, 7, True, "Silver")
        ethernet_heat_sink: HeatSink = HeatSink(
            scad_program, "RPi4 Ethernet Heat Sink",
            8.80, 8.90, heat_sink_dz, heat_sink_base_dz, fin_dx, 5, False, "Silver")
        usb_heat_sink: HeatSink = HeatSink(
            scad_program, "RPi4 USB Heat Sink",
            8.80, 8.90, heat_sink_dz, heat_sink_base_dz, fin_dx, 5, True, "Silver")

        origin3d: P3D = P3D(0.0, 0.0, 0.0)
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        repositioned_processor_heat_sink: Scad3D = (
            processor_heat_sink.module.use_module_get().
            reposition("Repositioned RPi4 Processor Heat Sink",
                       origin3d, z_axis, 0.0, processor_center3d))
        repositioned_sdram_heat_sink: Scad3D = (
            sdram_heat_sink.module.use_module_get().
            reposition("Repositioned RPi4 SDRAM Heat Sink",
                       origin3d, z_axis, 0.0, sdram_center3d))
        repositioned_ethernet_heat_sink: Scad3D = (
            ethernet_heat_sink.module.use_module_get().
            reposition("Repositioned RPi4 Ethernet Heat Sink",
                       origin3d, z_axis, 0.0, ethernet_center3d))
        repositioned_usb_heat_sink: Scad3D = (
            usb_heat_sink.module.use_module_get().
            reposition("Repositioned RPi4 UBB Heat Sink",
                       origin3d, z_axis, 0.0, usb_center3d))

        heatsinks_union: Union3D = Union3D("Heatsinks", [
            repositioned_processor_heat_sink,
            repositioned_sdram_heat_sink,
            repositioned_ethernet_heat_sink,
            repositioned_usb_heat_sink])
        heatsinks_union = heatsinks_union

        # Now place various cubes to represent the other connectors:
        connectors_center3d: P3D = holes_center3d + P3D(0.0, 0.0, pcb_dz)
        raspi4b_connectors: List[Scad3D] = [
            Color("Silver RJ45 Connector",
                  CornerCube("RJ45 Connecttor",
                             P3D(65.650, 2.495, pcb_dz + 0.000) - connectors_center3d,
                             P3D(87.000, 18.005, pcb_dz + 13.500) - connectors_center3d),
                  "Silver"),
            Color("Silver Lower USB2",
                  CornerCube("Lower USB2",
                             P3D(69.30, 22.43, pcb_dz + 0.00) - connectors_center3d,
                             P3D(87.00, 34.57, pcb_dz + 16.00) - connectors_center3d),
                  "Silver"),
            Color("Silver Upper USB2",
                  CornerCube("Upper USB2",
                             P3D(69.30, 40.43, pcb_dz + 0.00) - connectors_center3d,
                             P3D(87.00, 53.57, pcb_dz + 16.00) - connectors_center3d),
                  "Silver"),
            Color("Black Camera Connector",
                  CornerCube("Camera Connector",
                             P3D(43.55, 0.30, pcb_dz + 0.00) - connectors_center3d,
                             P3D(47.50, 22.70, pcb_dz + 5.50) - connectors_center3d),
                  "Black"),
            Color("Silver HDMI Connector",
                  CornerCube("HDMI Connector",
                             P3D(24.75, -1.50, pcb_dz + 0.00) - connectors_center3d,
                             P3D(39.25, 10.65, pcb_dz + 6.50) - connectors_center3d),
                  "Silver"),
            Color("Silver Power Connector",
                  CornerCube("Power Connector",
                             P3D(6.58, -1.22, pcb_dz + 0.00) - connectors_center3d,
                             P3D(14.62, 14.35, pcb_dz + 2.00) - connectors_center3d),
                  "Silver"),
            Color("Black LCD Connector",
                  CornerCube("LCD Connector",
                             P3D(2.65, 16.80, pcb_dz + 0.00) - connectors_center3d,
                             P3D(5.45, 39.20, pcb_dz + 5.50) - connectors_center3d),
                  "Black"),
        ]

        # Position the pin connectors:
        connectors_pcb_chunk = PCBChunk("Raspi4 Connectors", [], raspi4b_connectors)
        m2x20_pcb_chunk: PCBChunk = connectors.m2x20.pcb_chunk.reposition(
            origin2d, 0.0, connector2x20_center - holes_center)
        m2x2_pcb_chunk: PCBChunk = connectors.m2x2.pcb_chunk.reposition(
            origin2d, 0.0, connector2x2_center - holes_center)

        # Now do the *PCBChunk* version of this board.  There is *raspi3b_pcb_chunk*
        # which represents the RasPi4B+ PCB and there is *raspi3b_pcb_mate_chunk*
        # which represents the mating holes and connectors for the RasPi4B+:
        heat_sinks_pcb_chunk: PCBChunk = PCBChunk("xraspi4b", mount_pads, [heatsinks_union])
        raspi4b_pcb_chunk: PCBChunk = PCBChunk.join("SBC RasPi4", [
            connectors_pcb_chunk,
            m2x20_pcb_chunk,
            m2x2_pcb_chunk,
            heat_sinks_pcb_chunk])
        raspi4b_module: Module3D = raspi4b_pcb_chunk.pcb_update(
            scad_program, origin2d, 1.6, raspi4b_exterior, "Green", None, [])
        raspi4b_use_module: UseModule3D = raspi4b_module.use_module_get()

        # Now create the *rasp4b_mate_pcb_chunk* out of smaller chunks.
        # Start with the *raspi4b_f2x20_pcb_chunk*:
        raspi4b_f2x20_pcb_chunk: PCBChunk = (connectors.f2x20.pcb_chunk.scads_x_flip().sides_swap().
                                             reposition(origin2d, 0.0,
                                                        P2D(0.0, hole_pitch_dy / 2.0)))

        # To help visualize everything, create *mating_raspi4b_pcb_chunk* that mates with the f1x20:
        pi_z_offset: float = -12.10  # mm
        translated_rasp4b: Translate3D = Translate3D("Translated Rasp4B Board",
                                                     raspi4b_use_module,
                                                     P3D(0.0, 0.0, pi_z_offset))
        mating_rasp4b_pcb_chunk: PCBChunk = PCBChunk("Mating Rasp4B Board", [], [translated_rasp4b])

        # Likewise for the *other_pi*:
        translated_other_pi: Translate3D = Translate3D("Translated Other Pi Board",
                                                       other_pi.module.use_module_get(),
                                                       P3D(0.0, 0.0, pi_z_offset))
        mating_other_pi_pcb_chunk: PCBChunk = PCBChunk(
            "Mating OtherPi Board", [], [translated_other_pi])

        # Create each of the slots for the heat sinks, camera connector, and LCD connector:

        # Create the *camera_slot* connector:
        camera_sw: P2D = P2D(43.55, 0.30) - holes_center
        camera_ne: P2D = P2D(47.50, 22.70) - holes_center
        camera_center: P2D = (camera_sw + camera_ne) / 2.0
        camera_dx: float = abs(camera_ne.x - camera_sw.x)
        camera_dy: float = abs(camera_ne.y - camera_sw.y) - 2.0  # Shave off a little...
        camera_slot: SimplePolygon = Square(
            "Raspi4 Camera Slot", camera_dx, camera_dy, camera_center,
            corner_radius=1.0, corner_count=5)

        # Create the *lcd_slot*:
        lcd_sw: P2D = P2D(2.65, 16.80) - holes_center
        lcd_ne: P2D = P2D(5.45, 39.20) - holes_center
        lcd_center: P2D = (lcd_sw + lcd_ne) / 2.0 + P2D(3.5, 0.0)  # Offset for Nucleo connector
        lcd_dx: float = abs(lcd_ne.x - lcd_sw.x)
        lcd_dy: float = abs(lcd_ne.y - lcd_sw.y)
        lcd_slot: SimplePolygon = Square(
            "LCD Panel Slot", lcd_dx, lcd_dy, lcd_center, corner_radius=1.0, corner_count=5)

        # Create the *processor_slot*:
        processor_dx: float = abs(processor_ne2d.x - processor_sw2d.x)
        processor_dy: float = abs(processor_ne2d.y - processor_sw2d.y)
        processor_center: P2D = processor_center2d - holes_center
        processor_slot: SimplePolygon = Square(
            "Processor Heat Sink Slot", processor_dx, processor_dy, processor_center,
            corner_radius=1.0, corner_count=5)

        # Create the *sdram_slot*:
        sdram_dx: float = abs(sdram_ne2d.x - sdram_sw2d.x)
        sdram_dy: float = abs(sdram_ne2d.y - sdram_sw2d.y)
        sdram_center: P2D = sdram_center2d - holes_center
        sdram_slot: SimplePolygon = Square(
            "Sdram Heat Sink Slot", sdram_dx, sdram_dy, sdram_center,
            corner_radius=1.0, corner_count=5)

        # Create the *ethernet_slot*:
        ethernet_dx: float = abs(ethernet_ne2d.x - ethernet_sw2d.x)
        ethernet_dy: float = abs(ethernet_ne2d.y - ethernet_sw2d.y)
        ethernet_center: P2D = ethernet_center2d - holes_center
        ethernet_slot: SimplePolygon = Square(
            "Ethernet Heat Sink Slot", ethernet_dx, ethernet_dy, ethernet_center,
            corner_radius=1.0, corner_count=5)
        ethernet_slot = ethernet_slot

        # Create the *usb_slot*:
        usb_dx: float = abs(usb_ne2d.x - usb_sw2d.x)
        usb_dy: float = abs(usb_ne2d.y - usb_sw2d.y)
        usb_center: P2D = usb_center2d - holes_center
        usb_slot: SimplePolygon = Square(
            "Usb Heat Sink Slot", usb_dx, usb_dy, usb_center,
            corner_radius=1.0, corner_count=5)
        usb_slot = usb_slot  # Ignore for now.

        # Combined Ethernet/USB heat slot determined by trial and error.
        ethernet_usb_dx: float = 6.0
        ethernet_usb_dy: float = 22.0
        ethernet_usb_center: P2D = P2D(23.0, 2.5)  # (Y, -X) in master board coordinate state
        ethernet_usb_slot: SimplePolygon = Square(
            "Ethernet USB Heat Sink Slot", ethernet_usb_dx, ethernet_usb_dy, ethernet_usb_center,
            corner_radius=1.0, corner_count=5)

        # Now create the *raspi4b_slots_pcb_chunk*:
        cuts: List[SimplePolygon] = [
            camera_slot,
            lcd_slot,
            processor_slot,
            sdram_slot,
            # ethernet_slot,
            # usb_slot,
            ethernet_usb_slot,
        ]
        raspi4b_slots_pcb_chunk: PCBChunk = PCBChunk("Raspi4B Slots", [], [], cuts=cuts)

        raspi4b_mate_pcb_chunk: PCBChunk = PCBChunk.join("RASPI_F2X20", [
            raspi4b_slots_pcb_chunk,
            mating_rasp4b_pcb_chunk,
            mating_other_pi_pcb_chunk,
            raspi4b_f2x20_pcb_chunk,
        ])

        hr2_directory: Path = Path(os.environ["HR2_DIRECTORY"])
        master_board_directory: Path = hr2_directory / "electrical" / "master_board" / "rev_a"
        master_board_pretty_directory: Path = master_board_directory / "pretty"
        raspi4b_mate_pcb_chunk.footprint_generate("HR2", master_board_pretty_directory)
        raspi4b_mate_module: Module3D = raspi4b_mate_pcb_chunk.pcb_update(
             scad_program, pcb_origin, 1.6, raspi4b_exterior, "Green", None, [])
        raspi4b_mate_module = raspi4b_mate_module

        # Stuff some values into into *raspi4b* (i.e. *self*):
        # raspi4b: Raspberrypi4 = self
        # self.module: Module3D = raspi4b_module
        self.module: Module3D = raspi4b_module
        # self.pcb: PCB = raspi4b_pcb
        self.mate_pcb_chunk: PCBChunk = raspi4b_mate_pcb_chunk
        # TODO: add the Camera and LCD connector locations:


# RectangularConnector:
class RectangularConnector:
    """RectangularConnector represents an NxM connector."""

    # Note:
    # This code is now needlessly complicated.  Is supports all sorts of transforms that
    # are no longer used.  Really all it needs to do is produce male/female straight/right angle
    # rectangular connectors centered on the origin.  All transforms are now performed
    # down stream by the *PCBChunk* module.

    # RectangularConnector.__init__():
    def __init__(self, name: str, scad_program: ScadProgram, rows: int, columns: int,
                 insulation_height: float, pcb_pin_height: float, male_pin_height: float = 0.0,
                 center: P3D = P3D(0.0, 0.0, 0.0),
                 rows_pitch: float = 2.54, columns_pitch: float = 2.54,
                 is_top: bool = True,
                 vertical_rotate: float = 0.0,
                 pin_dx_dy: float = 0.640,  # 0.640mm = 0.25in,
                 rows_extra_insulation: float = 0.000,
                 columns_extra_insulation: float = 0.000,
                 right_angle_length: float = 0.000,
                 cut_out: bool = False,
                 pcb_polygon: Optional[Polygon] = None, pcb_hole_diameter: float = 0.0,
                 insulation_color: str = "Black", pin_color: str = "Gold",
                 # footprint: Footprint = Footprint("", ""),  # Old functionality
                 footprint_pin_number: int = 1,
                 footprint_pad_diameter: float = 0.0, footprint_drill_diameter: float = 0.0,
                 footprint_flags: str = "",
                 artwork: bool = True,
                 tracing: str = "") -> None:
        """Initialize RectangularConnector and append to ScadProgram.

        Create a rectangular mail header with through hole PCB pins.
        The center of the resulting header is at the center of the
        header at the surface of the PCB.  All lengths are specified
        in millimeters.

        Args:
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
            *footprint* (*Footprint*):
                (Optional: defaults to an footprint with an empty name)
                Specifies a footprint to draw pads into.
            *footprint_pin_number* (*int*): (Optional: defaults to 1)
                Specifies the starting pin number for a footprint.
            *footprint_pad_diameter* (*float*): (Optional: defaults to 0.0)
                Specifies the footprint pad diameter.
            *footprint_pad_drill* (*float*): (Optional: defaults to 0.0):
                Specifies the through hole drill diameter.
            *footprint_flags* (*str*): (Optional: defaults to ""):
                Specifies and flags needed to adjust the footprint output (TBD.)
            *artwork* (*bool*):
                (Optional: defaults to *False*)
                Specifies whether to supply artwork for the connector.

        """
        # Perform any requested *tracing*:
        next_tracing: str = tracing + " " if tracing else ""
        if tracing:
            print(f"{tracing}=>RectangularConnector('{name}', R:{rows}, C:{columns} "
                  f"VR:{degrees(vertical_rotate)})")

        # Stuff all of the values into *rectangular_connector* (i.e. *self*):
        # rectangular_connector: RectangularConnector = self
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
        # self.footprint: Footprint = footprint
        self.footprint_pin_number: int = footprint_pin_number
        self.footprint_pad_diameter: float = footprint_pad_diameter
        self.footprint_drill_diameter: float = footprint_drill_diameter
        self.footprint_flags: str = footprint_flags

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
        pads_group: PadsGroup = PadsGroup()
        pads: List[Pad] = []

        # Create each *colored_pin* (and optional *colored_right_angle_pin*) and
        # append to *connector_pins*.
        if pcb_hole_diameter == 0:
            pin_diagonal: float = sqrt(2.0 * pin_dx_dy * pin_dx_dy)
            pin_hole_extra: float = 0.100  # mm
            pcb_hole_diameter = pin_diagonal + pin_hole_extra
        row_start_y: float = -row_pins_dy / 2.0
        column_start_x: float = -column_pins_dx / 2.0
        connector_pins: List[Scad3D] = []
        first_footprint_pin: P2D
        last_footprint_pin: P2D
        row_index: int
        for row_index in range(rows):
            column_index: int
            y: float = row_start_y + row_index * rows_pitch
            for column_index in range(columns):
                # Create the *colored_vertical_pin* and append to *connector_pins*:
                x: float = column_start_x + column_index * columns_pitch

                # Compute put the pin corners:
                pin_bsw: P3D = P3D(x - half_pin_dx_dy, y - half_pin_dx_dy, -pcb_pin_height)
                pin_tne: P3D = P3D(x + half_pin_dx_dy, y + half_pin_dx_dy, pin_above_dz)
                vertical_pin_corner_cube: CornerCube = CornerCube(f"Pin {full_name} "
                                                                  f"({column_index}:{row_index})"
                                                                  "Corner Cube",
                                                                  pin_bsw, pin_tne)
                # colored_vertical_pin: Color = Color(f"{full_name} ({column_index},{row_index}) "
                #                                     f"{pin_color} Pin",
                #                                     vertical_pin_corner_cube, pin_color)
                # connector_pins.append(colored_vertical_pin)
                connector_pins.append(vertical_pin_corner_cube)

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
                    # colored_right_angle_pin: Color = Color(f"{full_name} "
                    #                                        f"({column_index}:{row_index})"
                    #                                        f"{pin_color} Right Angle Pin",
                    #                                        right_angle_pin_corner_cube,
                    #                                       pin_color)
                    # connector_pins.append(colored_right_angle_pin)
                    connector_pins.append(right_angle_pin_corner_cube)

                # For female receptacles, append a *receptcale_hole* to *insulation_polyon*:
                if is_female:
                    receptacle_hole: Square = Square(f"{full_name} "
                                                     f"({column_index}:{row_index})"
                                                     "Receptacle Hole", pin_dx_dy, pin_dx_dy,
                                                     center=P2D(x, y))
                    insulation_polygon.append(receptacle_hole)

                # Do any any needed PCB holes preforming any *vertical_rotate*:
                hole_rotate: float = vertical_rotate
                # Rotation of a point around the origin:
                # https://en.wikipedia.org/wiki/Rotation_(mathematics):
                # x' = x * cos(theta) - y * sin(theta)
                # y' = x * sin(theta) + y * cos(theta)
                cos_hole_rotate: float = cos(hole_rotate)
                sin_hole_rotate: float = sin(hole_rotate)
                rotated_x: float = x * cos_hole_rotate - y * sin_hole_rotate
                rotated_y: float = x * sin_hole_rotate + y * cos_hole_rotate
                hole_center: P2D = P2D(center_x + rotated_x,
                                       center_y + rotated_y)
                if pcb_polygon is not None:
                    hole: Circle = Circle(f"{full_name} ({column_index}:{row_index}) PCB Hole",
                                          pcb_hole_diameter, 8,
                                          center=hole_center)
                    pcb_polygon.append(hole)

                # ---  69 70
                # --+  36 01
                # -+-  01 02 <===
                # +++  35 70

                # Output a footprint pin number:
                # if footprint.name_get() != "":
                if True:
                    if row_index == 0 and column_index == 0:
                        first_footprint_pin = hole_center
                    else:
                        last_footprint_pin = hole_center
                    # swap_row_index: bool = False
                    # swap_column_index: bool = True
                    # transpose_rows_columns: bool = False
                    # footprint_row_index: int = (rows - row_index - 1
                    #                             if swap_row_index else row_index)
                    # footprint_column_index: int = (columns - column_index - 1
                    #                                if swap_column_index else column_index)
                    #                                if swap_column_index else column_index)
                    pin_number: int = (footprint_pin_number +
                                       column_index + 1 + row_index * columns)
                    # if transpose_rows_columns:
                    #    assert False
                    #    pin_number = footprint_pin_number + (footprint_row_index * columns +
                    #                                         columns - footprint_column_index - 1)
                    # else:
                    pin_number = 1 + column_index + row_index * columns
                    pad: Pad = Pad(f"{pin_number}", footprint_pad_diameter, footprint_pad_diameter,
                                   footprint_drill_diameter, hole_center, tracing=next_tracing)
                    pads_group.insert(pad)
                    pads.append(pad)
                    # footprint.thru_hole_pad(f"{pin_number}", hole_center,
                    #                         footprint_pad_diameter, footprint_drill_diameter)

        # Write out an artwork rectangle around the connector:
        front_artworks: List[SimplePolygon] = []
        # if footprint.name_get() != "":
        if artwork:
            x1: float = first_footprint_pin.x
            y1: float = first_footprint_pin.y
            x2: float = last_footprint_pin.x
            y2: float = last_footprint_pin.y
            columns_extra: float = (columns_pitch + columns_extra_insulation) / 2.0
            rows_extra: float = (rows_pitch + rows_extra_insulation) / 2.0
            x_maximum: float = max(x1, x2) + columns_extra
            x_minimum: float = min(x1, x2) - columns_extra
            y_maximum: float = max(y1, y2) + rows_extra
            y_minimum: float = min(y1, y2) - rows_extra
            # corner1: P2D = P2D(x_minimum, y_minimum)
            # corner2: P2D = P2D(x_maximum, y_maximum)
            # footprint.rectangle(corner1, corner2, "F.SilkS", 0.2)
            dx: float = abs(x_maximum - x_minimum)
            dy: float = abs(y_maximum - y_minimum)
            x_center: float = (x_maximum + x_minimum) / 2.0
            y_center: float = (y_maximum + y_minimum) / 2.0
            # original_y_center: float = y_center
            if have_right_angle:
                y_center -= right_angle_length
            square: Square = Square("name", dx, dy, P2D(x_center, y_center))
            front_artworks.append(square)
            circle: Circle
            if have_right_angle:
                column: int
                first_pin_x: float = -float(columns - 1) * columns_pitch / 2.0
                for column in range(columns):
                    pin_x: float = first_pin_x + float(column) * columns_pitch
                    line: SimplePolygon = SimplePolygon(
                        f"{name}:{column} Artwork Pin",
                        [P2D(pin_x, min(y1, y2) - footprint_pad_diameter / 2.0),
                         P2D(pin_x, y_center + dy / 2.0)])
                    front_artworks.append(line)
                circle = Circle("{name} Pin 1 Circle",  0.2, 8,
                                P2D(x1 - 0.75 * columns_pitch, max(y1, y2) - 0.50 * rows_pitch))
            else:
                # Draw the pin1 circle:
                circle = Circle("{name} Pin 1 Circle",  0.2, 8,
                                P2D(x1 - 0.50 * columns_pitch, max(y1, y2) + 0.75 * rows_pitch))
            front_artworks.append(circle)

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
        union_connector_pins: Union3D = Union3D(f"{full_name} Connector Pins Union",
                                                connector_pins, lock=True)
        colored_connector_pins: Color = Color(f"{full_name} {pin_color} Connector Pins",
                                              union_connector_pins, pin_color)

        union_connector: Union3D = Union3D(f"{full_name} Union",
                                           [colored_insulation] + [colored_connector_pins],
                                           lock=True)

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

        # Construct *connector_module*, append to *scad_program*:
        connector_module: Module3D = Module3D(name, [recentered_connector])
        pcb_chunk: PCBChunk = PCBChunk(name, pads, [recentered_connector],
                                       front_artworks=front_artworks)
        connector_module.tag_insert("PadsGroup", pads_group)
        scad_program.append(connector_module)
        self.module: Module3D = connector_module
        self.pcb_chunk: PCBChunk = pcb_chunk

        # Wrap up any requested *tracing*:
        if tracing:
            print(f"{tracing}=>RectangularConnector('{name}', R:{rows}, C:{columns})"
                  f"VR:{degrees(vertical_rotate)})")


# RomiBase:
class RomiBase:
    """Represents the Romi Chasis Base."""

    # RomiBase.__init__():
    def __init__(self, scad_program: ScadProgram, base_dxf: BaseDXF) -> None:
        """Initialize Romi and append to ScadProgram."""
        # Set *debugging* to *True* to print out debugging messages:
        debugging: bool = False  # True

        # Grab some Z values (X values are arbtrarily set to 0.0). We use the Y coordiante
        # which is actually the Z value:
        base_bottom_z: float = base_dxf.z_locate(-3.469098)
        battery_top_z: float = base_dxf.z_locate(-2.701374)
        base_top_z: float = base_dxf.z_locate(-3.095083)
        battery_dz: float = abs(battery_top_z - base_bottom_z)
        base_dz: float = abs(base_top_z - base_bottom_z)

        # Load some values into *romi_base* (i.e. *self*):
        romi_base: RomiBase = self
        self.debugging = debugging
        self.base_dxf: BaseDXF = base_dxf
        self.battery_dz: float = battery_dz
        self.base_dz: float = base_dz

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
        outline_polygon.arc_append(origin, radius, upper_start_angle, upper_end_angle, 0.0)

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
        outline_polygon.arc_append(origin, radius, lower_start_angle, lower_end_angle, 0.0)

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
            battery_clip.arc_append(arc_center, radius, start_angle, end_angle, 0.0)

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

    # RomiBase.spacer_positions_get():
    def spacer_positions_get(self) -> Dict[str, Tuple[P2D, str, float]]:
        """Generate a table of spacer positions."""
        # Grab some values from *romi_base* (i.e. *self*):
        romi_base: RomiBase = self
        romi_base_keys: List[Tuple[Any, ...]] = romi_base.keys_get()
        battery_dz: float = romi_base.battery_dz

        # Create *romi_base_keys_table* to lookup key tuples by key name:
        romi_base_key_table: Dict[str, Tuple[Any, ...]] = {}
        romi_base_key: Tuple[Any, ...]
        for romi_base_key in romi_base_keys:
            name: str = romi_base_key[1]
            romi_base_key_table[name] = romi_base_key

        # Find holes that are close to a named hole:
        def find(anchor_name: str, base_key_table: Dict[str, Tuple[Any, ...]]) -> None:
            """Find some holes that close."""
            anchor_key: Tuple[Any, ...] = base_key_table[anchor_name]
            anchor_position: P2D = P2D(float(anchor_key[2]), float(anchor_key[3]))
            distance_tuples: List[Tuple[float, str, float, float]] = []
            other_key: Tuple[Any, ...]
            distance_tuple: Tuple[float, str, float, float]
            distance: float
            name: str
            x: float
            y: float
            for other_key in base_key_table.values():
                assert isinstance(other_key[1], str)
                name = other_key[1]
                x = float(other_key[2])
                y = float(other_key[3])
                other_position: P2D = P2D(x, y)
                distance = anchor_position.distance(other_position)
                distance_tuple = (distance, name, x, y)
                distance_tuples.append(distance_tuple)
            distance_tuples = sorted(distance_tuples)
            index: int
            for index, distance_tuple in enumerate(distance_tuples[:15]):
                distance, name, x, y = distance_tuple
                print(f"[{index}]: {distance:.2f} '{name}' {x:.2f} {y:.2f}")
            print("")

        # Define the *spacer_tuples* list which is a list of 3-tuples of the spacer name (*str*),
        # key name (*str*), and base_level (*float*).
        base_dz: float = 0.0
        spacer_tuples: List[Tuple[str, str, str, float]] = [
            ("Pi NE", "BATTERY: Upper Slot (7, 1)", "H1", battery_dz),
            ("Pi NW", "BATTERY: Upper Slot (2, 1)", "H2", battery_dz),
            ("Pi SE", "RIGHT: LOWER Small Hex Slot (3, 0)", "H3", base_dz),
            ("Pi SW", "LEFT: LOWER Small Hex Slot (3, 0)", "H4", base_dz),
            ("Master NE", "BATTERY: Upper Hole (9, 2)", "H5", base_dz),
            ("Master NE", "RIGHT: Upper hole 0", "H5", base_dz),
            ("Master NW", "LEFT: Upper hole 0", "H6", base_dz),
            # ("MasterBoard SE", "RIGHT: Vector Hole 8", "H7", base_dz),
            ("MasterBoard SE", "RIGHT: Lower hole 0", "H7", base_dz),
            ("MasterBoard SW", "LEFT: Lower hole 0", "H8", base_dz),
            # ("MasterBoard SW", "LEFT: Vector Hole 1", "H8", base_dz),
        ]
        # find("RIGHT: Lower hole 0", romi_base_key_table)

        # Set *debug_dz* to non-zero to provide a little gap on top and bottom for visualization:
        bottom_z: float
        top_z: float
        male_height: float
        pi_center_x: float = 0.0
        pi_center_y: float = 0.0

        # Iterate over *spacer_tuples*:
        spacer_tuple: Tuple[str, str, float]
        spacer_positions: Dict[str, Tuple[P2D, str, float]] = {}
        spacer_name: str
        reference_name: str
        key_name: str
        base_level: float
        for spacer_name, key_name, reference_name, base_level in spacer_tuples:
            # Lookup the *key_name* and extract (*key_x*, *key_y*) location:
            key: Tuple[Any, ...] = romi_base_key_table[key_name]
            key2: Any = key[2]
            key3: Any = key[3]
            assert isinstance(key2, float) and isinstance(key3, float)
            key_x: float = float(key2)
            key_y: float = float(key3)

            # Insert value into *spacer_positions*:
            spacer_positions[spacer_name] = (P2D(key_x, key_y), reference_name, base_level)

            # Keep track of the Raspberry Pi center to ensure that it is very close to (0, 0).
            if name.startswith("Pi"):
                pi_center_x += key_x
                pi_center_y += key_y

        # Verify that *pi_center* is at the origin.
        pi_center: P2D = P2D(pi_center_x / 4.0, pi_center_y / 4.0)
        assert pi_center.length() < .000000001, f"pi_center: {pi_center} is not at origin"

        return spacer_positions

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

            # Sweep around *hole* in 60 degree increments putting in slots and holes:
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
    def large_holes_get(self) -> List[SimplePolygon]:
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
        large_holes: List[SimplePolygon] = []
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
                    large_hole: SimplePolygon = Circle(f"Large Hole[{x_index}, {y_index}]",
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
        expansion_outer.arc_append(origin, arc_radius, start_angle, end_angle, 0.0)  # Arc

        # Draw the right wheel well and lock up *expansion_outer*:
        expansion_outer.point_append(P2D(half_total_dx, -wheel_well_dy))       # Outer corner
        expansion_outer.point_append(P2D(half_wheel_well_dx, -wheel_well_dy))  # Inner corner
        expansion_outer.lock()

        # Fill in *simple_polygons* starting with *expansion_outer* and the all of the
        # other holes, rectangles and slots:
        simple_polygons: List[SimplePolygon] = (
            [expansion_outer] +
            romi_expansion_plate.large_holes_get() +
            romi_expansion_plate.small_holes_get() +
            romi_expansion_plate.hex_holes_slots_get() +
            romi_expansion_plate.standoff_holes_get() +
            romi_expansion_plate.top_slots_get() +
            romi_expansion_plate.arc_holes_get() +
            romi_expansion_plate.miscellaneous_polygons_get()
        )

        # Create and return *expansion_polygon* in an unlocked state:
        expansion_polygon: Polygon = Polygon("Expansion", simple_polygons, lock=True)
        return expansion_polygon

    # RomiExpansionPlate.small_holes_get():
    def small_holes_get(self) -> List[SimplePolygon]:
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
        small_holes: List[SimplePolygon] = []
        y_index: int
        for y_index in range(4):
            y: float = y_start + y_pitches[y_index]
            x_index: int
            for x_index in range(10):
                x: float = x_start + float(x_index) * x_pitch
                center: P2D = P2D(x, y)
                small_hole: SimplePolygon = Circle(f"Small Hole[{x_index}, {y_index}]",
                                                   small_hole_diameter, 8, center)
                small_holes.append(small_hole)
        return small_holes

    # RomiExpansionPlate.spacer_positions_get():
    def spacer_positions_get(self) -> Dict[str, Tuple[P2D, str, str]]:
        """Return the spacer positions for ExpansionPlate."""
        # Grab the *plate_keys* from *plate* (i.e. *self*):
        plate: RomiExpansionPlate = self
        plate_keys: List[Tuple[Any, ...]] = plate.keys_get()

        # Insert the contents of *plate_keys* into *plate_keys_table* keyed by key name:
        positions_table: Dict[str, P2D] = {}
        position: P2D
        plate_key: Tuple[Any, ...]
        for plate_key in plate_keys:
            key_name = plate_key[1]
            x = plate_key[2]
            y = plate_key[3]
            assert isinstance(key_name, str) and isinstance(x, float) and isinstance(y, float)
            position = P2D(x, y)
            positions_table[key_name] = position

        # Start with a list of *spacer_tuples*:
        spacer_tuples: List[Tuple[str, str, str, str]] = [
            ("Outer NE Arm Spacer NO_HOLE", "LEFT: Arc Hole 1", "H10", "center"),
            ("Outer NW Arm Spacer NO_HOLE", "RIGHT: Arc Hole 1", "H10", "center"),
            ("Inner NE Arm Spacer", "Slot[1,4]", "H12", "nw"),  # Spacer in center of slot
            ("Inner NW Arm Spacer", "Slot[5,5]", "H13", "ne"),  # Spacer in centef of slot
        ]

        # Create and populate *spacer_posistions* by iterating over *spacer_tuples*:
        spacer_positions: Dict[str, Tuple[P2D, str, str]] = {}
        spacer_name: str
        reference_name: str
        board_name: str
        for spacer_name, key_name, reference_name, board_name in spacer_tuples:
            assert key_name in positions_table, (f"Can not find '{key_name}' in positions table: "
                                                 f"{sorted(list(positions_table.keys()))}")
            position = positions_table[key_name]
            position_tuple: Tuple[P2D, str, str] = (
                position, reference_name, board_name)
            spacer_positions[spacer_name] = position_tuple
        return spacer_positions

    # RomiExpansionPlate.standoff_holes_get():
    def standoff_holes_get(self) -> List[SimplePolygon]:
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
        standoff_holes: List[SimplePolygon] = []
        standoff_center: P2D
        standoff_index: int
        for standoff_index, standoff_center in enumerate(standoff_centers):
            standoff_hole: SimplePolygon = Circle(f"Standoff[{standoff_index}",
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

    # RomiExpansionPlate.arc_holes_get():
    def arc_holes_get(self) -> List[SimplePolygon]:
        """Return The arc holes at the front."""
        # Grab *expansion_dxf* from *romi_expansion_plate* (i.e. *self*):
        romi_expansion_plate: RomiExpansionPlate = self
        expansion_dxf: ExpansionDXF = romi_expansion_plate.expansion_dxf
        right_arc_hole0: SimplePolygon = expansion_dxf.hole_locate("RIGHT: Arc Hole 0",  # Small
                                                                   0.260413, 2.275024,
                                                                   0.354858, 2.369469)
        right_arc_hole1: SimplePolygon = expansion_dxf.hole_locate("RIGHT: Arc Hole 1",  # Large
                                                                   0.134024, 2.095858,
                                                                   0.260413, 2.222248)
        right_arc_hole2: SimplePolygon = expansion_dxf.hole_locate("RIGHT: Arc Hole 2",  # Small
                                                                   0.039579, 1.948634,
                                                                   0.134024, 2.043079)
        right_arc_hole3: SimplePolygon = expansion_dxf.hole_locate("RIGHT: Arc Hole 3",  # Small
                                                                   -0.070142, 1.815303,
                                                                   0.024303, 1.909748)
        right_arc_hole4: SimplePolygon = expansion_dxf.hole_locate("RIGHT: Arc Hole 4",  # Small
                                                                   -0.225697, 1.659748,
                                                                   -0.099307, 1.784748)
        right_arc_hole5: SimplePolygon = expansion_dxf.hole_locate("RIGHT: Arc Hole 5",  # Small
                                                                   -0.349307, 1.536134,
                                                                   -0.254866, 1.630579)
        right_arc_hole6: SimplePolygon = expansion_dxf.hole_locate("RIGHT: Arc Hole 6",  # Large
                                                                   -0.493752, 1.418079,
                                                                   -0.399307, 1.512524)
        right_arc_holes: List[SimplePolygon] = [
            right_arc_hole0,
            right_arc_hole1,
            right_arc_hole2,
            right_arc_hole3,
            right_arc_hole4,
            right_arc_hole5,
            right_arc_hole6,
        ]
        right_arc_hole: SimplePolygon
        left_arc_holes: List[SimplePolygon] = [right_arc_hole.y_mirror("RIGHT", "LEFT")
                                               for right_arc_hole in right_arc_holes]
        all_arc_holes: List[SimplePolygon] = right_arc_holes + left_arc_holes
        return all_arc_holes


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
                                        start_angle=pi/2.0, end_angle=3.0*pi/2.0)
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
                                     start_angle=3.0*pi/2.0, end_angle=pi/2.0)
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
    def __init__(self, scad_program: ScadProgram, base_dxf: BaseDXF, encoder: Encoder) -> None:
        """Initialize RomiWheelAssembly and append to ScadProgram."""
        # Create *romi_motor*, *romi_magnet*, and *encoder*:
        romi_motor: RomiMotor = RomiMotor(scad_program, base_dxf)
        romi_magnet: RomiMagnet = RomiMagnet(scad_program, base_dxf)

        # Construct *module*, append to *scad_program*, and store into *rom_wheel_assembly*
        # (i.e. *self*):
        module = Module3D("Romi Wheel Assembly Module", [
            romi_motor.module.use_module_get(),
            romi_magnet.module.use_module_get(),
            # translated_encoder,
        ])

        scad_program.append(module)
        # romi_wheel_assembly: RomiWheelAssembly = self
        self.module: Module3D = module
        scad_program.if3d.name_match_append("wheel_assembly", module, ["Wheel Assembly"])

        # Save some values into *romi_wheel_assembly* (i.e. *self*).
        # romi_wheel_assembly: Romi_Wheel_Assembly = self
        # self.east_encoder: Encoder = east_encoder
        self.romi_magnet: RomiMagnet = romi_magnet
        self.romi_motor: RomiMotor = romi_motor
        # self.west_encoder: Encoder = west_encoder


# Spacer:
class Spacer:
    """Represents a Spacer."""

    # Spacer.__init__():
    def __init__(self, scad_program: ScadProgram, name: str, height: float, screw_class: str,
                 inner_diameter: float = 0.0, outer_diameter: float = 0.0, is_hex: bool = False,
                 bottom_center: P3D = P3D(0.0, 0.0, 0.0),
                 bottom_height: float = 0.0, top_height: float = 0.0, color: str = "Silver",
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
            *inner_diameter*: (*float*):
                (Optional: Defaults to 0.0 which causes the diameter
                to be looked up from the screw class.
            *outer_diameter*: (*float*):
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
        assert inner_diameter >= 0.0, f"inner_diameter (={inner_diameter}) is negative"
        assert outer_diameter >= 0.0, f"outer_diameter (={outer_diameter}) is negative"
        assert bottom_height >= 0.0, f"bottom_height (={bottom_height}) is negative)"
        assert top_height >= 0.0, f"top_height (={top_height}) is negative)"
        washer: Tuple[float, float, str]
        for washer in bottom_washers + top_washers:
            assert len(washer) == 3, "top/bottom washer (={washer}) does not have 3 entries"

        # https://littlemachineshop.com/images/gallery/PDF/TapDrillSizes.pdf
        # Determine some standard constants based on *screw_class*:
        screw_size: float
        standard_drill_fit: float
        if screw_class == "M2":  # M2 spacers are not readily available, use M2.5 instead.
            screw_size = 2.00
            standard_drill_fit = 2.20
            class_inner_diameter = 2.70  # This is actually an M2.5 spacer inner diameter
            class_outer_diameter = 4.00  # This is actually an M2.5 spacer outer diameter
        elif screw_class == "M2.5":
            screw_size = 2.5
            standard_drill_fit = 2.75
            class_inner_diameter = 2.70
            class_outer_diameter = 4.00
        elif screw_class == "M3":
            screw_size = 3.00
            standard_drill_fit = 3.30
            class_inner_diameter = 3.30
            class_outer_diameter = 5.00
        else:
            assert False, "Bad screw class"  # pragma: no cover

        # If either *inner_diameter* or *outer_diamter* are zero, override with class based values:
        if inner_diameter <= 0.0:
            inner_diameter = class_inner_diameter
        if outer_diameter <= 0.0:
            outer_diameter = class_outer_diameter

        # Detemine the *hex_diameter* using math (http://mathworld.wolfram.com/RegularPolygon.html):
        #     (1) r = R * cos(pi / n)
        #     (2) R = r / cos(pi / n)
        # where:
        #     R is the circle diameter,
        #     r is the center to side distance, and
        #     n is the number of sides.
        hex_diameter: float = outer_diameter / cos(pi / float(6))  # Trail and error

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
            body_diameter: float = hex_diameter if is_hex else outer_diameter
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
                                     washer, standard_drill_fit, z, spacer_stack)

        # Stack up the *top_washers*:
        z = height
        for washer_index, washer in enumerate(top_washers):
            z = spacer.washer_append(f"{name} Top['{washer_index}]",
                                     washer, standard_drill_fit, z, spacer_stack)

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

    def __init__(self, scad_program: ScadProgram, connectors: Connectors, pcb_origin: P2D) -> None:
        """Initialize the STLink object."""
        # The mechanical drawing in the Nucleo144 User Manual does not give very much dimensional
        # information.  So much of the numbers below are measured with calipers:
        nucleo_main_bottom_y: float = 0
        cn2_pin2_y: float = nucleo_main_bottom_y + 12.70 + 114.30
        south_y: float = cn2_pin2_y - 18.22
        north_y1: float = cn2_pin2_y + 2.54 + 3.80  # .1inch spacing between pins
        north_y2: float = nucleo_main_bottom_y + 133.34
        assert abs(north_y1 - north_y2) < .001, f"north_y1(={north_y1}) ~= north_y2(={north_y2})"
        pcb_dy: float = abs(north_y2 - south_y)
        pcb_dx: float = 70.0
        # pcb_bottom_z: float = 0.0
        # pcb_dz: float = 1.60  # mm Calipers
        # pcb_top_z: float = pcb_bottom_z + pcb_dz

        # Define all of the connector locations:
        degrees180: float = pi
        degrees90: float = degrees180 / 2.0
        cn1_dx: float = 7.30  # Calipers
        cn1_bottom_y: float = pcb_dy / 2.0 - 4.00  # Calipers
        cn1_top_y: float = cn1_bottom_y + 5.36 + 3.0  # Extra large to poke through PCB
        cn1_dy: float = abs(cn1_top_y - cn1_bottom_y)
        cn1_dz: float = 2.75
        cn1_center_x: float = pcb_dx / 2.0 - 21.00  # Calipers
        cn1_center_y: float = cn1_bottom_y + cn1_dy / 2.0
        cn2_pin1_x: float = -pcb_dx / 2.0 + 3.25  # Calipers
        cn2_pin1_y: float = pcb_dy / 2.0 - 3.80  # Assume .1in spacing  (pin 1 on top)
        cn6_pin1_x: float = cn2_pin1_x + 5.00  # Calipers (pretty accurate)
        cn6_pin1_y: float = cn2_pin1_y - 2 * 2.54  # Pin 1 on on top
        jp2_pin1_x: float = cn2_pin1_x  # Visual inspection
        jp2_pin1_y: float = cn6_pin1_y - 5 * 2.54  # Visual inspection (pin 1 on bottom)
        cn4_pin1_x: float = cn2_pin1_x + 15.00  # Calipers (pretty accurate)
        cn4_pin1_y: float = cn2_pin1_y  # Visual inspection (pin 1 on top)
        cn5_pin1_x: float = cn4_pin1_x  # Visual inspecition
        cn5_pin1_y: float = jp2_pin1_y  # Visual inspection (pin 1 on bottom)
        jp1_pin1_x: float = -(2.54 / 2.0)  # It is pretty clear that it is centered along Y axis
        jp1_pin1_y: float = cn2_pin1_y  # Visual inspection
        cn3_pin1_x: float = -cn2_pin1_x
        cn3_pin1_y: float = cn2_pin1_y  # Visual inspection (pin 1 on top)

        # Compute the connector centers:
        pin_dx_dy: float = 2.54
        cn2_center: P2D = P2D(cn2_pin1_x, cn2_pin1_y - pin_dx_dy / 2.0)
        cn6_center: P2D = P2D(cn6_pin1_x, cn6_pin1_y - 5 * pin_dx_dy / 2.0)
        jp2_center: P2D = P2D(jp2_pin1_x, jp2_pin1_y + pin_dx_dy / 2.0)
        cn4_center: P2D = P2D(cn4_pin1_x, cn4_pin1_y - 3 * pin_dx_dy / 2.0)
        cn5_center: P2D = P2D(cn5_pin1_x, cn5_pin1_y + pin_dx_dy / 2.0)
        jp1_center: P2D = P2D(jp1_pin1_x + pin_dx_dy / 2.0, jp1_pin1_y)
        cn3_center: P2D = P2D(cn3_pin1_x, cn3_pin1_y - pin_dx_dy / 2.0)
        origin2d: P2D = P2D(0.0, 0.0)

        # Store the PCB outline into *st_link_exterior*:
        st_link_exterior: Square = Square("STLink Exterior Outline ", pcb_dx, pcb_dy)

        # Create the *st_link_pin_connectors_pcb_chunk* needed for *st_link_pcb_chunk*:
        st_link_pin_connectors_pcb_chunk: PCBChunk = PCBChunk.join("ST Link PinConnectors", [
            (connectors.m1x2.pcb_chunk.reposition(origin2d, -degrees90, cn2_center).  # CN2
             pads_rebase(20)),
            (connectors.m1x6.pcb_chunk.reposition(origin2d, -degrees90, cn6_center).  # CN6
             pads_rebase(60)),
            (connectors.m1x2.pcb_chunk.reposition(origin2d, -degrees90, jp2_center).  # JP2
             pads_rebase(200)),
            (connectors.m1x4.pcb_chunk.reposition(origin2d, -degrees90, cn4_center).  # CN4
             pads_rebase(40)),
            (connectors.m1x2.pcb_chunk.reposition(origin2d, -degrees90, cn5_center).  # CN5
             pads_rebase(50)),
            (connectors.m1x2.pcb_chunk.reposition(origin2d, 0.0, jp1_center).         # JP1
             pads_rebase(100)),
            (connectors.m1x2.pcb_chunk.reposition(origin2d, -degrees90, cn3_center).  # CN3
             pads_rebase(30)),
        ])

        # The power and signal connector centers a defined up here for so that temporary
        # cuts can be made into the st-link module to see the signal/power connections:
        power_signal_dy: float = 0.9
        signal_connector_center: P2D = P2D((cn4_center.x + jp1_center.x) / 2.0, power_signal_dy)
        power_connector_center: P2D = P2D((cn6_center.x + cn4_center.x) / 2.0, power_signal_dy)
        st_link_temporary_cuts_pcb_chunk: PCBChunk = PCBChunk(
            "ST Link Temporary Cuts", [], [], cuts=[
                Square("Power Cut", 2*2.54, 6*2.56, P2D(power_connector_center.x, 4.0)),
                Square("Signal Cut", 4*2.54, 6*2.56, P2D(signal_connector_center.x, 4.0)),
            ])

        # Create the *st_link_scads_pcb_chunk* needed for *st_link_pcb_chunk*:
        cn1_cube_center: P3D = P3D(cn1_center_x, cn1_center_y, cn1_dz / 2.0)
        cn1_cube: Cube = Cube("CN1 USB Connector", cn1_dx, cn1_dy, cn1_dz, cn1_cube_center)
        colored_cn1_cube: Color = Color("Colored CN1 USB Connector", cn1_cube, "Silver")
        st_link_scads_pcb_chunk: PCBChunk = PCBChunk("ST Link USB Scad", [], [colored_cn1_cube])

        # Create the *st_link_pcb_chunk* and associated *st_link_module*:
        st_link_pcb_chunk: PCBChunk = PCBChunk.join("ST_Link", [
            st_link_pin_connectors_pcb_chunk,
            st_link_scads_pcb_chunk,
            st_link_temporary_cuts_pcb_chunk,
        ])
        st_link_module: Module3D = st_link_pcb_chunk.pcb_update(
            scad_program, pcb_origin, 1.6, st_link_exterior, "Brown", None, [])

        # Now create an *translated_st_link_pcb_chunk* which is the *st_link_module* offset
        # to fit nicely with the upcoming *st_adapter_pcb_chunk*:
        xtranslated_st_link: Translate3D = Translate3D(
            "Translated ST Link Module", st_link_module.use_module_get(), P3D(0.0, 0.0, -12.1))
        translated_st_link_pcb_chunk: PCBChunk = PCBChunk(
            "Translated ST Link", [], [xtranslated_st_link])

        # Now create the *st_adapter_pin_connectors_pcb_chunk* need for the upcoming
        # *st_adapter_pcb_chunk*.  The pin are rebased using CNn=>(n x 10) and JPn=>(n * 100)
        st_adapter_pin_connectors_pcb_chunk: PCBChunk = PCBChunk.join("ST Adapter PinConnectors", [
            (connectors.f1x2.pcb_chunk.scads_y_flip().sides_swap().
             reposition(origin2d, -degrees90, cn2_center).pads_rebase(20)),   # CN2
            (connectors.f1x6.pcb_chunk.scads_y_flip().sides_swap().
             reposition(origin2d, -degrees90, cn6_center).pads_rebase(60)),   # CN6
            (connectors.f1x2.pcb_chunk.scads_y_flip().sides_swap().
             reposition(origin2d, -degrees90, jp2_center).pads_rebase(200)),  # JP2
            (connectors.f1x4.pcb_chunk.scads_y_flip().sides_swap().
             reposition(origin2d, -degrees90, cn4_center).pads_rebase(40)),   # CN4
            (connectors.f1x2.pcb_chunk.scads_y_flip().sides_swap().
             reposition(origin2d, -degrees90, cn5_center).pads_rebase(50)),   # CN5
            (connectors.f1x2.pcb_chunk.scads_y_flip().sides_swap().
             reposition(origin2d, 0.0, jp1_center).pads_rebase(100)),         # JP1
        ])

        # Create the *st_adapter_pcb_chunk*:
        st_adapter_power_connector_pcb_chunk: PCBChunk = (
            connectors.f1x2ra.pcb_chunk.scads_y_flip().sides_swap().
            reposition(origin2d, degrees180, power_connector_center))
        st_adapter_signal_connector_pcb_chunk: PCBChunk = (
            connectors.f1x4ra.pcb_chunk.scads_y_flip().sides_swap().pads_rebase(4).
            reposition(origin2d, degrees180, signal_connector_center))
        st_adapter_pcb_chunk: PCBChunk = PCBChunk.join("ST_Adapter", [
            st_adapter_pin_connectors_pcb_chunk,
            st_adapter_power_connector_pcb_chunk,
            st_adapter_signal_connector_pcb_chunk,
            translated_st_link_pcb_chunk,
        ])

        # Compute all directory and file names needed to generate footprint and update PCB's:
        assert "HR2_DIRECTORY" in os.environ, "HR2_DIRECTORY is not a defined environment variable."
        hr2_directory: Path = Path(os.environ["HR2_DIRECTORY"])
        st_adapter_directory: Path = hr2_directory / "electrical" / "st_adapter" / "rev_a"
        st_adapter_kicad_pcb: Path = st_adapter_directory / "st_adapter.kicad_pcb"
        st_adapter_pretty_directory: Path = st_adapter_directory / "pretty"

        # Generate the footprint for *st_adapter_pcb_chunk*:
        st_adapter_pcb_chunk.footprint_generate("HR2", st_adapter_pretty_directory)

        # Create *st_adapter_module* and update *st_adapter_kicad_pcb*:
        adapter_pcb_dy: float = pcb_dy
        adapter_pcb_dx: float = pcb_dx / 2.0 + 5.0
        # adapter_pcb_dz: float = 1.6
        st_adapter_exterior: Square = Square("STLink Adapter Exterior",
                                             adapter_pcb_dx, adapter_pcb_dy,
                                             center=P2D(-(pcb_dx - adapter_pcb_dx) / 2.0, 0.0),
                                             corner_radius=1.5, corner_count=5)
        st_adapter_module: Module3D = st_adapter_pcb_chunk.pcb_update(
            scad_program, pcb_origin, 1.6, st_adapter_exterior, "Olive", st_adapter_kicad_pcb, [])

        # Create the nylon tie down holes for the upcoming *st_mate_pcb_chunk*:
        tie_diameter: float = 2.4  # mm
        tie_east_x: float = -0.3  # mm  (all values determined by trial and error)
        tie_west_x: float = -29.0  # mm
        tie_north_y: float = 12.0  # mm
        tie_south_y: float = -3.0  # mm
        st_mate_pads: List[Pad] = [
            Pad("ST Adapter NE Nylon Tie Hole", 0.0, 0.0, tie_diameter,
                P2D(tie_east_x, tie_north_y), 0.0),
            Pad("ST Adapter Nw Nylon Tie Hole", 0.0, 0.0, tie_diameter,
                P2D(tie_west_x, tie_north_y), 0.0),
            Pad("ST Adapter SE Nylon Tie Hole", 0.0, 0.0, tie_diameter,
                P2D(tie_east_x, tie_south_y), 0.0),
            Pad("ST Adapter Sw Nylon Tie Hole", 0.0, 0.0, tie_diameter,
                P2D(tie_west_x, tie_south_y), 0.0),
        ]
        st_mate_tie_holes_pcb_chunk: PCBChunk = PCBChunk("ST Mate Tie holes", st_mate_pads, [])

        # Create the *st_adapter_mate_pcb_chunk*:
        st_mate_power_connector_pcb_chunk: PCBChunk = (
            connectors.m1x2.pcb_chunk.scads_y_flip().sides_swap().
            reposition(origin2d, 0.0, P2D(power_connector_center.x, 1.27)))
        st_mate_signal_connector_pcb_chunk: PCBChunk = (
            connectors.m1x4.pcb_chunk.scads_y_flip().sides_swap().pads_rebase(2).
            reposition(origin2d, 0.0, P2D(signal_connector_center.x, 1.27)))
        st_mate_usb_cut_pcb_chunk: PCBChunk = PCBChunk("ST Mate Cuts", [], [], cuts=[
            Square("USB Cut", cn1_dx + 0.50, 1.5 * cn1_dz, P2D(cn1_center_x, 7.75),  # Trial & error
                   0.0, corner_radius=1.0, corner_count=3)])
        origin3d: P3D = P3D(0.0, 0.0, 0.0)
        x_axis: P3D = P3D(1.0, 0.0, 0.0)
        repositioned_st_adapter: Scad3D = st_adapter_module.use_module_get().reposition(
            "Repositoned ST Adapter", origin3d, x_axis, degrees90, P3D(0.0, 0.0, -12.1))
        st_mate_repositioned_pcb_chunk: PCBChunk = PCBChunk(
            "ST Mate Repositioned", [], [repositioned_st_adapter]).sides_swap()
        st_mate_pcb_chunk: PCBChunk = PCBChunk.join("ST_MATE", [
            st_mate_tie_holes_pcb_chunk,
            st_mate_repositioned_pcb_chunk,
            st_mate_usb_cut_pcb_chunk,
            st_mate_power_connector_pcb_chunk,
            st_mate_signal_connector_pcb_chunk,
        ])

        # Create both the footprint and a temporary module for viewing purposes:
        st_mate_exterior: Square = Square("ST Mate Exterior", 70.0, 30.0)
        st_mate_pcb_chunk.footprint_generate("HR2", st_adapter_pretty_directory)
        st_mate_pcb_module: Module3D = st_mate_pcb_chunk.pcb_update(
            scad_program, pcb_origin, 1.6, st_mate_exterior, "Blue", None, [])

        # Stuff some values into *st_link* (i.e. *self*):
        # st_link: STLink = self
        self.st_adapter_module: Module3D = st_adapter_module
        self.st_adapter_pcb_chunk: PCBChunk = st_adapter_pcb_chunk
        self.st_mate_module: Module3D = st_mate_pcb_module
        self.st_mate_pcb_chunk: PCBChunk = st_mate_pcb_chunk


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


# U3V70x:
class U3V70x:
    """Represents a Pololu UV70x Step-Up Voltage Regulator."""

    # U3V70x.__init__:
    def __init__(self, scad_program: ScadProgram,
                 output_voltage: float, connectors: Connectors) -> None:
        """Initialize a U3V70x voltage regulator."""
        # Dimensions obtained from:
        #    https://www.robitshop.com/urun/5v-step-up-voltage-regulator-u3v70f5

        # Some X dimensions:
        pcb_dx: float = 15.20  # mm
        pcb_edge_e: float = pcb_dx / 2.0
        pcb_edge_w: float = -pcb_edge_e
        sw_mount_x: float = pcb_edge_w + 2.20
        ne_mount_x: float = sw_mount_x + 10.90
        nw_terminal_x: float = pcb_edge_w + 2.54
        ne_terminal_x: float = nw_terminal_x + 5.00
        se_terminal_x: float = pcb_edge_e - 2.54
        sw_terminal_x: float = se_terminal_x - 5.00
        n_connector_center_x: float = pcb_edge_w + 1.27 + ((4 - 1) * 2.54) / 2.0
        s_connector_center_x: float = pcb_edge_e - 1.27 - ((5 - 1) * 2.54) / 2.0

        # Some Y dimensions:
        pcb_dy: float = 40.60  # mm
        pcb_edge_n: float = pcb_dy / 2.0
        pcb_edge_s: float = -pcb_edge_n
        s_mount_y: float = pcb_edge_s + 2.20
        n_mount_y: float = s_mount_y + 36.30
        n_terminal_y: float = pcb_edge_n - 2.54
        s_terminal_y: float = pcb_edge_s + 2.54
        n_connector_center_y: float = pcb_edge_n - 5.08
        s_connector_center_y: float = pcb_edge_s + 5.08

        # The switching transistor is the largest object on the board.  It dimensions and locations
        # are obtained by obtained using GIMP reading off pixel locations.  The pixel Y axis is
        # downwards, so a minus flips it to standard right hand rule Cartesian coordinates:
        sw_pcb_corner_pixels: P2D = P2D(88.6, -265.1)
        sw_transistor_corner_pixels: P2D = P2D(106.5, -222.1)
        ne_transistor_corner_pixels: P2D = P2D(139.6, -186.6)
        ne_pcb_corner_pixels: P2D = P2D(162.4, -67.6)
        pcb_dx_dy_pixels: P2D = ne_pcb_corner_pixels - sw_pcb_corner_pixels
        pcb_center_pixels: P2D = (sw_pcb_corner_pixels + ne_pcb_corner_pixels) / 2.0
        transistor_dx_dy_pixels: P2D = ne_transistor_corner_pixels - sw_transistor_corner_pixels
        transistor_center_pixels: P2D = (
            sw_transistor_corner_pixels + ne_transistor_corner_pixels) / 2.0
        x_pixels2mm: float = pcb_dx / pcb_dx_dy_pixels.x
        y_pixels2mm: float = pcb_dy / pcb_dx_dy_pixels.y
        transistor_dx: float = transistor_dx_dy_pixels.x * x_pixels2mm
        transistor_dy: float = transistor_dx_dy_pixels.y * y_pixels2mm
        transistor_center_x: float = (transistor_center_pixels - pcb_center_pixels).x * x_pixels2mm
        transistor_center_y: float = (transistor_center_pixels - pcb_center_pixels).y * y_pixels2mm

        # Some miscellaneous dimensions:
        pcb_dz: float = 1.57
        transistor_dz: float = 3.0
        mount_diameter: float = 2.18  # mm for #2 or 2mm screw.
        # terminal_diameter: float = 2.18  # Same.

        # Create the *mounts_pcb_chunk*:
        ne_mount_center: P2D = P2D(ne_mount_x, n_mount_y)
        sw_mount_center: P2D = P2D(sw_mount_x, s_mount_y)
        ne_terminal_center: P2D = P2D(ne_terminal_x, n_terminal_y)
        nw_terminal_center: P2D = P2D(nw_terminal_x, n_terminal_y)
        se_terminal_center: P2D = P2D(se_terminal_x, s_terminal_y)
        sw_terminal_center: P2D = P2D(sw_terminal_x, s_terminal_y)
        ne_mount_pad: Pad = Pad("U3V70x NE Mount Hole", 0.0, 0.0, mount_diameter, ne_mount_center)
        sw_mount_pad: Pad = Pad("U3V70x SE Mount Hole", 0.0, 0.0, mount_diameter, sw_mount_center)
        mount_pads: List[Pad] = [ne_mount_pad, sw_mount_pad]
        ne_terminal_pad: Pad = Pad(
            "U3V70x NE Terminal", 0.0, 0.0, mount_diameter, ne_terminal_center)
        nw_terminal_pad: Pad = Pad(
            "U3V70x NW Terminal", 0.0, 0.0, mount_diameter, nw_terminal_center)
        se_terminal_pad: Pad = Pad(
            "U3V70x SE Terminal", 0.0, 0.0, mount_diameter, se_terminal_center)
        sw_terminal_pad: Pad = Pad(
            "U3V70x sW Terminal", 0.0, 0.0, mount_diameter, sw_terminal_center)
        terminal_pads: List[Pad] = [
            ne_terminal_pad, nw_terminal_pad, se_terminal_pad, sw_terminal_pad]
        mounts_pcb_chunk: PCBChunk = PCBChunk(
            "U3V70x Mount Holes", mount_pads + terminal_pads, [])

        # Create the *transistor_scad*:
        transistor_cube: Cube = Cube(
            "U3V70x Transistor Cube", transistor_dx, transistor_dy, transistor_dz,
            center=P3D(transistor_center_x, transistor_center_y, transistor_dz / 2.0))
        transistor_scad: Scad3D = Color("U3V70x Transistor", transistor_cube, "SaddleBrown")
        transistor_pcb_chunk: PCBChunk = PCBChunk("U3V70x Transistor", [], [transistor_scad])

        # Create the connector *PCBChunk*'s:
        origin2d: P2D = P2D(0.0, 0.0)
        n_connector_center: P2D = P2D(n_connector_center_x, n_connector_center_y)
        s_connector_center: P2D = P2D(s_connector_center_x, s_connector_center_y)
        connector_n_pcb_chunk: PCBChunk = (
            connectors.m1x4.pcb_chunk.
            scads_y_flip().sides_swap().pads_rebase(5).
            reposition(origin2d, 0.0, n_connector_center))
        connector_s_pcb_chunk: PCBChunk = (
            connectors.m1x5.pcb_chunk.
            scads_y_flip().sides_swap().
            reposition(origin2d, 0.0, s_connector_center))

        # Create the *pcb_exterior* and associated *exterior_pcb_chunk*:
        pcb_exterior: SimplePolygon = Square("U3V70x Exterior", pcb_dx, pcb_dy)
        outline_pcb_chunk: PCBChunk = PCBChunk(
            "U3V7x Exterior Outline", [], [], back_artworks=[pcb_exterior])

        # Create *u3v70x_pcb_chunk*:
        u3v70x_pcb_chunk: PCBChunk = PCBChunk.join("U3V70x", [
            connector_n_pcb_chunk,
            connector_s_pcb_chunk,
            outline_pcb_chunk,
            mounts_pcb_chunk,
            transistor_pcb_chunk,
        ])

        # Generate the *u3v70x_module*:
        u3v70x_module: Module3D = u3v70x_pcb_chunk.pcb_update(
            scad_program, origin2d, pcb_dz, pcb_exterior, "SeaGreen", None, [])

        # Create the *spacer_
        spacer_dz: float = 9.0
        spacer_diameter: float = 4.00  # Pololu Spacers: L=4/8/10/12/15mm OD=4mm ID=M2
        spacer: Spacer = Spacer(scad_program, "U3V70x Spacer", spacer_dz, "M2", spacer_diameter)
        ne_translated_spacer: Scad3D = Translate3D(
            "Translated NE U3V70x Spacer", spacer.module.use_module_get(),
            P3D(ne_mount_x, n_mount_y, 0.0))
        sw_translated_spacer: Scad3D = Translate3D(
            "Translated SW U3V70x Spacer", spacer.module.use_module_get(),
            P3D(sw_mount_x, s_mount_y, 0.0))

        # Create *u3v70x_mate_pcb_chunk:
        connector_mate_n_pcb_chunk: PCBChunk = (
            connectors.f1x4.pcb_chunk.pads_rebase(5).
            reposition(origin2d, 0.0, n_connector_center))
        connector_mate_s_pcb_chunk: PCBChunk = (
            connectors.f1x5.pcb_chunk.
            reposition(origin2d, 0.0, s_connector_center))
        translated_u3v70x: Scad3D = Translate3D("U3V70x", u3v70x_module.use_module_get(),
                                                P3D(0.0, 0.0, spacer_dz))
        mating_u3v70x_pcb_chunk: PCBChunk = PCBChunk(
            "Mating U3V70x", [], [translated_u3v70x, ne_translated_spacer, sw_translated_spacer])
        mate_mounts_pcb_chunk: PCBChunk = PCBChunk("U3V70x Mate Mount Holes", mount_pads, [])
        u3v70x_mate_pcb_chunk: PCBChunk = PCBChunk.join("U3V70xMate", [
            outline_pcb_chunk.sides_swap(),
            mate_mounts_pcb_chunk,
            connector_mate_n_pcb_chunk,
            connector_mate_s_pcb_chunk,
            mating_u3v70x_pcb_chunk,
        ])

        # Figure out *master_pcb_directory* and *master_pcb_pretty_directory*:
        assert "HR2_DIRECTORY" in os.environ, "HR2_DIRECTORY environment variable not set"
        hr2_directory: Path = Path(os.environ["HR2_DIRECTORY"])
        master_pcb_directory: Path = hr2_directory / "electrical" / "master_board" / "rev_a"
        master_pcb_pretty_directory: Path = master_pcb_directory / "pretty"

        # Output the *u3v70x_mate_pcb_chunk* footprint:
        u3v70x_mate_pcb_chunk.footprint_generate("HR2", master_pcb_pretty_directory)
        u3v70x_mate_module: Module3D = u3v70x_mate_pcb_chunk.pcb_update(
            scad_program, origin2d, 1.6, pcb_exterior, "MediumSlateBlue", None, [])

        # Stuff some values into xxx (i.e. *self*):
        # u3v70x: U3V70x = self
        self.module: Module3D = u3v70x_module
        self.pcb_chunk: PCBChunk = u3v70x_pcb_chunk
        self.mate_module: Module3D = u3v70x_mate_module
        self.mate_pcb_chunk: PCBChunk = u3v70x_mate_pcb_chunk


def main() -> int:  # pragma: no cover
    """Generate the openscand file."""
    # print("hr2_models.main() called")
    # scad_file: IO[Any]
    # with open("romi_base_dxf.scad", "w") as scad_file:
    #     romi_base_polygon.scad_file_write(scad_file)

    # Create the top level *scad_program* program that we will stuff everything into:
    scad_program: ScadProgram = ScadProgram("Scad models")

    # Update the `README.md` file:
    pcb_origin: P2D = P2D(100.0, 100.0)
    hr2_robot: HR2Robot = HR2Robot(scad_program, pcb_origin)
    hr2_robot = hr2_robot

    # Create the `scad_show` program in the HR2 `bin` directory:
    assert "HR2_DIRECTORY" in os.environ, "HR2_DIRECTORY enviroment variable is not set"
    hr2_directory: Path = Path(os.environ["HR2_DIRECTORY"])
    hr2_bin_directory: Path = hr2_directory / "bin"
    scad_show: Path = hr2_bin_directory / "scad_show"
    hr2_mechanical_directory: Path = hr2_directory / "mechancial"
    hr2_models_scad: Path = hr2_mechanical_directory / "hr2_models.scad"
    scad_program.scad_show_create(scad_show, hr2_models_scad)

    # Generate `hr2_models.scad`:
    scad_lines: List[str] = []
    scad_program.scad_lines_append(scad_lines, "")
    # Funky slice syntax for inserting a list in the middle of another list:
    # scad_lines[2:2] = scad_comment_lines
    scad_lines.append("")
    scad_text: str = '\n'.join(scad_lines) + '\n'
    scad_file: IO[Any]
    with open("hr2_models.scad", "w") as scad_file:
        # print("writing out to hr2_models.scad")
        scad_file.write(scad_text)

    return 0


# Footprint:
class Footprint:
    """Footprint represents a KiCad footprint file."""

    # Footprint.__init__():
    def __init__(self, library_name: str, footprint_name: str) -> None:
        """Initialize a footprint."""
        # Note to print the current time in seconds from 1970 use the following command:
        #     date +%s
        # The *time.time()* method returns the time in seconds as a float.
        # if time_stamp == 0:
        #     time_stamp = int(time.time())
        # Create *lines* and start with the module line:
        timestamp: int = int(time.time())
        lines: List[str] = [f"(module {library_name}:{footprint_name} "
                            f"(layer F.Cu) (tedit {timestamp:08X})"]
        # print(f"lines={lines}")

        # Save *name* and *lines* into *footprint* (i.e. *self*):
        # footprint: FootPrint = self
        self.lines: List[str] = lines
        self.library_name: str = library_name
        self.footprint_name: str = footprint_name

    def header_match(self, previous_header: str) -> bool:
        """See whether a previous header matches the current one."""
        footprint: Footprint = self
        lines: List[str] = footprint.lines
        assert len(lines) >= 1
        header: str = lines[0]
        # The tail end of the header looks like "... (tedit XXXXXXXX)", where "XXXXXXXX"
        # is an eight digit upper case hex number.  We want to identically match everything
        # *except* the hex number:
        # print(f"len(header):{len(header)}")
        # print(f"len(previoius_header):{len(previous_header)}")
        # print(f"last characters match:{header[-1:] == previous_header[-1:]}")
        # print(f"front characters match:{header[:-9] == previous_header[:-9]}")
        match: bool = (len(header) == len(previous_header) and
                       header[-1:] == previous_header[-1:] and
                       header[:-9] == previous_header[:-9])
        # print(f"match:{match}")
        return match

    # Footprint.hole():
    def hole(self, name: str, center: P2D, pad_dx: float, pad_dy: float, drill: float,
             tracing: str = "") -> None:
        """Append a hole to a footprint."""
        # Perform any requested *tracing*:
        if tracing:
            print(f"{tracing}=>Footprint.hole(*, '{name}', {center}, {pad_dx}, {pad_dy}, {drill})")

        # Unpack some values from *footprint* (i.e. *self*):
        footprint: Footprint = self
        lines: List[str] = footprint.lines

        # Construct *line* based on the argument values:
        line: str
        if pad_dx == 0.0 or pad_dy == 0.0:
            line = (f"  (pad \"{name}\" np_thru_hole circle "
                    f"(at {center.x:.2f} {-center.y:.2f}) "
                    f"(size {drill:.2f} {drill:.2f}) "
                    f"(drill {drill:.2f}) "
                    "(layers *.Cu *.Mask))")
        else:
            small_distance: float = 0.0000001
            if abs(pad_dx - pad_dy) < small_distance:
                # Do a simple circular pad:
                line = (f"  (pad \"{name}\" thru_hole circle "
                        f"(at {center.x:.2f} {-center.y:.2f}) "
                        f"(size {pad_dx:.2f} {pad_dy:.2f}) "
                        f"(drill {drill:.2f}) "
                        "(layers *.Cu *.Mask))")
            else:
                # Do an oval pad:
                pad_dx_dy_minimum: float = min(pad_dx, pad_dy)
                pad_extra: float = pad_dx_dy_minimum - drill
                drill_dx: float = pad_dx - pad_extra
                drill_dy: float = pad_dy - pad_extra
                line = (f"  (pad \"{name}\" thru_hole oval "
                        f"(at {center.x:.2f} {-center.y:.2f}) "
                        f"(size {pad_dx:.2f} {pad_dy:.2f}) "
                        f"(drill oval {drill_dx:.2f} {drill_dy:.2f}) "
                        "(layers *.Cu *.Mask))")
        if tracing:
            print(f"{tracing}Footprint.hole:'{line}'")
        lines.append(line)

        # Wrap up any requested *tracing*:
        if tracing:
            print(f"{tracing}<=Footprint.hole(*, '{name}', {center}, {pad_dx}, {pad_dy}, {drill})")

    # Footprint.library_get():
    def librarty_get(self) -> str:
        """Return the name of the library."""
        # Use *footprint* instead of *self*:
        footprint: Footprint = self
        return footprint.library_name

    # Footprint.line():
    def line(self, point1: P2D, point2: P2D, layer: str, width: float, tracing: str = "") -> None:
        """Append a line to a footprint."""
        # Perform any requested *tracing*:
        if tracing:
            print(f"{tracing}=>Footprint.line({point1}, {point2}, '{layer}', {width}")

        # Extract some values from *footprint* (i.e. *self*)
        footprint: Footprint = self
        lines: List[str] = footprint.lines
        x1: float = point1.x
        y1: float = point1.y
        x2: float = point2.x
        y2: float = point2.y

        # Create the *line* and append to *lines*:
        line: str = (f"  (fp_line (start {x1:.2f} {-y1:.2f}) (end {x2:.2f} {-y2:.2f}) "
                     f"(layer {layer}) (width {width:.2f}))")
        lines.append(line)

        # Wrap up any requested *tracing*:
        if tracing:
            print(f"{tracing}<=Footprint.line({point1}, {point2}, '{layer}', {width}")

    # Footprint.name_get():
    def name_get(self) -> str:
        """Return the name of the footprint."""
        # Use *footprint* instead of *self*:
        footprint: Footprint = self
        return footprint.footprint_name

    # Footprint.rectangle():
    def rectangle(self, point1: P2D, point2: P2D, layer: str, width: float) -> None:
        """Append a rectangle to a footprint."""
        footprint: Footprint = self
        x1: float = point1.x
        y1: float = point1.y
        x2: float = point2.x
        y2: float = point2.y
        point3: P2D = P2D(x1, y2)
        point4: P2D = P2D(x2, y1)
        footprint.line(point1, point3, layer, width)
        footprint.line(point3, point2, layer, width)
        footprint.line(point2, point4, layer, width)
        footprint.line(point4, point1, layer, width)

    # Footprint.reference():
    def reference(self, center: P2D) -> None:
        """Append a reference to a footprint."""
        footprint: Footprint = self
        lines: List[str] = footprint.lines
        x: float = center.x
        y: float = center.y
        lines.append(f"  (fp_text reference REF** (at {x:.2f} {-y:.2f}) (layer F.SilkS)")
        lines.append("    (effects (font (size 1 1) (thickness 0.2)))")
        lines.append("  )")
        # If value is not specified, KiCad currently defaults to putting it into the F.SilkS layer.
        # KiCad completely ignores whatever text that is supplied.  The solution seems to be to put
        # it into the F.Fab layer and you can disable seeing that layer.  Kind of silly, but
        # it works.  Ship it!:
        lines.append(f"  (fp_text value VALUE (at {x:.2f} {-y-2.0:.2f}) (layer F.Fab)")
        lines.append("    (effects (font (size 1 1) (thickness 0.2)))")
        lines.append("  )")
        # lines.append(f"  (fp_text user USER (at {x:.2f} {-y+2.0+2.0:.2f}) (layer B.SilkS)")
        # lines.append("    (effects (font (size 1 1) (thickness 0.2)))")
        # lines.append("  )")

    # KiCadFootprint.save():
    def save(self, footprint_path: Path, flags: str) -> None:
        """Save a footprint to a file.

        Args:
            * *footprint_path* (*Path*):
              The path to the file to write.  Any specified directores
              must already be present.
            * *flags* (*str*):
              Single character flags that control behavior:
              * 'w':
                Aways overwrite the previous file with a new embedded
                KiCad timestamp.
              * 't':
                Always the update file system timestamp for the file.
                If the new content matches the previous content except
                excluding the `tedit` field, the previous content is
                used so that the `tedit` field does not change.
                This is useful for files that are tracked by `git` which
                always matches the file content, but not file system
                timestamp.  The 'w' option takes precedence over this
                option.

        """
        # Grab the *lines* from *footprint* (i.e. *self*):
        footprint: Footprint = self
        lines: List[str] = footprint.lines

        # Figure out if there the *previous_file_matches*:
        previous_text: str
        previous_file_matches: bool = False
        footprint_file: Any[IO]
        if footprint_path.exists():
            # print("file_exists")
            with open(footprint_path, "r") as footprint_file:
                previous_text = footprint_file.read()
                previous_lines: List[str] = previous_text.split('\n')
                # print(f"len(previous_lines):{len(previous_lines)}")
                # print(f"len(lines):{len(lines)}")
                # The +/- 2is because the last two lines have not been tacked onto *lines* yet:
                line_lengths_match: bool = (len(previous_lines) - 2 == len(lines))
                contents_match: bool = (previous_lines[1:-2] == lines[1:])
                header_match: bool = footprint.header_match(previous_lines[0])
                # print(f"line_lengths_match:{line_lengths_match}")
                # print(f"header_match:{header_match}")
                # print(f"contents_match:{contents_match}")
                previous_file_matches = (line_lengths_match and contents_match and header_match)

        #   write    matches  touch  operation
        #   0        0        0      write new
        #   0        0        1      write new
        #   0        1        0      do nothing
        #   0        1        1      write previous
        #   1        0        0      write new
        #   1        0        1      write new
        #   1        1        0      write new
        #   1        1        1      write new
        #
        #   1        x        x      write new
        #   x        0        x      write new
        #   0        1        1      write previous
        #   0        1        0      do nothing

        write_required: bool = 'w' in flags
        touch_required: bool = 't' in flags
        # print(f"write:{write_required} matches:{previous_file_matches} touch:{touch_required}")
        # print(f"footprint_path={footprint_path}")
        if write_required or not previous_file_matches:
            # We need to write out *footprint* to *footprint_path* with any new content
            # and a new `tedit` timestamp.
            #
            # Convert *lines* into *footprint_text*.  We do this by tacking on a closing
            # parenthesis and an empty line to ensure we get the final new-line character
            # at the end of the file:
            footprint_text: str = '\n'.join(lines + [")", ""])
            with open(footprint_path, "w") as footprint_file:
                footprint_file.write(footprint_text)
            # print("write out new")
        elif touch_required:
            # Write out the *previous_text* to force the file system timestamp to be updated.
            with open(footprint_path, "w") as footprint_file:
                footprint_file.write(previous_text)
            # print("write out previous")
        else:
            # print("do nothing")
            pass

    # Footprint.simple_polygon_append():
    def simple_polygon(self, simple_polygon: "SimplePolygon", layer: str, width: float,
                       tracing: str = "") -> None:
        """Append a SimpelePolygon to a Footprint."""
        # Use *footprint* instead of *self*:
        footprint: Footprint = self

        # Perform any requested *tracing*:
        next_tracing: str = tracing + " " if tracing else ""
        if tracing:
            print(f"{tracing}=>Footprint.simplepolygon(*, *, '{layer}', {width}")

        # Extract adjacent point pair from *simple_polygon* and draw a line:
        point1: P2D
        point2: P2D
        simple_polygon_size: int = len(simple_polygon)
        if 1 <= simple_polygon_size <= 2:
            # Draw circle or a line:
            point1 = simple_polygon[0]
            # Trigger a circle of diameter *width* by drawing a really, really short line:
            point2 = simple_polygon[1] if simple_polygon_size == 2 else point1 + P2D(0.0, 0.001)
            footprint.line(point1, point2, layer, width, tracing=next_tracing)
        else:
            # Iterate through *simplepoly* drawing a close loop from point-to-point:
            index: int
            for index in range(simple_polygon_size):
                point1 = simple_polygon[index]
                point2 = simple_polygon[(index + 1) % simple_polygon_size]
                footprint.line(point1, point2, layer, width, tracing=next_tracing)

        # Wrap up any requested *tracing*:
        if tracing:
            print(f"{tracing}=>Footprint.simplepolygon(*, *, '{layer}', {width}")

    # Footprint.thru_hole_pad():
    def thru_hole_pad(self, name: str, center: P2D,
                      pad_diameter: float, drill_diameter: float) -> None:
        """Append a circular through hole pad to a Footprint."""
        footprint: Footprint = self
        lines: List[str] = footprint.lines
        line: str = (f"  (pad {name} thru_hole circle (at {center.x:.2f} {-center.y:.2f}) "
                     f" (size {pad_diameter:.2f} {pad_diameter:.2f}) "
                     f"(drill {drill_diameter:.2f}) (layers *.Cu *.Mask))")
        lines.append(line)

    # Footprint.value():
    def value(self, center: P2D, hide: bool) -> None:
        """Append a value to a Footprint."""
        # Unpack some values from *footprint* (i.e. *self*):
        footprint: Footprint = self
        lines: List[str] = footprint.lines
        footprint_name: str = footprint.footprint_name

        # Now append some lines to *lines*:
        x: float = center.x
        y: float = center.y
        hide_text: str = " hide" if hide else ""
        lines.append(f"  (fp_text value {footprint_name} (at {x:.2f} {-y:.2f}) "
                     f"(layer F.Fab) {hide_text}")
        lines.append("    (effects (font (size 1 1) (thickness 0.2)))")
        lines.append("  )")


# KicadPCB:
class KicadPCB:
    """Represents a KiCAD PCB."""

    # KicadPCB.__init__():
    def __init__(self, file_name: Path, offset: P2D) -> None:
        """Bind to a .kicad_pcb file."""
        # Read in *file_name* and split into *lines*:
        kicad_pcb_file: IO[Any]
        lines: List[str]
        with open(file_name, "r") as kicad_pcb_file:
            kicad_pcb_text: str = kicad_pcb_file.read()
            lines = kicad_pcb_text.split('\n')
            # print(f"KicadPcb.__init__('{file_name}') read in {len(lines)} lines")

        # Sweep through *lines* and strip out cut lines:
        removed_cut_lines: List[str] = []
        line: str
        for line in lines:
            if line.startswith("  (gr_line") and line.find("(layer Edge.Cuts)") > 0:
                # Have a cut line that will *NOT* be copied over.
                pass
            else:
                # We do *NOT* have a cut line and we copy it over:
                removed_cut_lines.append(line)
        lines = removed_cut_lines

        # Save values into *kicad_pcb* (i.e. *self*):
        # kicad_pcb: KicadPcb = self
        self.file_name: Path = file_name
        self.lines: List[str] = lines
        self.offset: P2D = offset

    # KicadPCB.cuts_update():
    def cuts_update(self, cuts: List[SimplePolygon], tracing: str = ""):
        """Update the cuts in in a KicadPCB."""
        # Perform any request *tracing*:
        if tracing:
            print(f"{tracing}=>KicadPCB.cuts_update(len(cuts)={len(cuts)})")

        # Grab some values from *kicad_pcb* (i.e. *self*):
        kicad_pcb: KicadPCB = self
        offset: P2D = kicad_pcb.offset
        lines: List[str] = kicad_pcb.lines

        # Construct *cut_lines* from *cuts*:
        offset_x = offset.x
        offset_y = offset.y
        width: float = 0.05
        layer: str = "Edge.Cuts"
        cut_lines: List[str] = []
        cut: SimplePolygon
        for cut in cuts:
            cut_size: int = len(cut)
            index: int
            for index in range(cut_size):
                point1: P2D = cut[index]
                point2: P2D = cut[(index + 1) % cut_size]
                cut_line: str = ("  (gr_line "
                                 f"(start {offset_x + point1.x:.2f} {offset_y - point1.y:.2f}) "
                                 f"(end {offset_x + point2.x:.2f} {offset_y - point2.y:.2f}) "
                                 f"(layer {layer}) "
                                 f"(width {width:1.2f}))")
                cut_lines.append(cut_line)

        # Now insert *cut_lines* into *lines* 2 lines back:
        assert len(lines) >= 2
        lines[-2:-2] = cut_lines

        # Wrap up any requested *tracing*:
        if tracing:
            print(f"{tracing}<=KicadPCB.cuts_update(len(cuts)={len(cuts)})")

    # KicadPCB.layer_remove():
    def layer_remove(self, layer: str) -> None:
        """Remove the a layer."""
        # Grab some values from *kicad_pcb* (i.e. *self*):
        kicad_pcb: KicadPCB = self
        lines: List[str] = kicad_pcb.lines
        new_lines: List[str] = []
        pattern: str = f"(layer {layer})"
        line: str
        for line in lines:
            if line.find(pattern) < 0:
                # Not a layer we care about, copy it over:
                new_lines.append(line)

        # Save *new_lines* back into *kicad*:
        kicad_pcb.lines = new_lines

    # KicadPCB.modules_update():
    def modules_update(self, references: List["Reference"], tracing: str = "") -> None:
        """Update the module positions."""
        # Avoid circular import loop by importing inside of this method:
        from scad_models.hr2_models import Reference

        # Perform any requested *tracing*:
        # next_tracing: str = tracing + " " if tracing else ""
        assert isinstance(tracing, str)
        if tracing:
            print("********************************************************")
            print(f"{tracing}=>KicadPCB.modules_update()")

        # Grab some values from *kicad_pcb*:
        kicad_pcb: KicadPCB = self
        lines: List[str] = kicad_pcb.lines
        offset: P2D = kicad_pcb.offset

        # Define some variables and constants:
        line_number: int
        line: str
        at_line_index: int
        reference_line_index: int
        reference_prefix: str = "    (fp_text reference "
        reference_prefix_size: int = len(reference_prefix)

        # Construct *references_table* from *referneces*:
        references_table: Dict[str, Reference] = {}
        reference: Reference
        for reference in references:
            if tracing:
                print(f"{tracing}reference_table['{reference.name}] = {reference}")
            references_table[reference.name] = reference

        # KiCad basically copies the entire footprint (e.g. `.kicad_mod`) into the PCB file
        # (e.g. `.kicad_pcb`.)  So far so good.  The first `(at ...)` is the position and
        # orientation of the footprint.  The footprint basically consists of three classes
        # of declarations:
        #
        # * *pad*: This is a pad:
        #
        #   (pad "Name" SHAPE (at X Y ROTATE) (size DX DY) (drill DIAM)(layers *.Cu *.Mask))
        #
        #   Where SHAPE is one of "np_thru_hole", "circle", or "oval" and the remaining values
        #   are a rotation (ROTATE), pad size (DX and dY), and a drill diameter (DIAM).
        #
        # * *fp_line*: This i

        # Sweep through *lines* looking for modules:
        references_found_table: Dict[str, bool] = {}
        in_module: bool = False
        for line_index, line in enumerate(lines):
            if line.startswith("  (module "):
                # Beginning of mounting hole:
                if tracing:
                    print(f"{tracing}Module start found:{line_index}")
                in_module = True
                at_line_index = -1
                reference_line_index = -1
            elif line.startswith("  )") and in_module:
                # We have reached the module end:
                if tracing:
                    print(f"{tracing}Module End found:{line_index}")
                if at_line_index < 0 or reference_line_index < 0:
                    # The module is not well formatted:
                    print(f"Incomplete module: at:{at_line_index} "
                          f"reference:{reference_line_index}")  # pragma: no cover
                else:
                    # Grab the hole label from label_line:
                    reference_line: str = lines[reference_line_index]
                    at_index: int = reference_line.find(" (at ")
                    assert at_index > reference_prefix_size
                    reference_name: str = reference_line[reference_prefix_size:at_index]
                    if tracing:
                        print(f"{tracing}Module Reference Name: '{reference_name}'")

                    # Look up the *reference* from *references_table* and stuff updated location
                    # back into *lines* at *position_index*:
                    if reference_name in references_table:
                        # Unpack the *reference* information from *references_table*:
                        reference = references_table[reference_name]
                        # name: str = reference.name
                        # is_front: bool = reference.is_front
                        position: P2D = reference.position
                        # rotate: float = reference.rotate
                        # FIXME: Add orientation!!!
                        at_text: str = (f"    (at "
                                        f"{offset.x + position.x:0.2f} "
                                        f"{offset.y - position.y:0.2f})")  # KiCad inverts Y axis!
                        if tracing:
                            print(f"{tracing}Found '{reference_name}' in table.")
                            print(f"{tracing}Old: '{lines[at_line_index]}'")
                            print(f"{tracing}New: '{at_text}'")
                        lines[at_line_index] = at_text
                        references_found_table[reference_name] = True

                # Reset all of the values:
                in_module = False
                at_line_index = -1
                reference_line_index = -1
                if tracing:
                    print(f"{tracing}Module Pocessed; line indices reset")
            elif in_module:
                if line.startswith("    (at "):
                    at_line_index = line_index
                    if tracing:
                        print(f"{tracing}at_line_index={at_line_index}")
                elif line.startswith("    (fp_text reference "):
                    reference_line_index = line_index
                    if tracing:
                        print(f"{tracing}reference_line_index={reference_line_index}")

        # Iterate through *references_table* looking for modules that have not been placed:
        for reference_name in references_table.keys():
            if reference_name not in references_found_table:
                print(f"Reference '{reference}' is not currently placed in the PCB.")

        # Wrap up any requested *tracing*:
        if tracing:
            print(f"{tracing}=>KicadPCB.modules_update()")

    # KicadPCB.simple_polygon_append():
    def simple_polygon_append(self, simple_polygon: "SimplePolygon",
                              layer: str, width: float) -> None:
        """Append a SimpelePolygon to a PCB."""
        # kicad_pcb: KicadPCB = self
        # simple_polygon_size: int = len(simple_polygon)
        # index: int
        # for index in range(simple_polygon_size):
        #     point1: P2D = simple_polygon[index]
        #     point2: P2D = simple_polygon[(index + 1) % simple_polygon_size]
        #     kicad_pcb.line_append(point1, point2, layer, width)
        pass

    # KicadPCb.save():
    def save(self):
        """Save contents back file."""
        # Grab some values from *kicad_pcb* (i.e. *self*):
        kicad_pcb: KicadPCB = self
        file_name: str = kicad_pcb.file_name
        lines: List[str] = kicad_pcb.lines

        # Delete blank lines from the end of the file:
        while len(lines) > 0 and lines[-1] == "":
            del lines[-1]

        # Write *kicad_lines* out to *kicad_file_name*:
        kicad_pcb_text: str = '\n'.join(lines) + '\n'
        kicad_pcb_file: IO[Any]
        with open(file_name, "w") as kicad_pcb_file:
            kicad_pcb_file.write(kicad_pcb_text)

    # KicadPCB.number():
    @staticmethod
    def number(value: float, maximum_fractional_digits: int) -> str:
        """Convert value into KiCad style number."""
        # Small positive/negative number close to zero to zero.
        if abs(value) < .000000001:
            value = 0.0

        # KiCad numbers are trimmed of trailing zeros and optionally the trailing decimal point:
        number_text: str = str(round(value, maximum_fractional_digits)).rstrip('0').rstrip('.')
        return number_text

    # KicadPCB.string():
    @staticmethod
    def string(text: str) -> str:
        """Convert a string into a Kicad style string."""
        # This not a perfect conversion for S-expression atoms, but it is good enough:
        assert text.isprintable() and all(ord(c) < 128 for c in text)  # Is ASCII.
        has_space: bool = ' ' in text
        has_double_quote: bool = '"' in text
        has_single_quote: bool = "'" in text
        has_open_parenthesis: bool = "(" in text
        has_close_parenthesis: bool = ")" in text
        is_empty: bool = len(text) == 0
        needs_quotes: bool = (has_space or has_double_quote or has_single_quote or
                              has_open_parenthesis or has_close_parenthesis) or is_empty
        result_text: str = f'"{text}"' if needs_quotes else text
        # print(f"KicadPCB.string('{text}') => '{result_text}'")
        return result_text

    # KicadPCB.fp_line():
    @staticmethod
    def fp_line(prefix: str, pcb_origin: P2D, start_point: P2D, end_point: P2D, layer: str) -> str:
        """Return a KiCad fp_line S-expression."""
        origin_x: float = pcb_origin.x
        origin_y: float = pcb_origin.y
        line: str = (
            f"{prefix}  (fp_line "
            f"(start {KicadPCB.number(origin_x + start_point.x, 2)} "
            f"{KicadPCB.number(origin_y - start_point.y, 2)}) "
            f"(end {KicadPCB.number(origin_x + end_point.x, 2)} "
            f"{KicadPCB.number(origin_y - end_point.y, 2)}) "
            f"(layer {layer}) (width 0.2))")
        return line


if __name__ == "__main__":    # pragma: no cover
    main()
