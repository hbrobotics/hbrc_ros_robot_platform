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
"""Code that generates an OpenSCAD model for HR2 (HBRC ROS Robot)."""

from scad_models.scad import (
    Circle, Color, CornerCube, Cube, Cylinder, If2D, Difference2D, LinearExtrude,
    Module2D, Module3D, P2D, P3D, Polygon, Rotate3D, Scad2D, Scad3D, SimplePolygon, ScadProgram,
    Square, Translate3D, UseModule2D, UseModule3D, Union3D)
import os
import time
from pathlib import Path
from typing import Any, Dict, IO, List, Optional, Set, Tuple
from math import asin, atan2, cos, degrees, nan, pi, sin, sqrt

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

        # Common 1x4 connectors:
        m1x4: RectangularConnector = RectangularConnector(
            "M1x4", scad_program, 1, 4, pins_dx_dy, pcb_pin_height,
            insulation_color="Maroon", male_pin_height=4.04,
            footprint_drill_diameter=1.016, footprint_pad_diameter=1.524)
        # HCSR04 Right angle:
        m1x4ra: RectangularConnector = RectangularConnector(
            "M1x4RA", scad_program, 1, 4, pins_dx_dy, pcb_pin_height,
            insulation_color="Cyan", male_pin_height=6.00,
            right_angle_length=3.00 - 0.127,  # <=calipers / schematic=>6.00 + 0.127
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

        # Common 1x5 connectors:
        f1x5ra: RectangularConnector = RectangularConnector(
            "F1x5RA", scad_program, 1, 5, female_insulation_height, pcb_pin_height,
            insulation_color="Fuchsia",
            right_angle_length=3.00 - 0.127,  # <=calipers / schematic=>6.00 + 0.127
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
        self.m1x3: RectangularConnector = m1x3
        self.m1x3ra: RectangularConnector = m1x3ra
        self.f1x3: RectangularConnector = f1x3
        self.m1x4: RectangularConnector = m1x4
        self.f1x4lp: F1x4LP = f1x4lp
        self.m1x4ra: RectangularConnector = m1x4ra
        self.f1x4: RectangularConnector = f1x4
        self.f1x4h: RectangularConnector = f1x4h
        self.f1x5ra: RectangularConnector = f1x5ra
        self.m1x6: RectangularConnector = m1x6
        self.f1x6: RectangularConnector = f1x6
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


# PCB:
class PCB:
    """Represents a printed circuit board in KiCad and OpenSCAD."""

    # PCB.__init__():
    def __init__(self, name: str, scad_program: "ScadProgram", dz: float,
                 pcb_exterior: SimplePolygon) -> None:
        """Initialize a PCB with a name and pcb exterior."""
        # Create the *pcb_polygon*:
        pcb_polygon: Polygon = Polygon(f"name PCB Polygon", [pcb_exterior], lock=False)
        # Load values into *pcb* (i.e. *self*):
        # pcb: PCB = self
        self.dz: float = dz
        # self.groups: Dict[str, List[PadsGroup]] = {}
        self.name: str = name
        self.pcb_cut_holes: List[SimplePolygon] = []
        self.pcb_parent: Optional[PCB] = None
        self.pcb_exterior: SimplePolygon = pcb_exterior
        self.pcb_polygon: Polygon = pcb_polygon
        self.pcb_groups: Dict[str, PCBGroup] = {}
        self.scad3ds: List[Scad3D] = []
        self.scad_program: ScadProgram = scad_program

    # PCB.footprint_generate():
    def footprint_generate(self, directory: Path, base_name: str, group_names: Set[str],
                           reference_prefix: str, tracing: str = "") -> None:
        """Generate a footprint from PCB.

        Args:
            * *base_name* (*str*):
              The base name of the footprint without the `.kicad_mod`
              suffix.
            * *directory* (*Path*):
              The directory to store the footprint into.
            * *groups_names* (*Set*[*str*]):
              The names of the groups to use for the footprint.
            * *reference_prefix* (*str*):
              The default reference prefix to put into the footprint.

        """
        # Unpack some values from *pcb* (i.e. *self*):
        pcb: PCB = self
        pcb_groups: Dict[str, PCBGroup] = pcb.pcb_groups

        # Now start the *footprint*:
        footprint: Footprint = Footprint("HR2", base_name)
        footprint.reference(P2D(0.0, 0.0))

        # Iterate through all of the *group_names* whose content is to be pushed into *footprint*:
        group_name: str
        for group_name in group_names:
            # Fail if *group_name* does not exist:
            if group_name not in pcb_groups:
                all_group_names: Set[str] = set(list(pcb_groups.keys()))
                raise ValueError(f"Group '{group_name}' is not one of {all_group_names}")

            # Iterate over each *pad* in the *pads_group* associated with *group_name*:
            pcb_group: PCBGroup = pcb_groups[group_name]
            pads_groups: List[PadsGroup] = pcb_group.pads_groups
            pads_group: PadsGroup
            for pads_group in pads_groups:
                pads: List[Pad] = pads_group.pads
                pad: Pad
                for pad in pads:
                    # Output a pad defintion:
                    footprint.hole(pad.name, pad.pad_center,
                                   pad.pad_dx, pad.pad_dy, pad.drill_diameter)

        # Save the *footprint* into the file system:
        full_base_name = base_name + ".kicad_mod"
        footprint.save(directory / full_base_name, "t")

    # PCB.kicad_merge():
    def kicad_merge(self, kicad_pcb_path: Path) -> None:
        """Merge the appropriate into a KiCad .kicad_pcb file."""
        pass

    # PCB.module3d_place():
    def module3d_place(self, module_name: str, group_names: Set[str], flags: str,
                       center: "P2D", z_rotate: float, translate: "P2D",
                       pads_base: int = 0, tracing: str = "") -> None:
        """Place a Module3D onto a PCB.

        Place module onto module onto a PCB doing any requested flips,
        rotations and translations.

        Args:
        * *module_name* (*str*):
          The name of the *Module3D* object to lookup from the
          *ScadProgram* object that was passed into the *PCB* object
          when it was created.  A *ValueError* is thrown if this
          *module_name* is not present.
        * *group_names* (*Set*[*Str*]):
          A set of footprint group names to associate with the
          placement.
        * *flags* (*str*):
          A possibly empty string of 1 character modification codes to
          the module placement.  The codes are:
          * 'b':
            Place the module on the bottom of the board.  This flag is
            usually followed by either the 'x' or 'y' flag.
          * 'n':
            Do NOT actually place the module on the actual PCB.
            Instead, just leave it available for other PCB's to place.
          * 'N':
            Do NOT actually place pads module on the actual PCB.
            Instead, just leave it available for other PCB's to place.
          * 'x':
            Rotate the module and along the X axis by 180 degrees
          * 'X':
            Rotate any associated pads along the X axis by 180 degrees.
          * 'y':
            Rotate the module along the Y axis by 180 degrees.
          * 'Y':
            Rotate any associated pads along the Y axis by 180 degrees.
        * *center* (*PD2D*):
          The virtual center of module in X and Y.
        * *z_rotate* (*float*):
          The amount to rotate the module by around the Z axis
          at its *center*.
        * *translate* (*float*):
          The location to translate the module *center* to.
        * *pad_base* (*int*) (Optional: defaults to 0):
          An offset to be added to each pin number.

        """
        # print(f"=>PCB.module3d_place('{module_name}')")
        # Unpack some values from *pcb* (i.e. *self*):
        pcb: PCB = self
        pcb_groups: Dict[str, PCBGroup] = pcb.pcb_groups
        name: str = pcb.name
        pcb_polygon: Polygon = pcb.pcb_polygon
        scad_program: ScadProgram = pcb.scad_program
        scad3ds: List[Scad3D] = pcb.scad3ds
        dz: float = pcb.dz

        # Perform any requested *tracing*:
        next_tracing: str = ""
        if tracing:
            next_tracing = tracing + " "
            print(f"{tracing}=>PCB.module3d_place('{name}', {group_names}, '{flags}', ...)")

        # Mirror the entire operation on *pcb_parent* if it is set:
        if isinstance(pcb.pcb_parent, PCB):
            pcb_parent: PCB = pcb.pcb_parent
            pcb_parent.module3d_place(module_name, group_names, flags, center, z_rotate, translate,
                                      pads_base=pads_base, tracing=next_tracing)

        # Verify that *flags* are valid:
        flag: str
        for flag in flags:
            if flag not in "NXYbnxy":
                raise ValueError(f"Flag '{flag}' is not allowed.")
        bottom_place: bool = 'b' in flags
        scad3d_place: bool = 'n' not in flags
        pads_place: bool = 'N' not in flags
        x_flip: bool = 'x' in flags
        y_flip: bool = 'y' in flags
        if x_flip and y_flip:
            raise ValueError(f"Flags '{flags}' specifies both 'x' and 'y'.  Only one is allowed.")
        if (x_flip or y_flip) and not bottom_place:
            x_y_flag: str = 'x' if x_flip else 'y'
            raise ValueError(f"Flags '{flags}' does not specify 'b' option for '{x_y_flag}' flag. "
                             "'b' must be specified.")
        x_mirror: bool = 'X' in flags
        y_mirror: bool = 'Y' in flags

        # Display flag values during *tracing*:
        if tracing:
            print(f"{tracing}bottom_place:{bottom_place} "
                  f"pads_place:{pads_place} scad3d_place:{scad3d_place}")
            print(f"{tracing}x_flip:{x_flip} y_flip:{y_flip} "
                  f"x_mirror:{x_mirror} y_mirror:{y_mirror}")

        # Ensure that each *group_name* in *group_names* has a corresponding *PCB_Group*:
        for group_name in group_names:
            if group_name not in pcb_groups:
                pcb_groups[group_name] = PCBGroup(group_name)

        # Lookup the *module3d* and its associated *use_module3d* using *module_name*:
        module3d: Module3D
        error_message: str
        try:
            module3d = scad_program.module3d_get(module_name)
        except ValueError as error_message:
            raise ValueError(error_message)
        use_module3d: UseModule3D = module3d.use_module3d

        # Extract the PADS from the from *module3d*:
        tag: Any = module3d.tag_lookup("PadsGroup")
        if tracing:
            print(f"{tracing}isinstance(tag, PadsGroup):{isinstance(tag, PadsGroup)}")
        if pads_place and isinstance(tag, PadsGroup):
            pads_group: PadsGroup = tag
            if tracing:
                print(f"{tracing}Module '{name}' has pads_group(len={len(pads_group)})")
            x_flag: str = "x" if x_mirror else ""
            y_flag: str = "y" if y_mirror else ""
            xy_flags: str = x_flag + y_flag
            repositioned_pads_group: PadsGroup = pads_group.reposition(xy_flags,
                                                                       center, z_rotate, translate,
                                                                       pads_base)
            if tracing:
                print(f"{tracing}Before holes_append: len(pcb_polygon):{len(pcb_polygon)}")
            repositioned_pads_group.holes_append(pcb_polygon)
            if tracing:
                print(f"{tracing}After holes_append: len(pcb_polygon):{len(pcb_polygon)}")

            # Now add *repositioned_pads_group* to each *group_name* in *groups*:
            # print(f"group_names:{group_names}")
            for group_name in group_names:
                pcb_group: PCBGroup = pcb_groups[group_name]
                pcb_group.pads_groups.append(repositioned_pads_group)
        else:
            if tracing:
                print(f"{tracing}Skipping pads placement.")

        # Perform any requested rotations by moving to *center* to the origin,
        # performing the rotations and move it back to *center* again:
        center3d: P3D = P3D(center.x, center.y, 0.0)
        origin_centered: Scad3D = Translate3D(f"{name} Origin Centered", use_module3d, -center3d)
        if x_flip:
            x_axis: P3D = P3D(1.0, 0.0, 0.0)
            origin_centered = Rotate3D(f"{name} X Flip", origin_centered, pi, x_axis)
        elif y_flip:
            y_axis: P3D = P3D(0.0, 1.0, 0.0)
            origin_centered = Rotate3D(f"{name} Y Flip", origin_centered, pi, y_axis)
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        rotated: Rotate3D = Rotate3D(f"{name} Z Rotate", origin_centered, z_rotate, z_axis)

        # Do the final *tranlation* and append to *scads3ds*:
        z: float = 0.0 if bottom_place else dz
        translate3d: P3D = P3D(translate.x, translate.y, z)
        translated: Translate3D = Translate3D(f"{name} Translated", rotated, center3d + translate3d)

        # Only *place* translated if 'n' is not specified in flags:
        if scad3d_place:
            scad3ds.append(translated)

        # Stuff *translated* into the appropiate *pcb_groups*:
        for group_name in group_names:
            pcb_group = pcb_groups[group_name]
            pcb_group.scad3ds.append(translated)

        # Perform any requested tracing:
        if tracing:
            print(f"{tracing}<=PCB.module3d_place('{name}', {group_names}, '{flags}', ...)")

    # PCB.mount_hole_append():
    def mount_hole_append(self, name: str, group_names: Set[str],
                          diameter: float, position: "P2D") -> None:
        """Append a mounting hole to a PCB.

        Args:
              * *name* (*str):
                The name of the mounting hole.
              * *group_names* (*Set*[*str*]):
                The groups to add mounting hole to.
              * *diameter* (*diameter*):
                The diameter of the mounting hole.
              * *position* (*P2D*):
                The position of the mounting hole:

        """
        # Unpack some values from *pcb* (i.e. *self*):
        pcb: PCB = self
        pcb_polygon: Polygon = pcb.pcb_polygon
        pcb_groups: Dict[str, PCBGroup] = pcb.pcb_groups

        # Mirror the entire operation on *pcb_parent* if it is set:
        if isinstance(pcb.pcb_parent, PCB):
            pcb_parent: PCB = pcb.pcb_parent
            pcb_parent.mount_hole_append(name, group_names, diameter, position)

        # Interate through all of the *group_names*:
        group_name: str
        for group_name in group_names:
            if group_name not in pcb_groups:
                pcb_groups[group_name] = PCBGroup(group_name)
                # print(f"PCB.mount_hole_append: Creating group '{group_name}'")
            pcb_group: PCBGroup = pcb_groups[group_name]
            pads_groups: List[PadsGroup] = pcb_group.pads_groups
            pad: Pad = Pad(name, 0.0, 0.0, diameter, position)
            pads_group: PadsGroup = PadsGroup()
            pads_group.insert(pad)
            pads_groups.append(pads_group)
        pad.hole_append(pcb_polygon)

    # PCB.pad_append():
    def pad_append(self, name: str, group_names: Set[str], flags: str,
                   pad_dx: float, pad_dy: float, drill_diameter: float,
                   center: "P2D" = P2D(0.0, 0.0), rotate=0.0) -> None:
        """Install a bare Pad onto a PCB."""
        # Unpack soem values from *pcb* (i.e. *self*):
        pcb: PCB = self
        pcb_parent: Optional[PCB] = pcb.pcb_parent
        pcb_polygon: Polygon = pcb.pcb_polygon
        pcb_groups: Dict[str, PCBGroup] = pcb.pcb_groups

        # Mirror operation onto *pcb_parent* (if present*):
        if isinstance(pcb_parent, PCB):
            pcb_parent.pad_append(name, group_names, flags, pad_dx, pad_dy, drill_diameter,
                                  center=center, rotate=rotate)

        # Iterate through all of the *group_names*:
        group_name: str
        for group_name in group_names:
            if group_name not in pcb_groups:
                pcb_groups[group_name] = PCBGroup(group_name)
            pcb_group: PCBGroup = pcb_groups[group_name]
            pads_groups: List[PadsGroup] = pcb_group.pads_groups
            pad: Pad = Pad(name, pad_dx, pad_dy, drill_diameter, center, rotate)
            pads_group: PadsGroup = PadsGroup()
            pads_group.insert(pad)
            pads_groups.append(pads_group)

        # Append the hole/slot to *pcb_polygon*:
        pad.hole_append(pcb_polygon)

    # PCB.pcb_cut_hole_append():
    def pcb_cut_hole_append(self, cut_hole: "SimplePolygon") -> None:
        """Append cut hole to a PCB."""
        # Unpack some values from *pcb* (i.e. *self*):
        pcb: PCB = self
        pcb_parent: Optional[PCB] = pcb.pcb_parent
        pcb_polygon: Polygon = pcb.pcb_polygon
        pcb_cut_holes: List[SimplePolygon] = pcb.pcb_cut_holes

        # Mirror the operation on *pcb_parent* (if present):
        if isinstance(pcb_parent, PCB):
            pcb_parent.pcb_cut_hole_append(cut_hole)

        # Do the actual append:
        pcb_polygon.append(cut_hole)
        pcb_cut_holes.append(cut_hole)

    # PCB.pcb_parent_set():
    def pcb_parent_set(self, pcb_parent: "PCB") -> None:
        """Set parent PCB to replicate operations on."""
        # Set the *pcb_parent* in *pcb* (i.e. *self*):
        pcb: PCB = self
        pcb.pcb_parent = pcb_parent

    # PCB.pcb_place():
    def pcb_place(self, other_pcb: "PCB", group_names: Set[str], flags: str,
                  center: "P2D" = P2D(0.0, 0.0), rotate: float = 0.0,
                  translate: "P2D" = P2D(0.0, 0.0), tracing: str = "") -> None:
        """Place the connectors to mate to another PCB.

        Args:
            * *other_pcb* (*PCB*):
              The other *PCB* object to place on the *PCB*.
            * *group_names* (*Set*[*str*]):
              The list of groups to select from the other *PCB*.
            * *flags* (*str*):
              A list of 1 character flags for tweaking things.
              Currently, no flags are defined.
            * *center* (*P2D*)
              (Optional: Defaults to *P2D*(0.0, 0.0):
              The virtual center of the object to rotate around
              the Z axis.
            * *rotate* (*float*): (Optional: Defaults to 0.0):
              The number of radians to rotate the object by around
              the *center*.
            * *translate* (*P2D*)
              (Optional: Defaults to *P2D*(0.0, 0.0)):
              The location to translate *center* to.

        """
        # Unpack some values from *pcb* (i.e. *self*):
        pcb: PCB = self
        name: str = pcb.name
        scad3ds: List[Scad3D] = pcb.scad3ds
        pcb_parent: Optional[PCB] = pcb.pcb_parent
        pcb_polygon: Polygon = pcb.pcb_polygon

        # Perform any requested *tracing*:
        next_tracing: str = ""
        if tracing:
            next_tracing = tracing + " "
            print(f"{tracing}=>PCB.pcb_place('{name}', '{other_pcb.name}', "
                  f"{group_names}, '{flags}'...)")
            print(f"{tracing}len(scad3ds):{len(scad3ds)}")
            print(f"{tracing}(pcb_polygon):{len(pcb_polygon)}")
            other_pcb.show("", next_tracing)

        # Mirror the operation to *pcb_parent* (if present):
        if isinstance(pcb_parent, PCB):
            pcb_parent.pcb_place(other_pcb, group_names, flags, center=center,
                                 rotate=rotate, translate=translate, tracing=next_tracing)

        # Unpack some values from *other_pcb*:
        other_pcb_groups: Dict[str, PCBGroup] = other_pcb.pcb_groups

        # Process each *group_name* in *group_names*:
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        group_name: str
        center3d: P3D = P3D(center.x, center.y, 0.0)
        translate3d: P3D = P3D(translate.x, translate.y, 0.0)
        for group_name in group_names:
            if tracing:
                print(f"{tracing}Group['{group_name}']")
            # Grab the *other_pcb_group* from *other_pcb_groupes*:
            assert group_name in other_pcb_groups, (
                f"Group name '{group_name}' not one of {set(other_pcb_groups.keys())}.")
            other_pcb_group: PCBGroup = other_pcb_groups[group_name]

            # Take *other_pcb_scad3s, contstruct a union, reposition it and append to *scad3ds*:
            other_pcb_scad3ds: List[Scad3D] = other_pcb_group.scad3ds
            if tracing:
                print(f"{tracing}len(other_pcb_scad3ds):{len(other_pcb_scad3ds)}")
            other_pcb_union: Union3D = Union3D(
                f"'{name}' PCB import from '{group_name}'", other_pcb_scad3ds)
            other_pcb_repositioned: Scad3D = other_pcb_union.reposition(
                f"{name} Reposition", center3d, z_axis, rotate, translate3d)
            scad3ds.append(other_pcb_repositioned)

            # Do the same for pads:
            other_pads_groups: List[PadsGroup] = other_pcb_group.pads_groups
            if tracing:
                print(f"{tracing}len(other_pads_groups):{len(other_pads_groups)}")
            other_pads_group: PadsGroup
            for index, other_pads_group in enumerate(other_pads_groups):
                if tracing:
                    print(f"{tracing} [{index}]len(other_pads_group)={len(other_pads_group)}")
                other_pads_group_repositioned: PadsGroup = other_pads_group.reposition(
                    "", center, rotate, translate)
                if tracing:
                    print(f"{tracing} [{index}]len(other_pads_group_repositioned)="
                          f"{len(other_pads_group_repositioned)}")
                other_pads_group_repositioned.holes_append(pcb_polygon, tracing=next_tracing)
        if tracing:
            print(f"{tracing}len(scad3ds):{len(scad3ds)}")
            print(f"{tracing}len(pcb_polygon):{len(pcb_polygon)}")
            print(f"{tracing}<=PCB.pcb_place('{name}', '{other_pcb.name}', "
                  f"{group_names}, '{flags}'...)")

    # PCB.polygon_get():
    def polygon_get(self) -> Polygon:
        """Return the PCB polygon."""
        return self.pcb_polygon

    # PCB.scad3d_place():
    def scad3d_place(self, scad3d: "Scad3D", flags: str, center: "P2D" = P2D(0.0, 0.0),
                     rotate: float = 0.0, translate: "P2D" = P2D(0.0, 0.0)) -> None:
        """Add a Scad3D to a PCB.

        Args:
            * *scad3d* (*Scad3D*):
              The *Scad3D* object to append to the PCB.
            * *flags* (*str*):
              Some one character flags for placing the *scad3d*:
              * 'b': Place the *scad3d* on the back surface.
              * 't': Place the *scad3d* on the top surface.
            * *center* (*P2D*)
              (Optional: Defaults to *P3D*(0.0, 0.0, 0.0):
              The virtual center of the object to rotate around
              the Z axis.
            * *rotate* (*float*): (Optional: Defaults to 0.0):
              The number of radians to rotate the object by around
              the *center*.
            * *translate* (*P3D*)
              (Optional: Defaults to *P3D*(0.0, 0.0, 0.0)):
              The location to translate *center* to.

        """
        # Unpack some values from *pcb* (i.e. *self*):
        pcb: PCB = self
        dz: float = pcb.dz
        pcb_parent: Optional[PCB] = pcb.pcb_parent
        scad3ds: List[Scad3D] = pcb.scad3ds

        # Mirror operation on *pcb_parent* (if present):
        if isinstance(pcb_parent, PCB):
            pcb_parent.scad3d_place(scad3d, flags,
                                    center=center, rotate=rotate, translate=translate)

        # Verify *flags*:
        flag: str
        for flag in flags:
            assert flag in "bt", f"'{flag}' is not one of 'b' or 't'."

        # Reposition *scad3d* into *repositioned_scad3d* and append to *scad3ds*:
        center3d: P3D = P3D(center.x, center.y, 0.0)
        translate_dz: float = 0.0 if 'b' in flags else dz
        translate3d: P3D = P3D(translate.x, translate.y, translate_dz)
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        repositioned_scad3d: Scad3D = scad3d.reposition(
            f"{scad3d.name} Reposition", center3d, z_axis, rotate, translate3d)
        scad3ds.append(repositioned_scad3d)

    # PCB.scad_program_append():
    def scad_program_append(self, scad_program: ScadProgram,
                            color: str, tracing: str = "") -> Module3D:
        """Merge the..."""
        # Unpack some values from *pcb* (i.e. *self*):
        pcb: PCB = self
        dz: float = pcb.dz
        name: str = pcb.name
        scad3ds: List[Scad3D] = pcb.scad3ds

        # Perform any requested *tracing*:
        # next_tracing: str = ""
        pcb_polygon: Polygon = pcb.pcb_polygon
        pcb_polygon.lock()
        if tracing:
            print(f"{tracing}len(pcb_polygon)={len(pcb_polygon)}")

        duplicates_list: List[List[SimplePolygon]] = pcb_polygon.duplicates_find()
        duplicates_list_size = len(duplicates_list)
        if duplicates_list_size > 0:
            print(f"{tracing}{duplicates_list_size} Duplicate polygons found in PCB '{name}':")
            duplicates: List[SimplePolygon]
            for index, duplicates in enumerate(duplicates_list):
                simple_polygon: SimplePolygon = duplicates[0]
                sw_corner: P2D
                ne_corner: P2D
                sw_corner, ne_corner = simple_polygon.bounding_box_get()
                center: P2D = sw_corner + ne_corner
                dx: float = abs(ne_corner.x - sw_corner.x)
                dy: float = abs(ne_corner.y - sw_corner.y)
                size: int = len(duplicates)
                print(f"{tracing}[{index}] "
                      f"({center.x:.3f}, {center.y:.3f}) {dx:.3f}, {dy:.3f} {size}")

        # Generate the *pcb_module2d* that contains the PCB exterior, connector holes, slots, etc.
        pcb_module2d: Module2D = Module2D(f"{name} PCB", [pcb_polygon])
        scad_program.append(pcb_module2d)
        scad_program.if2d.name_match_append(f"{name.lower()}_pcb",
                                            pcb_module2d, [f"{name} PCB"])
        pcb_module2d_use_module: UseModule2D = pcb_module2d.use_module

        # Extrude end Color the *pcb_polygon*:
        extruded_pcb: LinearExtrude = LinearExtrude(f"{name} Extruded PCB",
                                                    pcb_module2d_use_module, dz)
        translated_pcb: Translate3D = Translate3D(f"{name} Translated PCB", extruded_pcb,
                                                  P3D(0.0, 0.0, 0.0))
        colored_pcb: Color = Color(f"{name} Colored PCB", translated_pcb, color)

        # Glue everything together:
        pcb_board: Module3D = Module3D(f"{name} Board", scad3ds + [colored_pcb])
        scad_program.append(pcb_board)
        scad_program.if3d.name_match_append(f"{name.lower()}_board",
                                            pcb_board, [f"{name} Board"])

        if tracing:
            print(f"{tracing}<=PCB.scad_program_append('{name}, *, '{color}')")
        return pcb_board

    # PCB.show():
    def show(self, flags: str, indent: str):
        """Show the contents of a PCB."""
        pcb: PCB = self
        name: str = pcb.name
        print(f"{indent}=>PCB.show('{name}', '{flags}')")
        print(f"{indent}PCB:  dz:{pcb.dz}")
        pcb_polygon: Polygon = pcb.pcb_polygon
        pcb_polygon_size: int = len(pcb_polygon)
        index: int
        sizes: List[str] = []
        for index in range(pcb_polygon_size):
            sizes.append(str(len(pcb_polygon[index])))
        # print(f"{indent}pcb_polygon:[{' '.join(sizes)}]")
        pcb_groups: Dict[str, PCBGroup] = pcb.pcb_groups
        group_name: str
        pcb_group: PCBGroup
        pairs: List[str] = []
        for group_name, pcb_group in pcb_groups.items():
            pairs.append(f"'{group_name}': {{"
                         f"P{len(pcb_group.pads_groups)};S{len(pcb_group.scad3ds)} }}")
            pass
        print(f"{indent}pcb_groups: {' '.join(pairs)}")

        scad3ds: List[Scad3D] = pcb_group.scad3ds
        names: List[str] = []
        scad3d: Scad3D
        for scad3d in scad3ds:
            names.append(f"'{scad3d.name}'")
            pass
        print(f"{indent}scad3ds:[{' '.join(names)}]")
        print(f"{indent}<=PCB.show('{name}', '{flags}')")

    # PCB.simple_polygon_append():
    def simple_polygon_append(self, simple_polygon: "SimplePolygon") -> None:
        """Append a SimplePolygon to the PCB polygon."""
        # Unpack some values from *pcb* (i.e. *self*):
        pcb: PCB = self
        pcb_parent: Optional[PCB] = pcb.pcb_parent
        pcb_polygon: Polygon = pcb.pcb_polygon

        # Mirror the operation on *pcb_parent* (if present):
        if isinstance(pcb_parent, PCB):
            pcb_parent.simple_polygon_append(simple_polygon)

        # Do the actual append:
        pcb_polygon.append(simple_polygon)


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
        # (Note, PCB_Chunk always makes shallow copies of the lists):
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
        # cuts: List[SimplePolygon] = pcb_chunk.cuts
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
        # next_tracing: str = tracing + " " if tracing else ""
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
            0.0, origin2d, False)

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

        # This stuff is OLD and nees to be eventually deleted:
        # footprint_path = directory / (name + ".kicad_mod")

        # Now start the *footprint*:
        # footprint: Footprint = Footprint("HR2", name)
        # footprint.reference(P2D(0.0, 0.0))

        # Iterate over each *pad* in the *pads*:
        # index: int
        # pad: Pad
        # for index, pad in enumerate(pads):
        #     # Output a pad defintion:
        #     # print(f"footprint_generate[{index}]:{pad}")
        #     footprint.hole(pad.name, pad.pad_center, pad.pad_dx, pad.pad_dy, pad.drill_diameter)

        # back_artwork: SimplePolygon
        # for back_artwork in back_artworks:
        #     footprint.simple_polygon(back_artwork, "B.SilkS", 0.2, tracing=next_tracing)

        # front_artwork: SimplePolygon
        # for front_artwork in front_artworks:
        #     footprint.simple_polygon(front_artwork, "F.SilkS", 0.2, tracing=next_tracing)

        # Save the *footprint* into the file system:
        # footprint.save(footprint_path, "t")

        # For now do not mirror this operation up to the *parent*:
        # parent = parent

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
        module3d: Module3D = Module3D(f"{name}_xpcb", module3d_scads)
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

        # OLD CODE to be removed:

        # Update the `.kicad_pcb` file specified by *kicad_pcb_path* (if not *None*):
        # if isinstance(kicad_pcb_path, Path):
        #     kicad_pcb: KicadPCB = KicadPCB(kicad_pcb_path, P2D(100.0, 100.0))
        #     kicad_pcb.modules_update(all_references, tracing=next_tracing)
        #     kicad_pcb.cuts_update([pcb_exterior] + cuts, tracing=next_tracing)
        #    kicad_pcb.save()

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

        # Create *x_mirrored_pcb_chunk*, update the pads with X axis mirrored ones, and return it:
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

        # Create *y_mirrored_pcb_chunk*, update the pads with Y axis mirrored ones, and return it:
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
        pcb_west_x: float = -14.0  # Trail and error
        pcb_connector_x: float = -motor_casing_dz / 2.0 - 3.0   # Note the X/Z coordinate swap
        pcb_east_x: float = motor_casing_dz / 2.0
        pcb_header_dx: float = 6.5  # Trial and error
        pcb_corner_x: float = -2.5  # Trial and error
        pcb_dy_extra: float = 9.0 * 2.54
        pcb_dz: float = 1.0  # mm
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
        #    |      |
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

        # Create *encoder_pcb* iwth *encoder_exterior* as the PCB outline:
        encoder_pcb: PCB = PCB("Encoder", scad_program, pcb_dz, encoder_exterior)

        # Create *north_header* and *south_header* for (Digikey: 2057-PH1RB-03-UA-ND (Adam Tech))
        # and make sure the header pin holes are appended to *pcb_polygon*:
        header_offset: float = 2.0 * 2.54
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
                                            reposition(origin2d, degrees90, north_header_center2d))
        # m1x3ra_pcb_chunk_north.pads_show("m1x3ra_chunks_north:")
        m1x3ra_pcb_chunk_south: PCBChunk = (m1x3ra_pcb_chunk.
                                            sides_swap().
                                            pads_y_mirror().
                                            scads_y_flip().
                                            reposition(origin2d, degrees90, south_header_center2d))

        # Create *motor_slots_pcb_chunk* (put "8" before "7" because the footprint looks better):
        north_motor_pad: Pad = Pad("8", pcb_pad_dx, pcb_pad_dy,
                                   pcb_drill_diameter, pcb_north_slot_center)
        south_motor_pad: Pad = Pad("7", pcb_pad_dx,
                                   pcb_pad_dy, pcb_drill_diameter, pcb_south_slot_center)
        motor_slots_pcb_chunk: PCBChunk = PCBChunk("motor_slots",
                                                   [north_motor_pad, south_motor_pad], [])

        # Figure out *encoder_pcb_directory* and *encoder_pcb_pretty_directory*:
        assert "HR2_DIRECTORY" in os.environ, "HR2_DIRECTORY environement variable not set"
        hr2_directory: Path = Path(os.environ["HR2_DIRECTORY"])
        encoder_pcb_directory: Path = hr2_directory / "electrical" / "encoder" / "rev_a"
        encoder_pcb_pretty_directory: Path = encoder_pcb_directory / "pretty"
        encoder_pcb_path: Path = encoder_pcb_directory / "encoder.kicad_pcb"

        # Create *encoder_pcb_chunk* that is used on the *encoder_pcb* (see below):
        encoder_pcb_chunk: PCBChunk = PCBChunk.join(
            "Encoder", [m1x3ra_pcb_chunk_north, m1x3ra_pcb_chunk_south, motor_slots_pcb_chunk])
        encoder_pcb_chunk.footprint_generate("HR2", encoder_pcb_pretty_directory)
        encoder_pcb_chunk.pcb_update(scad_program, pcb_origin, 1.6, encoder_exterior, "Purple",
                                     encoder_pcb_path, [])

        # encoder_pcb_chunk.pads_show("encoder_pcb_chunk:")
        # encoder_pcb_chunk.references_show("encoder_pcb_chunk:")

        # references: List[Reference] = [Reference(
        #     "CN1", True, 0.0, P2D(0.0, 0.0), encoder_pcb_chunk)]  # pcb_chunk is bogus; no matter!
        # xencoder_module: Module3D = encoder_pcb_chunk.pcb_update(
        #     scad_program, pcb_dz, encoder_exterior, "Purple", encoder_pcb_path, references)
        # xencoder_module = xencoder_module

        encoder_pcb.pad_append("7", {"motors"}, "",
                               pcb_pad_dx, pcb_pad_dy, pcb_drill_diameter, pcb_north_slot_center)
        encoder_pcb.pad_append("8", {"motors"}, "",
                               pcb_pad_dx, pcb_pad_dy, pcb_drill_diameter, pcb_south_slot_center)

        # f1x3_pcb_chunk_north: PCBChunk = f1x3.reposition(origin2d, degrees90, degrees)
        # f1x3_pcb_chunk_south: PCBChunk = f1x3.reposition(origin2d,
        #                                                  degrees90, south_header_center_2d)
        # master_pcb_chunk: PCB_Chunk = PCBCHunk.Join(
        #     "Encoder_Master", [f1x3_pcb_chunk_north])  # , f1x3_pcb_chunk_south])
        # master_pcb_chunk = master_pcb_chunk

        # Install the connectors to the encoder board:
        encoder_pcb.module3d_place("M1x3RA", {"connectors"}, "bx",
                                   origin2d, degrees90, north_header_center2d)
        encoder_pcb.module3d_place("F1x3", {"connectors_mate"}, "nN",
                                   origin2d, degrees90, north_header_center2d)
        encoder_pcb.module3d_place("M1x3RA", {"connectors"}, "bx",
                                   origin2d, degrees90, south_header_center2d, pads_base=3)
        encoder_pcb.module3d_place("F1x3", {"connectors_mate"}, "nN",
                                   origin2d, degrees90, south_header_center2d, pads_base=3)
        encoder_pcb.mount_hole_append("SHAFT", {"shaft"}, pcb_shaft_hole_diameter, origin2d)

        # Constuct the slots for the motor tabs:
        encoder_pcb.pad_append("7", {"motors"}, "",
                               pcb_pad_dx, pcb_pad_dy, pcb_drill_diameter, pcb_north_slot_center)
        encoder_pcb.pad_append("8", {"motors"}, "",
                               pcb_pad_dx, pcb_pad_dy, pcb_drill_diameter, pcb_south_slot_center)

        # Digikey: 2057-PH1RB-03-UA-ND (Adam Tech):
        # encoder_pcb.footprint_generate(Path("/tmp"), "En",
        #                                {"connectors", "motors", "shaft"}, "U")
        # encoder_pcb.footprint_generate(Path("/tmp"), "ENCODER_MATE",
        #                                {"connectors"}, "U")

        # Wrap up the *encoder_pcb*:

        encoder_module: Module3D = encoder_pcb.scad_program_append(scad_program, "Purple")

        # *translate* is the point on the east motor where the shaft comes out:
        # No need to worry about the west motor because the entire east wheel assembly is
        # is rotated and placed on the west side of the robot.
        translate: P3D = P3D(motor_casing_east_x, 0.0, motor_shaft_z)

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
        self.pcb: PCB = encoder_pcb
        self.translate: P3D = translate

        # Wrap up any requested *tracing*:
        if tracing:
            print(f"{tracing}<=Encoder.__init__(...)")


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
        connector_center: P2D = P2D(0.0, pcb_dy / 2.0 - 4.0 * 2.54 / 2.0)
        transducer_pitch: float = 26.00  # mm
        transducer_diameter: float = 16.00  # mm
        transducer_dz: float = 12.00  # mm
        transducer_left_x: float = transducer_pitch / 2.0
        transducer_right_x: float = -transducer_left_x
        transducer_left_center: P2D = P2D(transducer_left_x, 0.0)
        transducer_right_center: P2D = P2D(transducer_right_x, 0.0)

        # *center_y* is Y center line value:
        center_y: float = 0.0

        # Create the *sonar_pcb*:
        sonar_exterior: Square = Square("HCSR04 PCB", pcb_dx, pcb_dy, center=P2D(0.0, center_y))
        sonar_pcb: PCB = PCB("HCSR04", scad_program, pcb_dz, sonar_exterior)

        # Create the *colored_transducer* and place two of them on *sonar_pcb*:
        transducer_start: P3D = P3D(0.0, center_y, 0.0)
        transducer_end: P3D = P3D(0.0, center_y, transducer_dz)
        transducer_cylinder: Cylinder = Cylinder("Sonar Transducer", transducer_diameter,
                                                 transducer_start, transducer_end, 16)
        colored_transducer: Color = Color("Colored Transducer", transducer_cylinder, "Silver")
        origin2d = P2D(0.0, 0.0)
        sonar_pcb.scad3d_place(colored_transducer, "", origin2d, 0.0, transducer_left_center)
        sonar_pcb.scad3d_place(colored_transducer, "", origin2d, 0.0, transducer_right_center)

        # Install the two mounting holes:
        sonar_pcb.mount_hole_append("NE Mount Hole", {"mounts"}, mount_hole_diameter, mount_hole_ne)
        sonar_pcb.mount_hole_append("SW Mount Hole", {"mounts"}, mount_hole_diameter, mount_hole_sw)

        # Install the connector:
        sonar_pcb.module3d_place("M1x4RA", {"connector"}, "by", origin2d, 0.0, connector_center)

        # Wrap up *sonar_pcb*:
        sonar_module: Module3D = sonar_pcb.scad_program_append(scad_program, "LightGreen")

        # Now create a *vertical_sonar_module* which is centered over the connector and points
        # in +X direction:
        sonar_use_module: UseModule3D = sonar_module.use_module_get()
        recentered_sonar: Translate3D = Translate3D("Recentered HCSR04", sonar_use_module,
                                                    P3D(0.0, -pcb_dy / 2.0, 0.5 * 2.54))
        degrees180: float = pi
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        z_rotated_sonar: Rotate3D = Rotate3D("Z Rotated HCSR04", recentered_sonar,
                                             degrees180, z_axis)
        degrees90: float = degrees180 / 2.0
        x_axis: P3D = P3D(1.0, 0.0, 0.0)
        x_rotated_sonar: Rotate3D = Rotate3D("X Rotated HCS404", z_rotated_sonar,
                                             degrees90, x_axis)
        vertical_sonar_module: Module3D = Module3D("Veritcal HCSR04", [x_rotated_sonar])
        scad_program.append(vertical_sonar_module)
        scad_program.if3d.name_match_append(
            "vertical_sonar", vertical_sonar_module, ["Vertical Sonar"])

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
        tie_down_hole_pad_h_e: Pad = Pad("HCSR04 East Tie Down Hole (Hight)",
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
        origin3d: P3D = P3D(0.0, 0.0, 0.0)
        hcsr04_transducer_module: Module3D = Module3D("HCSR04 Transducer", [colored_transducer])
        hcsr04_transducer_use_module: UseModule3D = hcsr04_transducer_module.use_module_get()
        left_transducer: Scad3D = hcsr04_transducer_use_module.reposition(
            "HCSR04 Left Transducer", origin3d, z_axis, 0.0,
            P3D(transducer_left_center.x, transducer_left_center.y, 0.0))
        right_transducer: Scad3D = hcsr04_transducer_use_module.reposition(
            "HCSR04 Right Transducer", origin3d, z_axis, 0.0,
            P3D(transducer_right_center.x, transducer_right_center.y, 0.0))
        hcsr04_transducers: PCBChunk = PCBChunk("HCSR04 Transducers",
                                                [], [left_transducer, right_transducer])

        # The standard pin number is with pin 1 on the left when viewed from the top.
        # The right-angle connector is installed on the back side so we swap sides
        # without swapping pad numbering:
        m1x4ra: PCBChunk = connectors.m1x4ra.pcb_chunk
        sides_swapped_m1x4ra: PCBChunk = m1x4ra.sides_swap()
        repositioned_m1x4: PCBChunk = sides_swapped_m1x4ra.reposition(origin2d, 0.0,
                                                                      connector_center)
        centered_hcsr04: PCBChunk = PCBChunk.join("Centered HCSR04",
                                                  [hcsr04_transducers, repositioned_m1x4])
        edge_center: P2D = P2D(0.0, pcb_dy / 2.0)
        hcsr04_pcb_chunk: PCBChunk = centered_hcsr04.reposition(origin2d, 0.0, edge_center)
        edge_sonar_exterior: SimplePolygon = sonar_exterior.reposition(origin2d, 0.0, edge_center)

        # Generate the *hrsr04_module* but there is no associated `.kicad_pcb`):
        hcsr04_module: Module3D = hcsr04_pcb_chunk.pcb_update(
            scad_program, pcb_origin, pcb_dz, edge_sonar_exterior, "LightGreen", None, [])
        hcsr04_module = hcsr04_module  # Eventual this will be used.

        # Stuff some values into *hrcsr04* (i.e. *self*):
        # hcsr04: HCSR04 = self
        self.module: Module3D = sonar_module
        self.pcb: PCB = sonar_pcb
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
                 base_dz: float, fin_dx: float, fin_count: int, color: str) -> None:
        """Initialize a HeatSink 3D model."""
        fin_spacing: float = float(fin_count - 1) * fin_dx
        start_x: float = -(dx / 2.0 - fin_dx / 2.0)
        # Note that outline is in X/Y plane, so Z is mapped to Y:
        top_y: float = dz
        middle_y: float = base_dz
        bottom_y: float = 0

        # Crude ASCII art of what is desired:
        #
        #     B--C   B--C   B--C   B--C   B--C
        #     |  |   |  |   |  |   |  |   |  |
        #     |  |   |  |   |  |   |  |   |  |
        #     |  |   |  |   |  |   |  |   |  |
        #     A  D---A  D---A  D---A  D---A  D
        #     |                              |
        #     a------------------------------d

        # Create the end *profile* as show immediately above:
        heat_sink_points: List[P2D] = []
        index: int
        x: float
        for index in range(fin_count):
            x = start_x + float(index) * fin_spacing
            x1: float = x - fin_dx / 2.0
            x2: float = x + fin_dx / 2.0
            start_y: float = bottom_y if index == 0 else middle_y  # a or A
            end_y: float = bottom_y if index == fin_count - 1 else middle_y  # d or D
            heat_sink_points.append(P2D(start_y, x1))  # A or a (pick "a" for index == first)
            heat_sink_points.append(P2D(top_y, x1))    # B
            heat_sink_points.append(P2D(top_y, x2))    # C
            heat_sink_points.append(P2D(end_y, x2))    # D or d (pick "d" for index == last)
        heat_sink_profile: SimplePolygon = SimplePolygon(f"{name} Outline",
                                                         heat_sink_points, lock=True)

        # Now make *extruded_heat_sink*:
        extruded_heat_sink: LinearExtrude = LinearExtrude(f"Extruded {name}", heat_sink_profile, dy)

        # Now rotate and translate to get a new *repositioned_heat_sink* that is origin centered:
        x_axis: P3D = P3D(1.0, 0.0, 0.0)
        bottom_center: P3D = P3D(0.0, 0.0, dz / 2.0)
        origin3d: P3D = P3D(0.0, 0.0, 0.0)
        degrees90: float = pi / 2.0
        repositioned_heat_sink: Scad3D = extruded_heat_sink.reposition(
            f"Repositioned {name}", bottom_center, x_axis, degrees90, origin3d)
        repositioned_heat_sink = repositioned_heat_sink
        heat_sink_module: Module3D = Module3D(f"{name} Module", [repositioned_heat_sink])
        scad_program.append(heat_sink_module)
        scad_program.if3d.name_match_append(name, heat_sink_module, ["name"])

        # Load values into *heat_sink* (i.e. *self*):
        # heat_sink: HeatSink = self
        self.module: Module3D = heat_sink_module


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
    def __init__(self, scad_program: ScadProgram, base_dxf: BaseDXF, connectors: Connectors,
                 pi_board_z: float, master_board_z: float, arm_z: float) -> None:
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
             base_battery_top_z, pi_board_z, spacer_male_height),
            ("Pi NW", "BATTERY: Upper Slot (2, 1)",
             base_battery_top_z, pi_board_z, spacer_male_height),
            ("Pi SE", "RIGHT: LOWER Small Hex Slot (3, 0)",
             base_top_z, pi_board_z, 0.0),
            ("Pi SW", "LEFT: LOWER Small Hex Slot (3, 0)",
             base_top_z, pi_board_z, 0.0),
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
        debug_dz: float = 0.0  # + 0.250
        spacers: List[Scad3D] = []
        spacer_name: str
        key_name: str
        bottom_z: float
        top_z: float
        male_height: float
        for spacer_name, key_name, bottom_z, top_z, male_height in spacer_tuples:
            # Lookup the *key_name* and extract (*key_x*, *key_y*) location:
            key: Tuple[Any, ...] = romi_base_key_table[key_name]
            key2: Any = key[2]
            key3: Any = key[3]
            assert isinstance(key2, float) and isinstance(key3, float)
            key_x: float = float(key2)
            key_y: float = float(key3)

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
    def __init__(self,
                 scad_program: ScadProgram, pcb_origin: P2D, connectors: Connectors,
                 hr2_pi_assembly: "HR2PiAssembly", base_dxf: BaseDXF,
                 pi_board_z, master_board_z: float, nucleo_board_z: float, arm_z: float,
                 pi_offset2d: P2D, nucleo_offset2d: P2D, st_link_offset2d: P2D,
                 encoder: "Encoder", raspi3b: "RaspberryPi3", nucleo144: "Nucleo144",
                 st_link: "STLink",
                 romi_base_keys: List[Tuple[Any, ...]],
                 romi_expansion_plate_keys: List[Tuple[Any, ...]]) -> None:
        """Initialize the HR2MasterAssembly."""
        # print(f"HR2MasterAssembly: nucleo_offset2d:{nucleo_offset2d}")
        master_board: MasterBoard = MasterBoard(scad_program, pcb_origin, base_dxf, connectors,
                                                encoder, raspi3b, nucleo144, st_link,
                                                pi_offset2d, nucleo_offset2d, st_link_offset2d,
                                                master_board_z, nucleo_board_z, arm_z,
                                                romi_base_keys, romi_expansion_plate_keys)

        # Create *module*, append to *scad_program* and save into *hr2_master_assembly*
        # (i.e. *self*):
        master_board_use_module: UseModule3D = master_board.module.use_module_get()
        translated_master_board: Translate3D = Translate3D(
            "MasterBoard Translated", master_board_use_module, P3D(0.0, 0.0, master_board_z))
        module: Module3D = Module3D("HR2 Master Assembly", [
            hr2_pi_assembly.module.use_module_get(),
            translated_master_board])
        scad_program.append(module)
        # hr2_master_assembly: HR2MasterAssembly = self
        self.module = module
        scad_program.if3d.name_match_append("hr2_master_assembly", module, ["HR2 Base Assembly"])


# HR2NucleoAssembly:
class HR2NucleoAssembly:
    """Represents the HR2 up to the Nucleo144."""

    # HR2NucleoAssembly.__init__():
    def __init__(self, scad_program: ScadProgram, connectors: Connectors,
                 hr2_wheel_assembly: "HR2WheelAssembly",
                 nucleo144: "Nucleo144", nucleo_board_z: float, nucleo_offset2d: P2D) -> None:
        """Initialize the HR2NucleoAssembly."""
        # Create *module*, append to *scad_program* and save into *hr2_nucleo_assembly*
        # (i.e. *self*):

        nucleo144_module: Module3D = nucleo144.module
        nucleo144_use_module: UseModule3D = nucleo144_module.use_module_get()
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        degrees90: float = pi / 2.0
        rotated_nucleo144: Rotate3D = Rotate3D("Rotated Nucleo144 PCB",
                                               nucleo144_use_module, -degrees90, z_axis)
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
                 connectors: Connectors, raspi3b: "RaspberryPi3", st_link: "STLink",
                 pi_board_z: float, pi_offset2d: P2D, st_link_offset: P2D) -> None:
        """Initialize the HR2BaseAssembly."""
        # Define some constants
        degrees90: float = pi / 2.0
        origin3d: P3D = P3D(0.0, 0.0, 0.0)
        z_axis: P3D = P3D(0.0, 0.0, 1.0)

        # Create the *other_pi* and rotate and translate to the correct location:
        OtherPi(scad_program)
        other_pi_board: Module3D = scad_program.module3d_get("OtherPi_Board")
        other_pi_board_use_module: UseModule3D = other_pi_board.use_module_get()
        pi_reposition: P3D = P3D(pi_offset2d.x, pi_offset2d.y, pi_board_z)
        repositioned_other_pi: Scad3D = other_pi_board_use_module.reposition(
            "Reposition Other Pi", origin3d, z_axis, degrees90, pi_reposition)

        # Create the *raspberry_pi3* and rotate and translate to the correct location.:
        raspi3b_board_module: Module3D = scad_program.module3d_get("RasPi3B_Board")
        raspi3b_board_use_module: UseModule3D = raspi3b_board_module.use_module_get()
        repositioned_raspi3b: Scad3D = raspi3b_board_use_module.reposition(
            "Reposition Raspberry Pi 3B", origin3d, z_axis, degrees90, pi_reposition)

        # Create *module*, append to *scad_program* and save into *hr2_base_assembly* (i.e. *self*):
        module: Module3D = Module3D("HR2 Pi Assembly", [
            hr2_base_assembly.module.use_module_get(),
            repositioned_other_pi, repositioned_raspi3b])
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
        arm_z: float = master_board_z + 26.00

        # Now create the *raspi3b* and the *st_link*
        raspi3b: RaspberryPi3 = RaspberryPi3(scad_program, connectors)
        st_link: STLink = STLink(scad_program, connectors)

        # Create the *nucleo144* before *master_board* so it can be passed in:
        nucleo_offset2d: P2D = P2D(pi_x + 8.5, pi_y - 1.0)
        # print(f"HR2PiAssembly:nucleo_offset2d:{nucleo_offset2d}")
        nucleo144: Nucleo144 = Nucleo144(scad_program, connectors)

        # Create the *romi_expansion_plate* before *master_board* so it can be passed in:
        romi_expansion_plate: RomiExpansionPlate = RomiExpansionPlate(scad_program)
        romi_expansion_plate_keys: List[Tuple[Any, ...]] = romi_expansion_plate.keys_get()

        # Create the *hr2_base_assembly* object that can accept the various PCB's and assemblies
        # that go on top of it:
        hr2_base_assembly: HR2BaseAssembly = HR2BaseAssembly(scad_program, base_dxf, connectors,
                                                             pi_board_z, master_board_z, arm_z)
        hr2_pi_assembly: HR2PiAssembly = HR2PiAssembly(scad_program, hr2_base_assembly,
                                                       connectors, raspi3b, st_link, pi_board_z,
                                                       pi_offset2d, st_link_offset2d)
        romi_base_keys: List[Tuple[Any, ...]] = hr2_base_assembly.romi_base_keys_get()
        hr2_master_assembly: HR2MasterAssembly = HR2MasterAssembly(
            scad_program, pcb_origin, connectors, hr2_pi_assembly, base_dxf,
            pi_board_z, master_board_z, nucleo_board_z, arm_z,
            pi_offset2d, nucleo_offset2d, st_link_offset2d,
            encoder, raspi3b, nucleo144, st_link,
            romi_base_keys, romi_expansion_plate_keys)
        hr2_wheel_assembly: HR2WheelAssembly = HR2WheelAssembly(scad_program, hr2_master_assembly,
                                                                connectors,
                                                                base_dxf, encoder)

        hr2_nucleo_assembly: HR2NucleoAssembly = HR2NucleoAssembly(
            scad_program, connectors, hr2_wheel_assembly, nucleo144,
            nucleo_board_z, nucleo_offset2d)
        hr2_arm_assembly: HR2ArmAssembly = HR2ArmAssembly(scad_program, hr2_nucleo_assembly,
                                                          romi_expansion_plate, arm_z)
        hr2_arm_assembly = hr2_arm_assembly


# HR2WheelAssembly:
class HR2WheelAssembly:
    """Represents HR2 with wheels, motors, and encoders installed."""

    # HR2WheelAssemlby.__init__():
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
        next_tracing: str = ""
        if tracing:
            print(f"{tracing}=>Nucleo144.__init()")
            next_tracing = tracing + " "

        # Misc. constants:
        degrees90: float = pi / 2.0
        origin: P2D = P2D(0, 0)
        # morpho_pin_columns: int = 35
        # morpho_pin_rows: int = 2
        mount_hole_diameter: float = 3.20

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
        nucleo_pcb: PCB = PCB("Nucleo144", scad_program, pcb_dz, nucleo_exterior)
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
        nucleo_pcb.scad3d_place(colored_ethernet, "")

        # To improve visibility cut a hole into the center of the PCB:
        pcb_cut_out: Square = Square("PCB Cut-Out", .575 * pcb_dx, .75 * pcb_dy)
        nucleo_pcb.simple_polygon_append(pcb_cut_out)

        # Insert the 4 ZIO connectors:
        nucleo_pcb.module3d_place("F2x10_Long", {"zio"}, "", origin, degrees90,
                                  P2D(cn7_zio_center_x, cn7_zio_center_y))
        nucleo_pcb.module3d_place("F2x8_Long", {"zio"}, "", origin, degrees90,
                                  P2D(cn8_zio_center_x, cn8_zio_center_y))
        nucleo_pcb.module3d_place("F2x15_Long", {"zio"}, "", origin, degrees90,
                                  P2D(cn9_zio_center_x, cn9_zio_center_y))
        nucleo_pcb.module3d_place("F2x17_Long", {"zio"}, "", origin, degrees90,
                                  P2D(cn10_zio_center_x, cn10_zio_center_y))

        # Create the 2 morpho connectors and 2 associated ground connectors:
        # # Digikey: SAM1066-40-ND; pcb_pin=2.79  insulation=2.54  mating_length=15.75
        # # Digikey: S2212EC-40-ND; pcb_pin=3.05  insulation=2.50  mating_length=8.08  price=$1.15/1
        nucleo_pcb.module3d_place("M2x35Long", {"morpho"}, "bx", origin, degrees90,
                                  cn11_morpho_center, pads_base=1100, tracing=next_tracing)
        nucleo_pcb.module3d_place("M2x35Long", {"morpho"}, "bx", origin, degrees90,
                                  cn12_morpho_center, pads_base=1200)
        nucleo_pcb.module3d_place("M1x2", {"ground"}, "bx", origin, 0.0,
                                  P2D(cn11_morpho_center_x, ground_south_y))
        nucleo_pcb.module3d_place("M1x2", {"ground"}, "bx", origin, 0.0,
                                  P2D(cn12_morpho_center_x, ground_south_y))

        # Create the associated 2 mating morpho connectors:
        nucleo_pcb.module3d_place("F2x35", {"morpho_mate"}, "nN", origin, degrees90,
                                  cn11_morpho_center, pads_base=1100)
        nucleo_pcb.module3d_place("F2x35", {"morpho_mate"}, "nN", origin, degrees90,
                                  cn12_morpho_center, pads_base=1200)

        # Create a list of *mount_hole_keys* that specify where each mount holes goes:
        nucleo_pcb.mount_hole_append("NE Mount Hole H2", {"arduino"}, mount_hole_diameter,
                                     P2D(ne_mount_hole_x, ne_mount_hole_y))
        nucleo_pcb.mount_hole_append("NW Mount Hole H3", {"arduino"}, mount_hole_diameter,
                                     P2D(nw_mount_hole_x, nw_mount_hole_y))
        nucleo_pcb.mount_hole_append("SE Mount Hole H4", {"arduino"}, mount_hole_diameter,
                                     P2D(se_mount_hole_x, se_mount_hole_y))
        nucleo_pcb.mount_hole_append("SW Mount Hole H5", {"arduino"}, mount_hole_diameter,
                                     P2D(sw_mount_hole_x, sw_mount_hole_y))
        nucleo_pcb.mount_hole_append("Center Mount Hole H1", {"align"}, mount_hole_diameter,
                                     P2D(center_mount_hole_x, center_mount_hole_y))

        # Generate the ST144MORPHO KiCad footprint:
        nucleo_pcb.footprint_generate(Path("/tmp"), "ST144MORPHO",
                                      {"morpho", "arduino", "align"}, "U")

        # Wrap up *nucleo_pcb*:
        nucleo_board: Module3D = nucleo_pcb.scad_program_append(scad_program, "White")

        # Stuff a some values into *nucleo144* (i.e. *self*):
        # nucleo144: Nucleo144 = self
        self.module: Module3D = nucleo_board
        self.pcb: PCB = nucleo_pcb

        if tracing:
            nucleo_pcb.show("", next_tracing)
            print(f"{tracing}<=Nucleo144.__init()")


# MasterBoard:
class MasterBoard:
    """Represents Master PCB that the various Pi boards mount to."""

    # MasterBoard.__init__():
    def __init__(self, scad_program: ScadProgram, pcb_origin: P2D, base_dxf: BaseDXF,
                 connectors: Connectors, encoder: "Encoder", raspi3b: "RaspberryPi3",
                 nucleo144: "Nucleo144", st_link: "STLink", pi_offset: P2D, nucleo_offset: P2D,
                 st_link_offset: P2D, master_board_bottom_z: float, nucleo_board_z: float,
                 arm_z: float, romi_base_keys: List[Tuple[Any, ...]],
                 romi_expansion_plate_keys: List[Tuple[Any, ...]], tracing: str = "") -> None:
        """Initialize the MasterBoard."""
        master_board: MasterBoard = self

        next_tracing: str = tracing + " " if tracing else ""
        if tracing:
            next_tracing = tracing + " "
            print(f"{tracing}=>MasterBoard.__init__(...)")

        # Define some PCB constants
        pcb_bottom_z: float = 0.0
        pcb_dz: float = 1.6
        pcb_top_z: float = pcb_bottom_z + pcb_dz

        # Create the various PCB's:
        master_pcb: PCB
        center_pcb: PCB
        ne_pcb_pcb: PCB
        nw_pcb_pcb: PCB
        se_pcb_pcb: PCB
        sw_pcb_pcb: PCB
        (master_pcb, center_pcb, ne_pcb, nw_pcb, se_pcb,
         sw_pcb) = master_board.pcbs_create(scad_program, pcb_dz)

        # Install various cut out slots, mount_holes and spacers:
        master_board.cut_holes_append(center_pcb, pi_offset)
        master_board.spacer_mounts_append(center_pcb, ne_pcb, nw_pcb, romi_base_keys)

        # Install the nucleo144 and Raspberry Pi connectors and mounting holes:
        origin2d: P2D = P2D(0.0, 0.0)
        degrees180: float = pi
        degrees90: float = degrees180 / 2.0
        center_pcb.pcb_place(nucleo144.pcb, {"morpho_mate", "arduino"}, "",
                             origin2d, -degrees90, nucleo_offset)
        center_pcb.pcb_place(raspi3b.pcb, {"connector_mate"}, "",
                             origin2d, degrees90, pi_offset)

        # Install the two encoder connectors:
        encoder_offset: P2D = encoder.offset
        center_pcb.pcb_place(encoder.pcb, {"connectors_mate"},
                             "West Encoder Connectors", origin2d, 0.0, encoder_offset)
        center_pcb.pcb_place(encoder.pcb, {"connectors_mate"},
                             "East Encoder Connectors", origin2d, degrees180, -encoder_offset)

        # center_pcb.pcb_place(st_link.pcb, {"connectors_mate"}, "b",
        #                       origin2d, 0.0, st_link_offset)

        # Create the *arm_spacers* and install the appropriate arm spacer mounting holes:
        arm_spacers: List[Scad3D] = master_board.arm_spacers_append(
            scad_program, romi_expansion_plate_keys, center_pcb, ne_pcb, nw_pcb, pcb_top_z)

        # Install the *nucleo144* mounting holes into *pcb_polygon* and create a list
        # of *nucleo144_spacer*:
        # Set *spacer_debug_dz* to non zero to show some gap space above and below the spacer:
        # spacer_debug_dz: float = 0.0 + 0.250
        # nucleo144_offset: P3D = nucleo144.offset
        # spacer_height: float = abs(nucleo_board_z - pcb_top_z) - 2.0 * spacer_debug_dz
        nucleo144_spacers: List[Scad3D] = []

        # Create and install all of the sonars:
        hcsr04: HCSR04 = HCSR04(scad_program, connectors, pcb_origin)
        master_board.sonar_modules_create(scad_program, hcsr04, connectors)
        center_sonars_pcb_chunk: PCBChunk
        ne_sonars_pcb_chunk: PCBChunk
        nw_sonars_pcb_chunk: PCBChunk
        center_sonars_pcb_chunk, ne_sonars_pcb_chunk, nw_sonars_pcb_chunk = (
            master_board.sonars_install(hcsr04, center_pcb, ne_pcb, nw_pcb, connectors,
                                        tracing=next_tracing))

        # This is where we *st_link_use_modle* and translate it to the desired location:
        st_link_use_module: UseModule3D = st_link.module.use_module_get()
        translated_st_link: Scad3D = Translate3D(
            "Translated ST Link", st_link_use_module,
            P3D(st_link_offset.x, st_link_offset.y, 0.0))  # 40.0))  # -10.0))

        # Write the *external_polygon* and *kicad_holes* out to *kicad_file_name*:
        # kicad_file_name: str = "../electrical/master_board/rev_a/master_board.kicad_pcb"
        # kicad_pcb: KicadPcb = KicadPcb(kicad_file_name, P2D(100.0, 100.0))
        # edge_cuts: str = "Edge.Cuts"
        # margin: str = "Margin"
        # kicad_pcb.layer_remove(edge_cuts)
        # kicad_pcb.layer_remove(margin)
        # kicad_pcb.mounting_holes_update(kicad_mounting_holes)
        # kicad_pcb.save()

        # Create the 6 PCB modules:
        master_pcb_module: Module3D = master_pcb.scad_program_append(scad_program, "Yellow")
        center_pcb_module: Module3D = center_pcb.scad_program_append(scad_program, "Gray")
        ne_pcb_module: Module3D = ne_pcb.scad_program_append(scad_program, "Blue")
        nw_pcb_module: Module3D = nw_pcb.scad_program_append(scad_program, "Orange")
        se_pcb_module: Module3D = se_pcb.scad_program_append(scad_program, "Purple")
        sw_pcb_module: Module3D = sw_pcb.scad_program_append(scad_program, "Red")

        # Create *module*, append it to *scad_program* and stuff it into *master_pcb* (i.e. *self*):
        module: Module3D = Module3D("Master Board Module",
                                    (  # encoder_receptacles_use_modules +
                                     nucleo144_spacers +
                                     arm_spacers +
                                     # st_link_connectors +
                                     [  # master_pcb_module.use_module_get(),
                                      center_pcb_module.use_module_get(),
                                      ne_pcb_module.use_module_get(),
                                      nw_pcb_module.use_module_get(),
                                      se_pcb_module.use_module_get(),
                                      sw_pcb_module.use_module_get(),
                                      translated_st_link]))
        scad_program.append(module)
        scad_program.if3d.name_match_append("master_board", module, ["Master Board"])

        # Squirt everything into the associated KiCad PCB's:
        assert "HR2_DIRECTORY" in os.environ, "HR2_DIRECTORY environement variable not set"
        hr2_directory: Path = Path(os.environ["HR2_DIRECTORY"])
        master_board_directory: Path = hr2_directory / "electrical" / "master_board" / "rev_a"
        master_kicad_pcb_path: Path = master_board_directory / "master.kicad_pcb"

        center_pcb_chunk: PCBChunk = PCBChunk.join("XCenter", [
            center_sonars_pcb_chunk,
        ])
        center_kicad_pcb_path: Path = master_board_directory / "center.kicad_pcb"
        xcenter_module: Module3D = center_pcb_chunk.pcb_update(
            scad_program, pcb_origin, pcb_dz,
            center_pcb.pcb_exterior, "Tan", center_kicad_pcb_path, [])
        xcenter_module = xcenter_module
        scad_program.if3d.name_match_append("xcenter_xboard", xcenter_module, ["XCenterBoard"])
        print(f"xcenter_module.name='{xcenter_module.name}'==================")

        ne_pcb_chunk: PCBChunk = PCBChunk.join("XNE", [
            ne_sonars_pcb_chunk,
        ])
        ne_kicad_pcb_path: Path = master_board_directory / "ne.kicad_pcb"
        xne_module: Module3D = ne_pcb_chunk.pcb_update(
            scad_program, pcb_origin, pcb_dz, ne_pcb.pcb_exterior, "Green", ne_kicad_pcb_path, [])
        xne_module = xne_module

        nw_pcb_chunk: PCBChunk = PCBChunk.join("XNW", [
            nw_sonars_pcb_chunk,
        ])
        nw_kicad_pcb_path: Path = master_board_directory / "nw.kicad_pcb"
        xnw_module: Module3D = nw_pcb_chunk.pcb_update(
            scad_program, pcb_origin, pcb_dz, nw_pcb.pcb_exterior, "Orange", nw_kicad_pcb_path, [])
        xnw_module = xnw_module

        se_pcb_chunk: PCBChunk = PCBChunk.join("XSE", [])
        se_kicad_pcb_path: Path = master_board_directory / "se.kicad_pcb"
        xse_module: Module3D = se_pcb_chunk.pcb_update(
            scad_program, pcb_origin, pcb_dz, se_pcb.pcb_exterior, "Purple", se_kicad_pcb_path, [])
        xse_module = xse_module

        sw_pcb_chunk: PCBChunk = PCBChunk.join("XSW", [])
        sw_kicad_pcb_path: Path = master_board_directory / "sw.kicad_pcb"
        xsw_module: Module3D = sw_pcb_chunk.pcb_update(
            scad_program, pcb_origin, pcb_dz, sw_pcb.pcb_exterior, "Red", sw_kicad_pcb_path, [])
        xsw_module = xsw_module

        # Create the *PCBChunk*'s for all 6 boards:
        master_pcb_chunk: PCBChunk = PCBChunk.join("Master", [
            center_pcb_chunk,
            ne_pcb_chunk,
            nw_pcb_chunk,
            se_pcb_chunk,
            sw_pcb_chunk])

        xmaster_module: Module3D = master_pcb_chunk.pcb_update(
            scad_program, pcb_origin, pcb_dz, master_pcb.pcb_exterior,
            "Yellow", master_kicad_pcb_path, [])
        xmaster_module = xmaster_module

        # Stuff some values into *master_board* (i.e. *self*):
        # master_board: MasterBoard = self
        self.module: Module3D = module
        self.master_pcb_chunk: PCBChunk = master_pcb_chunk
        self.master_pcb_module: Module3D = master_pcb_module
        self.center_pcb_chunk: PCBChunk = center_pcb_chunk
        self.center_pcb_module: Module3D = center_pcb_module
        self.ne_pcb_chunk: PCBChunk = ne_pcb_chunk
        self.ne_pcb_module: Module3D = ne_pcb_module
        self.nw_pcb_chunk: PCBChunk = nw_pcb_chunk
        self.nw_pcb_module: Module3D = nw_pcb_module
        self.se_pcb_chunk: PCBChunk = se_pcb_chunk
        self.se_pcb_module: Module3D = se_pcb_module
        self.sw_pcb_chunk: PCBChunk = sw_pcb_chunk
        self.sw_pcb_module: Module3D = nw_pcb_module

        # Wrap up any requested *tracing*:
        if tracing:
            next_tracing = tracing + " "
            print(f"{tracing}<=MasterBoard.__init__(...)")

    # MasterBoard.arm_spacers_append():
    def arm_spacers_append(self, scad_program: ScadProgram,
                           romi_expansion_plate_keys: List[Tuple[Any, ...]],
                           center_pcb: PCB, ne_pcb: PCB, nw_pcb: PCB,
                           pcb_top_z: float) -> List[Scad3D]:
        """Return the arm spacers and install the mounting holes."""
        # Grab the *arm_plate_keys* and build *arm_plate_keys_table*:
        arm_plate_keys: List[Tuple[Any, ...]] = romi_expansion_plate_keys
        arm_plate_key_table: Dict[str, Tuple[Any, ...]] = {}
        arm_plate_key: Tuple[Any, ...]
        for arm_plate_key in arm_plate_keys:
            name = arm_plate_key[1]
            arm_plate_key_table[name] = arm_plate_key

        # Define all of the arm spacer hole locations
        # Note: due to 180 degree rotation large hole origin is in upper right:
        arm_spacer_tuples: List[Tuple[str, str, str, str, PCB]] = [
            ("SE Spacer", "LEFT: Middle Triple Hole", "H10", "H14", center_pcb),
            ("SW Spacer", "RIGHT: Middle Triple Hole", "H11", "H15", center_pcb),
            ("NE Spacer", "Angle Hole[0,0]", "H12", "H16", ne_pcb),
            ("NW Spacer", "Angle Hole[5,0]", "H13", "H17", nw_pcb),
        ]

        # Iterate over *arm_spacer_tuples* and append the results to *arm_spacers*:
        arm_spacers: List[Scad3D] = []
        arm_spacer_name: str
        arm_key_name: str
        arm_kicad_name1: str
        arm_kicad_name2: str
        for (arm_spacer_name, arm_key_name,
             arm_kicad_name1, arm_kicad_name2, pcb) in arm_spacer_tuples:
            # Unpack *arm_spacer_tuple*:

            # Lookup up *arm_key_name* and extract the (*arm_key_x*, *arm_key_y*) location:
            assert arm_key_name in arm_plate_key_table, (f"'{arm_key_name}' not in "
                                                         f"[{list(arm_plate_key_table.keys())}]")
            arm_key: Tuple[Any, ...] = arm_plate_key_table[arm_key_name]
            # Note that the expansion plate is rotated around the Z axis by 180 degrees.
            # This means that the coordinates need to be reflected with minus signs:
            arm_key2: Any = arm_key[2]
            arm_key3: Any = arm_key[3]
            assert isinstance(arm_key2, float) and isinstance(arm_key3, float)
            arm_key_x: float = -arm_key2
            arm_key_y: float = -arm_key3
            # arm_key_diameter: float = arm_key[4]
            # kicad_mounting_holes[arm_kicad_name1] = (P2D(arm_key_x, arm_key_y), arm_key_diameter)
            # kicad_mounting_holes[arm_kicad_name2]= (P2D(-arm_key_x, -arm_key_y), arm_key_diameter)

            # Construct the *arm_spacer* and append to the *spacers*:
            arm_spacer_height: float = 30.0
            arm_spacer_bottom_center: P3D = P3D(arm_key_x, arm_key_y, pcb_top_z)
            arm_spacer = Spacer(scad_program, f"{arm_spacer_name} Arm Spacer",
                                arm_spacer_height, "M2", diameter=3.50,
                                bottom_center=arm_spacer_bottom_center)
            arm_spacers.append(arm_spacer.module.use_module_get())

            # Now add the mounting holes to the PCB.
            arm_hole_diameter: float = 2.4
            pcb.mount_hole_append(f"{arm_spacer_name} Arm Spacer Hole 1", {"mounts"},
                                  arm_hole_diameter, P2D(arm_key_x, arm_key_y))
            pcb.mount_hole_append(f"{arm_spacer_name} Arm Spacer Hole 1", {"mounts"},
                                  arm_hole_diameter, P2D(-arm_key_x, -arm_key_y))
        assert len(arm_spacers), "Something failed"
        return arm_spacers

    def cut_holes_append(self, center_pcb: PCB, pi_offset: P2D):
        """Cut various holes into MasterBoard."""
        # We need to cut out the two slots needed for the two Raspberry Pi flex connectors:
        # The camera slot is 17mm x 2mm and it is centered at (45, 11.5) from the lower left corner.
        # The lcd slot is 5mm x 17mm and it is centered at (2.5, 19.5 + 17/2) from the lower left
        # corner.

        camera_slot_dx: float = 2.00
        camera_slot_dy: float = 17.00
        camera_slot_center_dx: float = 45.00
        camera_slot_center_dy: float = 11.50
        degrees90: float = pi / 2.0
        lcd_slot_dx: float = 5.00
        lcd_slot_dy: float = 17.00
        lcd_slot_center_dx: float = 0.0 + lcd_slot_dx / 2.0
        lcd_slot_center_dy: float = 19.50 + lcd_slot_dy / 2.0
        pi_dx: float = 65.00
        pi_dy: float = 56.50

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
        center_pcb.pcb_cut_hole_append(lcd_slot)
        center_pcb.pcb_cut_hole_append(camera_slot)

    # MasterBoard.pcbs_create():
    def pcbs_create(self, scad_program: "ScadProgram",
                    pcb_dz: float) -> Tuple[PCB, PCB, PCB, PCB, PCB, PCB]:
        """Create the master board PCB's with exterior contours."""
        # This routine creates 6 *PCB*'s named "Center", "NE", "NW", "SE", "SW", "Master".
        # When the first 5 boards are attached to one another, they form the final "Master" *PCB*.
        # The first 5 boards are designed to be manufatured using the Bantam Labs PCB milling
        # machine and associated software to allow rapid prototyping.  The Band Labs PCB milling
        # machine has a maximum size contraint of 5in by 4in.  There 5 individual boards will
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
        arm_well_dx: float = 15.0  # mm (trail and error)
        arm_well_north: float = -50.0  # mm (trail and error)
        # bantam_dx: float = 5 * 25.4  # mm (size of Bantam Labs PCB blank)
        bantam_dy: float = 4 * 25.4  # mm (size of Bantam Labs PCB blank)
        center_pcb_north: float = wheel_well_dy / 2.0 + 3.0  # mm (trail and error)
        center_pcb_south: float = center_pcb_north - (bantam_dy - 4.0)  # mm (trail and error)
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

        # Create the *master_board_pcb* polygon:
        master_pcb_exterior: SimplePolygon = SimplePolygon("Master PCB Exterior Simple Polygon",
                                                           [], lock=False)
        master_pcb_exterior.corner_arc_append(A, corner_radius, "SE")
        master_pcb_exterior.rounded_arc_append("BH++EV++", origin, radius,
                                               a_angle, c_angle, corner_radius)
        master_pcb_exterior.corner_arc_append(D, corner_radius, "NW")
        master_pcb_exterior.corner_arc_append(E, corner_radius, "EN")
        master_pcb_exterior.rounded_arc_append("BV-+EH++", origin, radius,
                                               f_angle, h_angle, corner_radius)
        master_pcb_exterior.corner_arc_append(H, corner_radius, "WS")
        master_pcb_exterior.corner_arc_append(I, corner_radius, "NE")
        master_pcb_exterior.corner_arc_append(J, corner_radius, "WS")
        master_pcb_exterior.corner_arc_append(K, corner_radius, "NW")
        master_pcb_exterior.corner_arc_append(L, corner_radius, "ES")
        master_pcb_exterior.corner_arc_append(M, corner_radius, "NW")
        master_pcb_exterior.rounded_arc_append("BH-+EV-+", origin, radius,
                                               m_angle, p_angle, corner_radius)
        master_pcb_exterior.corner_arc_append(Q, corner_radius, "SE")
        master_pcb_exterior.corner_arc_append(R, corner_radius, "WS")
        # master_pcb_exterior.point_append(Q)
        # master_pcb_exterior.point_append(R)
        master_pcb_exterior.rounded_arc_append("BV++EH-+", origin, radius,
                                               s_angle, v_angle, corner_radius)
        master_pcb_exterior.corner_arc_append(V, corner_radius, "EN")
        master_pcb_exterior.corner_arc_append(W, corner_radius, "SW")
        master_pcb_exterior.corner_arc_append(X, corner_radius, "EN")
        master_pcb_exterior.corner_arc_append(Y, corner_radius, "SE")
        master_pcb_exterior.corner_arc_append(Z, corner_radius, "WN")
        master_pcb_exterior.corner_arc_append(A, corner_radius, "SE")  # Strange! no rounded corner!
        master_pcb_exterior.lock()
        master_pcb: PCB = PCB("Master", scad_program, pcb_dz, master_pcb_exterior)

        # Compute the *center_pcb_exterior*:
        center_pcb_exterior: SimplePolygon = SimplePolygon("Center PCB Exterior SimplePolygon",
                                                           [], lock=False)
        center_pcb_exterior.corner_arc_append(B, corner_radius, "SW")
        center_pcb_exterior.point_append(C)
        center_pcb_exterior.corner_arc_append(D, corner_radius, "NW")
        center_pcb_exterior.corner_arc_append(E, corner_radius, "EN")
        center_pcb_exterior.point_append(F)
        center_pcb_exterior.corner_arc_append(G, corner_radius, "ES")
        # Skip H
        center_pcb_exterior.corner_arc_append(I, corner_radius, "NE")
        center_pcb_exterior.corner_arc_append(J, corner_radius, "WS")
        center_pcb_exterior.corner_arc_append(K, corner_radius, "NW")
        center_pcb_exterior.corner_arc_append(L, corner_radius, "ES")
        # Skip M
        center_pcb_exterior.arc_append(origin, inner_radius, N_angle, O_angle, degrees5)
        center_pcb_exterior.point_append(P)
        center_pcb_exterior.corner_arc_append(Q, corner_radius, "SE")
        center_pcb_exterior.corner_arc_append(R, corner_radius, "WS")
        center_pcb_exterior.point_append(S)
        center_pcb_exterior.arc_append(origin, inner_radius, T_angle, U_angle, degrees5)
        # Skip V
        center_pcb_exterior.corner_arc_append(W, corner_radius, "SW")
        center_pcb_exterior.corner_arc_append(X, corner_radius, "EN")
        center_pcb_exterior.corner_arc_append(Y, corner_radius, "SE")
        center_pcb_exterior.corner_arc_append(Z, corner_radius, "WN")
        # Skip A
        center_pcb_exterior.lock()
        center_pcb: PCB = PCB("Center", scad_program, pcb_dz, center_pcb_exterior)
        center_pcb.pcb_parent_set(master_pcb)

        # Compute the *ne_pcb_exterior* and stuff it into *ne_pcb_polygon*:
        ne_pcb_exterior: SimplePolygon = SimplePolygon("NE PCB Exterior SimplePolygon",
                                                       [], lock=False)
        ne_pcb_exterior.point_append(A)
        ne_pcb_exterior.rounded_arc_append("BH++EV++", origin, radius,
                                           a_angle, c_angle, corner_radius)
        ne_pcb_exterior.point_append(C)
        ne_pcb_exterior.corner_arc_append(B, corner_radius, "WS")
        ne_pcb_exterior.lock()
        ne_pcb: PCB = PCB("NE", scad_program, pcb_dz, ne_pcb_exterior)
        ne_pcb.pcb_parent_set(master_pcb)

        # Compute the *nw_pcb_exterior* and stuff it into the *nw_pcb_polygon*:
        nw_pcb_exterior: SimplePolygon = SimplePolygon("NW PCB Exterior SimplePolygon",
                                                       [], lock=False)
        nw_pcb_exterior.point_append(F)
        nw_pcb_exterior.rounded_arc_append("BV-+EH++", origin, radius,
                                           f_angle, h_angle, corner_radius)
        nw_pcb_exterior.point_append(H)
        nw_pcb_exterior.corner_arc_append(G, corner_radius, "SE")
        nw_pcb_exterior.lock()
        nw_pcb: PCB = PCB("NW", scad_program, pcb_dz, nw_pcb_exterior)
        nw_pcb.pcb_parent_set(master_pcb)

        # Compute the *se_pcb_exterior* and stuff it into the *se_pcb_polygon*:
        se_pcb_exterior: SimplePolygon = SimplePolygon("SE PCB Exterior SimplePolygon",
                                                       [], lock=False)
        se_pcb_exterior.point_append(S)
        se_pcb_exterior.rounded_arc_append("BV++EH-+", origin, radius,
                                           s_angle, v_angle, corner_radius)
        se_pcb_exterior.point_append(V)
        # se_pcb_exterior.point_append(U)
        se_pcb_exterior.arc_append(origin, inner_radius, U_angle, T_angle, 0.0)
        # se_pcb_exterior.point_append(T)
        se_pcb_exterior.lock()
        se_pcb: PCB = PCB("SE", scad_program, pcb_dz, se_pcb_exterior)
        se_pcb.pcb_parent_set(master_pcb)

        # Compute the *sw_pcb_exterior*:
        sw_pcb_exterior: SimplePolygon = SimplePolygon("SW PCB Exterior SimplePolygon",
                                                       [], lock=False)
        sw_pcb_exterior.point_append(M)
        sw_pcb_exterior.rounded_arc_append("BH-+EV-+", origin, radius,
                                           m_angle, p_angle, corner_radius)
        sw_pcb_exterior.point_append(P)
        # sw_pcb_exterior.point_append(O)
        sw_pcb_exterior.arc_append(origin, inner_radius, O_angle, N_angle, 0.0)
        # sw_pcb_exterior.point_append(N)
        sw_pcb_exterior.lock()
        sw_pcb: PCB = PCB("SW", scad_program, pcb_dz, sw_pcb_exterior)
        sw_pcb.pcb_parent_set(master_pcb)

        # Return the resulting *PCB*s:
        return master_pcb, center_pcb, ne_pcb, nw_pcb, se_pcb, sw_pcb

    # MasterBoard.sonar_modules_create():
    def sonar_modules_create(self, scad_program: ScadProgram,
                             hcsr04: HCSR04, connectors: Connectors) -> None:
        """Create all of the sonar modules."""
        # Create "F1x4LP" low profile 1x4 .1in female connector:
        f1x4lp: F1x4LP = connectors.f1x4lp
        f1x4lp_top_z: float = f1x4lp.top_z

        # Create the *hcsr04_module* and grab some values out of it:
        hcsr04_pcb_dy: float = hcsr04.pcb_dy
        hcsr04_module: Module3D = hcsr04.module
        hcsr04_use_module: UseModule3D = hcsr04_module.use_module_get()

        # Create an *upright_sonar_module* that has repositioned *hcs04_use_module* so that
        # fits properly into the *f1x41p*:
        degrees180: float = pi
        degrees90: float = degrees180 / 2.0
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        z_rotated_hcsr04: Rotate3D = Rotate3D("Z-Axis Rotated HC-SR04",
                                              hcsr04_use_module, degrees180, z_axis)
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
    def sonars_install(self, hcsr04: HCSR04, center_pcb: PCB, ne_pcb: PCB, nw_pcb: PCB,
                       connectors: Connectors,
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
        sonar_poses: List[Tuple[str, str, PCB, List[PCBChunk], List[Reference], str,
                                float, float, float, bool]] = [
            ("Rear Left Sonar", "high", ne_pcb, ne_pcb_chunks, ne_references, "CN60",
             (90.0 - 1.8 * 22.5) * degrees2radians,               # Trial and error to fit iniside
             (90.0 + 0.5 * 22.5) * degrees2radians,
             2.0 * 2.54, True),  # robot perimeter and miss spacer.
            ("Rear Right Sonar", "high", nw_pcb, nw_pcb_chunks, nw_references, "CN70",
             (90.0 + 1.8 * 22.5) * degrees2radians,               # Same Trial and error as above.
             (90.0 - 0.5 * 22.5) * degrees2radians,
             2.0 * 2.540, True),
            ("Front Right Top Sonar", "normal", center_pcb,
             center_pcb_chunks, center_references, "CN50",
             (270.0 - 2 * 22.5 + 2.5) * degrees2radians,
             (270.0 - 2 * 22.5) * degrees2radians,
             1.5 * 2.54, True),
            ("Front Right Bottom Sonar", "low", center_pcb,
             center_pcb_chunks, center_references, "CN51",
             (270.0 - 1 * 22.5 - 2.5) * degrees2radians,
             (270.0 - 1 * 22.5) * degrees2radians,
             1.5 * 2.54, False),
            ("Front Center Sonar", "low", center_pcb,
             center_pcb_chunks, center_references, "CN52",
             (270.0) * degrees2radians,
             (270.0) * degrees2radians,
             23.0, True),
            ("Front Left Center Sonar", "low", center_pcb,
             center_pcb_chunks, center_references, "CN53",
             (270.0 + 1 * 22.5 + 2.5) * degrees2radians,
             (270.0 + 1 * 22.5) * degrees2radians,
             1.5 * 2.54, False),
            ("Front Right Center Sonar", "normal", center_pcb,
             center_pcb_chunks, center_references, "CN54",
             (270.0 + 2 * 22.5 - 2.5) * degrees2radians,
             (270.0 + 2 * 22.5) * degrees2radians,
             1.5 * 2.54, True),
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
        pcb: PCB
        pcb_chunks: List[PCBChunk]
        foot_print_name: str
        references: List[Reference]
        rim_angle: float
        beam_angle: float
        radius_offset: float
        is_front: bool
        placements_file: IO[Any] = open("/tmp/placments.csv", "w")
        for (sonar_name, height, pcb, pcb_chunks, references, reference_name,
             rim_angle, beam_angle, sonar_offset, is_front) in sonar_poses:
            sonar_radius: float = sonars_radius - sonar_offset
            sonar_x: float = sonar_radius * cos(rim_angle)
            sonar_y: float = sonar_radius * sin(rim_angle)
            sonar_flags: str = "" if is_front else "byY"
            sonar_translate: P2D = P2D(sonar_x, sonar_y)
            connector_pcb_chunk: PCBChunk
            rotate: float = beam_angle + degrees90
            if height == "low":
                pcb.module3d_place("F1x4LP", {"sonar_connectors"}, sonar_flags,
                                   origin2d, rotate, sonar_translate)
                pcb.module3d_place("HCSR04Low", {"sonars"}, sonar_flags,
                                   origin2d, rotate, sonar_translate)
                connector_pcb_chunk = (top_surface_hcsr04_f1x4lp_pcb_chunk
                                       if is_front else bottom_surface_hcsr04_f1x4lp_pcb_chunk)
                footprint_name = "HCSR04;F1x4LP"
            elif height == "normal":
                pcb.module3d_place("F1x4", {"sonar_connectors"}, sonar_flags,
                                   origin2d, rotate, sonar_translate)
                pcb.module3d_place("HCSR04Medium", {"sonars"}, sonar_flags,
                                   origin2d, rotate, sonar_translate)
                connector_pcb_chunk = f1x4_hcsr04_pcb_chunk
                footprint_name = "HCSR04;F1x4"
            elif height == "high":
                pcb.module3d_place("F1x4H", {"sonar_connectors"}, sonar_flags,
                                   origin2d, rotate, sonar_translate)
                pcb.module3d_place("HCSR04High", {"sonars"}, sonar_flags,
                                   origin2d, rotate, sonar_translate)
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

    # MasterBoard.spacer_mounts_append():
    def spacer_mounts_append(self, center_pcb: PCB, ne_pcb: PCB, nw_pcb: PCB,
                             romi_base_keys: List[Tuple[Any, ...]]) -> None:
        """Append some spacers mounts to MasterBoard."""
        # Use *romi_base_keys* to build *romi_base_keys_table*:
        romi_base_keys_table: Dict[str, Tuple[Any, ...]] = {}
        romi_base_key: Tuple[Any, ...]
        romi_base_key_name: str
        for romi_base_key in romi_base_keys:
            romi_base_key_name = romi_base_key[1]
            romi_base_keys_table[romi_base_key_name] = romi_base_key

        # Create the *romi_base_mounting holes* which are the holes for mounting the
        # master board to the Romi base:
        spacer_tuples: List[Tuple[str, str, str, PCB]] = [
            ("NE MasterBoard", "BATTERY: Upper Hole (9, 2)", "H6", ne_pcb),
            ("NW MasterBoard", "BATTERY: Upper Hole (0, 2)", "H7", nw_pcb),
            # ("SE MasterBoard", "RIGHT: Misc Small Upper Right 90deg", "H8"),
            # ("SW MasterBoard", "LEFT: Misc Small Upper Right 90deg", "H9"),
            ("SE MasterBoard", "RIGHT: Vector Hole 8", "H8", center_pcb),
            ("SW MasterBoard", "LEFT: Vector Hole 8", "H9", center_pcb),
        ]

        # Put in a spacer_hole for each *spacer_tuple*:
        spacer_hole_diameter: float = 2.2
        kicad_mounting_holes: Dict[str, Tuple[P2D, float]] = {}
        kicad_hole_name: str
        spacer_name: str
        spacer_tuple: Tuple[str, str, str]
        pcb: PCB
        for spacer_name, romi_base_key_name, kicad_hole_name, pcb in spacer_tuples:
            romi_base_key = romi_base_keys_table[romi_base_key_name]
            romi_base_key_x: float = romi_base_key[2]
            romi_base_key_y: float = romi_base_key[3]
            # romi_base_key_diameter: float = romi_base_key[4]  # Use *space_hole_diameter* instead!
            kicad_mounting_holes[kicad_hole_name] = (P2D(romi_base_key_x, romi_base_key_y),
                                                     spacer_hole_diameter)

            # Consturct the *spacer_hole*:
            spacer_hole_center: P2D = P2D(romi_base_key_x, romi_base_key_y)
            pcb.mount_hole_append(f"{spacer_name} Spacer Hole", {"mounts"},
                                  spacer_hole_diameter, spacer_hole_center)


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
        # Define the board dimensions:
        pcb_dx: float = 104.00
        pcb_center_x: float = pcb_dx / 2
        pcb_dy: float = 70.00
        pcb_center_y: float = pcb_dy / 2
        pcb_dz: float = 1.0
        pcb_center: P2D = P2D(pcb_center_x, pcb_center_y)
        pcb_corner_radius: float = 1.5

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

        # The *Sqaure* class takes a *center* argument that specifies where the center
        # of the square is to be located in the X/Y plane.  We need to compute *square_center*
        # such that square is offset so that the holes center is at the origin (0, 0):
        # Create the *pcb_outline* with the origin in the lower left corner:
        square_center: P2D = pcb_center - holes_center
        other_pi_exterior: Square = Square("PCB Exterior", pcb_dx, pcb_dy,
                                           center=square_center, corner_radius=pcb_corner_radius)

        # Create the *other_pi_pcb*:
        other_pi_pcb: PCB = PCB("OtherPi", scad_program, pcb_dz, other_pi_exterior)

        # Define the mounting holes:
        hole_diameter: float = 2.75
        other_pi_pcb.mount_hole_append("NE Mount Hole", {"mounts"},
                                       hole_diameter, hole_ne - holes_center)
        other_pi_pcb.mount_hole_append("NW Mount Hole", {"mounts"},
                                       hole_diameter, hole_nw - holes_center)
        other_pi_pcb.mount_hole_append("SE Mount Hole", {"mounts"},
                                       hole_diameter, hole_se - holes_center)
        other_pi_pcb.mount_hole_append("SW Mount Hole", {"mounts"},
                                       hole_diameter, hole_sw - holes_center)

        # Create the Male 2x20 Header:
        male_2x20_header_center: P2D = P2D((7.37 + 57.67) / 2.0, (64.00 + 69.08) / 2.0)

        # male_2x20_header_center = P3D(0.0, 0.0, 0.0)
        origin2d: P2D = P2D(0.0, 0.0)
        other_pi_pcb.module3d_place("M2x20", {"connectory"}, "",
                                    origin2d, 0.0, male_2x20_header_center - holes_center)

        other_pi_pcb.scad3d_place(Color("Silver Ethernet Connector",
                                        CornerCube("Ethernet Connector",
                                                   P3D(80.00, 29.26, pcb_dz),
                                                   P3D(105.00, 44.51, pcb_dz + 13.08)),
                                        "Silver"),
                                  "", translate=-holes_center)
        other_pi_pcb.scad3d_place(Color("Silver USB2 Connector",
                                        CornerCube("USB2 Connector",
                                                   P3D(85.00, 12.82, pcb_dz),
                                                   P3D(104.00, 25.57, pcb_dz + 15.33)),
                                        "Silver"),
                                  "", translate=-holes_center)
        other_pi_pcb.scad3d_place(Color("Silver West Connector",
                                        CornerCube("West Connector",
                                                   P3D(98.00, 50.69, pcb_dz),
                                                   P3D(105.00, 58.09, pcb_dz + 3.00)),
                                        "Silver"),
                                  "", translate=-holes_center)
        other_pi_pcb.scad3d_place(Color("Black North Connector",
                                        CornerCube("North Connector",
                                                   P3D(70.03, 60.92, pcb_dz),
                                                   P3D(76.38, 66.00, pcb_dz + 5.00)),
                                        "Black"),
                                  "", translate=-holes_center)
        other_pi_pcb.scad3d_place(Color("Black Audio Connector",
                                        CornerCube("Audio Connector",
                                                   P3D(66.31, -1.00, pcb_dz),
                                                   P3D(72.31, 14.00, pcb_dz + 5.12)),
                                        "Black"),
                                  "", translate=-holes_center)
        other_pi_pcb.scad3d_place(Color("Silver USB3A Connector",
                                        CornerCube("USB3A Connector",
                                                   P3D(9.54, 0.00, pcb_dz),
                                                   P3D(18.52, 10.00, pcb_dz + 2.95)),
                                        "Silver"),
                                  "", translate=-holes_center)
        other_pi_pcb.scad3d_place(Color("Silver USB3B Connector",
                                        CornerCube("USB3B Connector",
                                                   P3D(22.73, 0.00, pcb_dz),
                                                   P3D(31.71, 10.00, pcb_dz + 2.95)),
                                        "Silver"),
                                  "", translate=-holes_center)
        other_pi_pcb.scad3d_place(Color("Silver Power Connector",
                                        CornerCube("Power Connector",
                                                   P3D(-1.00, 22.50, pcb_dz),
                                                   P3D(8.00, 29.00, pcb_dz + 2.95)),
                                        "Silver"),
                                  "", translate=-holes_center)
        other_pi_pcb.scad3d_place(Color("White JST Connector",
                                        CornerCube("JST Connector",
                                                   P3D(0.00, 55.01, pcb_dz),
                                                   P3D(3.00, 63.37, pcb_dz + 2.95)),  # 0.00? 3.00?
                                        "White"),
                                  "", translate=-holes_center)
        other_pi_pcb.scad3d_place(Color("Black Buttons Area",
                                        CornerCube("Buttons Area",
                                                   P3D(85.51, 62.12, pcb_dz),
                                                   P3D(98.23, 68.74, pcb_dz + 2.95)),  # 10.00?
                                        "Black"),
                                  "", translate=-holes_center)
        other_pi_module: Module3D = other_pi_pcb.scad_program_append(scad_program, "Yellow")

        # Stuff some values into *other_pi* (i.e. *self*):
        # other_pi: OtherPi = self
        self.module: Module3D = other_pi_module
        self.pcb: PCB = other_pi_pcb


# RaspberryPi3:
class RaspberryPi3:
    """Represents a Raspberry Pi 3B+."""

    # RaspberryPi3.__init__():
    def __init__(self, scad_program: ScadProgram, connectors: Connectors) -> None:
        """Initialize RaspberryPi3 and append to ScadProgram."""
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
        raspi3b_exterior: Square = Square("PCB Exterior", pcb_dx, pcb_dy,
                                          center=square_center, corner_radius=pcb_corner_radius)

        # Create the *raspib_pcb*:
        raspi3b_pcb: PCB = PCB("RasPi3B", scad_program, pcb_dz, raspi3b_exterior)

        # Create the 4 mount holes:
        hole_diameter: float = 2.75
        raspi3b_pcb.mount_hole_append("NE Mount Hole", {"mounts"},
                                      hole_diameter, hole_ne - holes_center)
        raspi3b_pcb.mount_hole_append("NW Mount Hole", {"mounts"},
                                      hole_diameter, hole_nw - holes_center)
        raspi3b_pcb.mount_hole_append("SE Mount Hole", {"mounts"},
                                      hole_diameter, hole_se - holes_center)
        raspi3b_pcb.mount_hole_append("SW Mount Hole", {"mounts"},
                                      hole_diameter, hole_sw - holes_center)

        # Install the header connectors:
        origin2d: P2D = P2D(0.0, 0.0)
        # The main Pi 2x20 header:
        # raspi3b_pcb.show("", "B:")
        connector2x20_center: P2D = P2D((57.90 + 7.10) / 2.0, 52.50)
        raspi3b_pcb.module3d_place("M2x20", {"connector"}, "",
                                   origin2d, 0.0, connector2x20_center - holes_center)
        raspi3b_pcb.module3d_place("F2x20", {"connector_mate"}, "bxnN",
                                   origin2d, 0.0, connector2x20_center - holes_center)
        # raspi3b_pcb.show("", "A:")

        # A 2x2 jumper header:
        connector2x2_center: P2D = P2D((58.918 + 64.087) / 2.0, (44.005 + 48.727) / 2.0)
        raspi3b_pcb.module3d_place("M2x2", {"jumpers"}, "",
                                   origin2d, 0.0, connector2x2_center - holes_center)
        # A 1x2 jumper header:
        connector1x2_center: P2D = P2D((58.90 + 64.10) / 2.0, (38.91 + 41.11) / 2.0)
        raspi3b_pcb.module3d_place("M1x2", {"jumpers"}, "",
                                   origin2d, 0.0, connector1x2_center - holes_center)

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
        processor_ne2d: P2D = P2D(36.75, 40.1)
        processor_sw2d: P2D = P2D(21.75, 24.9)
        processor_center2d: P2D = (processor_sw2d + processor_ne2d) / 2.0
        processor_center3d: P3D = P3D(processor_center2d.x, processor_center2d.y, 0.0)
        sdram_ne2d: P2D = P2D(50.65, 40.1)
        sdram_sw2d: P2D = P2D(38.95, 24.9)
        sdram_center2d: P2D = (sdram_sw2d + sdram_ne2d) / 2.0
        sdram_center3d: P3D = P3D(sdram_center2d.x, sdram_center2d.y, 0.0)
        ethernet_ne2d: P2D = P2D(61.5, 41.0)
        ethernet_sw2d: P2D = P2D(55.5, 35.0)
        ethernet_center2d: P2D = (ethernet_sw2d + ethernet_ne2d) / 2.0
        ethernet_center3d: P3D = P3D(ethernet_center2d.x, ethernet_center2d.y, 0.0)
        usb_ne2d: P2D = P2D(55.5, 28.0)
        usb_sw2d: P2D = P2D(55.1, 20.0)
        usb_center2d: P2D = (usb_sw2d + usb_ne2d) / 2.0
        usb_center3d: P3D = P3D(usb_center2d.x, usb_center2d.y, 0.0)

        # Construct the heat sinks:
        heat_sink_dz: float = 8.0
        heat_sink_base_dz: float = 2.5
        fin_dx: float = 2.5
        processor_heat_sink: HeatSink = HeatSink(
            scad_program, "RPi4 Processor Heat Sink",
            14.30, 14.10, heat_sink_dz, heat_sink_base_dz, fin_dx, 7, "Silver")
        sdram_heat_sink: HeatSink = HeatSink(
            scad_program, "RPi4 SDRam Heat Sink",
            14.30, 9.90, heat_sink_dz, heat_sink_base_dz, fin_dx, 7, "Silver")
        ethernet_heat_sink: HeatSink = HeatSink(
            scad_program, "RPi4 Ethernet Heat Sink",
            8.80, 8.90, heat_sink_dz, heat_sink_base_dz, fin_dx, 5, "Silver")
        usb_heat_sink: HeatSink = HeatSink(
            scad_program, "RPi4 USB Heat Sink",
            8.80, 8.90, heat_sink_dz, heat_sink_base_dz, fin_dx, 5, "Silver")

        center_offset: P3D = P3D(0.0, 0.0, 0.0)
        origin3d: P3D = P3D(0.0, 0.0, 0.0)
        z_axis: P3D = P3D(0.0, 0.0, 1.0)
        repositioned_processor_heat_sink: Scad3D = (
            processor_heat_sink.module.use_module_get().
            reposition("Repositioned RPi4 Processor Heat Sink",
                       origin3d, z_axis, 0.0, processor_center3d + center_offset))
        repositioned_sdram_heat_sink: Scad3D = (
            sdram_heat_sink.module.use_module_get().
            reposition("Repositioned RPi4 SDRAM Heat Sink",
                       origin3d, z_axis, 0.0, sdram_center3d + center_offset))
        repositioned_ethernet_heat_sink: Scad3D = (
            ethernet_heat_sink.module.use_module_get().
            reposition("Repositioned RPi4 Ethernet Heat Sink",
                       origin3d, z_axis, 0.0, ethernet_center3d + center_offset))
        repositioned_usb_heat_sink: Scad3D = (
            usb_heat_sink.module.use_module_get().
            reposition("Repositioned RPi4 UBB Heat Sink",
                       origin3d, z_axis, 0.0, usb_center3d + center_offset))

        heatsinks_union: Union3D = Union3D("Heatsinks", [
            repositioned_processor_heat_sink,
            repositioned_sdram_heat_sink,
            repositioned_ethernet_heat_sink,
            repositioned_usb_heat_sink])
        heatsinks_union = heatsinks_union

        # Now place various cubes to represent the other connectors:
        raspi3b_pcb.scad3d_place(Color("Silver RJ45 Connector",
                                       CornerCube("RJ45 Connecttor",
                                                  P3D(65.650, 2.495, pcb_dz + 0.000),
                                                  P3D(87.000, 18.005, pcb_dz + 13.500)),
                                       "Silver"),
                                 "", translate=-holes_center)
        raspi3b_pcb.scad3d_place(Color("Silver Lower USB2",
                                       CornerCube("Lower USB2",
                                                  P3D(69.30, 22.43, pcb_dz + 0.00),
                                                  P3D(87.00, 34.57, pcb_dz + 16.00)),
                                       "Silver"),
                                 "", translate=-holes_center)
        raspi3b_pcb.scad3d_place(Color("Silver Upper USB2",
                                       CornerCube("Upper USB2",
                                                  P3D(69.30, 40.43, pcb_dz + 0.00),
                                                  P3D(87.00, 53.57, pcb_dz + 16.00)),
                                       "Silver"),
                                 "", translate=-holes_center)
        raspi3b_pcb.scad3d_place(Color("Black Camera Connector",
                                       CornerCube("Camera Connector",
                                                  P3D(43.55, 0.30, pcb_dz + 0.00),
                                                  P3D(47.50, 22.70, pcb_dz + 5.50)),
                                       "Black"),
                                 "", translate=-holes_center)
        raspi3b_pcb.scad3d_place(Color("Silver HDMI Connector",
                                       CornerCube("HDMI Connector",
                                                  P3D(24.75, -1.50, pcb_dz + 0.00),
                                                  P3D(39.25, 10.65, pcb_dz + 6.50)),
                                       "Silver"),
                                 "", translate=-holes_center)
        raspi3b_pcb.scad3d_place(Color("Silver Power Connector",
                                       CornerCube("Power Connector",
                                                  P3D(6.58, -1.22, pcb_dz + 0.00),
                                                  P3D(14.62, 14.35, pcb_dz + 2.00)),
                                       "Silver"),
                                 "", translate=-holes_center)
        raspi3b_pcb.scad3d_place(Color("Black LCD Connector",
                                       CornerCube("LCD Connector",
                                                  P3D(2.65, 16.80, pcb_dz + 0.00),
                                                  P3D(5.45, 39.20, pcb_dz + 5.50)),
                                       "Black"),
                                 "", translate=-holes_center)

        # Wrap up the *raspi3b_pcb*:
        raspi3b_module: Module3D = raspi3b_pcb.scad_program_append(scad_program, "Green")

        # Stuff some values into into *raspi3b* (i.e. *self*):
        # raspi3b: RaspberryPi3 = self
        self.module: Module3D = raspi3b_module
        self.pcb: PCB = raspi3b_pcb
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
        expansion_outer.arc_append(origin, arc_radius, start_angle, end_angle, 0.0)  # Arc

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

        y_axis: P3D = P3D(0.0, 1.0, 0.0)
        degrees90: float = pi / 2.0
        encoder_use_module: UseModule3D = encoder.module.use_module_get()
        encoder_translate: P3D = encoder.translate
        rotated_encoder: Rotate3D = Rotate3D("Rotated Encoder", encoder_use_module,
                                             degrees90, y_axis)
        translated_encoder: Translate3D = Translate3D("Translated Encoder",
                                                      rotated_encoder, encoder_translate)

        # Construct *module*, append to *scad_program*, and store into *rom_wheel_assembly*
        # (i.e. *self*):
        module = Module3D("Romi Wheel Assembly Module", [
            romi_motor.module.use_module_get(),
            romi_magnet.module.use_module_get(),
            translated_encoder,
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

    def __init__(self, scad_program: ScadProgram, connectors: Connectors) -> None:
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
        pcb_dz: float = 1.60  # mm Calipers
        # pcb_top_z: float = pcb_bottom_z + pcb_dz

        # Define all of the connector locations:
        degrees180: float = pi
        degrees90: float = degrees180 / 2.0
        cn1_dx: float = 3.0  # Calipers
        cn1_bottom_y: float = pcb_dy / 2.0 - 1.2  # Calipers
        cn1_top_y: float = cn1_bottom_y + 10  # Extra large to poke through PCB
        cn1_dy: float = abs(cn1_top_y - cn1_bottom_y)
        cn1_dz: float = 1.0
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

        # Create the *st_link_pcb*:
        st_link_pcb: PCB = PCB("ST_Link", scad_program, pcb_dz, st_link_exterior)

        # Install CN1 USB connector:
        cn1_cube_center: P3D = P3D(0.0, 0.0, cn1_dz / 2.0)
        cn1_cube: Cube = Cube("CN1 USB Connector", cn1_dx, cn1_dy, cn1_dz, cn1_cube_center)
        colored_cn1_cube: Color = Color("Colored CN1 USB Connector", cn1_cube, "Silver")
        st_link_pcb.scad3d_place(colored_cn1_cube, "t", translate=P2D(cn1_center_x, cn1_center_y))

        # Install CN2 connectors:
        st_link_pcb.module3d_place("M1x2", {"ground_connectors"}, "",
                                   origin2d, -degrees90, cn2_center, pads_base=0)
        st_link_pcb.module3d_place("F1x2", {"ground_connectors_mate"}, "bynN",
                                   origin2d, -degrees90, cn2_center, pads_base=0)

        # Install CN6 connectors:
        st_link_pcb.module3d_place("M1x6", {"connectors"}, "",
                                   origin2d, -degrees90, cn6_center, pads_base=2)
        st_link_pcb.module3d_place("F1x6", {"connectors_mate"}, "bynN",
                                   origin2d, -degrees90, cn6_center, pads_base=2)

        # Install JP2 connectors:
        st_link_pcb.module3d_place("M1x2", {"connectors"}, "",
                                   origin2d, degrees90, jp2_center, pads_base=8)
        st_link_pcb.module3d_place("F1x2", {"connectors_mate"}, "bynN",
                                   origin2d, degrees90, jp2_center, pads_base=8)

        # Install CN4 connectors:
        st_link_pcb.module3d_place("M1x4", {"connectors"}, "",
                                   origin2d, -degrees90, cn4_center, pads_base=10)
        st_link_pcb.module3d_place("F1x4", {"connectors_mate"}, "bynN",
                                   origin2d, -degrees90, cn4_center, pads_base=10)

        # Install CN5 connectors:
        st_link_pcb.module3d_place("M1x2", {"connectors"}, "",
                                   origin2d, -degrees90, cn5_center, pads_base=14)
        st_link_pcb.module3d_place("F1x2", {"connectors_mate"}, "bynN",
                                   origin2d, -degrees90, cn5_center, pads_base=14)

        # Install JP1 connectors:
        st_link_pcb.module3d_place("M1x2", {"connectors"}, "",
                                   origin2d, 0.0, jp1_center, pads_base=16)
        st_link_pcb.module3d_place("F1x2", {"connectors_mate"}, "bxnN",
                                   origin2d, 0.0, jp1_center, pads_base=16)

        # Install CN3 connectors:
        st_link_pcb.module3d_place("M1x2", {"ground_connectors"}, "",
                                   origin2d, -degrees90, cn3_center, pads_base=18)
        st_link_pcb.module3d_place("F1x2", {"ground_connectors_mate"}, "bynN",
                                   origin2d, -degrees90, cn3_center, pads_base=18)

        # Wrap-up *st_link_pcb* and generate *st_link_module*:
        st_link_module: Module3D = st_link_pcb.scad_program_append(scad_program, "Beige")
        st_link_use_module: UseModule3D = st_link_module.use_module_get()

        # Now create *sandwich_pcb*:
        sandwich_pcb_dy: float = pcb_dy
        sandwich_pcb_dx: float = pcb_dx / 2.0 + 5.0
        sandwich_pcb_dz: float = 1.6
        adapter_exterior: Square = Square("STLink Adapter Exterior",
                                          sandwich_pcb_dx, sandwich_pcb_dy,
                                          center=P2D(-(pcb_dx - sandwich_pcb_dx) / 2.0, 0.0),
                                          corner_radius=1.5, corner_count=5)

        adapter_pcb: PCB = PCB("ST_Link_Adapter", scad_program, sandwich_pcb_dz, adapter_exterior)
        adapter_pcb.pcb_place(st_link_pcb, {"connectors_mate"}, "")
        f1x5ra_center: P2D = P2D(-9.0, 4.0 - 2.54)
        adapter_pcb.module3d_place("F1x5RA", {"connectors"}, "byY", origin2d, 0.0, f1x5ra_center)
        adapter_module: Module3D = adapter_pcb.scad_program_append(scad_program, "Tan")
        adapter_use_module: UseModule3D = adapter_module.use_module_get()

        translated_st_link: Translate3D = Translate3D("Translated ST Link", st_link_use_module,
                                                      P3D(0.0, 0.0, -10.0))
        sandwich_union: Union3D = Union3D("STLink Sandwich Union",
                                          [adapter_use_module, translated_st_link])

        # y_axis: P3D = P3D(0.0, 1.0, 0.0)
        # y_axis_rotated_sandwich: Rotate3D = Rotate3D("Y Axis Rotated ST Link Sandwich",
        #                                              sandwich_union, degrees180, y_axis)
        x_axis: P3D = P3D(1.0, 0.0, 0.0)
        x_axis_rotated_sandwich: Rotate3D = Rotate3D("X Axis Rotated ST Link Sandwich",
                                                     sandwich_union, degrees90, x_axis)
        # There are all sorts of rotations that happened Y and Z are swapped:
        translated_sandwich: Translate3D = Translate3D("Connector Centered Sandwich",
                                                       x_axis_rotated_sandwich,
                                                       P3D(f1x5ra_center.x, 1.27, -pcb_dy / 2.0))

        sandwich_module: Module3D = Module3D("ST_Link_Sandwich_Module", [translated_sandwich])
        scad_program.append(sandwich_module)
        scad_program.if3d.name_match_append("st_link_sandwich", sandwich_module,
                                            ["ST-Link Adapter Sandwich"])

        # Stuff some values into *st_link* (i.e. *self*):
        # st_link: STLink = self
        self.module: Module3D = sandwich_module
        self.adapter_module: Module3D = adapter_module
        self.adapter_pcb: PCB = adapter_pcb
        self.st_link_module: Module3D = st_link_module
        self.st_link_pcb: PCB = st_link_pcb


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
    # print("hr2_models.main() called")
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
    # scad_program.append(Variable2D("Name", "name", '"hr2_arm_assembly"'))

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

    # # KicadPCB.line_append():
    # def line_append(self, point1: P2D, point2: P2D, layer: str, width:
    #                 float) -> None:
    #     """Append a line cut."""
    #     # Grab some values from *kicad_pcb* (i.e. *self*):
    #     kicad_pcb: KicadPCB = self
    #     cut_lines_insert_index: int = kicad_pcb.cut_lines_insert_index
    #     lines: List[str] = kicad_pcb.lines
    #     offset: P2D = kicad_pcb.offset
    #
    #     # Make sure we know where to insert cut lines:
    #     if cut_lines_insert_index < 0:
    #         line: str
    #         index: int
    #         net_class_default_found: bool = False
    #         for index, line in enumerate(lines):
    #             # print(f"[{index}]:'{line}'")
    #             if line.startswith("  (net_class Default"):
    #                 net_class_default_found = True
    #             if net_class_default_found and line.startswith("  )"):
    #                 cut_lines_insert_index = index + 2
    #                 kicad_pcb.cut_lines_insert_index = cut_lines_insert_index
    #
    #     # Create the *cut_line_text* into *lines*:
    #     cut_line_text: str = (
    #         "  (gr_line (start {0:.2f} {1:.2f}) (end {2:.2f} {3:.2f}) "
    #         "(layer {4}) (width {5:1.2f}))").format(
    #             offset.x + point1.x, offset.y - point1.y,
    #             offset.x + point2.x, offset.y - point2.y, layer, width)
    #     lines.insert(cut_lines_insert_index, cut_line_text)

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
        # KiCad numbers are trimmed of trailing zeros and optionally the end decimal point:
        number_text: str = str(round(value, maximum_fractional_digits)).rstrip('0').rstrip('.')

        # Convert "-0..." to "0...":
        if len(number_text) >= 2 and number_text[:2] == "-0":
            number_text = number_text[1:]
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
