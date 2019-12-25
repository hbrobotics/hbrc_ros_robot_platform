# <--------------------------------------- 100 characters ---------------------------------------> #

"""Code to genarate and openscad model of the Pololu Romi Base."""

# Pumpkin Pi: 104mm x 70mm.  Holes are the same as the Raspberry Pi with the upper left
# hole in the upper left corner of the PCB.  The extra PCB space goes to the right and
# down on the Pumpkin Pi.

# http://docplayer.net/42910792-
# Hardware-assisted-tracing-on-arm-with-coresight-and-opencsd-mathieu-poirier.html
from scad_models.scad import (Color, Circle, CornerCube, Cube, If2D, If3D, LinearExtrude, Module2D,
                              Module3D, P2D, P3D, Polygon, Scad, Scad3D, SimplePolygon,
                              ScadProgram, Square, Translate3D, UseModule2D, UseModule3D,
                              Union3D, Variable2D)
from typing import Any, Dict, IO, List, Set, Tuple
from math import asin, atan2, cos, degrees, nan, pi, sin, sqrt


# MalePinConnector:
class MalePinConnector:
    """Represents an NxM .1 inch male pin connector."""

    # MalePinConnector.__init__():
    def __init__(self, rows: int, columns: int, mating_length: float) -> None:
        """Initialize .1 inch MakePinConnector."""
        # Define a hole bunch of constants:
        # Save the arguments into *male_pin_connector* (i.e. *self*):
        # male_pin_connector: MalePinConnector = self
        self.rows: int = rows
        self.columns: int = columns
        self.mating_length: float = mating_length

    # MalePinConnector.connector_get():
    def connector_get(self) -> Union3D:
        """Return the MalePinConnector as a Union3D."""
        # Grab some values out of *male_pin_connector* (i.e. *self*):
        male_pin_connector: MalePinConnector = self
        rows: int = male_pin_connector.rows
        columns: int = male_pin_connector.columns
        mating_length: float = male_pin_connector.mating_length

        # Compute some constants:
        name: str = f"M{rows}x{columns}"
        pin_pitch = 2.54
        half_pin_pitch: float = pin_pitch / 2.0
        pin_dx_dy: float = half_pin_pitch / 2.0
        half_pin_dx_dy: float = pin_dx_dy / 2.0
        insulation_height = pin_pitch
        total_height: float = insulation_height + mating_length
        dx: float = columns * pin_pitch
        dx_offset = -(dx - pin_pitch) / 2.0
        dy: float = rows * pin_pitch
        dy_offset: float = -(dy - pin_pitch) / 2.0

        # Create the *Union3D* *pin_connector* and append *insulation_base* to it:
        pin_connector: Union3D = Union3D(f"{name} Male Pin Connector", [], lock=False)
        insulation_base: Cube = CornerCube(f"{name} Connector Base",
                                           P3D(-dx/2.0, -dy/2.0, 0.0),
                                           P3D(dx/2.0, dy/2.0, insulation_height))
        pin_connector.append(insulation_base)

        # Create each *pin* and append to *connector*:
        row_index: int
        for row_index in range(rows):
            column_index: int
            y: float = row_index * pin_pitch + dy_offset
            for column_index in range(columns):
                x: float = column_index * pin_pitch + dx_offset
                pin: Cube = CornerCube(f"Pin {column_index}:{row_index}",
                                       P3D(x - half_pin_dx_dy, y - half_pin_dx_dy, 0.0),
                                       P3D(x + half_pin_dx_dy, y + half_pin_dx_dy, total_height))
                pin_connector.append(pin)
        pin_connector.lock()
        return pin_connector


# RaspberryPi3:
class RaspberryPi3:
    """Represents a Raspberry Pi 3B+."""

    # RaspberryPi3.__init__():
    def __init__(self) -> None:
        """Initialize RaspberryPi3 object."""
        pass

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
                                                       upper_right_hole])
        return pcb_polygon

    # RaspberryPi3.connectors_get():
    def connectors_get(self) -> List[Scad3D]:
        """Return the RasPi3B connectors as Cube's."""
        # Create the connector2x20:
        connectors: List[Scad3D] = []
        # connectors.append(CornerCube("2x20 Connector", P3D(7.10, 50.00, 0.00),
        #                              P3D(57.90, 55.00, 8.50)))
        male_pin_connector2x20: MalePinConnector = MalePinConnector(2, 20, 5.840)
        connectors.append(Translate3D("Translate 2x20 Connector",
                                      male_pin_connector2x20.connector_get(),
                                      P3D((57.90 + 7.10) / 2.0, 52.50, 0.0)))
        connectors.append(CornerCube("RJ45 Connecttor", P3D(65.650, 2.495, 0.000),
                                     P3D(87.000, 18.005, 13.500)))
        connectors.append(CornerCube("Lower USB2", P3D(69.30, 22.43, 0.00),
                                     P3D(87.00, 34.57, 16.00)))
        connectors.append(CornerCube("Upper USB2", P3D(69.30, 40.43, 0.00),
                                     P3D(87.00, 53.57, 16.00)))
        connectors.append(CornerCube("2x2 Connector", P3D(58.918, 44.005, 0.00),
                                     P3D(64.087, 48.727, 8.50)))
        connectors.append(CornerCube("2x2 Connector", P3D(58.90, 38.91, 0.00),
                                     P3D(64.10, 41.11, 8.50)))
        connectors.append(CornerCube("2x2 Connector", P3D(50.00, 12.50, 0.00),
                                     P3D(57.00, -2.50, 6.00)))
        connectors.append(CornerCube("Camera Connector", P3D(43.55, 0.30, 0.00),
                                     P3D(47.50, 22.70, 5.50)))
        connectors.append(CornerCube("HDMI Connector", P3D(24.75, -1.50, 0.00),
                                     P3D(39.25, 10.65, 6.50)))
        connectors.append(CornerCube("Power Connector", P3D(6.58, -1.22, 0.00),
                                     P3D(14.62, 14.35, 2.00)))
        connectors.append(CornerCube("LCD Connector",  P3D(2.65, 16.80, 0.00),
                                     P3D(5.45, 39.20, 5.50)))
        return connectors

    # RaspberryPi3.scad_program_append():
    def scad_program_append(self, scad_program: ScadProgram, if2d: If2D, if3d: If3D) -> None:
        """Append RaspberryPi3 models to ScadProgram."""
        raspi3b: RaspberryPi3 = self
        raspi3b_pcb_polygon: Polygon = raspi3b.pcb_polygon_get()
        raspi3b_pcb_polygon_module: Module2D = Module2D("RasPi3B_PCB_Polygon_Module",
                                                        [raspi3b_pcb_polygon])
        scad_program.append(raspi3b_pcb_polygon_module)
        if2d.then_append('name == "raspi3b_pcb"',
                         [UseModule2D("RasPi3B PCB Polygon", raspi3b_pcb_polygon_module)])

        raspi3b_pcb: Scad3D = LinearExtrude("Rasp3B PCB", raspi3b_pcb_polygon, height=1.0)
        raspi3b_union: Union3D = Union3D("Rasp3B Model", [], lock=False)
        raspi3b_union.append(raspi3b_pcb)
        raspi3b_union.extend(raspi3b.connectors_get())
        raspi3b_union.lock()
        raspi3b_model: Color = Color("Green Rasp3B Model", raspi3b_union, "Green")
        raspi3b_model_module: Module3D = Module3D("RasPi3B_Model_Module", [raspi3b_model])
        scad_program.append(raspi3b_model_module)
        if3d.then_append('name == "raspi3b_model"',
                         [UseModule3D("RasPi3B_Model", raspi3b_model_module)])


# Romi:
class Romi:
    """A helper class for modeling the Pololu Romi robot platform."""

    # Romi.__init__():
    def __init__(self) -> None:
        """Initialize and create the Romi Platform."""
        # Figuring out where everything is located is done with a combination of reading
        # drawing in section 6 of the "Pololu Romi Chassis User's Guide" and extracting
        # values from the `romi-chassis.dxf` file available from the Pololu web site.
        # `romi-chassis.dxf` is also stored in the git repository to make sure it does
        # not go away.
        #
        # It is important to notice that the section 6 drawings are upside-down from
        # the `romi-chassis.dxf`.  For the modeling of this platform, the `.dxf` file
        # orientation is used.  Sigh.  Also, the `.dxf` file seems to be in units of
        # inches rather than millimeters.  The rest of this code uses millimeters, so we
        # multiply inches coordinate values by *inches2mm* as soon as possible.
        #
        # Finally, the origin of the `.dxf` is off  to the lower right rather that in the
        # traditional center for differential drive robots.  There are various "locate"
        # methods that take values in inches from the `.dxf` file and generate an
        # appropriate data structure (usuually a *Polygon*.)  All of these "locate"
        # methods need to have *offset_origin* to convert to a robot center in millimeters.

        # Set *debugging* to *True* to print out debugging messages:
        debugging: bool = False  # True

        # Let's get started computing *origin_offet*:
        #
        # It is pretty clear that the motor axles are intended to go through the center of the
        # Romi platform along the X axis.  By reading the values for the top and bottom of the
        # axle (in inches) from the `.dxf` file we can compute *y_origin_offset*:
        # *y_origin_offset* in millimeters:
        inches2mm: float = 25.4
        axel_y_above: float = 2.967165 * inches2mm
        axel_y_below: float = 2.908110 * inches2mm
        y_origin_offset: float = (axel_y_above + axel_y_below) / 2.0

        # The *x_origin_offset* is computed using the upper castor hole location:
        upper_castor_x_left: float = -3.930756 * inches2mm
        upper_castor_x_right: float = -3.805256 * inches2mm
        x_origin_offset: float = (upper_castor_x_left + upper_castor_x_right) / 2.0

        # Finally we have *origin_offset* in millimeters and can save it back into *romi*
        # (i.e. *self*):
        origin_offset = P2D(x_origin_offset, y_origin_offset)
        self.debugging = debugging
        self.inches2mm: float = inches2mm
        self.origin_offset: P2D = origin_offset
        if debugging:  # pragma: no cover
            print(f"origin_offset={origin_offset}")

    # Romi.base_outline_polygon_get():
    def base_outline_polygon_get(self) -> SimplePolygon:
        """Return the outline of the Romi Base."""
        # Grab some values from *romi* (i.e. *self*):
        romi: Romi = self
        debugging: bool = romi.debugging

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
        inches2mm: float = romi.inches2mm
        left_neck_x: float = -5.704091 * inches2mm
        right_neck_x: float = -2.031646 * inches2mm
        neck_dx: float = right_neck_x - left_neck_x
        half_neck_dx: float = neck_dx / 2.0
        top_neck_y: float = 3.410067 * inches2mm
        bottom_neck_y: float = 2.465193 * inches2mm
        neck_dy: float = top_neck_y - bottom_neck_y
        assert neck_dy > 0.0
        half_neck_dy = neck_dy / 2.0
        # print(f"neck_dx={neck_dx}mm")

        # Perform any requested *debugging*:
        if debugging:  # pragma: no cover
            print(f"wheel_well_angle={wheel_well_angle}={degrees(wheel_well_angle)}deg")
            print(f"wheel_well_corner={wheel_well_corner}")

        # Verify that the *distance* from the *origin* to the *wheel_well_corner* matches *radius*:
        origin: P2D = P2D(0.0, 0.0)
        wheel_well_radius: float = origin.distance(wheel_well_corner)
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

        # Create the left wheel well:
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

    # Romi.base_polygon_get()
    def base_polygon_get(self) -> Polygon:
        """TODO."""
        # Grabe some values from *romi* (i.e. *self*):
        romi: Romi = self
        debugging: bool = romi.debugging

        # Grab the *base_outline_polygon*:
        base_outline_polygon: SimplePolygon = romi.base_outline_polygon_get()

        # Grab the *battery_polygons*:
        battery_polygons: List[SimplePolygon] = romi.battery_polygons_get()

        # Grab the *upper_hex_polygons*:
        upper_hex_polygons: List[SimplePolygon] = romi.upper_hex_polygons_get()
        if debugging:  # pragma: no cover
            print("************************")
            print(f"len(upper_hex_polygons)={len(upper_hex_polygons)}")

        # Grab the *lower_hex_polygons* and *lower_hex_table*:
        lower_hex_polygons: List[SimplePolygon]
        lower_hex_table: Dict[str, P2D]
        lower_hex_polygons, lower_hex_table = romi.lower_hex_polygons_table_get()
        if debugging:  # pragma: no cover
            print(f"len(lower_hex_polygons)={len(lower_hex_polygons)}")

        line_hole_polygons: List[SimplePolygon] = romi.line_hole_polygons_get(lower_hex_table)
        lower_arc_holes_rectangles: List[SimplePolygon] = romi.lower_arc_holes_rectangles_get()
        upper_arc_holes_rectangles: List[SimplePolygon] = romi.upper_arc_holes_rectangles_get()
        miscellaneous_holes: List[Circle] = romi.miscellaneous_holes_get()
        vertical_rectangles: List[Square] = romi.vertical_rectangles_get()

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
        bottom_rectangle: Square = romi.rectangle_locate("CENTER: Bottom Rectangle",
                                                         -3.942201, -0.169016,
                                                         -3.79352, -0.224126)
        bottom_castor_rectangle: Square = romi.rectangle_locate("CENTER: Bottom Castor Rectangle",
                                                                -3.930854, 1.164122,
                                                                -3.804870, 1.226996)
        top_castor_hole: Circle = romi.hole_locate("CENTER: Top Castor Hole",
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

    # Romi.battery_clips_get():
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

        romi: Romi = self
        # The lowest battery is identified by two rectangles -- the one on the left
        # encloses the "dome* and the one the right is a narrow slot.  The batteries
        # are numbered from 0 (lower most) to 3 (upper most):
        battery0_left_rectangle: Square = romi.rectangle_locate("Battery0 Left Rectangle",
                                                                -3.060783, 2.425815,
                                                                -2.922980, 2.701413)
        battery0_right_rectangle: Square = romi.rectangle_locate("Battery0 Right Rectangle",
                                                                 -2.844244, 2.311650,
                                                                 -2.745811, 2.815583)
        battery_clip0: SimplePolygon = battery_clip_helper("Battery Clip 0", True,
                                                           battery0_left_rectangle,
                                                           battery0_right_rectangle)

        battery1_left_rectangle: Square = romi.rectangle_locate("Battery1 Left Rectangle",
                                                                -2.941728, 3.002429,
                                                                -2.844244, 3.266539)
        battery1_right_rectangle: Square = romi.rectangle_locate("Battery1 Right Rectangle",
                                                                 -2.844244, 2.882512,
                                                                 -2.745811, 3.386457)
        battery_clip1: SimplePolygon = battery_clip_helper("Battery Clip 1", False,
                                                           battery1_left_rectangle,
                                                           battery1_right_rectangle)

        battery2_left_rectangle: Square = romi.rectangle_locate("Battery2 Left Rectangle",
                                                                -2.051925, 3.665984,
                                                                -1.9141182, 3.941567)
        battery2_right_rectangle: Square = romi.rectangle_locate("Battery2 Right Rectangle",
                                                                 -1.835382, 3.551803,
                                                                 -1.736965, 4.055748)
        battery_clip2: SimplePolygon = battery_clip_helper("Battery Clip 2", True,
                                                           battery2_left_rectangle,
                                                           battery2_right_rectangle)

        battery3_left_rectangle: Square = romi.rectangle_locate("Battery3 Left Rectangle",
                                                                -1.932882, 4.242594,
                                                                -1.835382, 4.506693)
        battery3_right_rectangle: Square = romi.rectangle_locate("Battery3 Right Rectangle",
                                                                 -1.835382, 4.122677,
                                                                 -1.736965, 4.626610)
        battery_clip3: SimplePolygon = battery_clip_helper("Battery Clip 3", False,
                                                           battery3_left_rectangle,
                                                           battery3_right_rectangle)

        return [battery_clip0, battery_clip1, battery_clip2, battery_clip3]

    # Romi.battery_polygons_get():
    def battery_polygons_get(self) -> List[SimplePolygon]:
        """Return the holes for the Romi battery case."""
        # Grab some values from *romi* (i.e. *self*):
        romi: Romi = self
        # debugging: bool = romi.debugging
        inches2mm: float = romi.inches2mm
        origin_offset: P2D = romi.origin_offset

        # All of the battery holes are done relative to the *battery_reference_hole*
        # indicated on the drawing of the dimensions and mounting holes seciont of the
        # "Pololu Romi Chassis User's Guide".
        reference_hole: Circle = romi.hole_locate("ZILCH: Battery Hole",
                                                  -3.913146, 3.376610, -3.822591, 3.286051)

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
                                                             f"({2-x_index}, {y_index})"),
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
                upper_hole: Circle = reference_hole.copy(("BATTERY: Upper Hole "
                                                         f"({2-x_index}, {y_index})"),
                                                         center=upper_hole_center)
                simple_polygons.append(upper_hole)

        # There are 6 rectangular slots that have a nub on the end to cleverly indicate battery
        # polarity direction.  We will model these as simple rectangular slots and skip the
        # cleverly battery nubs.  There are 4 batteries slots at the top called
        # *upper_left_battery_slot*, *upper_right_battery_slot*, *lower_left_battery_slot*, and
        # *lower_righ_battery_slot*.  Underneath these 4 batteries are 2 more centered battery
        # slots that are called *upper_center_battery_slot* and *lower_center_battery_slot*:
        upper_left_battery_slot: Square = romi.rectangle_locate("BATTERY: Upper Left Battery Slot",
                                                                -5.550937, 4.252594,
                                                                -4.074563, 4.473067)
        upper_right_battery_slot: Square = romi.rectangle_locate(("BATTERY: "
                                                                  "Upper Right Battery Slot"),
                                                                 -3.621799, 4.252594,
                                                                 -2.145425, 4.473067)
        lower_left_battery_slot: Square = romi.rectangle_locate("BATTERY: Lower Left Battery Slot",
                                                                -5.590311, 3.705346,
                                                                -4.113925, 3.925815)
        lower_right_battery_slot: Square = romi.rectangle_locate("BATTERY Lower Right Battery Slot",
                                                                 -3.661173, 3.705346,
                                                                 -2.184799, 3.925815)
        upper_center_battery_slot: Square = romi.rectangle_locate(("BATTERY: Upper Center "
                                                                  "Battery Slot"),
                                                                  -4.58637, 3.012429,
                                                                  -3.109992, 3.232913)
        lower_center_battery_slot: Square = romi.rectangle_locate(("BATTERY: Lower Center "
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
        upper_outer_left_cutout: Square = romi.rectangle_locate("BATTERY: Upper Outer Left Cutout",
                                                                -5.984008, 4.74865,
                                                                -5.302909, 4.965193)
        upper_inner_left_cutout: Square = romi.rectangle_locate("BATTERY: Upper Inner Left Cutout",
                                                                -5.23598, 4.669913,
                                                                -4.861965, 4.858886)
        upper_inner_right_cutout: Square = romi.rectangle_locate(("BATTERY: "
                                                                  "Upper Inner Right Cutout"),
                                                                 -2.873772, 4.669913,
                                                                 -2.499756, 4.858886)
        upper_outer_right_cutout: Square = romi.rectangle_locate(("BATTERY: "
                                                                  "Upper Outer Right Cutout"),
                                                                 -2.432827, 4.74865,
                                                                 -1.751728, 4.965193)

        # There are three cutouts across the bottom of the battery case and they are called
        # *lower_left_cutout*, *lower_center_cutout*, and *lower_right_cutout*:
        lower_left_cutout: Square = romi.rectangle_locate("BATTERY: Lower Left Cutout",
                                                          -5.572591, 1.939594,
                                                          -4.655272, 2.189594)
        lower_center_cutout: Square = romi.rectangle_locate("BATTERY: Lower Center Cutout",
                                                            -4.340311, 2.032122,
                                                            -3.395425, 2.189594)
        lower_right_cutout: Square = romi.rectangle_locate("BATTERY: Lower Right Cutout",
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
        # We will measure two on the right side and mirror them to the left side:
        x1: float = -2.031646 * inches2mm - origin_offset.x - 8.85
        dx: float = 2.5
        x2: float = x1 - dx
        x3: float = x2 - 1.65
        x4: float = x3 - dx
        dy: float = 0.70 * inches2mm

        outer_center: P2D = P2D((x1 + x2) / 2.0, 0.0)
        outer_encoder_slot: Square = Square("RIGHT: Outer Encoder Slot", dx, dy, outer_center,
                                            corner_radius=dx/2.0, corner_count=3)
        inner_center: P2D = P2D((x3 + x4) / 2.0, 0.0)
        inner_encoder_slot: Square = Square("RIGHT: Inner Encoder Slot", dx, dy, inner_center,
                                            corner_radius=dx/2.0, corner_count=3)

        # Collect the encoder slots into *encoder_slots* and append to *polygons*:
        encoder_slots: List[SimplePolygon] = [
            outer_encoder_slot, outer_encoder_slot.y_mirror("RIGHT:", "LEFT:"),
            inner_encoder_slot, inner_encoder_slot.y_mirror("RIGHT:", "LEFT:")
        ]
        simple_polygons.extend(encoder_slots)

        battery_clips: List[SimplePolygon] = romi.battery_clips_get()
        simple_polygons.extend(battery_clips)

        return simple_polygons

    # Romi.expansion_hex_holes_slot_get():
    def expansion_hex_holes_slots_get(self) -> List[SimplePolygon]:
        """Return the expansion chasis holes and slots."""
        # romi: Romi = self
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
                    hole = Circle(f"Hole[{hole_index},{angle_index}]",
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

    # Romi.expansion_large_holes_get():
    def expansion_large_holes_get(self) -> List[Circle]:
        """Return the expansion chasis small hole pattern."""
        # romi: Romi = self
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
                    # print(f"  Skip [{x_index}, {y_index}]")
                    pass
                elif y_index in (2, 3) and x_index in (1, 2, 3, 4, 5, 6, 7):  # Skip middle columns
                    # print(f"  SKIP [{x_index}, {y_index}]")
                    pass
                else:
                    # print(f"  [{x_index},{y_index}]")
                    center: P2D = P2D(x, y)
                    large_hole: Circle = Circle(f"Large Hole[{x_index}, {y_index}]",
                                                large_hole_diameter, 8, center)
                    large_holes.append(large_hole)
        return large_holes

    # Romi.expansion_polygon_get():
    def expansion_polygon_get(self) -> Polygon:
        """Return outline of Romi Expansion Chasis."""
        romi: Romi = self
        # The various dimensions are from `mechanical/pdf/romi-chasis-expansion_plate.pdf`
        # We use the same orientation as in the `.pdf` file, where the rounded portion is
        # towards the bottom and the flat portion is on top.  The orgin is set to be at the
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
        expansion_large_holes: List[Circle] = romi.expansion_large_holes_get()
        simple_polygons.extend(expansion_large_holes)
        expansion_small_holes: List[Circle] = romi.expansion_small_holes_get()
        simple_polygons.extend(expansion_small_holes)
        expansion_hex_holes_slots: List[SimplePolygon] = romi.expansion_hex_holes_slots_get()
        simple_polygons.extend(expansion_hex_holes_slots)
        expansion_standoff_holes: List[Circle] = romi.expansion_standoff_holes_get()
        simple_polygons.extend(expansion_standoff_holes)

        # Create and return *expansion_polygon* in an unlocked state:
        expansion_polygon: Polygon = Polygon("Expansion", simple_polygons, lock=True)
        return expansion_polygon

    # Romi.expansion_small_holes_get():
    def expansion_small_holes_get(self) -> List[Circle]:
        """Return the expansion chasis small hole pattern."""
        # romi: Romi = self
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

    # Romi.expansion_standoff_holes_get():
    def expansion_standoff_holes_get(self) -> List[Circle]:
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

    # Romi.hex_pattern_get():
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
        romi: Romi = self
        debugging: bool = romi.debugging

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
        lower_right_vertical_slot: Square = romi.rectangle_locate("Lower Right Hex Slot",
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
                        # Put in the *right_hole*:
                        hole: Circle = Circle(f"RIGHT: {label} Hex Hole ({x_index}, {y_index})",
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
            short_slot_rectangle: Square = romi.rectangle_locate("Short Slot Rectangle",
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

    # Romi.hole_locate():
    def hole_locate(self, name: str, x1: float, y1: float, x2: float, y2: float) -> Circle:
        """TODO."""
        romi: Romi = self
        origin_offset: P2D = romi.origin_offset
        inches2mm: float = romi.inches2mm
        x1 *= inches2mm
        y1 *= inches2mm
        x2 *= inches2mm
        y2 *= inches2mm

        dx: float = abs(x2 - x1)
        dy: float = abs(y2 - y1)
        center_x: float = (x1 + x2) / 2.0
        center_y: float = (y1 + y2) / 2.0
        diameter: float = (dx + dy) / 2.0
        center: P2D = P2D(center_x, center_y) - origin_offset
        hole: Circle = Circle(name, diameter, 8, center)
        return hole

    # Romi.line_hole_polygons_get():
    def line_hole_polygons_get(self, lower_hex_table: Dict[str, P2D]) -> List[SimplePolygon]:
        """TODO."""
        # Grab some values from *romi* (i.e. *self*):
        romi: Romi = self
        debugging: bool = romi.debugging

        # There is a line of holes along the bottom that have a smaller hole diameter.
        # We locate the smallest hole at the end of the line:
        small_hole: Circle = romi.hole_locate("Small Hole",
                                              -3.289535, 0.256524, -3.198980, 0.165984)
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

    # Romi.lower_arc_holes_rectangles_get():
    def lower_arc_holes_rectangles_get(self) -> List[SimplePolygon]:
        """TODO."""
        # Grab some values from *romi*:
        romi: Romi = self
        debugging: bool = romi.debugging
        inches2mm: float = romi.inches2mm
        origin_offset: P2D = romi.origin_offset

        # The resulting *Polygon*'s are collected into *lower_arc_holes_rectangles*:
        lower_arc_holes_rectangles: List[SimplePolygon] = []

        # There are arcs of holes and and rectangular slots along the upper and lower rims.
        # Since they are mirrored across the Y axis, we only worry about the right side.
        # The hole closest to the wheel is the "start" hole and the one farthest from the
        # wheel is the "end" hole.  We have to locate each of these holes:
        lower_start: Circle = romi.hole_locate("Lower Start Hole",
                                               -1.483063, 1.348929, -1.357508, 1.223803)
        lower_start_diameter: float = lower_start.diameter
        lower_start_center: P2D = lower_start.center
        lower_arc_start_angle: float = atan2(lower_start_center.y, lower_start_center.x)
        if debugging:  # pragma: no cover
            print(f"lower_start_diameter={lower_start_diameter}")
            print(f"lower_start_center={lower_start_center}")
            print(f"lower_start_angle={degrees(lower_arc_start_angle)}deg")

        # We pick the smallest hole that is next to the hole at the end to get the
        # *small_hole_diameter*:
        lower_end: Circle = romi.hole_locate("Lower End Hole",
                                             -3.144020, 0.125287, -3.053465, 0.034732)
        lower_end_diameter: float = lower_end.diameter
        lower_end_center: P2D = lower_end.center
        small_hole_diameter: float = lower_end_diameter

        lower_arc_end_angle: float = atan2(lower_end_center.y, lower_end_center.x)
        if debugging:  # pragma: no cover
            print(f"lower_start_diameter={lower_start_diameter}")
            print(f"lower_start_center={lower_start_center}")

        # Compute the *lower_arc_radius*:
        origin: P2D = P2D(0.0, 0.0)
        lower_hole_radius: float = origin.distance(lower_start_center)

        # There are two sizes of rectangle -- small and large.  The width appears to
        # be the same for both, so we only need *rectangle_width*, *small_rectangle_length*
        # and *large_rectangle_length*.  Lastly, we need to find one *rectangle_center*
        # so we can determine the *rectangle_radius* from the *origin*:
        large_upper_left_corner: P2D = (P2D(-1.248201 * inches2mm, 1.259484 * inches2mm) -
                                        origin_offset)
        large_lower_left_corner: P2D = (P2D(-1.33137 * inches2mm, 1.136248 * inches2mm) -
                                        origin_offset)
        large_upper_right_corner: P2D = (P2D(-1.205772 * inches2mm, 1.230858 * inches2mm) -
                                         origin_offset)
        large_rectangle_length: float = large_upper_left_corner.distance(large_lower_left_corner)
        rectangle_width: float = large_upper_left_corner.distance(large_upper_right_corner)
        rectangle_center: P2D = (large_upper_right_corner + large_lower_left_corner) / 2.0
        rectangle_radius: float = origin.distance(rectangle_center)
        small_upper_left_corner: P2D = (P2D(-1.368228 * inches2mm, 1.081638 * inches2mm) -
                                        origin_offset)
        small_lower_left_corner: P2D = (P2D(-1.431575 * inches2mm, 0.987760 * inches2mm) -
                                        origin_offset)
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

    # Romi.lower_hex_polygons_table_get():
    def lower_hex_polygons_table_get(self) -> Tuple[List[SimplePolygon], Dict[str, P2D]]:
        """TODO."""
        # The "User's Guide" identifies the lower hex whole used to reference the hex
        # pattern off of:
        # Grab some values from *romi* (i.e. *self*):
        romi: Romi = self
        debugging: bool = romi.debugging

        # Extract the origin hole information for the lower hexagon pattern:
        lower_hex_hole: Circle = romi.hole_locate("Lower Hex Hole",
                                                  -2.454646, 1.553776, -2.329091, 1.428650)
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
        lower_hex_polygons, lower_holes_table = romi.hex_pattern_get(lower_pattern_rows,
                                                                     lower_slot_pairs,
                                                                     lower_hex_hole_center,
                                                                     lower_hex_hole_diameter,
                                                                     "LOWER", True)
        return lower_hex_polygons, lower_holes_table

    # Romi.miscellaneous_holes_get():
    def miscellaneous_holes_get(self) -> List[Circle]:
        """Return the miscellaneous holes."""
        # Around the lower hex pattern there are a bunch of miscellaneous holes
        # around the periphery of the patten.  This code just picks them off:
        romi: Romi = self
        lower_left_hole: Circle = romi.hole_locate("RIGHT: Misc Small Lower Left",
                                                   -3.119047, 0.658110,
                                                   -3.041756, 0.74865)
        upper_left_hole: Circle = romi.hole_locate("RIGHT: Misc Small Upper Left",
                                                   -3.028492, 1.642358,
                                                   -3.119047, 1.732913)
        upper_right_hole_minus30: Circle = romi.hole_locate("RIGHT: Misc Small Upper Right -30deg",
                                                            -1.755228, 1.642358,
                                                            -1.664673, 1.732913)
        upper_right_hole_30: Circle = romi.hole_locate("RIGHT: Misc Small Upper Right 30deg",
                                                       -1.755228, 1.839205,
                                                       -1.664673, 1.929760)
        upper_right_hole_90: Circle = romi.hole_locate("RIGHT: Misc Small Upper Right 90deg",
                                                       -1.925717, 1.937638,
                                                       -1.835161, 2.028177)
        upper_right_hole_120: Circle = romi.hole_locate("RIGHT: Misc Small Upper Right 120deg",
                                                        -2.096189, 1.839205,
                                                        -2.005634, 1.929760)
        upper_right_large: Circle = romi.hole_locate("RIGHT: Misc Large Upper Right",
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

    # Romi.rectangle_locate():
    def rectangle_locate(self, name: str, x1: float, y1: float,
                         x2: float, y2: float) -> Square:
        """Locate a non-rotated rectangle and return it as a Square.

        Args:
            *name* (*str*): The name to assign to the returned
                *Polygon*.
            *x1* (*float*): One of sides of the rectangle in inches.
            *y1* (*float*): Either the top or bottom edge the rectangle
                in inches.
            *x1* (*float*): One of sides of the rectangle in inches.
            *y2* (*float*): The other top/bottom edge of the rectangle
                in inches:

        Returns:
            (*Polygon*) Returns a *Polygon* that represents the
            rectangle.

        """
        # Grab some values from *romi* (i.e. *self*):
        romi: Romi = self
        origin_offset: P2D = romi.origin_offset
        inches2mm: float = romi.inches2mm

        # Convert *x1*, *y1*, *x2*, and *y2* from inches to millimeters:
        x1 *= inches2mm
        y1 *= inches2mm
        x2 *= inches2mm
        y2 *= inches2mm
        dx: float = abs(x2 - x1)
        dy: float = abs(y2 - y1)
        center: P2D = P2D((x1 + x2) / 2.0, (y1 + y2) / 2.0) - origin_offset

        # Create and return the *rectangle* (which is stored in a *Square* object):
        rectangle: Square = Square(name, dx, dy, center)
        return rectangle

    # Romi.scad_program_append():
    def scad_program_append(self, scad_program: ScadProgram, if2d: If2D, if3d: If3D) -> None:
        """Append all of the Romi models to a ScadProgram."""
        # Use *romi* instead of *self*:a
        romi: Romi = self

        # Start with the just *rome_base_polygon* and append it to *scad_program* and *if2d*:
        romi_base_polygon: Polygon = romi.base_polygon_get()
        romi_base_polygon_module: Module2D = Module2D("Romi_Base_Polygon_Module",
                                                      [romi_base_polygon])
        scad_program.append(romi_base_polygon_module)
        if2d.then_append('name == "romi_base"',
                         [UseModule2D("Romi_Base_Polygon", romi_base_polygon_module)])

        # Next append the *expansion_polygon*:
        expansion_polygon: Polygon = romi.expansion_polygon_get()
        expansion_module: Module2D = Module2D("Romi_Expansion_Polygon_Module", [expansion_polygon])
        scad_program.append(expansion_module)
        if2d.then_append('name == "expansion"',
                         [UseModule2D("Romi_Expansion_Polygon_Module", expansion_module)])

    # Romi.upper_arc_holes_rectangles_get():
    def upper_arc_holes_rectangles_get(self) -> List[SimplePolygon]:
        """TODO."""
        # Grab some values from *romi*:
        romi: Romi = self
        debugging: bool = romi.debugging
        inches2mm: float = romi.inches2mm
        origin_offset: P2D = romi.origin_offset

        # The resulting *Polygon*'s are collected into *upper_arc_holes_rectangles*:
        upper_arc_holes_rectangles: List[SimplePolygon] = []

        # There are arcs of holes and and rectangular slots along the upper and lower rims.
        # Since they are mirrored across the Y axis, we only worry about the right side.
        # The hole closest to the wheel is the "start" hole and the one farthest from the
        # wheel is the "end" hole.  We have to locate each of these holes:
        upper_start: Circle = romi.hole_locate("Upper Start Hole",
                                               -1.483063, 4.651469, -1.357508, 4.526346)
        upper_start_diameter: float = upper_start.diameter
        upper_start_center: P2D = upper_start.center
        upper_arc_start_angle: float = atan2(upper_start_center.y, upper_start_center.x)
        if debugging:  # pragma: no cover
            print(f"upper_start_diameter={upper_start_diameter}")
            print(f"upper_start_center={upper_start_center}")
            print(f"upper_start_angle={degrees(upper_arc_start_angle)}deg")

        upper_end: Circle = romi.hole_locate("Upper End Hole",
                                             -3.14402, 5.840524, -3.053465, 5.749969)
        upper_end_diameter: float = upper_end.diameter
        upper_end_center: P2D = upper_end.center
        upper_arc_end_angle: float = atan2(upper_end_center.y, upper_end_center.x)
        if debugging:  # pragma: no cover
            print(f"upper_end_diameter={upper_end_diameter}")
            print(f"upper_end_center={upper_end_center}")

        # Compute the *upper_hole_radius*:
        origin: P2D = P2D(0.0, 0.0)
        upper_hole_radius: float = origin.distance(upper_start_center)

        # There are two sizes of rectangle -- small and large.  The width appears to
        # be the same for both, so we only need *rectangle_width*, *small_rectangle_length*
        # and *large_rectangle_length*.  Lastly, we need to find one *rectangle_center*
        # so we can determine the *rectangle_radius* from the *origin*:
        large_upper_inner_corner: P2D = (P2D(-1.33137 * inches2mm, 4.739012 * inches2mm) -
                                         origin_offset)
        large_lower_inner_corner: P2D = (P2D(-1.248201 * inches2mm, 4.615776 * inches2mm) -
                                         origin_offset)
        large_lower_outer_corner: P2D = (P2D(-1.205772 * inches2mm, 4.644402 * inches2mm) -
                                         origin_offset)
        large_rectangle_length: float = large_upper_inner_corner.distance(large_lower_inner_corner)
        rectangle_width: float = large_lower_inner_corner.distance(large_lower_outer_corner)
        rectangle_center: P2D = (large_upper_inner_corner + large_lower_outer_corner) / 2.0
        rectangle_radius: float = origin.distance(rectangle_center)
        small_upper_inner_corner: P2D = (P2D(-1.431575 * inches2mm, 4.887512 * inches2mm) -
                                         origin_offset)
        small_lower_inner_corner: P2D = (P2D(-1.368228 * inches2mm, 4.793638 * inches2mm) -
                                         origin_offset)
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

    # Romi.upper_hex_polygons_get():
    def upper_hex_polygons_get(self) -> List[SimplePolygon]:
        """TODO."""
        # Grab some values from *romi* (i.e. *self*):
        romi: Romi = self
        debugging: bool = romi.debugging

        # For the upper hex pattern, the hole that is at the end of the 4 slots is selected
        # as the upper hex hole:
        upper_hex_hole: Circle = romi.hole_locate("Upper Hex Hole",
                                                  -2.749535, 5.441567, -2.629075, 5.316441)
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
        upper_hex_polygons, upper_holes_table = romi.hex_pattern_get(upper_pattern_rows,
                                                                     upper_slot_pairs,
                                                                     upper_hex_hole_center,
                                                                     upper_hex_hole_diameter,
                                                                     "UPPER", False)

        # The *upper_holes_table* is not needed, we just return *upper_hex_polygons*:
        return upper_hex_polygons

    # Romi.vertical_rectangles_get():
    def vertical_rectangles_get(self) -> List[Square]:
        """Return the vertical wheel well rectangles."""
        romi: Romi = self
        upper_rectangle0: Square = romi.rectangle_locate("RIGHT: UPPER: Rectangle 0",
                                                         -1.511965, 3.569984,
                                                         -1.460783, 3.683232)
        upper_rectangle1: Square = romi.rectangle_locate("RIGHT: UPPER: Rectangle 1",
                                                         -1.511965, 3.749122,
                                                         -1.460783, 3.897803)
        upper_rectangle2: Square = romi.rectangle_locate("RIGHT: UPPER: Rectangle 2",
                                                         -1.511965, 3.963677,
                                                         -1.460783, 4.076929)
        upper_rectangle3: Square = romi.rectangle_locate("RIGHT: UPPER: Rectangle 3",
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

    # Romi.holes_slots_rectangles_write():
    def holes_slots_rectangles_write(self):
        """Write out the holes, slots, and rectangle lcoations."""
        # Obtain the hole, slot and rectangle locations:
        romi: Romi = self
        romi_base_polygon: Polygon = romi.base_polygon_get()
        romi_base_simple_polygons: List[SimplePolygon] = romi_base_polygon.simple_polygons_get()
        romi_base_simple_polygon: SimplePolygon
        # Generate *romi_base_keys* skipping the first *SimplePolygon* which is the outer one:
        romi_base_keys: List[Tuple[Any, ...]] = [romi_base_simple_polygon.key()
                                                 for romi_base_simple_polygon
                                                 in romi_base_simple_polygons[1:]]
        romi_base_keys.sort()

        # Now output the hole/slot/rectangle `.csv` file:
        csv_file: IO[Any]
        with open("romi_base.csv", "w") as csv_file:
            Scad.keys_csv_file_write(romi_base_keys, csv_file)

        # Output the hole/slot/recantangle `.html` file:
        html_file: IO[Any]
        with open("romi_base.html", "w") as html_file:
            Scad.keys_html_file_write(romi_base_keys, html_file, "Romi Base Holes and Rectangles")


# OtherPi:
class OtherPi:
    """Represents a different SBC compatible with Raspberry Pi."""

    # OtherPi.__init__():
    def __init__(self) -> None:
        """Initialize OtherPi."""
        pass

    # OtherPi.board_polygon_get():
    def pcb_polygon_get(self) -> Polygon:
        """Return the PCB Polygon that represents the PCB."""
        # Define the board dimensions:
        pcb_dx: float = 103.00
        pcb_dy: float = 70.49

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

        # Lock *pcb_polygon* and return it:
        pcb_polygon.lock()
        return pcb_polygon

    # OtherPi.connectors_get():
    def connectors_get(self) -> List[Scad3D]:
        """Return the connector Cube's for the OtherPi."""
        male_2x20_header: MalePinConnector = MalePinConnector(2, 20, 5.840)
        connectors: List[Scad3D] = []
        connectors.append(CornerCube("Ethernet Connector", P3D(80.00, 29.26, 0.00),
                                     P3D(105.00, 44.51, 13.08)))  # 80.00? 105?
        connectors.append(CornerCube("USB2 Connector", P3D(85.00, 12.82, 0.00),
                                     P3D(104.00, 25.57, 15.33)))  # 85.00? 104.00?
        connectors.append(CornerCube("West Connector", P3D(98.00, 50.69, 0.00),
                                     P3D(105.00, 58.09, 3.00)))  # 98.00? 3.00? 105??
        connectors.append(CornerCube("North Connector", P3D(70.03, 60.92, 0.00),
                                     P3D(76.38, 66.00, 5.00)))  # 5.00? 66.00??
        # connectors.append(CornerCube("2x20 Connector", P3D(7.37, 64.00, 0.00),
        #                              P3D(57.67, 69.08, 7.00)))  # 7.00? 69.08? 64.00?
        connectors.append(Translate3D("Moved 2x20 Header", male_2x20_header.connector_get(),
                                      P3D((7.37 + 57.67) / 2.0, (64.00 + 69.08) / 2.0)))
        connectors.append(CornerCube("Audio Connector", P3D(66.31, -1.00, 0.00),
                                     P3D(72.31, 14.00, 5.12)))  # -1.00? 14.00?
        connectors.append(CornerCube("USB3A Connector", P3D(9.54, 0.00, 0.00),
                                     P3D(18.52, 10.00, 2.95)))  # 10.00?
        connectors.append(CornerCube("USB3B Connector", P3D(22.73, 0.00, 0.00),
                                     P3D(31.71, 10.00, 2.95)))  # 10.00?
        connectors.append(CornerCube("Power Connector", P3D(-1.00, 22.50, 0.00),
                                     P3D(8.00, 29.00, 2.95)))  # -1.00? 8.00?
        connectors.append(CornerCube("JST Connector", P3D(0.00, 55.01, 0.00),
                                     P3D(3.00, 63.37, 2.95)))  # 0.00? 3.00?
        connectors.append(CornerCube("Buttons Area", P3D(85.51, 62.12, 0.00),
                                     P3D(98.23, 68.74, 2.95)))  # 10.00?
        return connectors

    # OtherPi.scad_program_append():
    def scad_program_append(self, scad_program: ScadProgram, if2d: If2D, if3d: If3D) -> None:
        """Append OtherPi OpenScad code to a ScadProgram."""
        # Create the PCB *Polygon* and *Module2D*:
        other_pi: OtherPi = self
        other_pi_pcb_polygon: Polygon = other_pi.pcb_polygon_get()
        other_pi_pcb_module: Module2D = Module2D("Other_Pi_PCB_Polygon_Module",
                                                 [other_pi_pcb_polygon])
        scad_program.append(other_pi_pcb_module)
        if2d.then_append('name == "other_pi_pcb"',
                         [UseModule2D("OtherPi PCB Polygon", other_pi_pcb_module)])

        # Now create the *other_pi_module*:
        other_pi_model: Union3D = Union3D("OtherPi Model", [], lock=False)
        other_pi_pcb: Scad3D = LinearExtrude("OtherPiPCB", other_pi_pcb_polygon, 1.000)
        other_pi_model.append(Translate3D("Move Down 1mm", other_pi_pcb, P3D(-0.990, 0.0, 0.0)))
        other_pi_model.extend(other_pi.connectors_get())
        other_pi_model.lock()
        other_pi_model_module: Module3D = Module3D("OtherPi_Model_Module", [other_pi_model])
        scad_program.append(other_pi_model_module)
        if3d.then_append('name == "otherpi_model"',
                         [UseModule3D("OtherPi_Model", other_pi_model_module)])


def main() -> int:  # pragma: no cover
    """Generate the openscand file."""
    # print("romi_model.main() called")
    # scad_file: IO[Any]
    # with open("romi_base_dxf.scad", "w") as scad_file:
    #     romi_base_polygon.scad_file_write(scad_file)

    # romi_base_extruded_polygon: LinearExtrude = LinearExtrude("Romi Base Extrude",
    #                                                           romi_base_polygon, 9.6)
    # union: Union = Union("Base Union", [romi_base_extruded_polygon])
    # with open("romi_base.scad", "w") as scad_file:
    #     union.scad_file_write(scad_file)

    # Create the top level *scad_program* program and the two if-then-else trees for
    # decided which model to visualize::
    scad_program: ScadProgram = ScadProgram("Scad models")
    if2d: If2D = If2D("Polygon Modules", 'false', [])
    if3d: If3D = If3D("3D Models", 'false', [])

    # Create the *romi* object for constructing various portions of a Romi platform:
    romi: Romi = Romi()
    romi.scad_program_append(scad_program, if2d, if3d)
    romi.holes_slots_rectangles_write()

    # Now create *other_pi* and append it as well:
    other_pi: OtherPi = OtherPi()
    other_pi.scad_program_append(scad_program, if2d, if3d)

    # Create *raspi3b*
    raspi3b: RaspberryPi3 = RaspberryPi3()
    raspi3b.scad_program_append(scad_program, if2d, if3d)

    # Now construct an if-then-else sequence to showing the desired module:
    scad_program.append(Variable2D("Name", "name", '"romi_base"'))

    # Now do the if-then-else sequences for showing models:
    # Lock *if2d* and *if3d* and append them to *scad_program*:
    if2d.lock()
    scad_program.append(if2d)
    if3d.lock()
    scad_program.append(if3d)

    # Generate `scad_models.scad`:
    scad_lines: List[str] = []
    scad_program.scad_lines_append(scad_lines, "")
    scad_lines.append("")
    scad_text: str = '\n'.join(scad_lines)
    scad_file: IO[Any]
    with open("scad_models.scad", "w") as scad_file:
        scad_file.write(scad_text)

    return 0
