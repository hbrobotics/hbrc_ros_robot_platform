# <--------------------------------------- 100 characters ---------------------------------------> #

"""Code to genarate and openscad model of the Pololu Romi Base."""

# Pumpkin Pi: 104mm x 70mm.  Holes are the same as the Raspberry Pi with the upper left
# hole in the upper left corner of the PCB.  The extra PCB space goes to the right and
# down on the Pumpkin Pi.

# http://docplayer.net/42910792-
# Hardware-assisted-tracing-on-arm-with-coresight-and-opencsd-mathieu-poirier.html
from typing import Any, Dict, IO, List, Tuple
from math import asin, atan2, ceil, cos, degrees, pi, sin, sqrt


class P:
    """Represents a 2 dimensional point."""

    def __init__(self, x: float, y: float):
        """Initialize a point."""
        self.x: float = x
        self.y: float = y

    def __add__(self, point2: "P") -> "P":
        """Add two points together."""
        point1: P = self
        return P(point1.x + point2.x, point1.y + point2.y)

    def __truediv__(self, scale: float) -> "P":
        """Divide a point by a scale factor."""
        point: P = self
        return P(point.x / scale, point.y / scale)

    def __mul__(self, scale: float) -> "P":
        """Multiply a point by a scale factor."""
        point: P = self
        return P(point.x * scale, point.y * scale)

    def __rmul__(self, scale: float) -> "P":
        """Multiply a point by a scale factor."""
        point: P = self
        return P(point.x * scale, point.y * scale)

    def __sub__(self, point2: "P") -> "P":
        """Subtract two points from one another."""
        point1: P = self
        return P(point1.x - point2.x, point1.y - point2.y)

    def __str__(self) -> str:
        """Convert a point to a string."""
        point: P = self
        return f"P({point.x}, {point.y})"

    def distance(self, point2) -> float:
        """Compute the distance between two points."""
        point1: P = self
        dx: float = point1.x - point2.x
        dy: float = point1.y - point2.y
        return sqrt(dx * dx + dy * dy)

    def rotate(self, angle: float):
        """Rotate a point by angle around the origin."""
        # To rotate a point (x, y) around the origin, use the following formula:
        #
        #   x' = x * cos(angle) - y * sin(angle)
        #   y' = y * cos(angle) + x * sin(angle)
        point: P = self
        x: float = point.x
        y: float = point.y
        sin_angle: float = sin(angle)
        cos_angle: float = cos(angle)
        rotated_point: P = P(x * cos_angle - y * sin_angle, y * cos_angle + x * sin_angle)
        return rotated_point

    def y_mirror(self) -> "P":
        """Return the point mirrored across the Y axis."""
        point: P = self
        return P(-point.x, point.y)


class ComplexPolygon:
    """Represent a compleax polygon with holes in it."""

    def __init__(self, name: str, outer_polygon: "Polygon") -> None:
        """Initialize a complex polygon with an outer Polygon."""
        self.holes: List[Polygon] = list()
        self.name: str = name
        self.outer_polygon: Polygon = outer_polygon

    def hole_append(self, hole: "Polygon") -> None:
        """Append an inner polygon."""
        complex_polygon: ComplexPolygon = self
        holes: List[Polygon] = complex_polygon.holes
        holes.append(hole)

    def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:
        """Append an OpenSCAD polygon to a list of lines."""
        # Grab some values from *complex_polygon* (i.e. *self*):
        complex_polygon: ComplexPolygon = self
        holes: List[Polygon] = complex_polygon.holes
        name: str = complex_polygon.name
        outer_polygon: Polygon = complex_polygon.outer_polygon

        # Construct *all_polygons* and output the initial OpenSCAD `polygon` statement:
        all_polygons: List[Polygon] = [outer_polygon] + holes
        scad_lines.append(f"{indent}polygon(points=[ // '{name}'")

        # Output the actual points:
        next_indent: str = indent + " "
        base_index: int = 0
        polygon: Polygon
        for polygon in all_polygons:
            base_index = polygon.scad_lines_points_append(scad_lines, next_indent, base_index)

        # Output the point indices:
        scad_lines.append(f"{next_indent}], paths=[")
        base_index = 0
        for polygon in all_polygons:
            base_index = polygon.scad_lines_point_indices_append(scad_lines,
                                                                 next_indent, base_index)

        # Wrap up `polygon` statement:
        scad_lines.append(f"{indent}]);")


class Polygon:
    """Represent a Polygon as a sequence of points."""

    def __init__(self, name: str) -> None:
        """Initialize the Polygon."""
        self.name: str = name
        self.points: List[P] = list()

    def append(self, point: P):
        """Append a point to the Polygon."""
        polygon: Polygon = self
        polygon.points.append(point)

    def arc(self, origin: P, radius: float, start_radian: float, end_radian: float,
            points_count: int) -> None:
        """Append an arc to a Polygon."""
        polygon: Polygon = self
        span_radian: float = end_radian - start_radian
        delta_radian: float = span_radian / float(points_count - 1)
        # radians2degrees: float = 180.0 / pi
        # print(f"start_radian={start_radian}={start_radian * radians2degrees}deg")
        # print(f"end_radian={end_radian}={end_radian * radians2degrees}deg")
        # print(f"span_radian={span_radian}={span_radian * radians2degrees}deg")
        # print(f"delta_radian={delta_radian}={delta_radian * radians2degrees}deg")
        index: int
        for index in range(points_count):
            radian: float = start_radian + index * delta_radian
            x: float = origin.x + radius * cos(radian)
            y: float = origin.y + radius * sin(radian)
            # print(f"[{index}]radian={radian}={radian * radians2degrees} x={x} y={y}")
            polygon.append(P(x, y))

    def circle(self, origin: P, diameter: float, points_count):
        """Append a circle to Polygon."""
        polygon: Polygon = self
        delta_radian: float = (2 * pi) / points_count
        radius: float = diameter / 2.0
        index: int
        for index in range(points_count):
            radian: float = index * delta_radian
            x: float = origin.x + radius * cos(radian)
            y: float = origin.y + radius * sin(radian)
            polygon.append(P(x, y))

    def rotated_rectangle(self, center: P, dx: float, dy: float, angle: float):
        """Append a rotated rectangle to Polygon."""
        # Compute some the four unrotated points of the rectangle:
        polygon: Polygon = self
        half_dx: float = dx / 2.0
        half_dy: float = dy / 2.0
        upper_right_corner: P = P(half_dx, half_dy)
        lower_right_corner: P = P(half_dx, -half_dy)
        upper_left_corner: P = P(-half_dx, half_dy)
        lower_left_corner: P = P(-half_dx, -half_dy)

        # Compute the 4 rotated corners offset by *center*:
        rotated_upper_right_corner: P = center + upper_right_corner.rotate(angle)
        rotated_lower_right_corner: P = center + lower_right_corner.rotate(angle)
        rotated_upper_left_corner: P = center + upper_left_corner.rotate(angle)
        rotated_lower_left_corner: P = center + lower_left_corner.rotate(angle)

        # Append the points to the polygon:
        polygon.append(rotated_upper_right_corner)
        polygon.append(rotated_lower_right_corner)
        polygon.append(rotated_lower_left_corner)
        polygon.append(rotated_upper_left_corner)

    def scad_lines_point_indices_append(self, scad_lines: List[str],
                                        indent: str, start_index: int) -> int:
        """Append the Polygon points to a list of lines."""
        # Grab some values from *polygon* (i.e. *self*):
        polygon: Polygon = self
        name: str = polygon.name
        points: List[P] = polygon.points

        # Compute *end_index* from *start_index* and *points_size*:
        points_size: int = len(points)
        end_index: int = start_index + points_size

        # Figure out how many slices to output (i.e. *slice_count*):
        slice_size: int = 25
        slices_count: int = int(ceil(float(points_size) / float(slice_size)))

        # Output some debugging information:
        scad_lines.append(f"{indent}// '{name}' ")

        slice_index: int
        for slice_index in range(slices_count):
            start_slice_index: int = slice_index * slice_size
            end_slice_index: int = min((slice_index + 1) * slice_size, points_size)
            line_text: str = ", ".join([f"{start_index + index}"
                                        for index in range(start_slice_index, end_slice_index)])
            front_text: str = "[" if slice_index == 0 else ""
            end_text: str = "]," if end_slice_index == points_size else ","
            scad_lines.append(f"{indent}{front_text}{line_text}{end_text}")
        return end_index

    def scad_lines_points_append(self, scad_lines: List[str], indent: str, start_index: int) -> int:
        """Append the PolyPoints points to a list of lines."""
        # Grab some values from *polygon* (i.e. *self*):
        polygon: Polygon = self
        name: str = polygon.name
        points: List[P] = polygon.points

        # Compute *end_index* from *start_index* and *points_size*:
        points_size: int = len(points)
        end_index: int = start_index + points_size

        # Figure out the number of *slice_points* to output:
        slice_size: int = 4
        slices_count: int = int(ceil(float(points_size) / float(slice_size)))

        # Append a debugging line:
        scad_lines.append(f"{indent}// '{name}' {start_index}-{end_index-1}")

        # Sweep through *points* and output chunks of *slice_points*:
        slice_index: int
        for slice_index in range(slices_count):
            # Extract the next chunk of *slice_points*:
            slice_start: int = slice_index * slice_size
            slice_end: int = min((slice_index + 1) * slice_size, points_size)
            slice_points: List[P] = points[slice_start:slice_end]

            # Just to be paranoid, make sure we actually have at least one point:
            if slice_points:
                # Splice *splice_point* together as a comma separated list:
                point_texts: List[str] = ["[{0:.3f}, {1:.3f}]".format(point.x, point.y)
                                          for point in slice_points]
                slice_text: str = ', '.join(point_texts)
                scad_lines.append(f"{indent}{slice_text}, "
                                  f"// {start_index + slice_start}-"
                                  f"{start_index + slice_end - 1}")
        return end_index

    def slot(self, hole1: P, hole2: P,
             slot_length: float, slot_width: float, points_count: int) -> None:
        """TODO."""
        polygon: Polygon = self
        degrees90: float = pi / 2.0
        degrees180: float = pi
        center: P = (hole1 + hole2) / 2.0
        slot_angle: float = atan2(hole1.y - hole2.y, hole1.x - hole2.x)
        slot_radius: float = slot_width / 2.0
        half_slot_length: float = slot_length / 2.0
        center1: P = P(center.x + half_slot_length * cos(slot_angle),
                       center.y + half_slot_length * sin(slot_angle))
        center2: P = P(center.x + half_slot_length * cos(slot_angle + degrees180),
                       center.y + half_slot_length * sin(slot_angle + degrees180))
        polygon.arc(center1, slot_radius,
                    slot_angle - degrees90,
                    slot_angle + degrees90, 8)
        polygon.arc(center2, slot_radius,
                    slot_angle + degrees180 - degrees90,
                    slot_angle + degrees180 + degrees90, points_count)


class Romi:
    """Model the Polol Romi platform in OpenSCAD."""

    def __init__(self) -> None:
        """Initialize and create the Romi Platform."""
        outer_polygon: Polygon = Polygon("Romi Outer Polygon")
        self.outer_polygon: Polygon = outer_polygon
        self.complex_polygon: ComplexPolygon = ComplexPolygon("Romi Base", outer_polygon)
        self.inches2mm: float = 25.4
        self.origin_offset: P = P(-9999.0, -9999.0)  # Update to the correct value later on.

    def hole_locate(self, left_x: float, right_x: float,
                    above_y: float, below_y: float) -> Tuple[float, P]:
        """TODO."""
        romi: Romi = self
        origin_offset: P = romi.origin_offset
        inches2mm: float = romi.inches2mm
        left_x *= inches2mm
        right_x *= inches2mm
        above_y *= inches2mm
        below_y *= inches2mm

        dx: float = right_x - left_x
        dy: float = above_y - below_y
        x: float = (left_x + right_x) / 2.0
        y: float = (above_y + below_y) / 2.0
        diameter: float = ((dx + dy) / 2.0)
        center: P = P(x, y) - origin_offset
        return diameter, center

    def generate(self) -> None:
        """TODO."""
        romi: Romi = self
        complex_polygon: ComplexPolygon = romi.complex_polygon
        inches2mm: float = romi.inches2mm

        # These other dimensions are read off of the drawings in section 6 of the
        # the "Pololu Romi Chasis User's Guide":
        diameter: float = 163.0
        radius: float = diameter / 2.0
        overall_width: float = 149.0
        wheel_well_dx: float = 125.0
        wheel_well_dy: float = 72.0
        half_wheel_well_dx: float = wheel_well_dx / 2.0
        half_wheel_well_dy: float = wheel_well_dy / 2.0

        # Set *debugging* to *True* to print out some debugging stuff:
        debugging: bool = True
        if debugging:
            print(f"diameter={diameter}mm radius={radius}mm")
            print(f"overall_width={overall_width}mm")
            print(f"wheel_well_dx={wheel_well_dx}mm half_wheel_well_dx={half_wheel_well_dx}mm")
            print(f"wheel_well_dy={wheel_well_dy}mm half_wheel_well_dy={half_wheel_well_dy}mm")

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
        wheel_well_angle: float = asin(half_wheel_well_dy / radius)
        origin: P = P(0.0, 0.0)

        # Perform any requested *debugging*:
        if debugging:
            radians2degrees: float = 180.0 / pi
            print(f"wheel_well_angle={wheel_well_angle}={wheel_well_angle * radians2degrees}")
            wheel_well_x: float = radius * cos(wheel_well_angle)
            wheel_well_y: float = radius * sin(wheel_well_angle)
            print(f"wheel_well_x={wheel_well_x}")
            print(f"wheel_well_y={wheel_well_y}")
            distance: float = origin.distance(P(wheel_well_x, wheel_well_y))
            print(f"distance={distance}")

        # Now we can draw the outer outline of the Romi platform.  It conists of two arcs
        # with some straight line segments to form the wheel well.  Start by creating
        # an empty *outer_polygon*:a
        outer_polygon: Polygon = self.outer_polygon

        # Create the upper arc:
        upper_start_angle: float = wheel_well_angle
        upper_end_angle: float = pi - wheel_well_angle
        arc_count = 21
        outer_polygon.arc(origin, radius, upper_start_angle, upper_end_angle, arc_count)

        # Create the left wheel well:
        outer_polygon.append(P(-half_wheel_well_dx, half_wheel_well_dy))
        outer_polygon.append(P(-half_wheel_well_dx, -half_wheel_well_dy))

        # Create the lower arg:
        lower_start_angle: float = wheel_well_angle + pi
        lower_end_angle: float = upper_end_angle + pi
        outer_polygon.arc(origin, radius, lower_start_angle, lower_end_angle, arc_count)

        # Create the right wheel well:
        outer_polygon.append(P(half_wheel_well_dx, -half_wheel_well_dy))
        outer_polygon.append(P(half_wheel_well_dx, half_wheel_well_dy))

        # Figuring out where everything is located is done with a combination of reading
        # drawing in section 6 of the "Pololu Romi Chassis User's Guide" and extracting
        # values from the `romi-chassis.dxf` file available from the Pololu web site.
        # It is important to notice that the section 6 drawings are upside-down from
        # the `romi-chassis.dxf`.  For the modeling of this platform, the `.dxf` file
        # orientation is used.  Sigh.  Also, the `.dxf` file seems to be in units of
        # inches rather than millimeters.  The rest of this code uses millimeters, so we
        # multiply inches coordinate values by *inches2mm* as soon as possible.  Furthermore,
        # the origin of the `.dxf` is off  to the lower right rather that in the traditional
        # center for differential drive robots.    The *hole_locate*  method is used to take
        # a hole bounding box inches from the `.dxf` file and convert it into a diameter
        # and center in millimeters using the origin in the center of the robot; this method
        # needs to have the *origin_offset* computed*  before it can be used.

        # It is pretty clear that the motor axles are intended to go through the center of the
        # Romi platform along the X axis.  This is done by reading the values for the top
        # and bottom of the axle (in inches) from the `.dxf` file.  This allows us to compute
        # *y_origin_offset* in millimeters:
        axel_y_above: float = 2.967165 * inches2mm
        axel_y_below: float = 2.908110 * inches2mm
        y_origin_offset: float = (axel_y_above + axel_y_below) / 2.0

        # The *x_origin_offset* is computed using the upper castor hole location:
        upper_castor_x_left: float = -3.930756 * inches2mm
        upper_castor_x_right: float = -3.805256 * inches2mm
        x_origin_offset: float = (upper_castor_x_left + upper_castor_x_right) / 2.0

        # Finally we have *origin_offset* in millimeters and can save it back into *romi*.
        # From here on out we tend to use the *hole_locate* method to identify holes:
        origin_offset: P = P(x_origin_offset, y_origin_offset)
        romi.origin_offset = origin_offset
        if debugging:
            print(f"origin_offset={origin_offset}")

        # All of the battery holes are done relative to the *battery_reference_hole*
        # indicated on the drawing of the dimensions and mounting holes seciont of the
        # "Pololu Romi Chassis User's Guide".
        battery_reference_hole_diameter: float
        battery_reference_hole_center: P
        battery_reference_hole_diameter, battery_reference_hole_center = romi.hole_locate(-3.913146,
                                                                                          -3.822591,
                                                                                          3.376610,
                                                                                          3.286051)

        # The battery holes have an upper and lower group.  The lower group resides between
        # the motors and the upper group is above the motors.  The lower group is organized
        # in 3 rows by 9 columns and not all holes are poplulated.  We create a
        # *lower_battery_pattenrs* list to specify which of the holes need to be poputlated.
        # Remember, the `.dxf` orientation is being used and move down from the
        # *battery_reference_hole_center*:
        lower_battery_y_offsets: Tuple[float, ...] = (0.0, -12.3, -12.3 - 12.3)
        lower_battery_patterns: Tuple[str, ...] = (
            "--**O**--",  # Row with reference hole in the middle (at the 'O' location)
            "--*****--",  # Row below reference hole
            "*-*****-*")  # Two rows below referene hole
        battery_reference_hole_center_y: float = battery_reference_hole_center.y
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
                    y: float = battery_reference_hole_center_y + lower_battery_y_offsets[y_index]
                    lower_hole_center: P = P(x, y)
                    lower_hole: Polygon = Polygon(f"Lower Battery Hole ({2-x_index}, {y_index})")
                    lower_hole.circle(lower_hole_center, battery_reference_hole_diameter, 8)
                    complex_polygon.hole_append(lower_hole)

        # The upper battery holes above the lower battery motor holes are organized as
        # 3 rows by 9 columns where all positions are populated:
        upper_battery_y_offsets: Tuple[float, ...] = (7.0, 7.0 + 12.3, 7.0 + 12.3 + 12.3)
        column0_x = -4.5 * hole_dx_pitch
        for x_index in range(10):
            x = column0_x + x_index * hole_dx_pitch
            for y_index in range(3):
                y = battery_reference_hole_center_y + upper_battery_y_offsets[y_index]
                upper_hole_center: P = P(x, y)
                upper_hole: Polygon = Polygon(f"Upper Battery Hole ({2-x_index}, {y_index})")
                upper_hole.circle(upper_hole_center, battery_reference_hole_diameter, 8)
                complex_polygon.hole_append(upper_hole)

        # There are two hex hole and slot patterns; one is above the motors and the
        # other is below the motors.  The *hex_pattern* method is used to populate
        # the various holes and slots.  We need to get the bounding box from one
        # vertical slot (in inches) and compute the *v

        # We need to get the dimensions for one vertical and compute the vertical *slot_width*
        # and the *slot_length* which is distance between to centers of the arc end points:
        slot_left_x: float = -2.437146 * inches2mm
        slot_right_x: float = -2.346591 * inches2mm
        slot_top_y: float = 1.339205 * inches2mm
        slot_bottom_y: float = 1.051803 * inches2mm
        slot_dx: float = slot_right_x - slot_left_x
        slot_dy: float = slot_top_y - slot_bottom_y
        # Remember: *slot_length* is from center to center, *NOT* edge to edge:
        slot_length: float = slot_dy - slot_dx
        slot_width: float = slot_dx
        if debugging:
            print(f"slot_dx={slot_dx}")
            print(f"slot_dy={slot_dy}")
            print(f"slot_length={slot_length}")
            print(f"slot_width={slot_width}")

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

        # The "User's Guide" identifies the lower hex whole used to reference the hex
        # pattern off of:
        lower_hex_hole_center: P
        lower_hex_hole_diameter: float
        lower_hex_hole_diameter, lower_hex_hole_center = romi.hole_locate(-2.454646, -2.329091,
                                                                          1.553776, 1.428650)
        if debugging:
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

        # We need to specify the upper left origin for the grid.  The 'O' is 6 columns
        # over and one row up:
        lower_hex_origin: P = lower_hex_hole_center - P(6 * half_hex_dx_pitch, -hex_dy_pitch)

        # Now we can invoke the *hex_pattern* method to fill in the hex pattern and
        # mirror it across the Y axis to the other sise:
        lower_holes_table: Dict[str, P] = romi.hex_pattern(lower_pattern_rows, lower_slot_pairs,
                                                           lower_hex_origin,
                                                           lower_hex_hole_diameter,
                                                           slot_width, slot_length,
                                                           half_hex_dx_pitch, hex_dy_pitch)

        # For the upper hex pattern, the hole that is at the end of the 4 slots is selected
        # as the upper hex hole:
        upper_hex_hole_center: P
        upper_hex_hole_diameter: float
        upper_hex_hole_diameter, upper_hex_hole_center = romi.hole_locate(-2.749535, -2.629075,
                                                                          5.441567, 5.316441)
        if debugging:
            print(f"upper_hex_hole_center={upper_hex_hole_center}")
            print(f"upper_hex_hole_diameter={upper_hex_hole_diameter}")

        # For the *upper_pattern_rows*, the 'O' marks the *upper_hex_hole_center*:
        upper_pattern_rows: Tuple[str, ...] = (
            "a------",
            "-A-O---",
            "b-B-C-c",
            "---d---",
        )

        # The *upper_hex_origin* is located 3 columns to left and one column up:
        upper_hex_origin: P = upper_hex_hole_center - P(3 * half_hex_dx_pitch, -hex_dy_pitch)

        # *upper_slot_pairs* specifies which slots to render.  Now we can invoke the *hex_pattern*
        # method to render the hex pattern and mirror it across to the other side:
        upper_slot_pairs: List[str] = "aO:bO:dO:cO".split(':')
        romi.hex_pattern(upper_pattern_rows, upper_slot_pairs,
                         upper_hex_origin, upper_hex_hole_diameter,
                         slot_width, slot_length,
                         half_hex_dx_pitch, hex_dy_pitch)

        # There is a line of holes along the bottom that have a smaller hole diameter.
        # We locate the smallest hole at the end of the line:
        small_hole_diameter: float
        small_hole_center: P
        small_hole_diameter, small_hole_center = romi.hole_locate(-3.289535, -3.198980,
                                                                  0.256524, 0.165984)
        if debugging:
            print(f"small_hole_diameter={small_hole_diameter}")
            print(f"small_hole_center={small_hole_center}")

        # Now using *s_center* and *q_center* we compute a "unit" vector along the line.
        # We enter holes that do not over lap with the larger holes.  We wind up skipping
        # one hole in 3:
        right_s_center: P = lower_holes_table["S"]
        left_s_center: P = right_s_center.y_mirror()
        right_q_center: P = lower_holes_table["Q"]
        left_q_center: P = right_q_center.y_mirror()
        right_hole_vector: P = right_q_center - right_s_center
        left_hole_vector: P = left_q_center - left_s_center
        for vector_hole_index in range(9):
            if vector_hole_index % 3 != 1:
                # Do the hole on the right first:
                right_hole_center: P = (right_s_center +
                                        (vector_hole_index - 1) * right_hole_vector / 3.0)
                right_hole_polygon: Polygon = Polygon(f"Right Vector Hole {vector_hole_index}")
                right_hole_polygon.circle(right_hole_center, small_hole_diameter, 8)
                complex_polygon.hole_append(right_hole_polygon)

                # Do the hole on the left second:
                left_hole_center: P = (left_s_center +
                                       (vector_hole_index - 1) * left_hole_vector / 3.0)
                left_hole_polygon: Polygon = Polygon(f"Left Vector Hole {vector_hole_index}")
                left_hole_polygon.circle(left_hole_center, small_hole_diameter, 8)
                complex_polygon.hole_append(left_hole_polygon)

        # There are arcs of holes and and rectangular slots along the upper and lower rims.
        # Since they are mirrored across the Y axis, we only worry about the right side.
        # The hole closest to the wheel is the "start" hole and the one farthest from the
        # wheel is the "end" hole.  We have to locate each of these holes:
        lower_start_diameter: float
        lower_start_center: P
        lower_start_diameter, lower_start_center = romi.hole_locate(-1.483063, -1.357508,
                                                                    1.348929, 1.223803)
        lower_arc_start_angle: float = atan2(lower_start_center.y, lower_start_center.x)
        if debugging:
            print(f"lower_start_diameter={lower_start_diameter}")
            print(f"lower_start_center={lower_start_center}")
            print(f"lower_start_angle={degrees(lower_arc_start_angle)}deg")

        lower_end_diameter: float
        lower_end_center: P
        lower_end_diameter, lower_end_center = romi.hole_locate(-3.229035, -3.354591,
                                                                0.10461, -0.020516)
        lower_arc_end_angle: float = atan2(lower_end_center.y, lower_end_center.x)
        if debugging:
            print(f"lower_start_diameter={lower_start_diameter}")
            print(f"lower_start_center={lower_start_center}")

        # Compute the *lower_arc_radius*:
        lower_hole_radius: float = origin.distance(lower_start_center)

        # There are two sizes of rectangle -- small and large.  The width appears to
        # be the same for both, so we only need *rectangle_width*, *small_rectangle_length*
        # and *large_rectangle_length*.  Lastly, we need to find one *rectangle_center*
        # so we can determine the *rectangle_radius* from the *origin*:
        large_upper_left_corner: P = P(-1.248201, 1.259484)
        large_lower_left_corner: P = P(1.136248, -1.33137)
        large_upper_right_corner: P = P(-1.205772, 1.230858)
        large_rectangle_length: float = large_upper_left_corner.distance(large_lower_left_corner)
        rectangle_width: float = large_upper_left_corner.distance(large_upper_right_corner)
        rectangle_center: P = (large_upper_right_corner + large_lower_left_corner) / 2.0
        rectangle_radius: float = origin.distance(rectangle_center)
        small_upper_left_corner: P = P(-1.368228, 1.081638)
        small_lower_left_corner: P = P(-1.431575, 0.987760)
        small_rectangle_length: float = small_upper_left_corner.distance(small_lower_left_corner)

        # There *lower_holes_count* holes to create along the arc:
        lower_holes_count: int = 13
        delta_angle: float = (lower_arc_end_angle - lower_arc_start_angle) / (lower_holes_count - 1)
        lower_hole_index: int
        for lower_hole_index in range(lower_holes_count):
            # The same *lower_arc_hole_diameter* is used for both the left and right arc holes:
            lower_arc_hole_diameter: float = (lower_start_diameter if lower_hole_index % 3 == 0
                                              else small_hole_diameter)

            # Likewise the *lower_rectangle_length* is used both the left and right rectangle arcs:
            lower_rectangle_length: float = (large_rectangle_length if lower_hole_index % 3 == 0
                                             else small_rectangle_length)

            # Do the *lower_right_hole* first:
            lower_hole_angle: float = lower_arc_start_angle + float(lower_hole_index) * delta_angle
            lower_right_hole_x: float = lower_hole_radius * cos(lower_hole_angle)
            lower_right_hole_y: float = lower_hole_radius * sin(lower_hole_angle)
            lower_right_hole_center = P(lower_right_hole_x, lower_right_hole_y)
            lower_right_hole: Polygon = Polygon(f"Lower Right hole {lower_hole_index}")
            lower_right_hole.circle(lower_right_hole_center, lower_arc_hole_diameter, 8)
            complex_polygon.hole_append(lower_right_hole)

            # Next do the *lower_left_hole*:
            lower_left_hole_center: P = lower_right_hole_center.y_mirror()
            lower_left_hole: Polygon = Polygon(f"Lower left hole {lower_hole_index}")
            lower_left_hole.circle(lower_left_hole_center, lower_arc_hole_diameter, 8)
            complex_polygon.hole_append(lower_left_hole)

            # Next do the *lower_right_rectangle*:
            lower_right_rectangle_x: float = rectangle_radius * cos(lower_hole_angle)
            lower_right_rectangle_y: float = rectangle_radius * sin(lower_hole_angle)
            lower_right_rectangle_center: P = P(lower_right_rectangle_x, lower_right_rectangle_y)
            lower_right_rectangle: Polygon = Polygon(f"Lower left rectangle {lower_hole_index}")
            lower_right_rectangle.rotated_rectangle(lower_right_rectangle_center, rectangle_width,
                                                    lower_rectangle_length, lower_hole_angle)
            complex_polygon.hole_append(lower_right_rectangle)

    def output(self):
        """TODO."""
        romi: Romi = self
        complex_polygon: ComplexPolygon = romi.complex_polygon
        # Now we can output the `romi.scad` file:
        scad_lines: List[str] = list()
        complex_polygon.scad_lines_append(scad_lines, "")
        scad_lines.append("")
        scad_text: str = '\n'.join(scad_lines)
        # print(scad_text)
        scad_file: IO[Any]
        with open("romi.scad", "w") as scad_file:
            scad_file.write(scad_text)

    def hex_pattern(self, pattern_rows: Tuple[str, ...], slot_pairs: List[str],
                    hex_origin: P, hole_diameter: float, slot_width: float, slot_length: float,
                    half_hex_dx_pitch: float, hex_dy_pitch: float) -> Dict[str, P]:
        """TODO."""
        romi: Romi = self
        complex_polygon: ComplexPolygon = romi.complex_polygon

        # Set *debugging* to *True* to print out some debugging information:
        debugging: bool = True
        if debugging:
            print("-------------------")
            print(f"hex_origin={hex_origin}")
            print(f"hole_diameter={hole_diameter}")
            print(f"slot_width={slot_width}")
            print(f"slot_length={slot_length}")
            print(f"half_hex_dx_pitch={half_hex_dx_pitch}")
            print(f"hex_dy_pitch={hex_dy_pitch}")

        # *locations* contains the end-point locations for the hex pattern:
        hex_origin_x: float = hex_origin.x
        hex_origin_y: float = hex_origin.y
        points_count: int = 8
        locations: Dict[str, P] = dict()
        pattern_index: int
        pattern_row: str
        for y_index, pattern_row in enumerate(pattern_rows):
            y = hex_origin_y - (y_index * hex_dy_pitch)
            pattern_character: str
            for x_index, pattern_character in enumerate(pattern_row):
                if pattern_character != '-':
                    # Enter *left_hole_center* into *locations* keyed by *pattern_character*:
                    x = hex_origin_x + (x_index * half_hex_dx_pitch)
                    right_hole_center: P = P(x, y)
                    locations[pattern_character] = right_hole_center

                    # Only create holes when *pattern_character* is upper case:
                    if pattern_character.isupper():
                        # Put in the *right_hole*:
                        right_hole: Polygon = Polygon(f"Right Hex Hole ({x_index}, {y_index})")
                        right_hole.circle(right_hole_center, hole_diameter, points_count)
                        complex_polygon.hole_append(right_hole)

                        # Put in the mirrored *left_hole*:
                        left_hole: Polygon = Polygon(f"Left Hex Hole ({x_index}, {y_index})")
                        left_hole_center: P = P(-x, y)
                        left_hole.circle(left_hole_center, hole_diameter, points_count)
                        complex_polygon.hole_append(left_hole)

        # Now sweep through *slot_pairs* and install all of the slots:
        slot_pair: str
        for slot_pair in slot_pairs:
            # Do the right slots:
            hole1: P = locations[slot_pair[0]]
            hole2: P = locations[slot_pair[1]]
            right_slot: Polygon = Polygon(f"Right Slot '{slot_pair}'")
            right_slot.slot(hole1, hole2, slot_length, slot_width, points_count)
            complex_polygon.hole_append(right_slot)

            # Mirror the left left slots:
            mirror_hole1: P = P(-hole1.x, hole1.y)
            mirror_hole2: P = P(-hole2.x, hole2.y)
            left_slot: Polygon = Polygon(f"Left Slot '{slot_pair}'")
            left_slot.slot(mirror_hole1, mirror_hole2, slot_length, slot_width, points_count)
            complex_polygon.hole_append(left_slot)

        # Return the *locations* table for further hole drilling:
        return locations


def main() -> int:
    """Generate the openscand file."""
    print("romi_model.main() called")
    romi: Romi = Romi()
    romi.generate()
    romi.output()
    return 0
