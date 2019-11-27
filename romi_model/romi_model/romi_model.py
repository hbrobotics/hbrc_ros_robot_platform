# <--------------------------------------- 100 characters ---------------------------------------> #

"""Code to genarate and openscad model of the Pololu Romi Base."""

from typing import Any, Dict, IO, List, Tuple
from math import asin, atan2, ceil, cos, pi, sin, sqrt


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


class ComplexPolygon:
    """Represent a complex polygon with holes in it."""

    def __init__(self, name: str, outer_polygon: "Polygon") -> None:
        """Initialize a complex polygon with an outer Polygon."""
        self.holes: List[Polygon] = list()
        self.name: str = name
        self.outer_polygon: Polygon = outer_polygon

    def dxf_hole_append(self, dxf_name: str,
                        dxf_x: float, dxf_y: float, dxf_circumference: float) -> None:
        """Take a hole from `.dxf` file and add it."""
        complex_polygon: ComplexPolygon = self
        dxf_y_origin_above: float = 2.967165
        dxf_y_origin_below: float = 2.908110
        dxf_y_origin: float = (dxf_y_origin_above + dxf_y_origin_below) / 2.0

        dxf_x_origin: float = -3.899354
        inches2mm: float = 25.4
        x: float = (dxf_x - dxf_x_origin) * inches2mm
        y: float = (dxf_y - dxf_y_origin) * inches2mm
        circumference: float = dxf_circumference * inches2mm
        diameter: float = circumference / pi
        radius: float = diameter / 2.0
        hole: Polygon = Polygon(dxf_name)
        hole.circle(P(x, y), radius, 8)
        complex_polygon.hole_append(hole)

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

    def circle(self, origin: P, radius: float, points_count):
        """Append a circle to Polygon."""
        polygon: Polygon = self
        delta_radian: float = (2 * pi) / points_count
        index: int
        for index in range(points_count):
            radian: float = index * delta_radian
            x: float = origin.x + radius * cos(radian)
            y: float = origin.y + radius * sin(radian)
            polygon.append(P(x, y))

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
             slot_length: float, slot_radius: float, points_count: int) -> None:
        """TODO."""
        polygon: Polygon = self
        degrees90: float = pi / 2.0
        degrees180: float = pi
        center: P = (hole1 + hole2) / 2.0
        slot_angle: float = atan2(hole1.y - hole2.y, hole1.x - hole2.x)
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

    def generate(self) -> None:
        """TODO."""
        romi: Romi = self
        complex_polygon: ComplexPolygon = romi.complex_polygon

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

        # Perform any requested *debuggin*:
        if debugging:
            radians2degrees: float = 180.0 / pi
            print(f"wheel_well_angle={wheel_well_angle}={wheel_well_angle * radians2degrees}")
            wheel_well_x: float = radius * cos(wheel_well_angle)
            wheel_well_y: float = radius * sin(wheel_well_angle)
            print(f"wheel_well_x={wheel_well_x}")
            print(f"wheel_well_y={wheel_well_y}")
            origin: P = P(0.0, 0.0)
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
        # orientation is used.  Sigh.  Also, the `.dxf` file seems to be in untis of
        # inches rather than millimeters.  The rest of this code uses millimeters, so
        # we appropiately multiply dxf coordiate by *inches2mm*:
        inches2mm: float = 25.4

        # It is pretty clear that the motor axles are intended to go through the center
        # of the Romi platform.  Alas, the `.dxf` file origin is to lower right.  We
        # need to figure out *dxf_y_origin*.  This is done by selecting the left axle
        # in the `.dxf` file and extracting the above/below values.  These are averaged
        # to get the `.dxf` file value that corresponds to the Romi platform center.
        # Each of the line segments is a 4-point spline, but they all have the same Y
        # coordinate:
        dxf_above_axel_y: float = 2.967165
        dxf_below_axel_y: float = 2.908110
        dxf_y_origin: float = (dxf_above_axel_y + dxf_below_axel_y) / 2
        # y_origin: float = dxf_y_origin * inches2mm

        # From the "Pololu Romi Chassis User's Guide" one of the battery holder holes
        # is referenced.  We need to locate this hole using the `.dxf` file coordinates
        # which appears to use units of inches.  This hole is represented as a 64 point
        # spline for some reason.  Loading the `.dxf` file using the `qcad` program, it
        # is possible to cycle through all of the spline points and get the mininum/maximum
        # X/Y coordinates as follows:
        dxf_reference_hole_left: float = -3.913146
        dxf_reference_hole_right: float = -3.822591
        dxf_reference_hole_above: float = 3.376610
        dxf_reference_hole_below: float = 3.286051
        dxf_x_origin: float = (dxf_reference_hole_right + dxf_reference_hole_left) / 2.0

        # The dxf reference hole X/Y/Diameter are computed as follows:
        # dxf_reference_hole_x: float = (dxf_reference_hole_right + dxf_reference_hole_left) / 2.0
        dxf_reference_hole_y: float = (dxf_reference_hole_above + dxf_reference_hole_below) / 2.0
        dxf_reference_hole_dx: float = dxf_reference_hole_right - dxf_reference_hole_left
        dxf_reference_hole_dy: float = dxf_reference_hole_above - dxf_reference_hole_below
        dxf_reference_hole_diameter: float = (dxf_reference_hole_dx + dxf_reference_hole_dy) / 2.0

        # Now convert various values from inches to millimeters:
        reference_hole_y: float = (dxf_reference_hole_y - dxf_y_origin) * inches2mm
        reference_hole_diameter: float = dxf_reference_hole_diameter * inches2mm

        # Perform any requested *debugging*:
        if debugging:
            print(f"dxf_x_origin={dxf_x_origin}")
            print(f"dxf_y_origin={dxf_y_origin}")
            print(f"dxf_reference_hole_y={dxf_reference_hole_y}")
            print(f"dxf_reference_hole_dx={dxf_reference_hole_dx}")
            print(f"dxf_reference_hole_dy={dxf_reference_hole_dy}")
            print(f"dxf_reference_hole_diameter={dxf_reference_hole_diameter}")
            print(f"reference_hole_y={reference_hole_y}")
            print(f"reference_hole_diameter={reference_hole_diameter}")

        # The battery holes between motors is organized in 3 rows by 9 columns.  Not all holes
        # are poplulated.  We create append holes for each appropriate location:
        lower_row_ys: Tuple[float, ...] = (
            reference_hole_y,
            reference_hole_y - 12.3,
            reference_hole_y - 12.3 - 12.3
        )
        lower_battery_rows: Tuple[str, ...] = (
            "--*****--",  # Row with reference hole
            "--*****--",  # Row below reference hole
            "*-*****-*")  # Two rows below referene hole
        hole_dx_pitch: float = 10
        column0_x: float = -4.0 * hole_dx_pitch
        x_index: int
        for x_index in range(9):
            x: float = column0_x + x_index * hole_dx_pitch
            y_index: int
            lower_battery_row: str
            for y_index, lower_battery_row in enumerate(lower_battery_rows):
                if lower_battery_row[x_index] == '*':
                    # We need a hole:
                    y: float = lower_row_ys[y_index]
                    hole: Polygon = Polygon(f"Lower Battery Hole ({2-x_index}, {y_index})")
                    hole_center: P = P(x, y)
                    hole.circle(hole_center, reference_hole_diameter / 2.0, 8)
                    complex_polygon.hole_append(hole)

        # The battery holes above the motors is organized as 3 rows by 9 columns.  All holes
        # are populated:
        upper_row_ys: Tuple[float, ...] = (
            reference_hole_y + 7.0,
            reference_hole_y + 7.0 + 12.3,
            reference_hole_y + 7.0 + 12.3 + 12.3
        )
        column0_x = -4.5 * hole_dx_pitch
        for x_index in range(10):
            x = column0_x + x_index * hole_dx_pitch
            for y_index in range(3):
                y = upper_row_ys[y_index]
                hole = Polygon(f"Upper Battery Hole ({2-x_index}, {y_index})")
                hole.circle(P(x, y), reference_hole_diameter / 2.0, 8)
                complex_polygon.hole_append(hole)

        # One of the vertical slots on the `.dxf` has the following bounding box in inches:
        vertical_slot_left_x: float = -2.437146
        vertical_slot_right_x: float = -2.346591
        vertical_slot_top_y: float = 1.339205
        vertical_slot_bottom_y: float = 1.051803
        vertical_slot_dx: float = vertical_slot_right_x - vertical_slot_left_x
        vertical_slot_dy: float = vertical_slot_top_y - vertical_slot_bottom_y
        # *slot_length* is from center to center, *NOT* edge to edge:
        slot_length: float = (vertical_slot_dy - vertical_slot_dx) * inches2mm
        large_diameter: float = vertical_slot_dx * inches2mm
        # large_diameter = 0.5

        # The hexagonal slots and hole pattern is present on both the top and the
        # bottom of the platform.  The "User's Guide" implies that holes are space
        # by 7.5mm vertically.  The patterns are mirrored
        # across the Y axis, so we can just replicate holes across the Y axis.
        #
        # The math for equilateral triagngles is:
        #     b = equilateral triangle base width
        #     h = equalateral triangle height
        #     h = b*sqrt(3)/2
        #     b = 2*h/sqrt(3)
        x_origin: float = 0.0
        hex_dy_pitch: float = 7.50
        hex_dx_pitch: float = hex_dy_pitch / sqrt(3.0)  # We only want b/2 along X axis
        hex_dx_pitch += 0.0
        hex_x_origin: float = x_origin + 37.5
        hex_y_origin: float = reference_hole_y - 46.8
        # Using the `.dxf` image, the pattern below represent the locations of the hex pattern
        # holes in the lower left quadrant.  'O' is at (*hex_x_origin*, *hex_y_origin*).
        # Upper case letters indicate the location of a hole.  Lower case letters indicate
        # the end-point of a slot.  There is a weird little half slot above 'O' that is
        # not currently modeled:
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

        lower_hex_x_origin: float = hex_x_origin - 6 * hex_dx_pitch
        lower_hex_y_origin: float = hex_y_origin + 1 * hex_dy_pitch
        lower_holes_table: Dict[str, P] = romi.hex_pattern(lower_pattern_rows, lower_slot_pairs,
                                                           lower_hex_x_origin, lower_hex_y_origin,
                                                           large_diameter, slot_length,
                                                           hex_dx_pitch, hex_dy_pitch)
        lower_holes_table[""] = P(0.0, 0.0)

        upper_pattern_rows: Tuple[str, ...] = (
            "a------",
            "-A-B---",
            "b-C-D-c",
            "---d---",
        )
        upper_slot_pairs: List[str] = "aB:bB:dB:cB".split(':')

        right_upper_hex_hole_left: float = -2.964937
        right_upper_hex_hole_right: float = -3.090492
        right_upper_hex_hole_x: float = (right_upper_hex_hole_right +
                                         right_upper_hex_hole_left) / 2.0
        right_upper_hex_hole_above: float = 5.441567
        right_upper_hex_hole_below: float = 5.316441
        right_upper_hex_hole_y: float = (right_upper_hex_hole_above +
                                         right_upper_hex_hole_below) / 2.0

        upper_hex_x_origin: float = (right_upper_hex_hole_x - dxf_x_origin) * inches2mm
        upper_hex_y_origin: float = (right_upper_hex_hole_y -
                                     dxf_y_origin) * inches2mm + hex_dy_pitch
        print(f"upper_hex_x_origin={upper_hex_x_origin}")
        print(f"upper_hex_y_origin={upper_hex_y_origin}")
        upper_holes_table: Dict[str, P] = romi.hex_pattern(upper_pattern_rows, upper_slot_pairs,
                                                           upper_hex_x_origin, upper_hex_y_origin,
                                                           large_diameter, slot_length,
                                                           hex_dx_pitch, hex_dy_pitch)
        upper_holes_table[""] = P(0.0, 0.0)


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
                    hex_x_origin: float, hex_y_origin, diameter: float, slot_length: float,
                    hex_dx_pitch: float, hex_dy_pitch: float) -> Dict[str, P]:
        """TODO."""
        romi: Romi = self
        complex_polygon: ComplexPolygon = romi.complex_polygon
        radius: float = diameter / 2.0

        # *locations* contains the end-point locations for the hex pattern:
        points_count: int = 8
        locations: Dict[str, P] = dict()
        pattern_index: int
        pattern_row: str
        for y_index, pattern_row in enumerate(pattern_rows):
            y = hex_y_origin - (y_index * hex_dy_pitch)
            pattern_character: str
            for x_index, pattern_character in enumerate(pattern_row):
                if pattern_character != '-':
                    # Enter *left_hole_center* into *locations* keyed by *pattern_character*:
                    x = hex_x_origin + (x_index * hex_dx_pitch)
                    right_hole_center: P = P(x, y)
                    locations[pattern_character] = right_hole_center

                    # Put in the *right_hole*:
                    right_hole: Polygon = Polygon(f"Right Hex Hole ({x_index}, {y_index})")
                    right_hole.circle(right_hole_center, radius, points_count)
                    complex_polygon.hole_append(right_hole)

                    # Put in the mirrored *left_hole*:
                    left_hole: Polygon = Polygon(f"Left Hex Hole ({x_index}, {y_index})")
                    left_hole_center: P = P(-x, y)
                    left_hole.circle(left_hole_center, radius, points_count)
                    complex_polygon.hole_append(left_hole)

        # Now sweep through *slot_pairs* and install all of the slots:
        slot_pair: str
        for slot_pair in slot_pairs:
            # Do the right slots:
            hole1: P = locations[slot_pair[0]]
            hole2: P = locations[slot_pair[1]]
            right_slot: Polygon = Polygon(f"Right Slot '{slot_pair}'")
            right_slot.slot(hole1, hole2, slot_length, radius, points_count)
            complex_polygon.hole_append(right_slot)

            # Mirror the left left slots:
            mirror_hole1: P = P(-hole1.x, hole1.y)
            mirror_hole2: P = P(-hole2.x, hole2.y)
            left_slot: Polygon = Polygon(f"Left Slot '{slot_pair}'")
            left_slot.slot(mirror_hole1, mirror_hole2, slot_length, radius, points_count)
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
