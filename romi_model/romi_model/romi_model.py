# <--------------------------------------- 100 characters ---------------------------------------> #

"""Code to genarate and openscad model of the Pololu Romi Base."""

from typing import Any, IO, List, Tuple
from math import asin, ceil, cos, pi, sin, sqrt


class Points:
    """TODO."""

    def __init__(self):
        """TODO."""
        self.points: List[P] = list()


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

    def __sub__(self, point2: "P") -> "P":
        """Subtract two points from one another."""
        point1: P = self
        return P(point1.x - point2.x, point1.y - point2.y)

    def __mul__(self, scale: float) -> "P":
        """Multiply a point by a scale factor."""
        point: P = self
        return P(point.x * scale, point.y * scale)

    def __rmul__(self, scale: float) -> "P":
        """Multiply a point by a scale factor."""
        point: P = self
        return P(point.x * scale, point.y * scale)

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


class Romi:
    """Model the Polol Romi platform in OpenSCAD."""

    def __init__(self) -> None:
        """Initialize and create the Romi Platform."""
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
        outer_polygon: Polygon = Polygon("Base Perimeter")

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

        # Now create the *romi* *ComplexPolygon* and initialize with *outer_polygon*:
        romi: ComplexPolygon = ComplexPolygon("Base", outer_polygon)

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

        # The dxf reference hole X/Y/Diameter are computed as follows:
        # dxf_reference_hole_x: float = (dxf_reference_hole_right + dxf_reference_hole_left) / 2.0
        dxf_reference_hole_y: float = (dxf_reference_hole_above + dxf_reference_hole_below) / 2.0
        dxf_reference_hole_dx: float = dxf_reference_hole_right - dxf_reference_hole_left
        dxf_reference_hole_dy: float = dxf_reference_hole_above - dxf_reference_hole_below
        dxf_reference_hole_diameter: float = (dxf_reference_hole_dx + dxf_reference_hole_dy) / 2.0
        # dxf_x_origin: float = dxf_reference_hole_x

        # Now convert various values from inches to millimeters:
        reference_hole_y: float = (dxf_reference_hole_y - dxf_y_origin) * inches2mm
        reference_hole_diameter: float = dxf_reference_hole_diameter * inches2mm

        # Perform any requested *debugging*:
        if debugging:
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
        lower_row_pattern: Tuple[str, ...] = (
            "--*****--",  # Row with reference hole
            "--*****--",  # Row below reference hole
            "*-*****-*")  # Two rows below referene hole
        hole_dx_pitch: float = 10
        column0_x: float = -4.0 * hole_dx_pitch
        x_index: int
        for x_index in range(9):
            x: float = column0_x + x_index * hole_dx_pitch
            y_index: int
            for y_index in range(3):
                if lower_row_pattern[y_index][x_index] == '*':
                    # We need a hole:
                    y: float = lower_row_ys[y_index]
                    hole: Polygon = Polygon(f"Lower Battery Hole ({2-x_index}, {y_index})")
                    hole.circle(P(x, y), reference_hole_diameter / 2.0, 8)
                    romi.hole_append(hole)

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
                romi.hole_append(hole)

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
        patterns: Tuple[str, ...] = (
            "*-*------",  # [0]
            "-*-*-----",  # [1]
            "--*-*-*--",  # [2]
            "---*-*-*-",  # [3]
            "--*-O-*-*",  # [4]
            "-*-*-*-*-"   # [5]
        )
        # patterns = ("--*-O-*-*",)
        pattern_index: int
        pattern: str
        for pattern_index, pattern in enumerate(patterns):
            y = hex_y_origin + (pattern_index - 4) * hex_dy_pitch
            # y = hex_y_origin
            character: str
            for x_index, character in enumerate(pattern):
                if character != '-':
                    x = hex_x_origin + (x_index - 4) * hex_dx_pitch
                    # x = hex_x_origin
                    hole = Polygon(f"Right Hex Hole ({x_index}, {pattern_index})")
                    hole.circle(P(x, y), 1.5, 8)
                    romi.hole_append(hole)
                    hole = Polygon(f"Left Hex Hole ({x_index}, {pattern_index})")
                    hole.circle(P(-x, y), 1.5, 8)
                    romi.hole_append(hole)

        # base_complex_polygon.dxf_hole_append("Bat1a", -5.442673, 2.321484, 0.282635)
        # base_complex_polygon.dxf_hole_append("Bat1b", -4.655272, 2.321484, 0.282641)
        # base_complex_polygon.dxf_hole_append("Bat1c", -4.261563, 2.321484, 0.282641)
        # base_complex_polygon.dxf_hole_append("Bat1d", -3.867870, 2.321484, 0.282641)
        # base_complex_polygon.dxf_hole_append("Bat1e", -3.474161, 2.321484, 0.282641)
        # base_complex_polygon.dxf_hole_append("Bat1f", -3.080465, 2.321484, 0.282649)
        # base_complex_polygon.dxf_hole_append("Bat1g", -2.293063, 2.321484, 0.282652)

        # base_complex_polygon.dxf_hole_append("Bat2a", -4.655272, 2.803776, 0.282641)
        # base_complex_polygon.dxf_hole_append("Bat2b", -4.261563, 2.803776, 0.282641)
        # base_complex_polygon.dxf_hole_append("Bat2c", -3.867870, 2.803776, 0.282641)
        # base_complex_polygon.dxf_hole_append("Bat2d", -3.474161, 2.803776, 0.282641)
        # base_complex_polygon.dxf_hole_append("Bat2e", -3.080465, 2.803776, 0.282649)

        # base_complex_polygon.dxf_hole_append("Bat3a", -4.655272, 3.286051, 0.282656)
        # base_complex_polygon.dxf_hole_append("Bat3b", -4.261563, 3.286051, 0.282656)
        # base_complex_polygon.dxf_hole_append("Bat3c", -3.867870, 3.286051, 0.282656)
        # base_complex_polygon.dxf_hole_append("Bat3d", -3.474161, 3.286051, 0.282656)b
        # base_complex_polygon.dxf_hole_append("Bat3e", -3.080465, 3.286051, 0.282664)

        # base_complex_polygon.dxf_hole_append("Bat4a", -5.639520, 3.561650, 0.282643)
        # base_complex_polygon.dxf_hole_append("Bat4b", -5.245811, 3.561650, 0.282632)
        # base_complex_polygon.dxf_hole_append("Bat4c", -4.852118, 3.561650, 0.282649)
        # base_complex_polygon.dxf_hole_append("Bat4d", -4.458409, 3.561650, 0.282632)
        # base_complex_polygon.dxf_hole_append("Bat4e", -4.064717, 3.561650, 0.282640)
        # base_complex_polygon.dxf_hole_append("Bat4f", -3.671020, 3.561650, 0.282635)
        # base_complex_polygon.dxf_hole_append("Bat4g", -3.277311, 3.561650, 0.282645)
        # base_complex_polygon.dxf_hole_append("Bat4h", -2.883618, 3.561650, 0.282632)
        # base_complex_polygon.dxf_hole_append("Bat4i", -2.489909, 3.561650, 0.282647)
        # base_complex_polygon.dxf_hole_append("Bat4g", -2.096217, 3.561650, 0.282637)

        # base_complex_polygon.dxf_hole_append("Bat5a", -5.639520, 4.043929, 0.282657)
        # base_complex_polygon.dxf_hole_append("Bat5b", -5.245811, 4.043929, 0.282645)
        # base_complex_polygon.dxf_hole_append("Bat5c", -4.832437, 4.043929, 0.332260)
        # base_complex_polygon.dxf_hole_append("Bat5d", -4.458409, 4.043929, 0.282645)
        # base_complex_polygon.dxf_hole_append("Bat5e", -4.064717, 4.043929, 0.282653)
        # base_complex_polygon.dxf_hole_append("Bat5f", -3.671020, 4.043929, 0.282648)
        # base_complex_polygon.dxf_hole_append("Bat5g", -3.277311, 4.043929, 0.282658)
        # base_complex_polygon.dxf_hole_append("Bat5h", -2.883618, 4.036051, 0.332260)
        # base_complex_polygon.dxf_hole_append("Bat5i", -2.489909, 4.043929, 0.282661)
        # base_complex_polygon.dxf_hole_append("Bat5j", -2.096217, 4.043929, 0.282650)

        # base_complex_polygon.dxf_hole_append("Bat6a", -5.639520, 4.526220, 0.282647)
        # base_complex_polygon.dxf_hole_append("Bat6b", -5.245811, 4.526220, 0.282635)
        # base_complex_polygon.dxf_hole_append("Bat6c", -4.852118, 4.526220, 0.282653)
        # base_complex_polygon.dxf_hole_append("Bat6d", -4.458409, 4.526220, 0.282636)
        # base_complex_polygon.dxf_hole_append("Bat6e", -4.064717, 4.526220, 0.282643)
        # base_complex_polygon.dxf_hole_append("Bat6f", -3.671020, 4.526220, 0.282639)
        # base_complex_polygon.dxf_hole_append("Bat6g", -3.277311, 4.526220, 0.282648)
        # base_complex_polygon.dxf_hole_append("Bat6h", -2.883618, 4.526220, 0.282635)
        # base_complex_polygon.dxf_hole_append("Bat6i", -2.489909, 4.526220, 0.282651)
        # base_complex_polygon.dxf_hole_append("Bat6j", -2.096217, 4.526220, 0.282640)

        # base_complex_polygon.dxf_hole_append("Up1a", -6.315453, 4.526346, 0.393990)
        # base_complex_polygon.dxf_hole_append("Up1b", -1.420283, 4.526346, 0.394003)

        # base_complex_polygon.dxf_hole_append("Up2a", -6.205327, 4.707232, 0.282641)
        # base_complex_polygon.dxf_hole_append("Up2b", -1.530398, 4.707232, 0.282645)

        # base_complex_polygon.dxf_hole_append("Up3a", -5.219453, 5.146287, 0.393981)
        # base_complex_polygon.dxf_hole_append("Up3b", -4.878492, 5.146287, 0.393989)
        # base_complex_polygon.dxf_hole_append("Up3c", -2.857244, 5.146287, 0.393995)
        # base_complex_polygon.dxf_hole_append("Up3d", -2.516283, 5.146287, 0.393997)

        # base_complex_polygon.dxf_hole_append("Up4a", -5.816591, 5.119469, 0.282634)
        # base_complex_polygon.dxf_hole_append("Up4b", -1.919146, 5.119469, 0.282647)

        # base_complex_polygon.dxf_hole_append("Up5a", -5.672008, 5.238122, 0.282637)
        # base_complex_polygon.dxf_hole_append("Up5b", -2.063728, 5.238122, 0.282633)

        # base_complex_polygon.dxf_hole_append("Up6a", -5.508327, 5.329760, 0.394006)
        # base_complex_polygon.dxf_hole_append("Up6b", -2.227398, 5.329760, 0.393990)

        # base_complex_polygon.dxf_hole_append("Up7a", -5.048965, 5.441567, 0.393991)
        # base_complex_polygon.dxf_hole_append("Up7b", -4.708008, 5.441567, 0.393975)
        # base_complex_polygon.dxf_hole_append("Up7c", -3.027717, 5.441567, 0.393987)
        # base_complex_polygon.dxf_hole_append("Up7d", -2.686756, 5.441567, 0.393979)

        # base_complex_polygon.dxf_hole_append("Up8a", -5.344661, 5.456846, 0.282649)
        # base_complex_polygon.dxf_hole_append("Up8b", -2.391075, 5.456846, 0.282656)

        # base_complex_polygon.dxf_hole_append("Up9a", -5.179701, 5.545012, 0.282654)
        # base_complex_polygon.dxf_hole_append("Up9b", -2.556035, 5.545012, 0.282652)

        # base_complex_polygon.dxf_hole_append("Up10a", -4.997839, 5.602638, 0.393987)
        # base_complex_polygon.dxf_hole_append("Up10b", -2.737898, 5.602638, 0.394002)

        # base_complex_polygon.dxf_hole_append("Up11a", -4.815965, 5.695677, 0.282650)
        # base_complex_polygon.dxf_hole_append("Up11b", -2.919756, 5.695677, 0.282650)

        # base_complex_polygon.dxf_hole_append("Up12a", -4.636992, 5.749969, 0.282642)
        # base_complex_polygon.dxf_hole_append("Up12b", -3.098744, 5.749969, 0.282661)

        scad_lines: List[str] = list()
        romi.scad_lines_append(scad_lines, "")
        scad_lines.append("")
        scad_text: str = '\n'.join(scad_lines)
        # print(scad_text)
        scad_file: IO[Any]
        with open("romi.scad", "w") as scad_file:
            scad_file.write(scad_text)


def main() -> int:
    """Generate the openscand file."""
    print("romi_model.main() called")
    Romi()
    return 0
