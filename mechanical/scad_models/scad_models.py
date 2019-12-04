# <--------------------------------------- 100 characters ---------------------------------------> #

"""Code to genarate and openscad model of the Pololu Romi Base."""

# Pumpkin Pi: 104mm x 70mm.  Holes are the same as the Raspberry Pi with the upper left
# hole in the upper left corner of the PCB.  The extra PCB space goes to the right and
# down on the Pumpkin Pi.

# http://docplayer.net/42910792-
# Hardware-assisted-tracing-on-arm-with-coresight-and-opencsd-mathieu-poirier.html
from scad_models.scad import P, Polygon, Scad3DUnion, ScadLinearExtrude, ScadPolygon
from typing import Any, Dict, IO, List, Tuple
from math import asin, atan2, cos, degrees, nan, pi, sin, sqrt


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
        origin_offset = P(x_origin_offset, y_origin_offset)
        self.debugging = debugging
        self.inches2mm: float = inches2mm
        self.origin_offset: P = origin_offset
        if debugging:  # pragma: no cover
            print(f"origin_offset={origin_offset}")

    # Romi.base_outline_polygon_get():
    def base_outline_polygon_get(self) -> Polygon:
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
        wheel_well_corner: P = P(wheel_well_x, wheel_well_y)

        # Perform any requested *debugging*:
        if debugging:  # pragma: no cover
            print(f"wheel_well_angle={wheel_well_angle}={degrees(wheel_well_angle)}deg")
            print(f"wheel_well_corner={wheel_well_corner}")

        # Verify that the *distance* from the *origin* to the *wheel_well_corner* matches *radius*:
        origin: P = P(0.0, 0.0)
        wheel_well_radius: float = origin.distance(wheel_well_corner)
        assert abs(radius - wheel_well_radius) < .00001, "Something is not right"

        # Now we can draw the *outline_polygon* of the Romi platform.  It conists of two arcs
        # with some straight line segments to form the wheel well.  Start by creating
        # an empty *outline_polygon*:
        outline_polygon: Polygon = Polygon("Romi Base Exterior")

        # Create the upper arc:
        upper_start_angle: float = wheel_well_angle
        upper_end_angle: float = pi - wheel_well_angle
        arc_count = 21
        outline_polygon.arc_append(origin, radius, upper_start_angle, upper_end_angle, arc_count)

        # Create the left wheel well:
        outline_polygon.point_append(P(-half_wheel_well_dx, half_wheel_well_dy))
        outline_polygon.point_append(P(-half_wheel_well_dx, -half_wheel_well_dy))

        # Create the lower arc:
        lower_start_angle: float = wheel_well_angle + pi
        lower_end_angle: float = upper_end_angle + pi
        outline_polygon.arc_append(origin, radius, lower_start_angle, lower_end_angle, arc_count)

        # Create the right wheel well:
        outline_polygon.point_append(P(half_wheel_well_dx, -half_wheel_well_dy))
        outline_polygon.point_append(P(half_wheel_well_dx, half_wheel_well_dy))
        assert len(outline_polygon) == 2 * arc_count + 4

        # *outline_polygon* is done and can be returned:
        return outline_polygon

    # Romi.base_scad_polygon_generate()
    def base_scad_polygon_generate(self) -> ScadPolygon:
        """TODO."""
        # Grabe some values from *romi* (i.e. *self*):
        romi: Romi = self
        debugging: bool = romi.debugging

        # Grab the *base_outline_polygon*:
        base_outline_polygon: Polygon = romi.base_outline_polygon_get()

        # Grab the *battery_polygons*:
        battery_polygons: List[Polygon] = romi.battery_polygons_get()

        # Grab the *upper_hex_polygons*:
        upper_hex_polygons: List[Polygon] = romi.upper_hex_polygons_get()
        if debugging:  # pragma: no cover
            print("************************")
            print(f"len(upper_hex_polygons)={len(upper_hex_polygons)}")

        # Grab the *lower_hex_polygons* and *lower_hex_table*:
        lower_hex_polygons: List[Polygon]
        lower_hex_table: Dict[str, P]
        lower_hex_polygons, lower_hex_table = romi.lower_hex_polygons_table_get()
        if debugging:  # pragma: no cover
            print(f"len(lower_hex_polygons)={len(lower_hex_polygons)}")

        line_hole_polygons: List[Polygon] = romi.line_hole_polygons_get(lower_hex_table)

        lower_arc_hole_rectangle_polygons: List[Polygon]
        lower_arc_hole_rectangle_polygons = romi.lower_arc_hole_rectangle_polygons_get()

        upper_arc_hole_rectangle_polygons: List[Polygon]
        upper_arc_hole_rectangle_polygons = romi.upper_arc_hole_rectangle_polygons_get()

        # Concatenate all of the polygons together into *all_polygons* with *base_outline_polygon*
        # being the required first *Polygon*:
        mirrorable_polygons: List[Polygon] = (upper_hex_polygons + lower_hex_polygons +
                                              line_hole_polygons +
                                              lower_arc_hole_rectangle_polygons +
                                              upper_arc_hole_rectangle_polygons)
        mirrorable_polygon: Polygon
        mirrored_polygons: List[Polygon] = [mirrorable_polygon.y_mirror()
                                            for mirrorable_polygon in mirrorable_polygons]
        all_polygons: List[Polygon] = ([base_outline_polygon] + battery_polygons +
                                       mirrorable_polygons + mirrored_polygons)

        if debugging:  # pragma: no cover
            print(f"len(all_polygons)={len(all_polygons)}")

        # Create the final *base_scad_polygon*, write it out to disk and return it.
        base_scad_polygon: ScadPolygon = ScadPolygon("Romi Base ScadPolygon", all_polygons)
        return base_scad_polygon

    # Romi.battery_polygons_get():
    def battery_polygons_get(self) -> List[Polygon]:
        """Return the holes for the Romi battery case."""
        # Grab some values from *romi* (i.e. *self*):
        romi: Romi = self
        # debugging: bool = romi.debugging
        inches2mm: float = romi.inches2mm
        origin_offset: P = romi.origin_offset

        # All of the battery holes are done relative to the *battery_reference_hole*
        # indicated on the drawing of the dimensions and mounting holes seciont of the
        # "Pololu Romi Chassis User's Guide".
        reference_hole_diameter: float
        reference_hole_center: P
        reference_hole_diameter, reference_hole_center = romi.hole_locate(-3.913146, -3.822591,
                                                                          3.376610, 3.286051)

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
        polygons: List[Polygon] = list()
        reference_hole_center_y: float = reference_hole_center.y
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
                    lower_hole_center: P = P(x, y)
                    lower_hole: Polygon = Polygon(f"Lower Battery Hole ({2-x_index}, {y_index})")
                    lower_hole.circle_append(lower_hole_center, reference_hole_diameter, 8)
                    polygons.append(lower_hole)

        # The upper battery holes above the lower battery motor holes are organized as
        # 3 rows by 9 columns where all positions are populated:
        upper_battery_y_offsets: Tuple[float, ...] = (7.0, 7.0 + 12.3, 7.0 + 12.3 + 12.3)
        column0_x = -4.5 * hole_dx_pitch
        for x_index in range(10):
            x = column0_x + x_index * hole_dx_pitch
            for y_index in range(3):
                y = reference_hole_center_y + upper_battery_y_offsets[y_index]
                upper_hole_center: P = P(x, y)
                upper_hole_polygon: Polygon = Polygon("Upper Battery Hole "
                                                      f"({2-x_index}, {y_index})")
                upper_hole_polygon.circle_append(upper_hole_center, reference_hole_diameter, 8)
                polygons.append(upper_hole_polygon)

        # There are 6 rectangular slots that have a nub on the end to cleverly indicate battery
        # polarity direction.  We will model these as simple rectangular slots and skip the
        # cleverly battery nubs.  There are 4 batteries slots at the top called
        # *upper_left_battery_slot*, *upper_right_battery_slot*, *lower_left_battery_slot*, and
        # *lower_righ_battery_slot*.  Underneath these 4 batteries are 2 more centered battery
        # slots that are called *upper_center_battery_slot* and *lower_center_battery_slot*:
        upper_left_battery_slot: Polygon = romi.rectangle_locate("Upper Left Battery Slot",
                                                                 -5.550937, 4.252594,
                                                                 -4.074563, 4.473067)
        upper_right_battery_slot: Polygon = romi.rectangle_locate("Upper Right Battery Slot",
                                                                  -3.621799, 4.252594,
                                                                  -2.145425, 4.473067)
        lower_left_battery_slot: Polygon = romi.rectangle_locate("Lower Left Battery Slot",
                                                                 -5.590311, 3.705346,
                                                                 -4.113925, 3.925815)
        lower_right_battery_slot: Polygon = romi.rectangle_locate("Lower Right Battery Slot",
                                                                  -3.661173, 3.705346,
                                                                  -2.184799, 3.925815)
        upper_center_battery_slot: Polygon = romi.rectangle_locate("Upper Center Battery Slot",
                                                                   -4.58637, 3.012429,
                                                                   -3.109992, 3.232913)
        lower_center_battery_slot: Polygon = romi.rectangle_locate("Lower Center Battery Slot",
                                                                   -4.625744, 2.465193,
                                                                   -3.149354, 2.685665)

        # *battery_slot_polygons* list and append them to *polygons*:
        battery_slot_polygons: List[Polygon] = [
            upper_left_battery_slot, upper_right_battery_slot,
            lower_left_battery_slot, lower_right_battery_slot,
            upper_center_battery_slot, lower_center_battery_slot
        ]
        polygons.extend(battery_slot_polygons)

        # There 4 cutouts across the top of the battery case and 3 cutouts along the bottom.
        # The 4 upper cutouts are called *upper_outer_left_cutout*, *upper_inner_left_cutout*,
        # *upper_inner_right_cutout*, and *upper_outer_right_cutout*.
        upper_outer_left_cutout: Polygon = romi.rectangle_locate("Upper Outer Left Cutout",
                                                                 -5.984008, 4.74865,
                                                                 -5.302909, 4.965193)
        upper_inner_left_cutout: Polygon = romi.rectangle_locate("Upper Inner Left Cutout",
                                                                 -5.23598, 4.669913,
                                                                 -4.861965, 4.858886)
        upper_inner_right_cutout: Polygon = romi.rectangle_locate("Upper Inner Right Cutout",
                                                                  -2.873772, 4.669913,
                                                                  -2.499756, 4.858886)
        upper_outer_right_cutout: Polygon = romi.rectangle_locate("Upper Outer Right Cutout",
                                                                  -2.432827, 4.74865,
                                                                  -1.751728, 4.965193)

        # Thre are three cutouts across the bottom of the battery case and they are called
        # *lower_left_cutout*, *lower_center_cutout*, and *lower_right_cutout*:
        lower_left_cutout: Polygon = romi.rectangle_locate("Lower Left Cutout",
                                                           -5.572591, 1.939594,
                                                           -4.655272, 2.189594)
        lower_center_cutout: Polygon = romi.rectangle_locate("Lower Center Cutout",
                                                             -4.340311, 2.032122,
                                                             -3.395425, 2.189594)
        lower_right_cutout: Polygon = romi.rectangle_locate("Lower Right Cutout",
                                                            -3.080465, 1.939594,
                                                            -2.163146, 2.189594)

        # Collect all of the cutouts into *cutout_polygons* and append to *polygons*:
        cutout_polygons: List[Polygon] = [
            upper_outer_left_cutout, upper_inner_left_cutout,
            upper_inner_right_cutout, upper_outer_right_cutout,
            lower_left_cutout, lower_center_cutout, lower_right_cutout
        ]
        polygons.extend(cutout_polygons)

        # There are 4 slots for where the encoder connectors through hole pins land.
        # We will measure two on the right side and mirror them to the left side:
        x1: float = -2.031646 * inches2mm - origin_offset.x - 8.85
        dx: float = 2.5
        x2: float = x1 - dx
        x3: float = x2 - 1.65
        x4: float = x3 - dx
        dy: float = 0.70 * inches2mm
        outer_encoder_slot: Polygon = Polygon("Outer Encoder Slot")
        outer_encoder_slot.rotated_rectangle_append(P((x1 + x2) / 2.0, 0.0), dx, dy, 0.0)
        inner_encoder_slot: Polygon = Polygon("Inner Encoder Slot")
        inner_encoder_slot.rotated_rectangle_append(P((x3 + x4) / 2.0, 0.0), dx, dy, 0.0)

        # Collect the encoder slots into *encoder_slot_polygons* and append to *polygons*:
        encoder_slot_polygons: List[Polygon] = [
            outer_encoder_slot, outer_encoder_slot.y_mirror(),
            inner_encoder_slot, inner_encoder_slot.y_mirror()
        ]
        polygons.extend(encoder_slot_polygons)

        return polygons

    # Romi.hex_pattern_get():
    def hex_pattern_get(self, pattern_rows: Tuple[str, ...], slot_pairs: List[str],
                        hex_origin: P, hole_diameter: float) -> Tuple[List[Polygon], Dict[str, P]]:
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

        Returns:
            (*List*[*Polygon*], *Dict*[*str*, *P"]): Returns a list of
                hole and slot *Polygons*.  In addition, a *dict* that
                maps each hole letter name to a location is returned.

        """
        # Grab some values from *romi* (i.e. *self*):
        romi: Romi = self
        inches2mm: float = romi.inches2mm
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

        # We need to get the dimensions for one vertical slot and compute the vertical *slot_width*
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

        # Perform any requested debugging:
        if debugging:  # pragma: no cover
            print("-------------------")
            print(f"hex_origin={hex_origin}")
            print(f"hole_diameter={hole_diameter}")
            print(f"slot_width={slot_width}")
            print(f"slot_length={slot_length}")
            print(f"half_hex_dx_pitch={half_hex_dx_pitch}")
            print(f"hex_dy_pitch={hex_dy_pitch}")
            print(f"slot_dx={slot_dx}")
            print(f"slot_dy={slot_dy}")
            print(f"slot_length={slot_length}")
            print(f"slot_width={slot_width}")

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

        # The return values are *polygons* and *locations*:
        polygons: List[Polygon] = list()
        locations: Dict[str, P] = dict()

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
                    hole_center: P = P(x, y)
                    locations[pattern_character] = hole_center

                    # Only create holes when *pattern_character* is upper case:
                    if pattern_character.isupper():
                        # Put in the *right_hole*:
                        hole_polygon: Polygon = Polygon(f"Hex Hole ({x_index}, {y_index})")
                        hole_polygon.circle_append(hole_center, hole_diameter, points_count)
                        polygons.append(hole_polygon)

        # Now sweep through *slot_pairs* and install all of the slots:
        slot_pair: str
        for slot_pair in slot_pairs:
            # Do one slot for each *slot_pair8:
            hole1: P = locations[slot_pair[0]]
            hole2: P = locations[slot_pair[1]]
            slot_polygon: Polygon = Polygon(f"Slot '{slot_pair}'")
            slot_polygon.slot_append(hole1, hole2, slot_length, slot_width, points_count)
            polygons.append(slot_polygon)

        # Return the *polygons* and *locations*:
        return polygons, locations

    # Romi.hole_locate():
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

    # Romi.line_hole_polygons_get():
    def line_hole_polygons_get(self, lower_hex_table: Dict[str, P]) -> List[Polygon]:
        """TODO."""
        # Grab some values from *romi* (i.e. *self*):
        romi: Romi = self
        debugging: bool = romi.debugging

        # There is a line of holes along the bottom that have a smaller hole diameter.
        # We locate the smallest hole at the end of the line:
        small_hole_diameter: float
        small_hole_center: P
        small_hole_diameter, small_hole_center = romi.hole_locate(-3.289535, -3.198980,
                                                                  0.256524, 0.165984)
        if debugging:  # pragma: no cover
            print(f"small_hole_diameter={small_hole_diameter}")
            print(f"small_hole_center={small_hole_center}")

        # Now using *s_center* and *q_center* we compute a "unit" vector along the line.
        # We enter holes that do not over lap with the larger holes.  We wind up skipping
        # one hole in 3:
        line_hole_polygons: List[Polygon] = list()
        s_center: P = lower_hex_table["S"]
        q_center: P = lower_hex_table["Q"]
        hole_vector: P = q_center - s_center
        for vector_hole_index in range(9):
            if vector_hole_index % 3 != 1:
                # Do the hole on the right first:
                hole_center: P = (s_center + (vector_hole_index - 1) * hole_vector / 3.0)
                hole_polygon: Polygon = Polygon(f"Vector Hole {vector_hole_index}")
                hole_polygon.circle_append(hole_center, small_hole_diameter, 8)
                line_hole_polygons.append(hole_polygon)

        return line_hole_polygons

    # Romi.lower_arc_hole_rectangle_polygons_get():
    def lower_arc_hole_rectangle_polygons_get(self) -> List[Polygon]:
        """TODO."""
        # Grab some values from *romi*:
        romi: Romi = self
        debugging: bool = romi.debugging
        inches2mm: float = romi.inches2mm
        origin_offset: P = romi.origin_offset

        # The resulting *Polygon*'s are collected into *lower_arc_hole_rectangle_polygons*:
        lower_arc_hole_rectangle_polygons: List[Polygon] = list()

        # There are arcs of holes and and rectangular slots along the upper and lower rims.
        # Since they are mirrored across the Y axis, we only worry about the right side.
        # The hole closest to the wheel is the "start" hole and the one farthest from the
        # wheel is the "end" hole.  We have to locate each of these holes:
        lower_start_diameter: float
        lower_start_center: P
        lower_start_diameter, lower_start_center = romi.hole_locate(-1.483063, -1.357508,
                                                                    1.348929, 1.223803)
        lower_arc_start_angle: float = atan2(lower_start_center.y, lower_start_center.x)
        if debugging:  # pragma: no cover
            print(f"lower_start_diameter={lower_start_diameter}")
            print(f"lower_start_center={lower_start_center}")
            print(f"lower_start_angle={degrees(lower_arc_start_angle)}deg")

        # We pick the smallest hole that is next to the hole at the end to get the
        # *small_hole_diameter*:
        lower_end_diameter: float
        lower_end_center: P
        lower_end_diameter, lower_end_center = romi.hole_locate(-3.14402, -3.053465,
                                                                0.125287, 0.034732)
        small_hole_diameter: float = lower_end_diameter

        lower_arc_end_angle: float = atan2(lower_end_center.y, lower_end_center.x)
        if debugging:  # pragma: no cover
            print(f"lower_start_diameter={lower_start_diameter}")
            print(f"lower_start_center={lower_start_center}")

        # Compute the *lower_arc_radius*:
        origin: P = P()
        lower_hole_radius: float = origin.distance(lower_start_center)

        # There are two sizes of rectangle -- small and large.  The width appears to
        # be the same for both, so we only need *rectangle_width*, *small_rectangle_length*
        # and *large_rectangle_length*.  Lastly, we need to find one *rectangle_center*
        # so we can determine the *rectangle_radius* from the *origin*:
        large_upper_left_corner: P = (P(-1.248201 * inches2mm, 1.259484 * inches2mm) -
                                      origin_offset)
        large_lower_left_corner: P = (P(-1.33137 * inches2mm, 1.136248 * inches2mm) -
                                      origin_offset)
        large_upper_right_corner: P = (P(-1.205772 * inches2mm, 1.230858 * inches2mm) -
                                       origin_offset)
        large_rectangle_length: float = large_upper_left_corner.distance(large_lower_left_corner)
        rectangle_width: float = large_upper_left_corner.distance(large_upper_right_corner)
        rectangle_center: P = (large_upper_right_corner + large_lower_left_corner) / 2.0
        rectangle_radius: float = origin.distance(rectangle_center)
        small_upper_left_corner: P = (P(-1.368228 * inches2mm, 1.081638 * inches2mm) -
                                      origin_offset)
        small_lower_left_corner: P = (P(-1.431575 * inches2mm, 0.987760 * inches2mm) -
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
            lower_hole_center = P(lower_hole_x, lower_hole_y)
            lower_hole: Polygon = Polygon(f"Lower hole {lower_hole_index}")
            lower_hole.circle_append(lower_hole_center, lower_arc_hole_diameter, 8)
            if lower_hole_index < lower_holes_count + 1:
                lower_arc_hole_rectangle_polygons.append(lower_hole)

            # Next do the *lower_right_rectangle*:
            lower_rectangle_x: float = rectangle_radius * cos(lower_hole_angle)
            lower_rectangle_y: float = rectangle_radius * sin(lower_hole_angle)
            lower_rectangle_center: P = P(lower_rectangle_x, lower_rectangle_y)
            lower_rectangle: Polygon = Polygon(f"Lower left rectangle {lower_hole_index}")
            lower_rectangle.rotated_rectangle_append(lower_rectangle_center, rectangle_width,
                                                     lower_rectangle_length, lower_hole_angle)
            lower_arc_hole_rectangle_polygons.append(lower_rectangle)

        # Return the resuting *arc_hole_rectangle_polygons*:
        return lower_arc_hole_rectangle_polygons

    # Romi.lower_hex_polygons_table_get():
    def lower_hex_polygons_table_get(self) -> Tuple[List[Polygon], Dict[str, P]]:
        """TODO."""
        # The "User's Guide" identifies the lower hex whole used to reference the hex
        # pattern off of:
        # Grab some values from *romi* (i.e. *self*):
        romi: Romi = self
        debugging: bool = romi.debugging

        # Extract the origin hole information for the lower hexagon pattern:
        lower_hex_hole_center: P
        lower_hex_hole_diameter: float
        lower_hex_hole_diameter, lower_hex_hole_center = romi.hole_locate(-2.454646, -2.329091,
                                                                          1.553776, 1.428650)
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
        lower_hex_holes_table: Dict[str, P]

        lower_hex_polygons: List[Polygon]
        lower_hex_polygons, lower_holes_table = romi.hex_pattern_get(lower_pattern_rows,
                                                                     lower_slot_pairs,
                                                                     lower_hex_hole_center,
                                                                     lower_hex_hole_diameter)
        return lower_hex_polygons, lower_holes_table

    # Romi.lower_miscellaneous_polygons_get():
    def lower_miscellaneous_polygons_get(self) -> List[Polygon]:
        """Return the miscellaneous holes from bottom base."""
        # Grab some values from *romi* (i.e. *self*):

        misc_polygons: List[Polygon] = []
        # lower_left_misc_diameter: float
        # lower_left_misc_center: P
        # lower_left_misc_diameter, lower_left_misc_center = romi.hole_locate(-3.119047, -3.028492,
        # lower_left_misc_polygon: Polygon = Polygon("Lower Left Misc")
        # lower_left_misc_polygon.circle_append(lower_left_misc_center, lower_left_misc_diameter)
        return misc_polygons

    def rectangle_locate(self, name: str, x1: float, y1: float,
                         x2: float, y2: float) -> Polygon:
        """Locate a rectangle and return it as a Polygon.

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
        origin_offset: P = romi.origin_offset
        inches2mm: float = romi.inches2mm

        # Convert *x1*, *y1*, *x2*, and *y2* from inches to millimeters:
        x1 *= inches2mm
        y1 *= inches2mm
        x2 *= inches2mm
        y2 *= inches2mm
        dx: float = abs(x2 - x1)
        dy: float = abs(y2 - y1)
        center: P = P((x1 + x2) / 2.0, (y1 + y2) / 2.0) - origin_offset

        # Create and return the *rectangle_polygon*:
        rectangle_polygon: Polygon = Polygon(name)
        rectangle_polygon.rotated_rectangle_append(center, dx, dy, 0.0)
        return rectangle_polygon

    # Romi.upper_arc_hole_rectangle_polygons_get():
    def upper_arc_hole_rectangle_polygons_get(self) -> List[Polygon]:
        """TODO."""
        # Grab some values from *romi*:
        romi: Romi = self
        debugging: bool = romi.debugging
        inches2mm: float = romi.inches2mm
        origin_offset: P = romi.origin_offset

        # The resulting *Polygon*'s are collected into *upper_arc_hole_rectangle_polygons*:
        upper_arc_hole_rectangle_polygons: List[Polygon] = list()

        # There are arcs of holes and and rectangular slots along the upper and lower rims.
        # Since they are mirrored across the Y axis, we only worry about the right side.
        # The hole closest to the wheel is the "start" hole and the one farthest from the
        # wheel is the "end" hole.  We have to locate each of these holes:
        upper_start_diameter: float
        upper_start_center: P
        upper_start_diameter, upper_start_center = romi.hole_locate(-1.483063, -1.357508,
                                                                    4.651469, 4.526346)
        upper_arc_start_angle: float = atan2(upper_start_center.y, upper_start_center.x)
        if debugging:  # pragma: no cover
            print(f"upper_start_diameter={upper_start_diameter}")
            print(f"upper_start_center={upper_start_center}")
            print(f"upper_start_angle={degrees(upper_arc_start_angle)}deg")

        upper_end_diameter: float
        upper_end_center: P
        upper_end_diameter, upper_end_center = romi.hole_locate(-3.14402, -3.053465,
                                                                5.840524, 5.749969)
        upper_arc_end_angle: float = atan2(upper_end_center.y, upper_end_center.x)
        if debugging:  # pragma: no cover
            print(f"upper_end_diameter={upper_end_diameter}")
            print(f"upper_end_center={upper_end_center}")

        # Compute the *upper_hole_radius*:
        origin: P = P()
        upper_hole_radius: float = origin.distance(upper_start_center)

        # There are two sizes of rectangle -- small and large.  The width appears to
        # be the same for both, so we only need *rectangle_width*, *small_rectangle_length*
        # and *large_rectangle_length*.  Lastly, we need to find one *rectangle_center*
        # so we can determine the *rectangle_radius* from the *origin*:
        large_upper_inner_corner: P = (P(-1.33137 * inches2mm, 4.739012 * inches2mm) -
                                       origin_offset)
        large_lower_inner_corner: P = (P(-1.248201 * inches2mm, 4.615776 * inches2mm) -
                                       origin_offset)
        large_lower_outer_corner: P = (P(-1.205772 * inches2mm, 4.644402 * inches2mm) -
                                       origin_offset)
        large_rectangle_length: float = large_upper_inner_corner.distance(large_lower_inner_corner)
        rectangle_width: float = large_lower_inner_corner.distance(large_lower_outer_corner)
        rectangle_center: P = (large_upper_inner_corner + large_lower_outer_corner) / 2.0
        rectangle_radius: float = origin.distance(rectangle_center)
        small_upper_inner_corner: P = (P(-1.431575 * inches2mm, 4.887512 * inches2mm) -
                                       origin_offset)
        small_lower_inner_corner: P = (P(-1.368228 * inches2mm, 4.793638 * inches2mm) -
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
            upper_hole_center = P(upper_hole_x, upper_hole_y)
            upper_hole: Polygon = Polygon(f"Upper hole {upper_hole_index}")
            upper_hole.circle_append(upper_hole_center, upper_arc_hole_diameter, 8)
            # Skip 3 of the holes:
            if upper_hole_index not in (2, 3, 12, 13):
                upper_arc_hole_rectangle_polygons.append(upper_hole)

            # Next do the *lower_right_rectangle*:
            upper_rectangle_x: float = rectangle_radius * cos(upper_hole_angle)
            upper_rectangle_y: float = rectangle_radius * sin(upper_hole_angle)
            upper_rectangle_center: P = P(upper_rectangle_x, upper_rectangle_y)
            upper_rectangle: Polygon = Polygon(f"Upper right rectangle {upper_hole_index}")
            upper_rectangle.rotated_rectangle_append(upper_rectangle_center, rectangle_width,
                                                     upper_rectangle_length, upper_hole_angle)
            upper_arc_hole_rectangle_polygons.append(upper_rectangle)

        # Return the resuting *arc_hole_rectangle_polygons*:
        return upper_arc_hole_rectangle_polygons

    # Romi.upper_hex_polygons_get():
    def upper_hex_polygons_get(self) -> List[Polygon]:
        """TODO."""
        # Grab some values from *romi* (i.e. *self*):
        romi: Romi = self
        debugging: bool = romi.debugging

        # For the upper hex pattern, the hole that is at the end of the 4 slots is selected
        # as the upper hex hole:
        upper_hex_hole_center: P
        upper_hex_hole_diameter: float
        upper_hex_hole_diameter, upper_hex_hole_center = romi.hole_locate(-2.749535, -2.629075,
                                                                          5.441567, 5.316441)
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
        upper_holes_table: Dict[str, P]
        upper_hex_polygons: List[Polygon]
        upper_hex_polygons, upper_holes_table = romi.hex_pattern_get(upper_pattern_rows,
                                                                     upper_slot_pairs,
                                                                     upper_hex_hole_center,
                                                                     upper_hex_hole_diameter)

        # The *upper_holes_table* is not needed, we just return *upper_hex_polygons*:
        return upper_hex_polygons


def main() -> int:  # pragma: no cover
    """Generate the openscand file."""
    print("romi_model.main() called")
    romi: Romi = Romi()
    romi_base_scad_polygon: ScadPolygon = romi.base_scad_polygon_generate()
    scad_file: IO[Any]
    with open("romi_base_dxf.scad", "w") as scad_file:
        romi_base_scad_polygon.file_write(scad_file)

    extruded_romi_base_scad_polygon: ScadLinearExtrude
    extruded_romi_base_scad_polygon = ScadLinearExtrude("Romi Base Extrude",
                                                        romi_base_scad_polygon, 9.6)
    union: Scad3DUnion = Scad3DUnion("Base Union", [extruded_romi_base_scad_polygon])
    with open("romi_base.scad", "w") as scad_file:
        # extruded_romi_base_scad_polygon.file_write(scad_file)
        union.file_write(scad_file)
    return 0
