"""SCAD.

# Introduction

This module provides a Python interface to OpenSCAD.

The basic class tree is:

* P: Generic point
  * P2D: 2-dimensional Point
  * P3D: 3-dimensional Point
* Scad: Basic Scad Command
  * Scad: A generic OpenScad command
    * ScadProgram: A top level OpenSCAD program.
    * Scad2D: 2-dimensional Objects
      * Circle: A circle
      * If2D: An if-then-else tree for SCAD2D's.
      * Module2D: A 2D module definition.
      * Polygon: A outer most SimplePolygon with optional SimplePolygon holes.
      * SimplePolygon: A polygon contructed of line segments and arcs.
      * Square: A rotatable rectangle with optional rounded corners
      * UseModule3D: A use of a Module2D.
    * Scad3D:
      * Cube: A cube (technically a rectangular prism.)
      * LinearExtrude: Linear extrusion of SCAD2D into a SCAD3D.
      * Translate3D: Relocates a SCAD3 object to a different location.
      * Union3D: A Union of Scad3D objects.
"""

# MIT License
#
# Copyright (c) 2019 Wayne C. Gramlich (Wayne@Gramlich.Net)
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

# <----------------------------------------100 Characters----------------------------------------> #

# Import stuff from other libraries:
from math import acos, ceil, cos, degrees, pi, sin, sqrt
from typing import Any, Callable, IO, List, Optional, Set, Tuple


# P3D:
class P3D:
    """Represents a 3 dimensional point."""

    # P3D.__init__():

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
        """Initialize the point contents."""
        # Load values into *p3d* (i.e. *self*):
        # p3d: P3D = self
        self.x: float = x
        self.y: float = y
        self.z: float = z

    # P3D.__add__():
    def __add__(self, p3d2: "P3D") -> "P3D":
        """Add two points together."""
        # Use *p3d1* instead of *self*:
        p3d1: P3D = self
        return P3D(p3d1.x + p3d2.x, p3d1.y + p3d2.y, p3d1.z + p3d2.z)

    # P3D.__mul__():
    def __mul__(self, scale: float) -> "P3D":
        """Multiply a p3d by a scale factor."""
        # Use *p3d* instead of *self*:
        p3d: P3D = self
        return P3D(p3d.x * scale, p3d.y * scale, p3d.z * scale)

    # P3D.__neg__():
    def __neg__(self) -> "P3D":
        """Return the negative of of a point."""
        # USe *p3d* insetead of *self*:
        p3d: P3D = self
        negative: P3D = P3D(-p3d.x, -p3d.y, -p3d.z)
        return negative

    # P3D.__rmul__():
    def __rmul__(self, scale: float) -> "P3D":
        """Multiply a p3d by a scale factor."""
        # Use *p3d* instead of *self*:
        p3d: P3D = self
        return P3D(p3d.x * scale, p3d.y * scale, p3d.z * scale)

    # P3D.__sub__():
    def __sub__(self, p3d2: "P3D") -> "P3D":
        """Subtract two p3ds from one another."""
        # Use *p3d1* instead of *self*:
        p3d1: P3D = self
        return P3D(p3d1.x - p3d2.x, p3d1.y - p3d2.y, p3d1.z - p3d2.z)

    # P3D.__str__():
    def __str__(self) -> str:
        """Convert a p3d to a string."""
        # Use *p3d* instead of *self*:
        p3d: P3D = self
        x_text: str = "{0:.3f}".format(p3d.x)
        y_text: str = "{0:.3f}".format(p3d.y)
        z_text: str = "{0:.3f}".format(p3d.z)
        x_text = "0.000" if x_text == "-0.000" else x_text
        y_text = "0.000" if y_text == "-0.000" else y_text
        z_text = "0.000" if z_text == "-0.000" else z_text
        return f"P3D({x_text},{y_text},{z_text})"

    # P3D.__truediv__():
    def __truediv__(self, scale: float) -> "P3D":
        """Divide a p3d by a scale factor."""
        p3d: P3D = self
        return P3D(p3d.x / scale, p3d.y / scale, p3d.z / scale)

    # P3D.cross():
    def cross(self, p3d2: "P3D") -> "P3D":
        """Return the cross product of 2 points."""
        p3d1: P3D = self
        return P3D(p3d1.y * p3d2.z - p3d1.z * p3d2.y,
                   p3d1.z * p3d2.x - p3d1.x * p3d2.z,
                   p3d1.x * p3d2.y - p3d1.y * p3d2.x)

    # P3D.dot():
    def dot(self, p3d2: "P3D") -> float:
        """Return the dot product of 2 points."""
        p3d1: P3D = self
        dot_product: float = p3d1.x * p3d2.x + p3d1.y * p3d2.y + p3d1.z * p3d2.z
        return dot_product

    # P3D.distance():
    def distance(self, p3d2: "P3D") -> float:
        """Compute the distance between two p3ds."""
        p3d1: P3D = self
        dx: float = p3d1.x - p3d2.x
        dy: float = p3d1.y - p3d2.y
        dz: float = p3d1.z - p3d2.z
        total_distance: float = sqrt(dx * dx + dy * dy + dz * dz)
        return total_distance

    # P3D.length():
    def length(self) -> float:
        """Return the distance from origin to a point."""
        p3d: P3D = self
        x: float = p3d.x
        y: float = p3d.y
        z: float = p3d.z
        total_length: float = sqrt(x * x + y * y + z * z)
        return total_length


# P2D:
class P2D:
    """Represents a point in 2 demensions."""

    # P2D.__init__():
    def __init__(self, x: float, y: float):
        """Initialize a P2D."""
        # Load *x* and *y* into *p2d* (i.e. *self*):
        # p2d: P2D = self
        self.x: float = x
        self.y: float = y

    # P2D.__add__():
    def __add__(self, p2d2: "P2D") -> "P2D":
        """Add two two P2D's together."""
        # Use *p2d1* instead of *self*:
        p2d1: P2D = self
        return P2D(p2d1.x + p2d2.x, p2d1.y + p2d2.y)

    # P2D.__mul__():
    def __mul__(self, scale: float) -> "P2D":
        """Multiply a P2D by a scale factor."""
        # Use *p2d* instead of *self*:
        p2d: P2D = self
        return P2D(p2d.x * scale, p2d.y * scale)

    # P2D.__rmul__():
    def __rmul__(self, scale: float) -> "P2D":
        """Multiply a P2D by a scale factor."""
        # Use *p2d* instead of *self*:
        p2d: P2D = self
        return P2D(p2d.x * scale, p2d.y * scale)

    # P2D.__sub__():
    def __sub__(self, p2d2: "P2D") -> "P2D":
        """Subtract two P2D's from one another."""
        # Use *p2d1* instead of *self*:
        p2d1: P2D = self
        return P2D(p2d1.x - p2d2.x, p2d1.y - p2d2.y)

    # P2D.__str__():
    def __str__(self) -> str:
        """Convert a P2D to a string."""
        # Use *p2d* instead of *self*:
        p2d: P2D = self
        x_text: str = "{0:.3f}".format(p2d.x)
        y_text: str = "{0:.3f}".format(p2d.y)
        x_text = "0.000" if x_text == "-0.000" else x_text
        y_text = "0.000" if y_text == "-0.000" else y_text
        return f"P2D({x_text},{y_text})"

    # P2D.__truediv__():
    def __truediv__(self, scale: float) -> "P2D":
        """Divide a P2D by a scale factor."""
        p2d: P2D = self
        return P2D(p2d.x / scale, p2d.y / scale)

    # P2D.distance():
    def distance(self, p2d2: "P2D") -> float:
        """Compute the distance between two P2D's."""
        p2d1: P2D = self
        dx: float = p2d1.x - p2d2.x
        dy: float = p2d1.y - p2d2.y
        return sqrt(dx * dx + dy * dy)

    # P2D.rotate():
    def rotate(self, angle: float, center: "Optional[P2D]" = None) -> "P2D":
        """Rotate a P2D by angle around the origin."""
        # To rotate a *p2d* (i.e. self) around the origin, use the following formula:
        #
        #   x' = x * cos(angle) - y * sin(angle)
        #   y' = y * cos(angle) + x * sin(angle)
        p2d: P2D = self
        center_x: float = 0.0 if center is None else center.x
        center_y: float = 0.0 if center is None else center.y
        x: float = p2d.x - center_x
        y: float = p2d.y - center_y
        sin_angle: float = sin(angle)
        cos_angle: float = cos(angle)
        rotated_x: float = center_x + x * cos_angle - y * sin_angle
        rotated_y: float = center_y + y * cos_angle + x * sin_angle
        rotated_point: P2D = P2D(rotated_x, rotated_y)
        return rotated_point

    # PD2.y_mirror():
    def y_mirror(self) -> "P2D":
        """Return the p3d mirrored across the Y axis."""
        p2d: P2D = self
        return P2D(-p2d.x, p2d.y)


# Scad:
class Scad:
    """Base class that an OpenSCAD object, transform, etc.

    This base class basically just provides a name and is sub-classed to
    provide all of the functiontality.
    """

    # Scad.__init__()
    def __init__(self, name: str) -> None:
        """Create the base *Scad* object.

        Args:
            *name* (*str*): The name of the *Scad* object.

        """
        # Stuff *name* into the *scad* object (i.e. *self*):
        self.name: str = name

    # Scad.colors_set_get():
    @staticmethod
    def colors_set_get() -> Set[str]:
        """Return a set of lower case allowed colors."""
        colors: List[str] = [
            # Purples:
            "Lavender"
            "Thistle",
            "Plum",
            "Violet",
            "Orchid",
            "Fuchsia",
            "Magenta",
            "MediumOrchid",
            "MediumPurple",
            "BlueViolet",
            "DarkViolet",
            "DarkOrchid",
            "DarkMagenta",
            "Purple",
            "Indigo",
            "DarkSlateBlue",
            "SlateBlue",
            "MediumSlateBlue",
            # Pinks:
            "Pink",
            "LightPink",
            "HotPink",
            "DeepPink",
            "MediumVioletRed",
            "PaleVioletRed",
            # Blues:
            "Aqua",
            "Cyan",
            "LightCyan",
            "PaleTurquoise",
            "Aquamarine",
            "Turquoise",
            "MediumTurquoise",
            "DarkTurquoise",
            "CadetBlue",
            "SteelBlue",
            "LightSteelBlue",
            "PowderBlue",
            "LightBlue",
            "SkyBlue",
            "LightSkyBlue",
            "DeepSkyBlue",
            "DodgerBlue",
            "CornflowerBlue",
            "RoyalBlue",
            "Blue",
            "MediumBlue",
            "DarkBlue",
            "Navy",
            "MidnightBlue",
            # Reds:
            "IndianRed",
            "LightCoral",
            "Salmon",
            "DarkSalmon",
            "LightSalmon",
            "Red",
            "Crimson",
            "FireBrick",
            "DarkRed",
            # Greens:
            "GreenYellow",
            "Chartreuse",
            "LawnGreen",
            "Lime",
            "LimeGreen",
            "PaleGreen",
            "LightGreen",
            "MediumSpringGreen",
            "SpringGreen",
            "MediumSeaGreen",
            "SeaGreen",
            "ForestGreen",
            "Green",
            "DarkGreen",
            "YellowGreen",
            "OliveDrab",
            "Olive",
            "DarkOliveGreen",
            "MediumAquamarine",
            "DarkSeaGreen",
            "LightSeaGreen",
            "DarkCyan",
            "Teal",
            # Oranges
            "LightSalmon",
            "Coral",
            "Tomato",
            "OrangeRed",
            "DarkOrange",
            "Orange",
            # Yellows:
            "Gold",
            "Yellow",
            "LightYellow",
            "LemonChiffon",
            "LightGoldenrodYellow",
            "PapayaWhip",
            "Moccasin",
            "PeachPuff",
            "PaleGoldenrod",
            "Khaki",
            "DarkKhaki",
            # Browns:
            "Cornsilk",
            "BlanchedAlmond",
            "Bisque",
            "NavajoWhite",
            "Wheat",
            "BurlyWood",
            "Tan",
            "RosyBrown",
            "SandyBrown",
            "Goldenrod",
            "DarkGoldenrod",
            "Peru",
            "Chocolate",
            "SaddleBrown",
            "Sienna",
            "Brown",
            "Maroon",
            # Whites:
            "White",
            "Snow",
            "Honeydew",
            "MintCream",
            "Azure",
            "AliceBlue",
            "GhostWhite",
            "WhiteSmoke",
            "Seashell",
            "Beige",
            "OldLace",
            "FloralWhite",
            "Ivory",
            "AntiqueWhite",
            "Linen",
            "LavenderBlush",
            "MistyRose",
            # Grays:
            "Gainsboro",
            "LightGrey",
            "Silver",
            "DarkGray",
            "Gray",
            "DimGray",
            "LightSlateGray",
            "SlateGray",
            "DarkSlateGray",
            "Black",
        ]
        color: str
        colors_set: Set[str] = {color.lower() for color in colors}
        return colors_set

    # Scad.float_format():
    @staticmethod
    def float_format(value: float) -> str:
        """Convert a float into a string."""
        value_text: str = "{0:.3f}".format(value)
        value_text = "0.000" if value_text == "-0.000" else value_text
        return value_text

    # Scad.keys_csv_file_write():
    @staticmethod
    def keys_csv_file_write(keys: List[Tuple[Any, ...]], csv_file: IO[Any]) -> None:
        """Write out keys to an open file.

        Args:
            *keys* (*List*[*Tuple*[*Any*, ...]]: A list of keys to
                output.
            *csv_file* (*IO*[*Any*]): An open file to write to:

        """
        # Outut a header first:
        float_format: Callable[[float], str] = Scad.float_format
        csv_file.write("Type,Name,X,Y,DX,DY,Angle,Corner Radius,Corner Count\n")
        key: Tuple[Any, ...]
        for key in keys:
            key_value: Any
            key_texts: List[str] = []
            for key_value in key:
                key_text: str = (f'"{key_value}"' if isinstance(key_value, str)
                                 else (float_format(key_value) if isinstance(key_value, float)
                                       else str(key_value)))
                key_texts.append(key_text)
            csv_file.write(','.join(key_texts) + '\n')

    # Scad.html_table_file_write():
    @staticmethod
    def keys_html_file_write(keys: List[Tuple[Any, ...]], html_file: IO[Any], title: str) -> None:
        """Write out keys as an HTML table.

        Args:
            *keys* (*List*[*Tuple*[*Any*, ...]]: A list of keys to
                output.
            *html_file* (*IO*[*Any*]): An open file to write HTML to:
            *title* (*str*): The title to use in the generated HTML.

        """
        # Output the HTML Head and start the HTML Body with *title* and Table:
        html_file.write("<HTML>\n")
        html_file.write(" <Head>\n")
        html_file.write(f"  <Title>{title}</Title>\n")
        html_file.write(" </Head>\n")
        html_file.write(" <Body>\n")
        html_file.write(f"  <H1>{title}</H1>\n")
        html_file.write("  <Table>\n")

        # Output the *headings*:
        headings_text: str = "Index,Type,Name,X,Y,DX,DY,Angle,Corner Radius,Corner Count"
        headings: List[str] = headings_text.split(',')
        heading: str
        heading_row_text: str = (
            "   <TR>\n" +
            '\n'.join([f'    <TH align="left">{heading}</TH>' for heading in headings]) +
            "\n   </TR>\n"
        )
        # Now output all of the keys:
        float_format: Callable[[float], str] = Scad.float_format
        key: Tuple[Any, ...]
        for key_index, key in enumerate(keys):
            # Start a row for *key*:
            if key_index % 20 == 0:
                html_file.write(heading_row_text)
            html_file.write("   <TR>\n")
            html_file.write(f'    <TD align="left">{key_index}</TD>\n')
            key_value: Any
            for key_value in key:
                # Output *key_value*:
                key_text: str = (float_format(key_value) if isinstance(key_value, float)
                                 else str(key_value))
                html_file.write(f'    <TD align="left">{key_text}</TD>\n')
            html_file.write("   </TR>\n")

        # Wrap up the the Table, Body and HTML:
        html_file.write("  </Table>\n")
        html_file.write(" </Body>\n")
        html_file.write("</HTML>\n")

    # Scad.polygon_scad_lines_append():
    def polygon_scad_lines_append(self, simple_polygons: "List[SimplePolygon]",
                                  scad_lines: List[str], indent: str) -> None:  # pragma: no cover
        """`Polygon` template command to a list of lines."""
        # Grab *class_name* from *scad* (i.e *self*) and fail with a reasonable error message:
        scad: Scad = self
        class_name: str = scad.__class__.__name__
        assert False, f"{class_name}.polygon_scad_lines_append() is not implemented yet."

    # Scad.scad_file_write():
    def scad_file_write(self, scad_file: IO[Any]) -> None:
        """Write out a `.scad` file.

        Args:
            *scad_file* (*IO*[*Any*]):
                An IO object (usually to the file system) that can be
                written to.

        """
        # Grab some values from *scad* (i.e. *self*):
        scad: Scad = self
        name: str = scad.name

        # Store the contents of *scad* as a bunch of *scad_lines*:
        scad_lines: List[str] = list()
        scad_lines.append(f"// '{name}' File")
        scad.scad_lines_append(scad_lines, "")

        # Convert *scad_lines* into *scad_text*:
        scad_lines.append("")
        scad_text: str = '\n'.join(scad_lines)

        # Output *scad_text* to *scad_file*:
        assert scad_file.writable(), f"Unable to write out .scad for '{name}'"
        scad_file.write(scad_text)

    # Scad.scad_lines_append():
    def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:  # pragma: no cover
        """Place holder for virtual *scad_lines_append* method."""
        scad: Scad = self
        class_name: str = scad.__class__.__name__
        assert False, f"{class_name}.scad_lines_append() not implemented yet"


# ScadProgram:
class ScadProgram:
    """Represents top level OpenScad program."""

    # ScadProgram.__init__():
    def __init__(self, name: str) -> None:
        """Initialize a ScadProgram."""
        # Load values into *scad_program* (i.e. *self*):
        # scad_program: ScadProgram = self
        self.name: str = name
        self.scads: List[Scad] = []

    # ScadProgram.__str__():
    def __str__(self) -> str:
        """Return a string representation of a ScadProgram."""
        # Grab some values out of *scad_program* (i.e. *self*):
        scad_program: ScadProgram = self
        name: str = scad_program.name
        return f"ScadProgram('{name}')"

    # ScadProgram.append():
    def append(self, scad: Scad):
        """Append a Scad to a ScadProgram."""
        scad_program: ScadProgram = self
        scads: List[Scad] = scad_program.scads
        scads.append(scad)

    # ScadProgram.scad_lines_append():
    def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:
        """Append ScadProgram to lines list.

        Args:
            *scad_lines* (*List*[*str*]): The lines list to append the
                *circle* (i.e. *self*) to.
            *indent* (*str*): The indentatation prefix for each line.

        """
        # Grab some values out of *scad_program* (i.e. *self*):
        scad_program: ScadProgram = self
        name: str = scad_program.name
        scads: List[Scad] = scad_program.scads

        # Append the initial comment:
        scad_lines.append(f"{indent}// Begin ScadProgram('{name}')")

        # Append each *scad* to *scad_lines*:
        scad: Scad
        for scad in scads:
            scad.scad_lines_append(scad_lines, indent)

        # Append the final comment:
        scad_lines.append(f"{indent}// End ScadProgram('{name}')")


# Scad2D:
class Scad2D(Scad):
    """Represents 2-dimensional Scad objects."""

    # Scad2D.__init__():
    def __init__(self, name: str) -> None:
        """Set the name of the 2-dimensional SCAD object."""
        super().__init__(name)

    # Scad2D.polygon_scad_lines_append():
    def polygon_scad_lines_append(self, simple_polygons: "List[SimplePolygon]",
                                  scad_lines: List[str], indent: str) -> None:
        """Append an OpenSCAD `polygon` command as a list of lines.

        Args:
            *simple_polygons* (*List*[*SimplePolygon*]): A list of
                *SimplePolygon*'s where the first one is the outer
                perimiter of the polygon, and the remaining
                *SimplePolygon*'s are "hole" inside of the outer
                polygon perimeter.  None of these *SimplePolygon*'s
                are allowed to overlap in any way.
            *scad_lines* (*List*[*str*]): A list of strings to which
                individual lines of OpenSCAD code are appended.
            *indent* (*str*): The indentation text to prefixe each
                line with.

        """
        # Grab the *class_name* from *scad2d* (i.e. *self*):
        scad2d: Scad2D = self
        scad_name: str = scad2d.name
        scad_class_name: str = scad2d.__class__.__name__

        # Sweep through all of *simple_polygon*'s and compute *maximum_convexity* and
        # *all_points_size*:
        maximum_convexity: int = 1
        all_points_size: int = 0
        simple_polygon: SimplePolygon
        for simple_polygon in simple_polygons:
            assert simple_polygon.is_locked()
            convexity: int = simple_polygon.convexity
            maximum_convexity = max(maximum_convexity, convexity)
            all_points_size += len(simple_polygon)

        # This code outputs an OpenSCAD `polygon` command in following format:
        #
        #     polygon(points = [ // Begin CLASS_NAME 0-TOTAL_POINTS
        #      // POLYGON1_CLASS 'POLYGON1_NAME' START_INDEX-END_INDEX
        #       [x1, y1], ... , [xN, yN]  // INDEX_RANGE
        #      // POLYGON2_CLASS 'POLYGON2_NAME' START_INDEX-END_INDEX
        #       [x1, y1], ... , [xN, yN]  // INDEX_RANGE
        #      ...
        #      // POLYGONn_CLASS 'POLYGONn_NAME' START_INDEX-END_INDEX
        #       [x1, y1], ... , [xN, yN]  // INDEX_RANGE
        #      ], paths = [
        #       // POLYGON1_CLASS 'POLYGON1_NAME' START_INDEX-END_INDEX
        #       [[x1, y1], ... , [xN, yN]],  // INDEX_RANGE
        #       // POLYGON2_CLASS 'POLYGON2_NAME' START_INDEX-END_INDEX
        #       [[x1, y1], ... , [xN, yN]],  // INDEX_RANGE
        #       ...
        #       // POLYGONn_CLASS 'POLYGONn_NAME' START_INDEX-END_INDEX
        #       [[x1, y1], ... , [xN, yN]  // INDEX_RANGE
        #      ], convexity=CONVEXITY; // End CLASS_NAME 0-TOTAL_POINTS

        # Now start the output of the OpenScad `polygon` command:
        scad_lines.append(f"{indent}polygon(points = [  // Begin {scad_class_name} "
                          f"'{scad_name}' {0}:{all_points_size-1}")

        # Define some variables and constants (alphabetical order):
        float_format: Callable[[float], str] = Scad.float_format
        indices_slice_size: int = 10
        points_slice_size: int = 4
        polygon_class_name: str
        polygon_name: str
        slice_begin_index: int
        slice_end_index: int
        slice_index: int
        slice_last_index: int
        slices_count: int
        simple_polygon_size: int
        simple_polygons_size: int = len(simple_polygons)
        simple_polygon_last_index = simple_polygons_size - 1

        # Step 1: Output all of the points first:
        begin_index: int = 0
        end_index: int
        for simple_polygon_index, simple_polygon in enumerate(simple_polygons):
            # Output the *simple* polygon name and its index range:
            simple_polygon_size = len(simple_polygon)
            end_index = begin_index + simple_polygon_size - 1
            simple_polygon_name = simple_polygon.name
            simple_polygon_class_name = simple_polygon.__class__.__name__
            last_simple_polygon: bool = simple_polygon_index == simple_polygon_last_index
            scad_lines.append(f"{indent} // {simple_polygon_class_name} '{simple_polygon_name}' "
                              f"{begin_index}-{end_index}")

            # Now output 1 or more rows of *points* from *simple_polygon*, truncating to
            # *point_slice_size* to prevent excessively long lines in the `.scad` file:
            points: List[P2D] = simple_polygon.points_get()
            slices_count = int((ceil(float(simple_polygon_size) / float(points_slice_size))))
            slice_last_index = slices_count - 1
            for slice_index in range(slices_count):
                # Extract the *slice* of points from *points*:
                slice_begin_index = slice_index * points_slice_size
                slice_end_index = min(slice_begin_index + points_slice_size, simple_polygon_size)
                slice: List[P2D] = list(points[slice_begin_index:slice_end_index])

                # Figure out all of the text to output for the *slice*:
                point: P2D
                points_text: str = ", ".join([f"[{float_format(point.x)}, {float_format(point.y)}]"
                                              for point in slice])
                end_text: str = ("" if last_simple_polygon and slice_index == slice_last_index
                                 else ",")

                # Perform the append to *scad_lines*:
                scad_lines.append(f"{indent}  {points_text}{end_text}  "
                                  f"// {slice_begin_index}-{slice_end_index}")

            # Update *begin_index* for the next batch of *points* from the next *simple_polygon*:
            begin_index += simple_polygon_size

        # Step 2: Output all of the indices second:
        scad_lines.append(f"{indent} ], paths = [")
        begin_index = 0
        for simple_polygon_index, simple_polygon in enumerate(simple_polygons):
            # Output the *simple* polygon name and its index range:
            simple_polygon_size = len(simple_polygon)
            end_index = begin_index + simple_polygon_size - 1
            simple_polygon_name = simple_polygon.name
            simple_polygon_class_name = simple_polygon.__class__.__name__
            scad_lines.append(f"{indent}  // {simple_polygon_class_name} '{simple_polygon_name}' "
                              f"{begin_index}-{end_index}")
            last_polygon = simple_polygon_index == simple_polygon_last_index

            slices_count = int((ceil(float(simple_polygon_size) / float(indices_slice_size))))
            slice_last_index = slices_count - 1
            for slice_index in range(slices_count):
                slice_begin_index = slice_index * indices_slice_size
                slice_end_index = min(slice_begin_index + indices_slice_size, simple_polygon_size)
                indices: List[str] = [str(begin_index + slice_index)
                                      for slice_index in range(slice_begin_index, slice_end_index)]
                indices_text: str = ", ".join(indices)
                begin_text: str = "[" if slice_index == 0 else " "
                end_text = ("," if slice_index < slices_count - 1
                            else (f"]" if last_polygon else "],"))
                scad_lines.append(f"{indent}  {begin_text}{indices_text}{end_text}")

            # Update *begin_index* for the next batch of *points* for the next *simple_polygon*:
            begin_index += simple_polygon_size

        # Close out the paths and output the *maximum_convexity*:
        scad_lines.append(f"{indent} ], convexity={maximum_convexity});  "
                          f"// End {scad_class_name} '{scad_name}' {0}:{all_points_size-1}")

    # Scad2D.scad_lines_append():
    def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:  # pragma: no cover
        """Place holder for sub-class scad_lines_append methods."""
        # Grab *class_name* from *scad2d* (i.e. *self*) and fail:
        scad2d: Scad2D = self
        class_name: str = scad2d.__class__.__name__
        assert False, f"{class_name}.scad_lines_append() has not been implemented yet."


# Echo2D:
# class Echo2D(Scad2D):
#     """Represents an OpenCAD 2D echo command."""

#     # Echo2D.__init__():
#     def __init__(self, name: str, variable_names: List[str]) -> None:
#         """Initialize Echo2D."""
#         # Initiailize parent *Scad2D* object:
#         super().__init__(name)
#         # Stuff variable names into *echo2d* (i.e. *self*):
#         # echo2d: Echo2D = self
#         self.variable_names: List[str] = variable_names

#     # Echo2D.__str__():
#     def __str__(self) -> str:
#         """Return a string representaion."""
#         # Grab some values from *echo2d* (i.e. *self*):
#         echo2d: Echo2D = self
#         name: str = echo2d.name
#         variable_names: List[str] = echo2d.variable_names
#         variable_name: str
#         variable_names_text: str = ','.join([f"'{variable_name}'"
#                                              for variable_name in variable_names])
#         return f"Echo2D('{name}',[{variable_names_text}])"

#     # Echo2D.scad_lines_append():
#     def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:
#         """Append Echo2D to lines list.

#         Args:
#             *scad_lines* (*List*[*str*]): The lines list to append the
#                 *circle* (i.e. *self*) to.
#             *indent* (*str*): The indentatation prefix for each line.

#         """
#         # Grab some values from *module2d* (i.e. *self*):
#         echo2d: Echo2D = self
#         name: str = echo2d.name
#         variable_names: List[str] = echo2d.variable_names
#         variable_names_text: str = ','.join([f"{variable_name}={variable_name}"
#                                              for variable_name in variable_names])
#         scad_lines.append(f'{indent}echo("{name}:", {variable_names_text});')


# Module2D:
class Module2D(Scad2D):
    """Represents an OpenSCAD 2D Module."""

    # Module2D.__init__():
    def __init__(self, name: str, scad2ds: List[Scad2D], is_operator=False, lock=True) -> None:
        """Initialize a Module2D."""
        # Initialize the *Scad2D* parent class:
        super().__init__(name)

        # Load values into *module2d* (i.e. *self*):
        # module2d: Module2D = self
        self.is_operator: bool = is_operator
        self.locked: bool = lock
        self.scad2ds: List[Scad2D] = scad2ds[:]  # Make a copy

    # Module2D.__str__():
    def __str__(self) -> str:  # pragma: no cover
        """Return Module2D as a string."""
        # Grab some values from *module2d* (i.e. *self*):
        module2d: Module2D = self
        is_operator: bool = module2d.is_operator
        locked: bool = module2d.locked
        name: str = module2d.name
        return f"Module2D('{name}',[...],is_operator={is_operator},lock={locked})"

    # Module2D.__getitem__():
    def __getitem__(self, index: int) -> Scad2D:
        """Return a SCAD2 item from a Module2D."""
        # Grab some values from *module2d* (i.e. *self*):
        module2d: Module2D = self
        scad2ds: List[Scad2D] = module2d.scad2ds

        # Ensure that *index* is in bounds:
        scad2ds_size: int = len(scad2ds)
        if not (0 <= index < scad2ds_size):
            name: str = module2d.name
            raise IndexError(f"Index {index} exceeds {scad2ds_size} objects in Module2D '{name}'")

        # Fetch *scad2d* and return it:
        scad2d: Scad2D = scad2ds[index]
        return scad2d

    # Module2D.__len__():
    def __len__(self) -> int:
        """Return number of SCAD2D's in Module2D."""
        # Grab some values from *module2d* (i.e. *self*):
        module2d: Module2D = self
        scad2ds: List[Scad2D] = module2d.scad2ds
        scad2ds_size: int = len(scad2ds)
        return scad2ds_size

    # Module2D.append():
    def append(self, scad2d: Scad2D) -> None:
        """Append a Scad2D to a Module2D."""
        # Grab some values from *scad2d* (i.e. *self*):
        module2d: Module2D = self
        locked: bool = module2d.locked
        scad2ds: List[Scad2D] = module2d.scad2ds

        # Make sure that we are not *locked*:
        if locked:
            name: str = module2d.name
            raise ValueError(f"Can not append to Module2D '{name}' because is locked")

        # Perform the *append*:
        scad2ds.append(scad2d)

    # Module2D.extend():
    def extend(self, new_scad2ds: List[Scad2D]) -> None:
        """Append a Scad2D to a Module2D."""
        # Grab some values from *scad2d* (i.e. *self*):
        module2d: Module2D = self
        locked: bool = module2d.locked
        scad2ds: List[Scad2D] = module2d.scad2ds

        # Make sure that we are not *locked*:
        if locked:
            name: str = module2d.name
            raise ValueError(f"Can not extend Module2D '{name}' because is locked")

        # Perform the *append*:
        scad2ds.extend(new_scad2ds)

    # Module2d.lock():
    def lock(self):
        """Ensure that Module2D is locked."""
        module2d: Module2D = self
        module2d.locked = True

    # Module2D.scad_lines_append():
    def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:
        """Append Circle to lines list.

        Args:
            *scad_lines* (*List*[*str*]): The lines list to append the
                *circle* (i.e. *self*) to.
            *indent* (*str*): The indentatation prefix for each line.

        """
        # Grab some values from *module2d* (i.e. *self*):
        module2d: Module2D = self
        locked: bool = module2d.locked
        name: str = module2d.name
        scad2ds: List[Scad2D] = module2d.scad2ds

        # Make sure that we are *locked*:
        if not locked:
            raise ValueError(f"Module2D '{name}' is not locked yet.")

        # Output the module name defintition:
        scad_lines.append(f"{indent}module {name.replace(' ', '_')}() {{")

        # Output the *scad2d*s:
        next_indent: str = indent + " "
        scad2d: Scad2D
        for scad2d in scad2ds:
            scad2d.scad_lines_append(scad_lines, next_indent)

        # Output the closing '}':
        scad_lines.append(f"{indent}}}")


# Polygon:
class Polygon(Scad2D):
    """Represents an OpenScad `polygon` command."""

    # Polygon.__init__():
    def __init__(self, name: str, simple_polygons: "List[SimplePolygon]",
                 convexity: int = -1, lock=True) -> None:
        """Initialize an OpenSCAD polygon command.

        Initialze a *Polygon* object to initially contain a list of
        *simple_polygons.*  If *locked* is *True*, no additional
        *SimplePolygon*'s can be append to *polygon* (i.e. *self*);
        otherwise both the *Polygon*.*append*() and the
        *Polygon*.*extend*() methods can be used to append additional
        *SimplePolygon*'s.  The *Polygon*.*lock() forces *polygon*
        to be locked and it can not be unlocked afterwards.


        Args:
            *name*: (*str*): The name of OpenSCAD polygon command.
            *simple_polygons* (*List*[*SimplePolygon*]): A list of
                *SimplePolygon*'s to install into *polygon*
                (i.e. *self*).  Each of these *SimplePolygon*"s must
                be locked.
            *convexity* (*int*): A number to estimate the complexit
                of the polygon.  Higher numbers are needed for
                accurate complex polygon rendering.  If no value is
                provided, a resonable default is provided.
            *lock* (*bool*): If *True* the initialized *polygon*
                object (i.e. *self*) is locked from having additional
                *SimplePolygon*'s appended to it; other no additional
                *SimplePolygon*'s can be appended.

        Raises:
            *ValueError*(*str*): if any of the *SimplePolygon*'s in
                *simple_polygons* are not locked .

        """
        # Valid that all of the *simple_polygons* are locked:
        simple_polygon_index: int
        simple_polygon: SimplePolygon
        for simple_polygon_index, simple_polygon in enumerate(simple_polygons):
            if not simple_polygon.is_locked():
                raise ValueError(f"SimplePolygon ('{simple_polygon.name}') "
                                 f"at index {simple_polygon_index} is not locked.")

        # Intilize the base class and stuff values into *scad_polygon* (i.e. *self*):
        super().__init__(name)
        self.locked: bool = lock
        self.convexity: int = convexity
        self.simple_polygons: List[SimplePolygon] = simple_polygons[:]

    # Polygon.__getitem__():
    def __getitem__(self, index: int) -> "SimplePolygon":
        """Return the selected Polygon.

        Args:
            *index* (*int*): The index into the *polygon*
                (i.e. *self*) *Polygon*'s list to fetch.

        Returns:
            (*SimplePolygon*) Returns the selected *SimplePolygon*:

        """
        # Grab some values from *polygon* (i.e. *self*):
        polygon: Polygon = self
        simple_polygons: List[SimplePolygon] = polygon.simple_polygons
        simple_polygons_size: int = len(simple_polygons)
        if index < 0 or index >= simple_polygons_size:
            raise IndexError(f"index={index} and it is not in range 0:{simple_polygons_size-1}")
        simple_polygon: SimplePolygon = simple_polygons[index]
        return simple_polygon

    # Polygon.__len__()
    def __len__(self):
        """Return the number of SimplePolygon's in the Polygon.

        Returns:
            (*int*) Returns the number of *SimplePolygon*'s in *polygon*
                (i.e. *self*.)

        """
        # Grab the *polygons* from *scad_polygon* (i.e. *self*):
        polygon: polygon = self
        simple_polygons: List[SimplePolygon] = polygon.simple_polygons
        simple_polygons_size: int = len(simple_polygons)
        return simple_polygons_size

    # Polygon.__str__():
    def __str__(self) -> str:
        """Return string for *Polygon."""
        # Grab *name* from *polygon* (i.e. *self*) and return formatted string:
        polygon: Polygon = self
        name: str = polygon.name
        simple_polygons: List[SimplePolygon] = polygon.simple_polygons
        simple_polygons_size: int = len(simple_polygons)
        convexity: int = polygon.convexity
        return (f"Polygon('{name}',"
                f"len(simple_polygons)={simple_polygons_size},"
                f"convexity={convexity})")

    # Polygon.append():
    def append(self, simple_polygon: "SimplePolygon") -> None:
        """Append a SimplePolygon to the Polygon.

        Args:
            *simple_polygon*: (*SimplePolygon*): The *SimplePolygon*
                to append to *polygon* (i.e. *self*.)

        """
        # Grab some values from *polygon* (i.e. *self*):
        polygon: Polygon = self
        locked: bool = polygon.locked
        if locked:
            raise ValueError(f"Polygon '{polygon.name}' is locked and can not be appended to.")
        simple_polygons: List[SimplePolygon] = polygon.simple_polygons
        simple_polygons.append(simple_polygon)

    # Polygon.extend():
    def extend(self, additional_simple_polygons: "List[SimplePolygon]") -> None:
        """Append a list of SimplePolygon's to the Polygon.

        Args:
            *additional_simple_polygons*: (*List*[*SimplePolygon*]):
                The list of *SimplePolygon*'s  to append to
                *scad_polygon* (i.e. *self*.)

        """
        # Grab some values from *polygon* (i.e. *self*):
        polygon: Polygon = self
        locked: bool = polygon.locked
        if locked:
            raise ValueError(f"Polygon '{polygon.name}' is locked and can not be extended.")
        simple_polygons: List[SimplePolygon] = polygon.simple_polygons
        simple_polygons.extend(additional_simple_polygons)

    # Polygon.lock():
    def lock(self):
        """Lock Polygon from further expansion."""
        polygon: Polygon = self
        polygon.locked = True

    # Polygon.scad_lines_append():
    def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:
        """Append Polygon commands to a lines list.

        Args:
            *scad_lines* (*List*[*str*]): The lines list to append the
                *scad_polygon* (i.e. *self*) to.
            *indent* (*str*): The indentatation prefix for each line.

        """
        polygon: Polygon = self
        simple_polygons: List[SimplePolygon] = polygon.simple_polygons
        polygon.polygon_scad_lines_append(simple_polygons, scad_lines, indent)

    # Polygon.simple_polygons_get():
    def simple_polygons_get(self) -> "List[SimplePolygon]":
        """Return current list of Simple Polygons."""
        polygon: Polygon = self
        simple_polygons: List[SimplePolygon] = polygon.simple_polygons
        # Make a copy and return the result:
        simple_polygons = simple_polygons[:]
        return simple_polygons


# SimplePolygon:
class SimplePolygon(Scad2D):
    """Represents a simple closed polygon of points."""

    # SimplePolygon.__init__():
    def __init__(self, name: str, points: List[P2D] = [],
                 lock: bool = False, convexity: int = -1) -> None:
        """Initialize a SimplePolygon.

        Args:
            *name* (*str*): The name for the polygon.
            *points* (*List*[*P2D*]): The list of points to initialize
                *close_polygon* (i.e. *self* with.)
            *lock* (*bool*): If *True* no additional points can be
                appended to *simple_polygon* (*i.e. *self*); othewise
                additional points can be appended.
            *convexity* (*int*): Specifies the complexity of the
                polygon.  Larger numbers are needed render ever
                complex polygons.  The default is -1, which cause
                a reasonable initial guess to occur.

        """
        # Stuff values into *simpl_polygon* (i.e. *self*):
        # simple_polygon: SimplePolygon = self
        self.locked: bool = lock
        self.name: str = name
        self.points: List[P2D] = points[:]  # Copy the contents of *points*
        self.convexity: int = 4 if convexity <= 0 else convexity

    # SimplePolygon.__getitem__():
    def __getitem__(self, index: int) -> P2D:
        """Fetch a point from the SimplePolygon.

        Args:
            *index* (*int*): Index into the *simple_polygon*
                (i.e. *self*) points list.

        Returns:
            (*P2D*): Return the *index*'th *P2D* from *simple_polygon*.

        """
        # Grab some values from *simple_polygon* (i.e. *self*):
        simple_polygon: SimplePolygon = self
        points: List[P2D] = simple_polygon.points
        points_size: int = len(points)
        if index < 0 or index >= points_size:
            raise IndexError(f"index of {index} is not in range 0 to {points_size-1}")
        point: P2D = points[index]
        return point

    # SimplePolygon.__len__():
    def __len__(self) -> int:
        """Return the number of points currently in the Polygon."""
        # Grab some values from *simple_polygon* (i.e. *self*):
        simple_polygon: SimplePolygon = self
        points: List[P2D] = simple_polygon.points
        size: int = len(points)
        return size

    # SimplePolygon.__str__():
    def __str__(self):
        """Return a short string representation of a Polygon."""
        # Grab some values from *simplepolygon* (i.e. *self*):
        simple_polygon: SimplePolygon = self
        name: str = simple_polygon.name
        points: List[P2D] = simple_polygon.points
        selected_points: List[P2D] = points if len(points) <= 2 else [points[0]] + [points[-1]]
        join_text: str = ", " if len(points) <= 2 else ", ..., "
        selected_point: P2D
        selected_point_texts: List[str] = [f"{selected_point}"
                                           for selected_point in selected_points]
        selected_points_text = join_text.join(selected_point_texts)
        return f"SimplePolygon('{name}', [{selected_points_text}])"

    # SimplePolygon.arc():
    def arc_append(self, center: P2D, radius: float, start_angle: float, end_angle: float,
                   points_count: int) -> None:
        """Append an arc of points to a Polygon.

        Args:
            *origin* (*P*): The center of the arc.
            *radius* (*float*): The arc radius.
            *start_angle* (*float*): The starting angle for the arc.
            *end_angle* (*float*): The ending angle for the arc.
            *points_count* (*int*): The number of points along the arc.

        """
        # Grab some values from *simple_polygon* (i.e. *self*):
        simple_polygon: SimplePolygon = self
        points: List[P2D] = simple_polygon.points

        # Compute the total angle spanned and the delta angle increments:
        span_angle: float = end_angle - start_angle
        delta_angle: float = span_angle / float(points_count - 1)
        # print(f"start_angle={start_angle}={degrees(start_angle)}deg")
        # print(f"end_angle={end_angle}={degrees(end_angle}deg")
        # print(f"span_angle={span_angle}={degrees(span_angle)}deg")
        # print(f"delta_angle={delta_angle}={degrees(delta_angle}deg")
        center_x: float = center.x
        center_y: float = center.y
        index: int
        for index in range(points_count):
            angle: float = start_angle + index * delta_angle
            x: float = center_x + radius * cos(angle)
            y: float = center_y + radius * sin(angle)
            # print(f"[{index}]angle={degrees(angle} x={x} y={y}")
            points.append(P2D(x, y))

    # SimplePolygon.is_locked():
    def is_locked(self) -> bool:
        """Return whether SimplePolygon is locked or not."""
        # Grab *locked* flag from *simple_polygon* (i.e. *self*) and return it:
        simple_polygon: SimplePolygon = self
        locked: bool = simple_polygon.locked
        return locked

    # SimplePolygon.key():
    def key(self) -> Tuple[Any, ...]:
        """Return a key for *simple_polygon*."""
        simple_polygon: SimplePolygon = self
        name: str = simple_polygon.name
        points: List[P2D] = simple_polygon.points
        assert len(points) >= 1
        point0: P2D = points[0]
        x: float = point0.x
        y: float = point0.y
        x_maximum: float = x
        x_minimum: float = x
        y_maximum: float = y
        y_minimum: float = y
        point: P2D
        for point in points:
            x = point.x
            y = point.y
            x_maximum = max(x_maximum, x)
            x_minimum = min(x_minimum, x)
            y_maximum = max(y_maximum, y)
            y_minimum = min(y_minimum, y)
        dx: float = x_maximum - x_minimum
        dy: float = y_maximum - y_minimum
        x_center: float = (x_maximum + x_minimum) / 2.0
        y_center: float = (y_maximum + y_minimum) / 2.0
        key: Tuple[Any, ...] = ("SimplePolygon", name, x_center, y_center, dx, dy, 0.0)
        return key

    # SimplePolygon.lock():
    def lock(self) -> None:
        """Force SimplePolygon to be locked."""
        simple_polygon: SimplePolygon = self
        simple_polygon.locked = True

    # SimplePolygon.point_append():
    def point_append(self, point: P2D) -> None:
        """Append a point to a SimplePolygon.

        Args:
            *point* (*P2D*): The 2-dimensional point to the
            to *simple_polygon* (i.e. *self*.)

        Raises:
            *ValueError*(*str*): if *simple_polygon* (i.e. *self*.)
            is locked.

        """
        # Grab *points* from *simple_polygon* (i.e. *self*) and tack *point* onto the end:
        simple_polygon: SimplePolygon = self
        points: List[P2D] = simple_polygon.points
        points.append(point)

    # SimplePolygon.points_extend():
    def points_extend(self, new_points: List[P2D]) -> None:
        """Append a list of points to a SimplePolygon.

        Args:
            *new_points* (*List*[*P2D*]): A list of points to append.

        Raises:
            *ValueError*(*str*): if *simple_polygon* (i.e. *self*.)
            is locked.

        """
        # Grab *points* from *simple_polygon* (i.e. *self*) and tack *new_points* onto the end:
        simple_polygon: SimplePolygon = self
        points: List[P2D] = simple_polygon.points
        new_point: P2D
        for new_point in new_points:
            points.append(new_point)

    # Scad.points_get():
    def points_get(self) -> List[P2D]:
        """Return the points associated with SimplePolygon."""
        simple_polygon: SimplePolygon = self
        points: List[P2D] = simple_polygon.points
        # Make a copy:
        points = list(points[:])
        return points

    # SimplePolygon.points_rotate():
    def points_rotate(self, angle: float, center: P2D):
        """Rotate all SimplePolygon points by an angle."""
        # Grab some values from *simple_polygon* (i.e. *self*):
        simple_polygon: SimplePolygon = self
        locked: bool = simple_polygon.locked
        points: List[P2D] = simple_polygon.points
        if locked:
            raise ValueError(f"'{simple_polygon.name}' is locked")
        # For some reason, test coverage complains about the two variable declarations
        # even thought the following code is covered.  Weird!!
        index: int  # pragma: no cover
        point: P2D  # pragma: no cover
        for index, point in enumerate(points):
            points[index] = point.rotate(angle, center)

    # SimplePolygon.points_scad_lines_append():
    def points_scad_lines_append(self, scad_lines: List[str], indent: str, start_index: int) -> int:
        """Append the Polygon points to a list of lines.

        Args:
            *scad_lines* (*List*[*str*]): The list of OpenSCAD lines to
                append to.
            *indent (*str): The indentation text to prefix to each line.
            *start_index* (*int*): The starting index for points.

        Returns:
            (*int*) Returns the *end_index* after the points have been
                output.

        """
        # Grab some values from *simple_polygon* (i.e. *self*):
        simple_polygon: SimplePolygon = self
        name: str = simple_polygon.name
        points: List[P2D] = simple_polygon.points

        # Compute *end_index* from *start_index* and *points_size*:
        points_size: int = len(points)
        end_index: int = start_index + points_size

        # Figure out the number of *slice_points* to output:
        slice_size: int = 4
        slices_count: int = int(ceil(float(points_size) / float(slice_size)))

        # Append a debugging line:
        scad_lines.append(f"{indent} // Polygon '{name}' {start_index}:{end_index-1}")

        # Sweep through *points* and output chunks of *slice_points*:
        slice_index: int
        for slice_index in range(slices_count):
            # Extract the next chunk of *slice_points*:
            slice_start: int = slice_index * slice_size
            slice_end: int = min((slice_index + 1) * slice_size, points_size)
            slice_points: List[P2D] = points[slice_start:slice_end]

            # Just to be paranoid, make sure we actually have at least one point:
            if slice_points:
                # Splice *splice_point* together as a comma separated list:
                point_texts: List[str] = []
                slice_point: P2D
                for slice_point in slice_points:
                    x_text: str = "{0:.3f}".format(slice_point.x)
                    y_text: str = "{0:.3f}".format(slice_point.y)
                    x_text = "0.000" if x_text == "-0.000" else x_text
                    y_text = "0.000" if y_text == "-0.000" else y_text
                    point_texts.append(f"[{x_text}, {y_text}]")
                slice_text: str = ', '.join(point_texts)
                scad_lines.append(f"{indent}  {slice_text}, "
                                  f"// {start_index + slice_start}:"
                                  f"{start_index + slice_end - 1}")
        return end_index

    # SimplePolygon.scad_lines_append():
    def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:
        """TODO."""
        # Grab *class_name* from *simple_polygon* (i.e. *self*):
        simple_polygon: SimplePolygon = self

        # Use the parent *Scad2D*.*scad_lines_append* method to actually ouput the OpenSCAD
        # `polygon` command:
        super().polygon_scad_lines_append([simple_polygon], scad_lines, indent)

    # SimplePolygon.x_mirror():
    def x_mirror(self, name: str, replace: Optional[str] = None) -> "SimplePolygon":
        """Return an X-axis mirrored polygon.

        Args:
            *name* (*str*): The name of the new x mirrored polygon.
                The name is computed differently if *replace* is
                specified.
            *replace* (*str*): (Optional) If *replace* specified,
                the a new name is constructed by taking the name of
                the original *simple_polygon* (i.e. *self*) and
                replacing all occurrances of *name* with *replace*.

        """
        # Grab some values from *simple_polygon* (i.e. *self*):
        simple_polygon: SimplePolygon = self
        simple_polygon_name: str = simple_polygon.name

        points: List[P2D] = simple_polygon.points

        # Compute *new_name* and *x_mirrored_points*:
        new_name: str = (name if replace is None
                         else simple_polygon_name.replace(name, replace))
        point: P2D
        x_mirrored_points: List[P2D] = [P2D(point.x, -point.y) for point in points]

        # Construct the final *x_mirrored_simple_polygon* and return it.
        x_mirrored_simple_polygon: SimplePolygon = SimplePolygon(new_name,
                                                                 x_mirrored_points, lock=True)
        return x_mirrored_simple_polygon

    # SimplePolygon.y_mirror():
    def y_mirror(self, name: str, replace: Optional[str] = None) -> "SimplePolygon":
        """Return a Y-axis mirrored polygon.

        Args:
            *name* (*str*): The name of the new x mirrored polygon.
                The name is computed differently if *replace* is
                specified.
            *replace* (*str*): (Optional) If *replace* specified,
                the a new name is constructed by taking the name of
                the original *simple_polygon* (i.e. *self*) and
                replacing all occurrances of *name* with *replace*.

        """
        # Grab some values from *simple_polygon* (i.e. *self*):
        simple_polygon: SimplePolygon = self
        simple_polygon_name: str = simple_polygon.name
        points: List[P2D] = simple_polygon.points

        # Compute *new_name* and *y_mirrored_points*:
        new_name: str = (name if replace is None
                         else simple_polygon_name.replace(name, replace))
        point: P2D
        y_mirrored_points: List[P2D] = [P2D(-point.x, point.y) for point in points]

        # Construct the final *y_mirrored_simple_polygon* and return it.
        y_mirrored_simple_polygon: SimplePolygon = SimplePolygon(new_name,
                                                                 y_mirrored_points, lock=True)
        return y_mirrored_simple_polygon


# Circle:
class Circle(SimplePolygon):
    """Represents a circular SimplePolygon."""

    # Circle.__init__():
    def __init__(self, name: str, diameter: float, points_count: int,
                 center: P2D = P2D(0.0, 0.0)) -> None:
        """Create a circular SimplePolygon.

        Args:
            *name* (*str*): The debugging text name to output to the
                `.scad` file
            *diameter* (*float*): The diameter of the circle.,
            *points_count* (*int*): The number of points to approixmate
                the circle with.
            *center* (*P2D*): The center of the circle.  This defaults
                to the origin (i.e. *P2D*(0.0, 0.0).)

        """
        # Create the *radius *of *circle_points* list of *P2D*'s centered around *center*:
        center_x: float = center.x
        center_y: float = center.y
        radius: float = diameter / 2.0
        circle_points: List[P2D] = []
        delta_angle: float = (2 * pi) / float(points_count)
        point_index: int
        for point_index in range(points_count):
            angle: float = float(point_index) * delta_angle  # Radians
            x: float = center_x + radius * cos(angle)
            y: float = center_y + radius * sin(angle)
            circle_point: P2D = P2D(x, y)
            circle_points.append(circle_point)

        # Initialize the *SimplePolygon* parent class with *name* and *cicular_points*
        # and *lock* it:
        super().__init__(name, circle_points, lock=True)

        # Load values into *circle* (i.e. *self*):
        # circle: Circle = self
        self.center: P2D = center
        self.diameter: float = diameter
        self.points_count: int = points_count
        self.convexty: int = 4

    # Circle.__str__():
    def __str__(self) -> str:
        """Return a string representation of Circle*."""
        # Grab some values from *Circle* (i.e. *self*):
        circle: Circle = self
        center: P2D = circle.center
        convexity: int = circle.convexity
        diameter: float = circle.diameter
        name: str = circle.name
        points_count: int = circle.points_count

        # Return the formatted string reprentation:
        return f"Circle('{name}',{diameter},{points_count},{center},{convexity})"

    # Circle.copy():
    def copy(self, new_name: str, diameter: Optional[float] = None,
             points_count: Optional[int] = None, center: Optional[P2D] = None,
             replace: Optional[str] = None) -> "Circle":
        """Copy a circle replacing various values during copy.

        Returns a copy of *circle* (i.e. *self*) replacing any
        specified values.  A new *name* must be specified.  If
        *replace* is spacified, the new name is constructed by
        replacing all occurances of *name* in the original *circle*
        (i.e. *self*) name with *replace*.

        Args:
            *new_name* (*str*): The name of the new *Circle* object.
                This is modified if the *replace* argument is present
                (see below.)
            *diameter* (*float*): (Optional) The replacement value for
                the  circle diameter.
            *center* (*P2D*): (Optional) The replacement value for the
                rectangle center.
            *replace* (*str*): (Optional) If present, all occurances
                of *new_name* in the original *circle* (i.e. *self*)
                name are replaced with *replace*.

        Returns:
            (*Circle*): Returns a new *Circle* with the appropriate
                values updated.

        """
        # Grab some values from *circle* (i.e. *self*) or use the one new ones present from
        # the arguments:
        circle: Circle = self
        name: str = circle.name
        diameter = circle.diameter if diameter is None else diameter
        center = circle.center if center is None else center
        points_count = circle.points_count if points_count is None else points_count
        new_name = new_name if replace is None else name.replace(new_name, replace)

        # Create and return the *new_circle*:
        new_circle: Circle = Circle(new_name, diameter, points_count, center)
        return new_circle

    # Circle.key():
    def key(self) -> Tuple[Any]:
        """Return an immutable sorting key for a Circle."""
        # Grab some values from *circle* (i.e. *self*):
        circle: Circle = self
        center: P2D = circle.center
        diameter: float = circle.diameter
        name: str = circle.name

        # (TYPE, NAME, CENTER_X, CENTER_Y, DX, DY, ROTATE):
        key: Any[Tuple] = ("Circle", name, center.x, center.y, diameter, diameter, 0.0)
        return key

    # Circle.scad_lines_append():
    def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:
        """Append Circle to lines list.

        Args:
            *scad_lines* (*List*[*str*]): The lines list to append the
                *circle* (i.e. *self*) to.
            *indent* (*str*): The indentatation prefix for each line.

        """
        # Grab some values from *circle* (i.e. *self*):
        circle: Circle = self
        center: P2D = circle.center
        diameter: float = circle.diameter
        points_count: int = circle.points_count
        name: str = circle.name

        # Derives some values:
        center_x: float = center.x
        center_y: float = center.y
        float_format: Callable[[float], str] = Scad.float_format
        circle_indent: str = indent
        if abs(center_x) != 0.0 or abs(center_y) != 0.0:
            # We have to output a translate transform first:
            scad_lines.append(f"{indent}translate([{float_format(center_x)}, "
                              f"{float_format(center_y)}])")
            circle_indent += " "
        scad_lines.append(f"{circle_indent}circle(d={float_format(diameter)}, "
                          f"$fn={points_count});  // Circle '{name}'")

    # Circle.x_mirror():
    def x_mirror(self, new_name: str, replace: Optional[str] = None) -> "Circle":
        """Return an X-axis mirrored Circle.

        Args:
            *new_name* (*str*): The new name of the new x mirrored
                Circle.  The new name is computed differently if
                *replace* is specified (see immediately below.)
            *replace* (*str*): (Optional) If *replace* specified,
                the a new name is constructed by taking the name of
                the original *simple_polygon* (i.e. *self*) and
                replacing all occurrances of *new_name* with *replace*.

        """
        # Grab some values from *circle* (i.e. *self*):
        circle: Circle = self
        center: P2D = circle.center
        diameter: float = circle.diameter
        name: str = circle.name
        points_count: int = circle.points_count

        # Compute *final_name*:
        final_name: str = (new_name if replace is None
                           else name.replace(new_name, replace))

        # Construct the final *x_mirrored_simple_polygon* and return it.
        new_center: P2D = P2D(center.x, -center.y)
        x_mirrored_circle: Circle = Circle(final_name, diameter, points_count, new_center)
        return x_mirrored_circle

    # Circle.y_mirror():
    def y_mirror(self, new_name: str, replace: Optional[str] = None) -> "Circle":
        """Return an y-axis mirrored Circle.

        Args:
            *new_name* (*str*): The new name of the new y mirrored
                Circle.  The new name is computed differently if
                *replace* is specified (see immediately below.)
            *replace* (*str*): (Optional) If *replace* specified,
                the a new name is constructed by taking the name of
                the original *simple_polygon* (i.e. *self*) and
                replacing all occurrances of *new_name* with *replace*.

        """
        # Grab some values from *circle* (i.e. *self*):
        circle: Circle = self
        center: P2D = circle.center
        diameter: float = circle.diameter
        name: str = circle.name
        points_count: int = circle.points_count

        # Compute *final_name*:
        final_name: str = (new_name if replace is None
                           else name.replace(new_name, replace))

        # Construct the final *x_mirrored_simple_polygon* and return it.
        new_center: P2D = P2D(-center.x, center.y)
        y_mirrored_circle: Circle = Circle(final_name, diameter, points_count, new_center)
        return y_mirrored_circle


# If2D:
class If2D(Scad2D):
    """Represents a Scad2D if-then-else statement chain."""

    # If2D.__init__():
    def __init__(self, name: str, then_expression: str,
                 then_scad2ds: List[Scad2D], lock=False) -> None:
        """Initialize an If2D object."""
        # Initialize the *SCAD2D* parent class:
        super().__init__(name)
        # Load values into *If2D* (i.e. *self*):
        # if2d: If2D = self
        then_clause: Tuple[str, List[Scad2D]] = (then_expression, then_scad2ds[:])
        self.then_clauses: List[Tuple[str, List[Scad2D]]] = [then_clause]
        self.else_scad2ds: Optional[List[Scad2D]] = None
        self.locked: bool = lock
        self.named_mark_downs: List[Tuple[str, ...]] = []

    # If2D.__str__():
    def __str__(self) -> str:
        """Convert an If2D into a string."""
        # Grab some values from *if2d* (i.e. *self*):
        if2d: If2D = self
        name: str = if2d.name
        locked: bool = if2d.locked
        return f"If2D('{name}',...,lock={locked})"

    # If2D.else_set():
    def else_set(self, new_else_scad2ds: List[Scad2D]) -> None:
        """Set the final else caluse for an If2D."""
        # Grab some values from *if2d* (i.e. *self*):
        if2d: If2D = self
        else_scad2ds: Optional[List[Scad2D]] = if2d.else_scad2ds
        locked: bool = if2d.locked
        name: str = if2d.name
        if locked:
            raise ValueError(f"If2D('{name})' is locked and can not have an else clause set.")
        elif else_scad2ds is not None:
            raise ValueError(f"If2D('{name})' else clause is already set.")
        else:
            if2d.else_scad2ds = new_else_scad2ds[:]

    # If2D.lock():
    def lock(self):
        """Ensure that an If2D is locked."""
        if2d: If2D = self
        if2d.locked = True

    # If2D.name_match:
    def name_match_append(self, name: str, module2d: Module2D, mark_down: List[str]) -> None:
        """Append a then clause for mataching a name."""
        # Append a then clause to *if2d* (i.e. *self*) and remember the *mark_down*:
        if2d: If2D = self
        if2d.then_append(f'name == "{name}"', [UseModule2D(f"{name} Use Module", module2d)])
        named_mark_down: Tuple[str, ...] = (name,) + tuple(mark_down)
        if2d.named_mark_downs.append(named_mark_down)

    # If2D.scad_lines_append():
    def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:
        """Append If2D to a list of lines.

        Args:
            *scad_lines* (*List*[*str*]): The lines list to append the
                *square* (i.e. *self*) to.
            *indent* (*str*): The indentatation prefix for each line.

        """
        # Grab some values from *if2d* (i.e. *self*):
        if2d: If2D = self
        else_scad2ds: Optional[List[Scad2D]] = if2d.else_scad2ds
        name: str = if2d.name
        then_clauses: List[Tuple[str, List[Scad2D]]] = if2d.then_clauses
        locked: bool = if2d.locked
        if not locked:
            raise ValueError(f"If2D '{name}' is not locked.")

        next_indent: str = indent + " "
        then_clause: Tuple[str, List[Scad2D]]
        then_index: int
        for then_index, then_clause in enumerate(then_clauses):
            then_expression: str = then_clause[0]
            then_scad2ds: List[Scad2D] = then_clause[1]
            comment_text = f"  // If2D '{name}'" if then_index == 0 else ""
            if_text: str = "if" if then_index == 0 else "} else if"
            scad_lines.append(f"{indent}{if_text} ({then_expression}) {{{comment_text}")
            then_scad2d: Scad2D
            for then_scad2d in then_scad2ds:
                then_scad2d.scad_lines_append(scad_lines, next_indent)
        if else_scad2ds is not None:
            scad_lines.append(f"{indent}}} else {{")
            else_scad2d: Scad2D
            for else_scad2d in else_scad2ds:
                else_scad2d.scad_lines_append(scad_lines, next_indent)
        scad_lines.append(f"{indent}}}  // End If2D '{name}'")

    # If2D.then_append():
    def then_append(self, else_if_expression: str, else_if_scad2ds: List[Scad2D]) -> None:
        """Append a then clause to an If2D."""
        # Grab some values from *if2d* (i.e. *self*):
        if2d: If2D = self
        name: str = if2d.name
        then_clauses: List[Tuple[str, List[Scad2D]]] = if2d.then_clauses
        locked: bool = if2d.locked
        if locked:
            raise ValueError(f"If2D '{name}' is and locked can not accept another then clause")
        then_clause: Tuple[str, List[Scad2D]] = (else_if_expression, else_if_scad2ds)
        then_clauses.append(then_clause)


# Square:
class Square(SimplePolygon):
    """Represents a rectangular SimplePolygon."""

    # Square.__init__():
    def __init__(self, name: str, dx: float, dy: float, center: P2D = P2D(0.0, 0.0),
                 rotate: float = 0.0, corner_radius: float = 0.0, corner_count: int = 3) -> None:
        """Create a translated/rotated rectangular SimplePolygon.

        Args:
            *name* (*str*): The debugging text name to output to the
                `.scad` file.
            *dx* (*float*): The X dimension of the initial rectangle.
            *dy* (*float*): The Y dimension of the initial rectangle.
            *center* (*P2D*): The center of the rectangle.  This defaults
                to the origin (i.e. *P2D*(0.0, 0.0).)
            *rotate* (*float*): The amount to rotate the rectangle about
                its center (in radians.)  This defaults to 0.0.
            *corner_radius* (*float*): The amount to round all 4 corners by.
                This defaults to 0.0
            *corner_count* (*int*): The number of points on along corner
                arc excluding the arc end-points.  This defaults to 3.

        """
        # Compute some intermediate values:
        center_x: float = center.x
        center_y: float = center.y
        half_dx: float = dx / 2.0
        half_dy: float = dy / 2.0
        half_pi: float = pi / 2.0

        # Do some argument validation:
        float_format: Callable[[float], str] = Scad.float_format
        if dx <= 0.0:
            raise ValueError(f"dx={float_format(dx)} is not positive for '{name}'")
        if dy <= 0.0:
            raise ValueError(f"dy={float_format(dy)} is not positive for '{name}'")
        if corner_radius < 0.0:
            raise ValueError(f"corner_radius={float_format(corner_radius)} "
                             f"must be non-negative for '{name}'")
        if corner_radius > 0.0 and corner_count < 0:
            raise ValueError(f"corner_count={corner_count} must be non-negative for '{name}'")
        half_dx_dy_minimum: float = min(dx, dy) / 2.0
        if corner_radius > half_dx_dy_minimum:
            raise ValueError(f"corner radius={float_format(corner_radius)} is larger than half of"
                             f" min({float_format(dx)}, {float_format(dy)})/2.0 for '{name}'")
        if half_dx == half_dy == corner_radius:
            raise ValueError(f"dx/2={float_format(half_dx)}, dy/2={float_format(half_dy)}, "
                             f"corner_radius={float_format(corner_radius)}; use Circle instead!")

        # Initialize the *SimplePolygon* parent class:
        super().__init__(name, [])

        # Fill in *square* based on *dx*, *dy*, *corner_radius* and *corner_count*
        # with *center_x* and *center_y* offseting:
        square: Square = self
        if corner_radius <= 0.0:
            # No *corner_radius*, so we cand do simple 4 point *square* (i.e. *self*):
            square.point_append(P2D(center_x + half_dx, center_y + half_dy))
            square.point_append(P2D(center_x + half_dx, center_y - half_dy))
            square.point_append(P2D(center_x - half_dx, center_y - half_dy))
            square.point_append(P2D(center_x - half_dx, center_y + half_dy))
        elif corner_radius == half_dx_dy_minimum:
            # A *square* (i.e. *self*) rectangle with fully rounded corners:
            if dx < dy:
                # The rounded ends are on the top and bottom:
                upper_center: P2D = P2D(center_x, center_y + half_dy - half_dx)
                lower_center: P2D = P2D(center_x, center_y + half_dx - half_dy)
                square.arc_append(upper_center, corner_radius, 0.0, pi, 2 * corner_count + 3)
                square.arc_append(lower_center, corner_radius, pi, 2 * pi, 2 * corner_count + 3)
            elif dy < dx:
                # The rounded ends are on the left and right:
                right_center: P2D = P2D(center_x + half_dx - half_dy, center_y)
                left_center: P2D = P2D(center_x + half_dy - half_dx, center_y)
                square.arc_append(right_center, corner_radius,
                                  -half_pi, half_pi, 2 * corner_count + 3)
                square.arc_append(left_center, corner_radius,
                                  half_pi, 3 * half_pi, 2 * corner_count + 3)
            else:  # pragma: no cover
                assert False, "This should not be possible"
        elif 0.0 < corner_radius < half_dx_dy_minimum:
            # A significantly more complicated *Rectangle* with 4 rounded corners:
            corner_center_dx: float = half_dx - corner_radius
            corner_center_dy: float = half_dy - corner_radius
            upper_right_center: P2D = P2D(center_x + corner_center_dx, center_y + corner_center_dy)
            upper_left_center: P2D = P2D(center_x - corner_center_dx, center_y + corner_center_dy)
            lower_left_center: P2D = P2D(center_x - corner_center_dx, center_y - corner_center_dy)
            lower_right_center: P2D = P2D(center_x + corner_center_dx, center_y - corner_center_dy)
            square.arc_append(upper_right_center, corner_radius, 0.0, half_pi, corner_count + 2)
            square.arc_append(upper_left_center, corner_radius, half_pi, pi, corner_count + 2)
            square.arc_append(lower_left_center, corner_radius, pi, 3 * half_pi, corner_count + 2)
            square.arc_append(lower_right_center, corner_radius,
                              3 * half_pi, 2 * pi, corner_count + 2)
        else:  # pragma: no cover
            assert False, "Problem with corner_radius; this should not happen."

        # If *rotate* is non-zero, we replace *square_points* with a verision where each
        # point is rotated by *rotate* around *center*:
        if rotate != 0.0:
            square.points_rotate(rotate, center)

        # Load values into *square* (i.e. *self*) and *lock* it:
        self.center: P2D = center
        self.corner_count: int = corner_count
        self.corner_radius: float = corner_radius
        self.dx: float = dx
        self.dy: float = dy
        self.rotate: float = rotate
        square.lock()

    # Square.__str__():
    def __str__(self) -> str:
        """Return a string representation of Circle*."""
        # Grab some values from *square* (i.e. *self*):
        square: Square = self
        center: P2D = square.center
        corner_count: int = square.corner_count
        corner_radius: float = square.corner_radius
        dx: float = square.dx
        dy: float = square.dy
        name: str = square.name
        rotate: float = square.rotate

        # Only provide *center_text*, *rotate_text*, and *corner_text* if appropriate:
        float_format: Callable[[float], str] = Scad.float_format
        center_text: str = "" if center.x == 0.0 and center.y == 0.0 else f",center={center}"
        rotate_text: str = "" if rotate == 0.0 else f",rotate={float_format(degrees(rotate))}deg"
        corner_radius_text: str = ("" if corner_radius <= 0.0
                                   else f",corner_radius={float_format(corner_radius)}")
        corner_count_text: str = ("" if corner_radius <= 0.0 or corner_count == 3
                                  else f",corner_count={corner_count}")

        # Return the formatted string reprentation:
        return (f"Square('{name}',{float_format(dx)},{float_format(dy)}"
                f"{center_text}{rotate_text}{corner_radius_text}{corner_count_text})")

    # Square.copy():
    def copy(self, name: str, dx: Optional[float] = None, dy: Optional[float] = None,
             center: Optional[P2D] = None, rotate: Optional[float] = None,
             corner_radius: Optional[float] = None, corner_count: Optional[int] = None,
             replace: Optional[str] = None) -> "Square":
        """Copy a square replacing various values during copy.

        Returns a copy of *square* (i.e. *self*) replacing any
        specified values.  A new *name* must be specified.  If
        *replace* is spacified, the new name is constructed by
        replacing all occurances of *name* in the original *square*
        (i.e. *self*) name with *replace*.

        Args:
            *name* (*str*): The name of the new *Square* object.  This
                is modified if the *replace* argument is present (see
                below.)
            *dx* (*float*): (Optional) The replacement value for the
                rectangle width.
            *dy* (*float*): (Optional) The replacement value for the
                rectangle height.
            *center* (*P2D*): (Optional) The replacement value for the
                rectangle center.
            *rotate* (*float*): (Optional) The replacement rotation
                value (in radians) for the rectangle rotataion.
            *corner_radius* (*float*): (Optional)The replacement value
                to to round all 4 corners by.
            *corner_count* (*int*): (Optional) The replacement number
                points on along corner arc excluding the arc end-points.
            *replace* (*str*): (Optional) If present, all occurances
                of *name* in the original *square* (i.e. *self*) name
                are replaced with *replace*.

        Returns:
            (*Square*): Returns a new *Square* with the appropriate values
                updated.

        """
        # Grab some values from *square* (i.e. *self*) or use the one new ones present from
        # the arguments:
        square: Square = self
        dx = square.dx if dx is None else dx
        dy = square.dx if dy is None else dy
        center = square.center if center is None else center
        rotate = square.rotate if rotate is None else rotate
        corner_radius = square.corner_radius if corner_radius is None else corner_radius
        corner_count = square.corner_count if corner_count is None else corner_count
        name = square.name if replace is None else square.name.replace(name, replace)

        # Create and return the *new_square*:
        new_square: Square = Square(name, dx, dy, center, rotate, corner_radius, corner_count)
        return new_square

    # Square.key():
    def key(self) -> Tuple[Any]:
        """Return an immutable sorting key for a Circle."""
        # Grab some values from *square* (i.e. *self*):
        square: Square = self
        center: P2D = square.center
        corner_count: int = square.corner_count
        corner_radius: float = square.corner_radius
        dx: float = square.dx
        dy: float = square.dy
        name: str = square.name
        rotate: float = square.rotate

        # (TYPE, NAME, CENTER_X, CENTER_Y, DX, DY, ROTATE, CORNER_RADIUS, CORNER_COUNT):
        key: Any[Tuple] = ("Square", name, center.x, center.y, dx, dy, degrees(rotate),
                           corner_radius, corner_count)
        return key

    # Square.scad_lines_append():
    def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:
        """Append Circle to lines list.

        Args:
            *scad_lines* (*List*[*str*]): The lines list to append the
                *square* (i.e. *self*) to.
            *indent* (*str*): The indentatation prefix for each line.

        """
        # Grab some values from *circle* (i.e. *self*):
        # Grab some values from *square* (i.e. *self*):
        square: Square = self
        center: P2D = square.center
        corner_count: float = square.corner_count
        corner_radius: float = square.corner_radius
        dx: float = square.dx
        dy: float = square.dy
        name: str = square.name
        rotate: float = square.rotate

        # Output a debugging line
        float_format: Callable[[float], str] = Scad.float_format
        assert isinstance(scad_lines, list)
        scad_lines.append(f"{indent}// Square '{name}' dx={float_format(dx)} "
                          f"dy={float_format(dy)} center={center} "
                          f"corner_radius={float_format(corner_radius)} "
                          f"corner_count={corner_count}")

        if corner_radius == 0.0:
            # We can use the OpenSCAD `square` command with optional `translate` and
            # `rotate` transforms:
            center_x: float = center.x
            center_y: float = center.y
            square_indent: str = indent
            if center_x != 0.0 and center_y != 0.0:
                scad_lines.append(f"{square_indent}translate([{float_format(center.x)}, "
                                  f"{float_format(center_y)}])")
                square_indent += " "
            if rotate != 0.0:
                scad_lines.append(f"{square_indent}rotate(a = "
                                  f"[0, 0, {float_format(degrees(rotate))}])")
                square_indent += " "
            scad_lines.append(f"{square_indent}square([{float_format(dx)}, {float_format(dy)}], "
                              "center = true);")
        else:
            # Rounded corners need to be done with an OpenSCAD `polygon` command:
            square.polygon_scad_lines_append([square], scad_lines, indent)

    # Square.x_mirror():
    def x_mirror(self, new_name: str, replace: Optional[str] = None) -> "SimplePolygon":
        """Return an X-axis mirrored Square.

        Args:
            *new_name* (*str*): The new name of the new x mirrored
                Square.  The new name is computed differently if
                *replace* is specified (see immediately below.)
            *replace* (*str*): (Optional) If *replace* specified,
                the a new name is constructed by taking the name of
                the original *simple_polygon* (i.e. *self*) and
                replacing all occurrances of *new_name* with *replace*.

        """
        # Grab some values from *square* (i.e. *self*):
        square: Square = self
        center: P2D = square.center
        corner_count: int = square.corner_count
        corner_radius: float = square.corner_radius
        dx: float = square.dx
        dy: float = square.dy
        name: str = square.name
        rotate: float = square.rotate

        # Compute *final_name*:
        final_name: str = (new_name if replace is None
                           else name.replace(new_name, replace))

        # Construct the final *x_mirrored_simple_polygon* and return it.
        new_center: P2D = P2D(center.x, -center.y)
        new_rotate: float = -rotate
        x_mirrored_square: Square = Square(final_name, dx, dy, new_center,
                                           new_rotate, corner_radius, corner_count)
        return x_mirrored_square

    # Square.y_mirror():
    def y_mirror(self, new_name: str, replace: Optional[str] = None) -> "SimplePolygon":
        """Return an y-axis mirrored Square.

        Args:
            *new_name* (*str*): The new name of the new y mirrored
                Square.  The new name is computed differently if
                *replace* is specified (see immediately below.)
            *replace* (*str*): (Optional) If *replace* specified,
                the a new name is constructed by taking the name of
                the original *simple_polygon* (i.e. *self*) and
                replacing all occurrances of *new_name* with *replace*.

        """
        # Grab some values from *square* (i.e. *self*):
        square: Square = self
        center: P2D = square.center
        corner_count: int = square.corner_count
        corner_radius: float = square.corner_radius
        dx: float = square.dx
        dy: float = square.dy
        name: str = square.name
        rotate: float = square.rotate

        # Compute *final_name*:
        final_name: str = (new_name if replace is None
                           else name.replace(new_name, replace))

        # Construct the final *x_mirrored_simple_polygon* and return it.
        new_center: P2D = P2D(-center.x, center.y)
        new_rotate: float = -rotate
        y_mirrored_square: Square = Square(final_name, dx, dy, new_center,
                                           new_rotate, corner_radius, corner_count)
        return y_mirrored_square


# UseModule2D:
class UseModule2D(Scad2D):
    """Represents Module2D invocation."""

    # UseModule2D.__init__():
    def __init__(self, name: str, module2d: Module2D) -> None:
        """Invoke a Module2D."""
        super().__init__(name)
        # Stuff *module2d* into *use_module2d* (i.e. *self*):
        # use_module2d: UseModule2D = self
        self.module2d: Module2D = module2d

    # UseModule2D.__str__():
    def __str__(self) -> str:
        """Return UseModule2D as a string."""
        # Grab some values from *use_module2d* (i.e. *self*):
        use_module2d: UseModule2D = self
        module2d: Module2D = use_module2d.module2d
        name: str = use_module2d.name
        return f"UseModule2D('{name}',{module2d})"

    # UseModule2D.scad_lines_append():
    def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:
        """Append UseModule2D to list of lines.

        Args:
            *scad_lines* (*List*[*str*]): The lines list to append the
                *scad_polygon* (i.e. *self*) to.
            *indent* (*str*): The indentatation prefix for each line.

        """
        # Grab some values from *scad_linear_extrude* (i.e. *self*):
        use_module2d: UseModule2D = self
        module2d: Module2D = use_module2d.module2d
        use_module_name: str = use_module2d.name
        module_name: str = module2d.name.replace(' ', '_')
        is_operator: bool = module2d.is_operator
        end_text: str = "" if is_operator else ';'
        scad_lines.append(f"{indent}{module_name}(){end_text} // UseModule2D('{use_module_name}')")


# Variable2D:
class Variable2D(Scad2D):
    """Represents setting a variable in some 2D code."""

    # Variable2D.__init__():
    def __init__(self, name: str, variable_name: str, expression: str) -> None:
        """Set a variable value for a ScadD code."""
        # Initialize the *Scad2D* parent class:
        super().__init__(name)

        # Stuff the values into *varaible2d* (i.e. *self*):
        # variable2d: Variable2D = self
        self.variable_name: str = variable_name
        self.expression: str = expression

    # Variable2D.__str__():
    def __str__(self):
        """Return Variable2D as a text string."""
        # Grab some values from *Variable2D* (i.e. *self*):
        variable2d: Variable2D = self
        variable_name: str = variable2d.variable_name
        expression: str = variable2d.expression
        return f"Variable2D('{variable_name}={expression}')"

    # If2D.scad_lines_append():
    def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:
        """Append If2D to a list of lines.

        Args:
            *scad_lines* (*List*[*str*]): The lines list to append the
                *square* (i.e. *self*) to.
            *indent* (*str*): The indentatation prefix for each line.

        """
        # Grab some values from *Variable2D* (i.e. *self*):
        variable2d: Variable2D = self
        variable_name: str = variable2d.variable_name
        expression: str = variable2d.expression
        scad_lines.append(f"{indent}{variable_name} = {expression};")


# Scad3D:
class Scad3D(Scad):
    """Represents 3-dimensional Scad objects."""

    def __init__(self, name: str) -> None:
        """Set the name of the 3-dimensional SCAD object."""
        super().__init__(name)


# Cylinder:
class Cylinder(Scad3D):
    """Represents a cylinder in 3D space."""

    # Cylinder.__init__():
    def __init__(self, name: str, diameter: float, start_point: P3D, end_point: P3D,
                 sides: int) -> None:
        """Initialize Cylinder."""
        # Initialize parent *Scad3D* class first:
        super().__init__(name)
        # Verify that the cylinder has a positive height:
        height: float = start_point.distance(end_point)
        if height <= 0.0:
            raise ValueError(f"Cylinder '{name}' does not have positive height.")

        # Stuff values into *cylinder* (i.e. *self*):
        # cylinder: Cylinder = self
        self.diameter: float = diameter
        self.end_point: P3D = end_point
        self.sides: int = sides
        self.start_point: P3D = start_point

    # Cylinder.__str__():
    def __str__(self) -> str:
        """Return string representation of Cylinder."""
        # Grab some values from *cylinder* (i.e. *self*):
        cylinder: Cylinder = self
        diameter: float = cylinder.diameter
        end_point: P3D = cylinder.end_point
        name: str = cylinder.name
        sides: int = cylinder.sides
        start_point: P3D = cylinder.start_point
        return f"Cylinder('{name}',{diameter},{start_point},{end_point},{sides})"

    # Cylinder.scad_lines_append():
    def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:
        """Append OpenScad commands for cylinder to lines list."""
        # Grab some values from *cylinder* (i.e. *self*):
        cylinder: Cylinder = self
        diameter: float = cylinder.diameter
        end_point: P3D = cylinder.end_point
        name: str = cylinder.name
        sides: int = cylinder.sides
        start_point: P3D = cylinder.start_point

        height_vector: P3D = end_point - start_point
        height: float = height_vector.length()
        # Some algebra:
        #     (1) dot(A, B) = |A| * |B| * cos(theta)
        #     (2) dot(A, B) / (|A| * |B|) = cos(theta)
        #     (3) theta = acos(dot(A, B) / (|A| * |B|))
        # Now, let B be the unit length Z axis (i.e. (0, 0, 1)):
        #     (4) theta = acos(dot(A, [0,0,1]) / (|A| * |[0,0,1]|))
        #     (5) theta = acos((Ax*0 + Ay*0 + Az*1) / (|A| * 1))
        #     (6) theta = acos(Az / |A|)
        z_axis_angle: float = acos(height_vector.z / height)

        # Output an OpenSCAD translate command if needed:
        float_format: Callable[[float], str] = Scad.float_format
        center_point: P3D = (start_point + end_point) / 2.0
        if center_point.x != 0.0 or center_point.y != 0.0 or center_point.z != 0.0:
            scad_lines.append(f"{indent}translate(v = ["
                              f"{float_format(center_point.x)}, "
                              f"{float_format(center_point.y)}, "
                              f"{float_format(center_point.z)}])")

        # Output an OpenSCAD rotate command if needed:
        if z_axis_angle != 0.0:
            # We need to rotate by an angle.  We use a cross product to compute a *rotate_axis*:
            rotate_axis: P3D = P3D(0.0, 0.0, 1.0).cross(height_vector)
            scad_lines.append(f"{indent} rotate(a = {z_axis_angle * 180.0 / pi}, "
                              f"v = [{float_format(rotate_axis.x)}, "
                              f"{float_format(rotate_axis.y)}, "
                              f"{float_format(rotate_axis.z)}])")

        # Output an OpenSCAD cylinder command:
        scad_lines.append(f"{indent}  cylinder("
                          f"h = {float_format(height)}, "
                          f"d = {float_format(diameter)}, "
                          f"$fn = {sides}, "
                          "center = true);  "
                          f"// Cylinder: '{name}'")


# Cube:
class Cube(Scad3D):
    """Represents a cube in 3D space."""

    # Cube.__init__():
    def __init__(self, name: str, dx: float, dy: float, dz: float,
                 center: P3D = P3D(0.0, 0.0, 0.0)) -> None:
        """Initialize a cube in space."""
        # Initialize the *Scad3D* parent class:
        super().__init__(name)

        # Validate arguments:
        if dx <= 0.0:
            raise ValueError(f"Cube '{name}' dx={dx} is not positive")
        if dy <= 0.0:
            raise ValueError(f"Cube '{name}' dy={dy} is not positive")
        if dz <= 0.0:
            raise ValueError(f"Cube '{name}' dz={dz} is not positive")

        # Load the values into *cube* (i.e. *self*):
        # cube: Cube = self
        self.center: P3D = center
        self.dx: float = dx
        self.dy: float = dy
        self.dz: float = dz

    # Cube.__str__():
    def __str__(self) -> str:
        """Convert Cube into a string."""
        # Grab some values from *cube* (i.e. *self*):
        cube: Cube = self
        center: P3D = cube.center
        dx: float = cube.dx
        dy: float = cube.dy
        dz: float = cube.dz
        name: str = cube.name

        # Return the result:
        float_format: Callable[[float], str] = Scad.float_format
        return (f"Cube('{name}',{float_format(dx)},{float_format(dy)},"
                f"{float_format(dz)},center={center})")

    # Cube.scad_lines_append():
    def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:
        """Append Cube to lines list.

        Args:
            *scad_lines* (*List*[*str*]): The lines list to append the
                *circle* (i.e. *self*) to.
            *indent* (*str*): The indentatation prefix for each line.

        """
        # Grab some values from *cube* (i.e. *self*):
        cube: Cube = self
        center: P3D = cube.center
        dx: float = cube.dx
        dy: float = cube.dy
        dz: float = cube.dz
        name: str = cube.name

        # Grab the values out of *center*:
        center_x: float = center.x
        center_y: float = center.y
        center_z: float = center.z

        # Figure out if we need to output a preceeding `translate` command:
        float_format: Callable[[float], str] = Scad.float_format
        if center_x == 0.0 and center_y == 0.0 and center_z == 0.0:
            # Simple origin centered cube:
            scad_lines.append(f"{indent}cube(size = [{float_format(dx)}, "
                              f"{float_format(dy)}, {float_format(dz)}], center = true);  "
                              f"// Cube: '{name}'")
        else:
            # Cube centered somewhere other than the origin:
            scad_lines.append(f"{indent}translate(v = [{float_format(center_x)}, "
                              f"{float_format(center_y)}, {float_format(center_z)}]) {{")
            scad_lines.append(f"{indent} cube(size = [{float_format(dx)}, "
                              f"{float_format(dy)}, {float_format(dz)}], center = true);  "
                              f"// Cube: '{name}'")
            scad_lines.append(f"{indent}}}")


# Color:
class Color(Scad3D):
    """Represents a color."""

    # Color.__init__():
    def __init__(self, name: str, scad3d: Scad3D, color_name: str, alpha: float = 1.0) -> None:
        """Attach a color to Scad3D objet."""
        # Initialize the parent *Scad3D* class:
        super().__init__(name)
        # Now figure out what to stuff into *color* (i.e. *self*):
        # color: Color = self
        colors_set: Set[str] = Scad.colors_set_get()
        if color_name.lower() not in colors_set:
            raise ValueError(f"Color '{color_name} is not a recognized color.")
        if not (0.0 <= alpha <= 1.0):
            raise ValueError(f"Alpha ({alpha}) must be between 0.0 and 1.0 inclusive.")
        self.alpha: float = alpha
        self.color_name: str = color_name
        self.scad3d: Scad3D = scad3d

    # Color.str():
    def __str__(self) -> str:
        """Convert a Color into a string."""
        # Grab some values from *Color*:
        color: Color = self
        alpha: float = color.alpha
        color_name: str = color.color_name
        name: str = color.name
        scad3d: Scad3D = color.scad3d
        alpha_text: str = "" if alpha >= 1.0 else ",alpha={0:.2f}".format(alpha)
        return f"Color('{name}',{scad3d},'{color_name}'{alpha_text})"

    # Color.scad_lines_append():
    def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:
        """Append If3D to a list of lines.

        Args:
            *scad_lines* (*List*[*str*]): The lines list to append the
                *square* (i.e. *self*) to.
            *indent* (*str*): The indentatation prefix for each line.

        """
        # Grab some values from *Color*:
        color: Color = self
        alpha: float = color.alpha
        color_name: str = color.color_name
        name: str = color.name
        scad3d: Scad3D = color.scad3d
        alpha_text: str = "" if alpha >= 1.0 else ", a = {0:.2f}".format(alpha)
        scad_lines.append(f'{indent}color("{color_name}"{alpha_text}) {{  '
                          f"// Color: '{name}'")
        scad3d.scad_lines_append(scad_lines, indent + " ")
        scad_lines.append(f"{indent}}}")


# CornerCube:
class CornerCube(Cube):
    """Represents a cube specified by two corners."""

    def __init__(self, name: str, corner1: P3D, corner2: P3D) -> None:
        """Initialize a CornerCube."""
        # Grab values from *arguments*:
        corner1_x: float = corner1.x
        corner1_y: float = corner1.y
        corner1_z: float = corner1.z
        corner2_x: float = corner2.x
        corner2_y: float = corner2.y
        corner2_z: float = corner2.z

        # Compute *dx*, *dy*, and *dz*:
        dx: float = abs(corner2_x - corner1_x)
        dy: float = abs(corner2_y - corner1_y)
        dz: float = abs(corner2_z - corner1_z)

        # Valid that the volume is non-zero:
        if dx <= 0.0:
            raise ValueError(f"CornerCube '{name}' has dx of 0.0")
        if dy <= 0.0:
            raise ValueError(f"CornerCube '{name}' has dy of 0.0")
        if dz <= 0.0:
            raise ValueError(f"CornerCube '{name}' has dz of 0.0")

        # Compute *center*:
        center_x: float = (corner1_x + corner2_x) / 2.0
        center_y: float = (corner1_y + corner2_y) / 2.0
        center_z: float = (corner1_z + corner2_z) / 2.0
        center: P3D = P3D(center_x, center_y, center_z)

        # Now initialize the *Cube* parent class:
        super().__init__(name, dx, dy, dz, center=center)


# If3D:
class If3D(Scad3D):
    """Represents a Scad3D if-then-else statement chain."""

    # If3D.__init__():
    def __init__(self, name: str, then_expression: str,
                 then_scad3ds: List[Scad3D], lock=False) -> None:
        """Initialize an If3D object."""
        # Initialize the *SCAD3D* parent class:
        super().__init__(name)
        # Load values into *If3D* (i.e. *self*):
        # if3d: If3D = self
        then_clause: Tuple[str, List[Scad3D]] = (then_expression, then_scad3ds[:])
        self.then_clauses: List[Tuple[str, List[Scad3D]]] = [then_clause]
        self.else_scad3ds: Optional[List[Scad3D]] = None
        self.locked: bool = lock
        self.named_mark_downs: List[Tuple[str, ...]] = []

    # If3D.__str__():
    def __str__(self) -> str:
        """Convert an If3D into a string."""
        # Grab some values from *if3d* (i.e. *self*):
        if3d: If3D = self
        name: str = if3d.name
        locked: bool = if3d.locked
        return f"If3D('{name}',...,lock={locked})"

    # If3D.else_set():
    def else_set(self, new_else_scad3ds: List[Scad3D]) -> None:
        """Set the final else caluse for an If3D."""
        # Grab some values from *if3d* (i.e. *self*):
        if3d: If3D = self
        else_scad3ds: Optional[List[Scad3D]] = if3d.else_scad3ds
        locked: bool = if3d.locked
        name: str = if3d.name
        if locked:
            raise ValueError(f"If3D('{name})' is locked and can not have an else clause set.")
        elif else_scad3ds is not None:
            raise ValueError(f"If3D('{name})' else clause is already set.")
        else:
            if3d.else_scad3ds = new_else_scad3ds[:]

    # If3D.lock():
    def lock(self):
        """Ensure that an If3D is locked."""
        if3d: If3D = self
        if3d.locked = True

    # If3D.name_match():
    def name_match_append(self, name: str, module3d: "Module3D", mark_down: List[str]) -> None:
        """Append a then clause for mataching a name."""
        # Append a then clause to *if3d* (i.e. *self*) and remember the *mark_down*:
        if3d: If3D = self
        if3d.then_append(f'name == "{name}"', [UseModule3D(f"{name} Use Module", module3d)])
        named_mark_down: Tuple[str, ...] = (name,) + tuple(mark_down)
        if3d.named_mark_downs.append(named_mark_down)

    # If3D.scad_lines_append():
    def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:
        """Append If3D to a list of lines.

        Args:
            *scad_lines* (*List*[*str*]): The lines list to append the
                *square* (i.e. *self*) to.
            *indent* (*str*): The indentatation prefix for each line.

        """
        # Grab some values from *if3d* (i.e. *self*):
        if3d: If3D = self
        else_scad3ds: Optional[List[Scad3D]] = if3d.else_scad3ds
        name: str = if3d.name
        then_clauses: List[Tuple[str, List[Scad3D]]] = if3d.then_clauses
        locked: bool = if3d.locked
        if not locked:
            raise ValueError(f"If3D '{name}' is not locked.")

        next_indent: str = indent + " "
        then_clause: Tuple[str, List[Scad3D]]
        then_index: int
        for then_index, then_clause in enumerate(then_clauses):
            then_expression: str = then_clause[0]
            then_scad3ds: List[Scad3D] = then_clause[1]
            comment_text = f"  // If3D '{name}'" if then_index == 0 else ""
            if_text: str = "if" if then_index == 0 else "} else if"
            scad_lines.append(f"{indent}{if_text} ({then_expression}) {{{comment_text}")
            then_scad3d: Scad3D
            for then_scad3d in then_scad3ds:
                then_scad3d.scad_lines_append(scad_lines, next_indent)
        if else_scad3ds is not None:
            scad_lines.append(f"{indent}}} else {{")
            else_scad3d: Scad3D
            for else_scad3d in else_scad3ds:
                else_scad3d.scad_lines_append(scad_lines, next_indent)
        scad_lines.append(f"{indent}}}  // End If3D '{name}'")

    # If3D.then_append():
    def then_append(self, else_if_expression: str, else_if_scad3ds: List[Scad3D]) -> None:
        """Append a then clause to an If3D."""
        # Grab some values from *if3d* (i.e. *self*):
        if3d: If3D = self
        name: str = if3d.name
        then_clauses: List[Tuple[str, List[Scad3D]]] = if3d.then_clauses
        locked: bool = if3d.locked
        if locked:
            raise ValueError(f"If3D '{name}' is and locked can not accept another then clause")
        then_clause: Tuple[str, List[Scad3D]] = (else_if_expression, else_if_scad3ds)
        then_clauses.append(then_clause)


# LinearExtrude():
class LinearExtrude(Scad3D):
    """Represents an OpenScad `linear_extrude` command."""

    def __init__(self, name: str, scad2d: Scad2D, height: float, center: bool = False,
                 twist: float = 0.0, convexity: int = -1, slices: int = -1,
                 initial_scale: float = 1.0, final_scale: float = 1.0) -> None:
        """Initialize a `linear_extrude` command.

        Args:
            *name* (*str*): A name that will be output to the
                `.scad` file.
            *scad2d*: (*Scad2D*): A 2-dimensional *Scad* object to
                perform the linear extrusion on.
            *height* (*float*): The height of the extrusion.
            *center* (*bool*): If *True*, the extrusion is from
                -*height*/2 to *height*/2 in the Z direction;
                otherwise, it is from 0 to *height* in the Z direction.
            *convexity* (*int*): Specifies the complexity level to
                support for preview rendering.  Higher numbers support
                more complex objects.
            *twist* (*float*): Specifies the amount of twist around the
                Z axis in radians.
            *slices*: (*int*): Specifies
            *initial_scale* (*float*): Specifies the initial scale
                of the linear extrusion.
            *final_scale* (*float*): Specifies the final scale of
                the linear extrusion.

        """
        # Initialize the *Scad* base-class:
        super().__init__(name)

        # Do some argument range validation:
        assert final_scale > 0.0, f"final_scale={final_scale}; it needs to be positive"
        assert height > 0.0, f"height={height}; it needs to be positive"
        assert initial_scale > 0.0, f"initial_scale={initial_scale}; it needs to be positive"

        # Initialize the *scad_linear_extrude* (i.e. *self*):
        self.center: bool = center
        self.convexity: int = 10 if convexity <= 0 else convexity
        self.final_scale: float = final_scale
        self.height: float = height
        self.initial_scale: float = initial_scale
        self.scad2d: Scad2D = scad2d
        self.slices: int = slices
        self.twist: float = twist

    # LinearExtrude.scad_lines_append():
    def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:
        """Append ScadLinearExtrude to list of lines.

        Args:
            *scad_lines* (*List*[*str*]): The lines list to append the
                *scad_polygon* (i.e. *self*) to.
            *indent* (*str*): The indentatation prefix for each line.

        """
        # Grab some values from *scad_linear_extrude* (i.e. *self*):
        linear_extrude: LinearExtrude = self
        center: bool = linear_extrude.center
        convexity: int = linear_extrude.convexity
        final_scale: float = linear_extrude.final_scale
        initial_scale: float = linear_extrude.initial_scale
        height: float = linear_extrude.height
        name: str = linear_extrude.name
        scad2d: Scad2D = linear_extrude.scad2d
        slices: int = linear_extrude.slices
        twist: float = linear_extrude.twist

        # Compute *scale_text* and *slices_text*:
        float_format: Callable[[float], str] = Scad.float_format
        initial_scale_text: str = float_format(initial_scale)
        final_scale_text: str = float_format(final_scale)
        scale_text: str = (f", scale=[{initial_scale_text}, {final_scale_text}]"
                           if initial_scale != 1.0
                           else (f", scale={final_scale_text}"
                                 if final_scale != 1.0
                                 else ""))
        slices_text: str = (f", slices={slices}"
                            if slices > 0
                            else "")

        # Perform the the `linear_extrude` command append to *scad_lines*:
        scad_lines.append(f"{indent}// Begin LinearExtrude '{name}'")
        scad_lines.append(f"{indent}linear_extrude("
                          f"height={height}"
                          f", center={str(center).lower()}"
                          f", convexity={convexity}"
                          f", twist={degrees(twist)}"
                          f"{slices_text}{scale_text})")

        # Append the *scad2d* object next:
        scad2d.scad_lines_append(scad_lines, indent + " ")

        # Outut an end comment:
        scad_lines.append(f"{indent}// End LinearExtrude '{name}'")


# Module3D:
class Module3D(Scad3D):
    """Represents an OpenSCAD 3D Module."""

    # Module3D.__init__():
    def __init__(self, name: str, scad3ds: List[Scad3D], is_operator=False, lock=True) -> None:
        """Initialize a Module3D."""
        # Initialize the *Scad3D* parent class:
        super().__init__(name)

        # Load values into *module3d* (i.e. *self*):
        # module3d: Module3D = self
        self.is_operator: bool = is_operator
        self.locked: bool = lock
        self.scad3ds: List[Scad3D] = scad3ds[:]  # Make a copy

    # Module3D.__str__():
    def __str__(self) -> str:  # pragma: no cover
        """Return Module3D as a string."""
        # Grab some values from *module3d* (i.e. *self*):
        module3d: Module3D = self
        is_operator: bool = module3d.is_operator
        locked: bool = module3d.locked
        name: str = module3d.name
        return f"Module3D('{name}',[...],is_operator={is_operator},lock={locked})"

    # Module3D.__getitem__():
    def __getitem__(self, index: int) -> Scad3D:
        """Return a SCAD2 item from a Module3D."""
        # Grab some values from *module3d* (i.e. *self*):
        module3d: Module3D = self
        scad3ds: List[Scad3D] = module3d.scad3ds

        # Ensure that *index* is in bounds:
        scad3ds_size: int = len(scad3ds)
        if not (0 <= index < scad3ds_size):
            name: str = module3d.name
            raise IndexError(f"Index {index} exceeds {scad3ds_size} objects in Module3D '{name}'")

        # Fetch *scad3d* and return it:
        scad3d: Scad3D = scad3ds[index]
        return scad3d

    # Module3D.__len__():
    def __len__(self) -> int:
        """Return number of SCAD3D's in Module3D."""
        # Grab some values from *module3d* (i.e. *self*):
        module3d: Module3D = self
        scad3ds: List[Scad3D] = module3d.scad3ds
        scad3ds_size: int = len(scad3ds)
        return scad3ds_size

    # Module3D.append():
    def append(self, scad3d: Scad3D) -> None:
        """Append a Scad3D to a Module3D."""
        # Grab some values from *scad3d* (i.e. *self*):
        module3d: Module3D = self
        locked: bool = module3d.locked
        scad3ds: List[Scad3D] = module3d.scad3ds

        # Make sure that we are not *locked*:
        if locked:
            name: str = module3d.name
            raise ValueError(f"Can not append to Module3D '{name}' because is locked")

        # Perform the *append*:
        scad3ds.append(scad3d)

    # Module3D.extend():
    def extend(self, new_scad3ds: List[Scad3D]) -> None:
        """Append a Scad3D to a Module3D."""
        # Grab some values from *scad3d* (i.e. *self*):
        module3d: Module3D = self
        locked: bool = module3d.locked
        scad3ds: List[Scad3D] = module3d.scad3ds

        # Make sure that we are not *locked*:
        if locked:
            name: str = module3d.name
            raise ValueError(f"Can not extend Module3D '{name}' because is locked")

        # Perform the *append*:
        scad3ds.extend(new_scad3ds)

    # Module3D.lock():
    def lock(self):
        """Ensure that Module3D is locked."""
        module3d: Module3D = self
        module3d.locked = True

    # Module3D.scad_lines_append():
    def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:
        """Append Circle to lines list.

        Args:
            *scad_lines* (*List*[*str*]): The lines list to append the
                *circle* (i.e. *self*) to.
            *indent* (*str*): The indentatation prefix for each line.

        """
        # Grab some values from *module3d* (i.e. *self*):
        module3d: Module3D = self
        locked: bool = module3d.locked
        name: str = module3d.name
        scad3ds: List[Scad3D] = module3d.scad3ds

        # Make sure that we are *locked*:
        if not locked:
            raise ValueError(f"Module3D '{name}' is not locked yet.")

        # Output the module name defintition:
        scad_lines.append(f"{indent}module {name.replace(' ', '_')}() {{")

        # Output the *scad3d*s:
        next_indent: str = indent + " "
        scad3d: Scad3D
        for scad3d in scad3ds:
            scad3d.scad_lines_append(scad_lines, next_indent)

        # Output the closing '}':
        scad_lines.append(f"{indent}}}")


# Rotate3D(Scad3D):
class Rotate3D(Scad3D):
    """Rotate about a partiuclar axis."""

    # Rotate3D.__init__():
    def __init__(self, name: str, scad3d: Scad3D,
                 rotate: float, axis: P3D = P3D(0.0, 0.0, 1.0)) -> None:
        """Rotate a Scad3D about an axis."""
        # Initialize the parent *Scad3D* class:
        super().__init__(name)

        # Verify *axis* has a length:
        origin: P3D = P3D(0.0, 0.0, 0.0)
        length: float = origin.distance(axis)
        if length <= 0.0:
            raise ValueError("Rotate axis has no direction.")

        # Stuff arguments into *rotate3d* (i.e. *self*):
        # rotate3d: Rotate3D = self
        self.axis: P3D = axis
        self.rotate: float = rotate
        self.scad3d: Scad3D = scad3d

    # Rotate3D.__str__():
    def __str__(self) -> str:
        """Return string representation."""
        # Grab some values from *rotate3d* (i.e. *self*):
        rotate3d: Rotate3D = self
        axis: P3D = rotate3d.axis
        name: str = rotate3d.name
        rotate: float = rotate3d.rotate
        scad3d: Scad3D = rotate3d.scad3d

        float_format: Callable[[float], str] = Scad.float_format
        rotate_text: str = float_format(rotate * 180.0 / pi)
        return f"Rotate('{name}',{scad3d},{axis},{rotate_text}deg)"

    # Rotate3D.scad_lines_append():
    def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:
        """Append Rotate3D to lines list.

        Args:
            *scad_lines* (*List*[*str*]): The lines list to append the
                *circle* (i.e. *self*) to.
            *indent* (*str*): The indentatation prefix for each line.

        """
        # Grab some values from *rotate3d* (i.e. *self*):
        rotate3d: Rotate3D = self
        axis: P3D = rotate3d.axis
        name: str = rotate3d.name
        scad3d: Scad3D = rotate3d.scad3d
        rotate: float = rotate3d.rotate

        # Unpack *axis* X/Y/Z into text strings:
        float_format: Callable[[float], str] = Scad.float_format
        axis_x_text: str = float_format(axis.x)
        axis_y_text: str = float_format(axis.y)
        axis_z_text: str = float_format(axis.z)
        rotate_text: str = "{0:.6f}".format(rotate * 180.0 / pi)

        # Append everything to *scad_lines*:
        scad_lines.append(f"{indent}rotate(a = {rotate_text}, "
                          f"v=[{axis_x_text}, {axis_y_text}, {axis_z_text}]) {{  "
                          f"// Rotate3D: '{name}'")
        scad3d.scad_lines_append(scad_lines, indent + " ")
        scad_lines.append(f"{indent}}}")


# Translate3D(Scad3D):
class Translate3D(Scad3D):
    """Move Scad3D to a another location."""

    # Translate3D.__init__():
    def __init__(self, name: str, scad3d: Scad3D, offset: P3D) -> None:
        """Initialize a Translate3D object."""
        # Initialize the parent super *SCAD3D* class:
        super().__init__(name)
        # Stuff values into *translate3d* (i.e. *self*):
        # translate3d: Translate3D = self
        self.scad3d: Scad3D = scad3d
        self.offset: P3D = offset

    # Translate3D.__str__():
    def __str__(self) -> str:
        """Return the Translate3D as a text string."""
        # Grab some values from *translate3d* (i.e. *self*):
        translate3d: Translate3D = self
        name: str = translate3d.name
        offset: P3D = translate3d.offset
        scad3d: Scad3D = translate3d.scad3d
        return f"Translate3D('{name}',{scad3d},{offset})"

    # Translate3D.scad_lines_append():
    def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:
        """Append Translate3D to lines list.

        Args:
            *scad_lines* (*List*[*str*]): The lines list to append the
                *circle* (i.e. *self*) to.
            *indent* (*str*): The indentatation prefix for each line.

        """
        # Grab some values from *translate3d* (i.e. *self*):
        translate3d: Translate3D = self
        name: str = translate3d.name
        offset: P3D = translate3d.offset
        scad3d: Scad3D = translate3d.scad3d

        # Append the lines to *scad_lines*:
        float_format: Callable[[float], str] = Scad.float_format
        scad_lines.append(f"{indent}translate(v = [{float_format(offset.x)}, "
                          f"{float_format(offset.y)}, {float_format(offset.z)}]) {{  "
                          f"// Translate '{name}'")
        scad3d.scad_lines_append(scad_lines, indent + " ")
        scad_lines.append(f"{indent}}}")


# Union3D:
class Union3D(Scad3D):
    """Represents a boolean union of SCAD3D objects."""

    # Union3D.__init__():
    def __init__(self, name: str, scad3ds: List[Scad3D], lock=True) -> None:
        """Initialize a Union3D object.

        Args:
            *name* (*str*): The name that is output to the `.scad`
                file.
            *scad3ds* (*List*[*Scad3D*]): A list of *Scad3D* objects
                to be unioned together.
            *lock* (*bool*): If *True*, no additional *Scad3D*'s can
                be appended to the *union3d* (i.e. *self*); otherwise
                the *append* and *extend* methods can be used.

        Raises:
            *ValueError*(*str*): Exception that is raised when there are
                mixed *Scad2D* and *Scad3D* object in *scads*:

        """
        # Initialize the base class:
        super().__init__(name)

        # If we get there far, we can stuf *scads* into *union* (i.e. *self*):
        # union: Union3D = self
        self.locked: bool = lock
        self.scad3ds: List[Scad3D] = scad3ds[:]

    # Union3D.__len__():
    def __len__(self) -> int:
        """Return number of SCAD3D's in Union3D."""
        # Grab some values from *union3d* (i.e. *self*):
        union3d: Union3D = self
        scad3ds: List[Scad3D] = union3d.scad3ds
        scad3ds_size: int = len(scad3ds)
        return scad3ds_size

    # Union3D.append():
    def append(self, scad3d: Scad3D) -> None:
        """Append a Scad2D to a Union3d."""
        # Grab some values from *scad2d* (i.e. *self*):
        union3d: Union3D = self
        locked: bool = union3d.locked
        scad3ds: List[Scad3D] = union3d.scad3ds

        # Make sure that we are not *locked*:
        if locked:
            name: str = union3d.name
            raise ValueError(f"Can not append to Union3D '{name}' because is locked")

        # Perform the *append*:
        scad3ds.append(scad3d)

    # Union3D.extend():
    def extend(self, new_scad3ds: List[Scad3D]) -> None:
        """Append a list of Scad3D's to a Union3D."""
        # Grab some values from *scad2d* (i.e. *self*):
        union3d: Union3D = self
        locked: bool = union3d.locked
        scad3ds: List[Scad3D] = union3d.scad3ds

        # Make sure that we are not *locked*:
        if locked:
            name: str = union3d.name
            raise ValueError(f"Can not extend Union3D '{name}' because is locked")

        # Perform the *append*:
        scad3ds.extend(new_scad3ds)

    # Union3D.lock():
    def lock(self):
        """Ensure that Union3D is locked."""
        union3d: Union3D = self
        union3d.locked = True

    # Union3D.scad_lines_append():
    def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:
        """Append Union3D to list of lines.

        Args:
            *scad_lines* (*List*[*str*]): The lines list to append the
                *scad_polygon* (i.e. *self*) to.
            *indent* (*str*): The indentatation prefix for each line.

        """
        # Grab some values from *scad_3d_union* (i.e: *self*):
        union3d: Union3D = self
        name: str = union3d.name
        scad3ds: List[Scad3D] = union3d.scad3ds
        locked: bool = union3d.locked

        # Ensure that we are *locked*:
        if not locked:
            raise ValueError(f"Union3D '{name}' is not locked yet.")

        # Append the lines to *scad_lines*:
        scad_lines.append(f"{indent}union() {{  // Union3D '{name}'")
        next_indent: str = indent + " "
        scad3d: Scad3D
        for scad3d in scad3ds:
            scad3d.scad_lines_append(scad_lines, next_indent)
        scad_lines.append(f"{indent}}}  // End Union3D '{name}'")


# UseModule3D:
class UseModule3D(Scad3D):
    """Represents Module3D invocation."""

    # UseModule3D.__init__():
    def __init__(self, name: str, module3d: Module3D) -> None:
        """Invoke a Module3D."""
        super().__init__(name)
        # Stuff *module3d* into *use_module3d* (i.e. *self*):
        # use_module3d: UseModule3D = self
        self.module3d: Module3D = module3d

    # UseModule3D.__str__():
    def __str__(self) -> str:
        """Return UseModule3D as a string."""
        # Grab some values from *use_module3d* (i.e. *self*):
        use_module3d: UseModule3D = self
        module3d: Module3D = use_module3d.module3d
        name: str = use_module3d.name
        return f"UseModule3D('{name}',{module3d})"

    # UseModule3D.scad_lines_append():
    def scad_lines_append(self, scad_lines: List[str], indent: str) -> None:
        """Append UseModule3D to list of lines.

        Args:
            *scad_lines* (*List*[*str*]): The lines list to append the
                *scad_polygon* (i.e. *self*) to.
            *indent* (*str*): The indentatation prefix for each line.

        """
        # Grab some values from *scad_linear_extrude* (i.e. *self*):
        use_module3d: UseModule3D = self
        module3d: Module3D = use_module3d.module3d
        use_module_name: str = use_module3d.name
        module_name: str = module3d.name.replace(' ', '_')
        is_operator: bool = module3d.is_operator
        end_text: str = "" if is_operator else ';'
        scad_lines.append(f"{indent}{module_name}(){end_text} // UseModule3D('{use_module_name}')")

# Nucleo-32:
#                   Flash    RAM    Speed
# * Nucleo-L031K6:
# * Nucleo-F303K8:  65KB     16KB   72MHz
# * Nucleo-L011K4:
# * Nucleo-F031K6:  32KB     4KB    48MHz
# * Nucleo-F042K6:  32KB     6KB    48MHz
# * Nucleo-8S207K8  8-bit core
# * Nucleo-G431KB:  128KB    32K    170MHz   <==
