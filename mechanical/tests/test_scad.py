"""scad_test: Unit tests for the SCAD module."""

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

# <----------------------------------------100 Characters----------------------------------------> #

import io
from math import pi
from scad_models.scad import (Circle, LinearExtrude, P2D, P3D, Polygon, Scad, SimplePolygon,
                              Square, Union)
import scad_models.scad as scad
from typing import Any, IO, List, Tuple


def scad_writer(scad: Scad, scad_lines: List[str]) -> None:
    """Write out a Square to a `.scad` file."""
    scad_file: IO[Any]
    name: str = scad.name
    with open(name.replace(' ', '_') + ".scad", "w") as scad_file:
        scad_text: str = '\n'.join(scad_lines)
        scad_file.write(scad_text + "\n")


def test_p2d() -> None:
    """Test the point class."""
    origin: P2D = P2D(0.0, 0.0)
    assert f"{origin}" == "P2D(0.000,0.000)"
    p11: P2D = P2D(1.0, 1.0)
    assert f"{p11}" == "P2D(1.000,1.000)"
    p23: P2D = P2D(2.0, 3.0)
    assert f"{p23}" == "P2D(2.000,3.000)"

    # Addition:
    assert f"{origin + p11}" == "P2D(1.000,1.000)"

    # Subtraction:
    assert f"{p23 - p11}" == "P2D(1.000,2.000)"

    # Left/Right Multiplication:
    assert f"{p23 * 2.0}" == "P2D(4.000,6.000)"
    assert f"{2.0 * p23}" == "P2D(4.000,6.000)"

    # Test distance:
    p10: P2D = P2D(1.0, 0.0)
    assert f"{origin.distance(p10)}" == "1.0"
    p01: P2D = P2D(0.0, 1.0)
    assert f"{origin.distance(p01)}" == "1.0"

    # Division scaling:
    assert f"{p23 / 2.0}" == "P2D(1.000,1.500)"

    # Test *y_mirror* method:
    assert f"{p23.y_mirror()}" == "P2D(-2.000,3.000)"


def test_p3d() -> None:
    """Test the point class."""
    origin: P3D = P3D(0.0, 0.0, 0.0)
    assert f"{origin}" == "P3D(0.000,0.000,0.000)"
    p111: P3D = P3D(1.0, 1.0, 1.0)
    assert f"{p111}" == "P3D(1.000,1.000,1.000)"
    p123: P3D = P3D(1.0, 2.0, 3.0)
    assert f"{p123}" == "P3D(1.000,2.000,3.000)"

    # Addition:
    assert f"{origin + p111}" == "P3D(1.000,1.000,1.000)"

    # Subtraction:
    assert f"{p123 - p111}" == "P3D(0.000,1.000,2.000)"

    # Left/Right Multiplication:
    assert f"{p123 * 2.0}" == "P3D(2.000,4.000,6.000)"
    assert f"{2.0 * p123}" == "P3D(2.000,4.000,6.000)"

    # Test distance:
    p100: P3D = P3D(1.0, 0.0, 0.0)
    assert f"{origin.distance(p100)}" == "1.0"
    p010: P3D = P3D(0.0, 1.0, 0.0)
    assert f"{origin.distance(p010)}" == "1.0"
    p001: P3D = P3D(0.0, 0.0, 1.0)
    assert f"{origin.distance(p001)}" == "1.0"

    # Division scaling:
    assert f"{p123 / 2.0}" == "P3D(0.500,1.000,1.500)"


def test_simple_polygon() -> None:
    """Test the SimplePolygon class and associated methods."""
    # Test *empty_polygon*:
    empty_polygon: SimplePolygon = SimplePolygon("Empty")
    assert f"{empty_polygon}" == "SimplePolygon('Empty', [])"

    # Test both *Polygon.*__str__*() and the *Polygon*.size_get*() methods:
    p11: P2D = P2D(1.0, 1.0)
    p22: P2D = P2D(2.0, 2.0)
    p33: P2D = P2D(3.0, 3.0)
    simple_polygon0a: SimplePolygon = SimplePolygon("No Points A")
    assert len(simple_polygon0a) == 0
    assert f"{simple_polygon0a}" == "SimplePolygon('No Points A', [])"
    simple_polygon0b: SimplePolygon = SimplePolygon("No Points B", [])
    assert f"{simple_polygon0b}" == "SimplePolygon('No Points B', [])"
    assert len(simple_polygon0b) == 0
    simple_polygon1: SimplePolygon = SimplePolygon("One Point", [p11])
    assert len(simple_polygon1) == 1
    assert f"{simple_polygon1}" == "SimplePolygon('One Point', [P2D(1.000,1.000)])"
    simple_polygon2: SimplePolygon = SimplePolygon("Two Points", [p11, p22])
    assert len(simple_polygon2) == 2
    assert f"{simple_polygon2}" == ("SimplePolygon('Two Points', "
                                    "[P2D(1.000,1.000), P2D(2.000,2.000)])")
    simple_polygon3: SimplePolygon = SimplePolygon("Three Points", [p11, p22, p33])
    assert len(simple_polygon3) == 3
    assert f"{simple_polygon3}" == ("SimplePolygon('Three Points', "
                                    "[P2D(1.000,1.000), ..., P2D(3.000,3.000)])")

    # Now test the *Polygon*.*__getitem__*() method:
    assert f"{simple_polygon1[0]}" == "P2D(1.000,1.000)"
    assert f"{simple_polygon2[0]}" == "P2D(1.000,1.000)"
    assert f"{simple_polygon2[1]}" == "P2D(2.000,2.000)"
    assert f"{simple_polygon3[0]}" == "P2D(1.000,1.000)"
    assert f"{simple_polygon3[1]}" == "P2D(2.000,2.000)"
    assert f"{simple_polygon3[2]}" == "P2D(3.000,3.000)"

    # Validate that the out-of-bounds error is generated:
    index_error: IndexError
    try:
        point: P2D = simple_polygon3[4]
        point += point  # pragma: no cover
        assert False, "__getitem__ did not properly fail"  # pragma: no cover
    except IndexError as index_error:
        assert isinstance(index_error, IndexError)
        assert f"{index_error}" == "index of 4 is not in range 0 to 2"

    # Test Polygon.arc_append() and Polygon.point_append:
    slot: SimplePolygon = SimplePolygon("Slot")
    slot.arc_append(P2D(1.0, 0.0), 1.0, -pi/2, pi/2, 3)
    slot.point_append(P2D(0.0, 2.0))
    slot.arc_append(P2D(-1.0, 0.0), 1.0, pi/2, 3.0 * pi/2, 3)
    slot.point_append(P2D(0.0, -2.0))
    assert len(slot) == 8
    assert f"{slot[0]}" == "P2D(1.000,-1.000)"
    assert f"{slot[1]}" == "P2D(2.000,0.000)"
    assert f"{slot[2]}" == "P2D(1.000,1.000)"
    assert f"{slot[3]}" == "P2D(0.000,2.000)"
    assert f"{slot[4]}" == "P2D(-1.000,1.000)"
    assert f"{slot[5]}" == "P2D(-2.000,0.000)"
    assert f"{slot[6]}" == "P2D(-1.000,-1.000)"
    assert f"{slot[7]}" == "P2D(0.000,-2.000)"

    # Test Polygon.circle():
    # hole: SimplePolygon = SimplePolygon("Circle")
    # center: P2D = P2D(1.0, 0.0)
    # hole.circle_append(center, 2.0, 4)
    # assert False, [f"{point}" for point in hole.points]

    # assert len(hole) == 4
    # assert f"{hole[0]}" == "P2D(2.000,0.000)"
    # assert f"{hole[1]}" == "P2D(1.000,1.000)"
    # assert f"{hole[2]}" == "P2D(0.000,0.000)"
    # assert f"{hole[3]}" == "P2D(1.000,-1.000)"

    # Test Polygon.rotated_rectangle_append():
    rotated_rectangle: SimplePolygon = SimplePolygon("Rotated Rectangle")
    rotated_rectangle.rotated_rectangle_append(P2D(2.0, 3.0), 2.0, 4.0, pi/2)
    assert f"{rotated_rectangle[0]}" == "P2D(0.000,4.000)", "Index 0 failed"
    assert f"{rotated_rectangle[1]}" == "P2D(4.000,4.000)", "Index 1 failed"
    assert f"{rotated_rectangle[2]}" == "P2D(4.000,2.000)", "Index 2 failed"
    assert f"{rotated_rectangle[3]}" == "P2D(0.000,2.000)", "Index 3 failed"

    # Test Polygon.slot_append():
    slot2: SimplePolygon = SimplePolygon("Slot2")
    slot2.slot_append(P2D(1.0, 1.0), P2D(3.0, 1.0), 4.0, 2.0, 3)
    assert f"{slot2[0]}" == "P2D(0.000,2.000)", "Index 0 failed"
    assert f"{slot2[1]}" == "P2D(-1.000,1.000)", "Index 1 failed"
    assert f"{slot2[2]}" == "P2D(0.000,0.000)", "Index 2 failed"
    assert f"{slot2[3]}" == "P2D(4.000,0.000)", "Index 3 failed"
    assert f"{slot2[4]}" == "P2D(5.000,1.000)", "Index 4 failed"
    assert f"{slot2[5]}" == "P2D(4.000,2.000)", "Index 5 failed"

    # Test Polygon.points_scad_lines_append():
    scad_lines: List[str] = []
    start_index: int = 3
    end_index: int = slot.points_scad_lines_append(scad_lines, " ", start_index)
    assert end_index == start_index + len(slot)
    assert len(scad_lines) == 3
    assert scad_lines[0] == "  // Polygon 'Slot' 3:10", "Index 0 failed"
    assert scad_lines[1] == ("   [1.000, -1.000], [2.000, 0.000], [1.000, 1.000], "
                             "[0.000, 2.000], // 3:6"), "Index 1 failed"
    assert scad_lines[2] == ("   [-1.000, 1.000], [-2.000, 0.000], [-1.000, -1.000], "
                             "[0.000, -2.000], // 7:10"), "Index 2 failed"

    # Test Polygon.indices_scad_lines_append():
    # scad_lines = []
    # end_index = slot.indices_scad_lines_append(scad_lines, " ", start_index)
    # assert end_index == start_index + len(slot)
    # assert len(scad_lines) == 2
    # assert scad_lines[0] == "  // Polygon 'Slot' 3:10"
    # assert scad_lines[1] == "   [3, 4, 5, 6, 7, 8, 9, 10],"


def test_polygon() -> None:
    """Test SimplePolygon class and associated methods."""
    # Define the four corners of a square:
    upper_right: P2D = P2D(2.0, 2.0)
    lower_right: P2D = P2D(2.0, -2.0)
    lower_left: P2D = P2D(-2.0, -2.0)
    upper_left: P2D = P2D(-2.0, 2.0)

    # Convert the four corners into a *square_simple_polygon*:
    square_simple_polygon: SimplePolygon = SimplePolygon("Square SimplePolygon",
                                                         [upper_right, lower_right,
                                                          lower_left, upper_left], lock=True)
    # Now stuff *square_simple_polygon* into *x_square_polygon*:
    x_square_polygon: Polygon = Polygon("X Square Polygon", [square_simple_polygon])

    # Unpack *x_square_polygon* into *scad_lines*:
    scad_lines: List[str] = []
    x_square_polygon.scad_lines_append(scad_lines, "")
    scad_writer(x_square_polygon, scad_lines)

    # Now validate that we got the right values written into *scad*_lines*:
    assert len(scad_lines) == 8
    assert scad_lines[0] == "polygon(points = [  // Begin Polygon 'X Square Polygon' 0:3", "[0]!"
    assert scad_lines[1] == " // SimplePolygon 'Square SimplePolygon' 0-3", "[1]!"
    assert scad_lines[2] == ("  [2.000, 2.000], [2.000, -2.000], [-2.000, -2.000], "
                             "[-2.000, 2.000]  // 0-4"), "[2]!"
    assert scad_lines[3] == " ], paths = [", "[3]!"
    assert scad_lines[4] == "  // SimplePolygon 'Square SimplePolygon' 0-3", "[4]!"
    assert scad_lines[5] == "  [0, 1, 2, 3]", "[5]!"
    assert scad_lines[6] == " ], convexity=4);  // End Polygon 'X Square Polygon' 0:3", "[6]!"

    # Test Scad.file_write():
    scad_file: IO[Any]
    with open("/tmp/test_scad.scad", "w") as scad_file:
        x_square_polygon.file_write(scad_file)
    with io.StringIO("") as scad_file:
        x_square_polygon.file_write(scad_file)
        input_scad_text: str = scad_file.getvalue()
        input_scad_lines: List[str] = input_scad_text.split('\n')
        assert len(input_scad_lines) == 10
        assert input_scad_lines[0] == "// 'X Square Polygon' File", "[0]!"
        assert input_scad_lines[1] == ("polygon(points = [  "
                                       "// Begin Polygon 'X Square Polygon' 0:3"), "[1]!"
        assert input_scad_lines[2] == (" // SimplePolygon 'Square SimplePolygon' 0-3"), "[2]!"
        assert input_scad_lines[3] == ("  [2.000, 2.000], [2.000, -2.000], [-2.000, -2.000], "
                                       "[-2.000, 2.000]  // 0-4"), "[3]!"
        assert input_scad_lines[4] == " ], paths = [", "[4]!"
        assert input_scad_lines[5] == "  // SimplePolygon 'Square SimplePolygon' 0-3", "[5]!"
        assert input_scad_lines[6] == "  [0, 1, 2, 3]", "[6]!"
        assert input_scad_lines[7] == (" ], convexity=4);  "
                                       "// End Polygon 'X Square Polygon' 0:3"), "[7]!"
        assert input_scad_lines[8] == "", "[8]!"

    # Test the *Polygon*.*__getitem__*() and *Polygon*.*__size__*() methods:
    new_square_polygon: Polygon = Polygon("New Square Polygon", [])
    assert len(new_square_polygon) == 0
    new_square_polygon.append(square_simple_polygon)
    assert len(new_square_polygon) == 1
    assert new_square_polygon[0] is square_simple_polygon
    assert f"{new_square_polygon}" == ("Polygon('New Square Polygon',"
                                       "len(simple_polygons)=1,convexity=-1)")

    # Now Test the *Polygon*.*append*() and *Polygon*.*extend*() methods:
    hole_polygons: List[SimplePolygon] = list()
    hole_polygon: SimplePolygon
    diameter: float = 0.25
    x: float
    for x in (-1.0, 0.0, 1.0):
        y: float
        for y in (-1.0, 0.0, 1.0):
            center: P2D = P2D(x, y)
            hole: Circle = Circle(f"Hole[{x, y}]", diameter, 8, center)
            hole_polygons.append(hole)
    new_square_polygon.extend(hole_polygons)
    assert len(new_square_polygon) == 10

    # Test *Polygon*.*__get__item__*() method:
    hole_index: int
    for hole_index, hole_polygon in enumerate(hole_polygons):
        assert new_square_polygon[hole_index + 1] is hole_polygons[hole_index]
    try:
        new_square_polygon[10]
    except IndexError as index_error:
        assert f"{index_error}" == "index=10 and it is not in range 0:9"

    # Verify lock detection:
    simple_polygon: SimplePolygon = SimplePolygon("Unlocked SimplePolygon")
    try:
        Polygon("Unlocked Polygon", [simple_polygon])
    except ValueError as value_error:
        assert f"{value_error}" == ("SimplePolygon ('Unlocked SimplePolygon') "
                                    "at index 0 is not locked.")

    # Test Polygon.append() and Polygon.extend():


def test_circle() -> None:
    """Test Circle class."""
    center: P2D = P2D(2.0, 3.0)
    circle: Circle = Circle("Circle", 2.0, 8, center)
    assert f"{circle}" == "Circle('Circle',2.0,8,P2D(2.000,3.000),4)"

    scad_lines: List[str] = []
    circle.scad_lines_append(scad_lines, "")
    scad_text: str = '\n'.join(scad_lines)
    scad_file: IO[Any]
    with open("circle.scad", "w") as scad_file:
        scad_file.write(scad_text)
    assert len(circle) == 8
    assert len(scad_lines) == 2
    assert scad_lines[0] == "translate([2.000, 3.000])"
    assert scad_lines[1] == " circle(d=2.000, $fn=8);  // Circle 'Circle'"

    # Validate *Circle*.*key*() method:
    key: Tuple[Any] = circle.key()
    assert key == ("Circle", "Circle", 2.0, 3.0, 2.0, 2.0, 0.0), f"Bad key: {key}"


def test_linear_extrude() -> None:
    """Test LinearExtrude class."""
    unit_square: Square = Square("Unit Square", 1.0, 1.0)
    linear_extrude: LinearExtrude = LinearExtrude("Linear Extrude", unit_square, 1.0)
    scad_lines: List[str] = []
    linear_extrude.scad_lines_append(scad_lines, "")
    scad_writer(linear_extrude, scad_lines)
    assert len(scad_lines) == 5
    assert scad_lines[0] == ("// Begin LinearExtrude 'Linear Extrude'"), "[0]!"
    assert scad_lines[1] == ("linear_extrude(height=1.0, center=false, "
                             "convexity=10, twist=0.0)"), "[1]!"
    assert scad_lines[2] == (" // Square 'Unit Square' dx=1.000 dy=1.000 center=P2D(0.000,0.000) "
                             "corner_radius=0.000 corner_count=3"), "[2]!"
    assert scad_lines[3] == (" square([1.000, 1.000], center = true);"), "[3]!"
    assert scad_lines[4] == ("// End LinearExtrude 'Linear Extrude'"), "[4]!"


def test_square() -> None:
    """Test Square class."""
    # Start by pushing the *Square*.*__str__*() method through its paces:
    origin_square: Square = Square("Origin Square", 2.0, 3.0)
    assert f"{origin_square}" == "Square('Origin Square',2.000,3.000)"
    center: P2D = P2D(3.0, 2.0)
    offset_square: Square = Square("Offset Square", 2.5, 3.5, center)
    assert f"{offset_square}" == "Square('Offset Square',2.500,3.500,center=P2D(3.000,2.000))"
    rotated_square: Square = Square("Rotated Square", 2.0, 3.0, rotate=pi/2)
    assert f"{rotated_square}" == "Square('Rotated Square',2.000,3.000,rotate=90.000deg)"
    slot_square: Square = Square("Slot Square", 2.0, 4.0, corner_radius=1.0)
    assert f"{slot_square}" == "Square('Slot Square',2.000,4.000,corner_radius=1.000)"
    slot_square2: Square = Square("Slot Square2", 4.0, 2.0, corner_radius=1.0)
    assert f"{slot_square2}" == "Square('Slot Square2',4.000,2.000,corner_radius=1.000)"
    rounded_square: Square = Square("Rounded Square", 2.0, 4.0, corner_radius=0.5, corner_count=2)
    assert f"{rounded_square}" == ("Square('Rounded Square',2.000,4.000,"
                                   "corner_radius=0.500,corner_count=2)")

    # Perform some *ValueError* tests:
    value_error: ValueError
    try:
        Square("Negative dx", -1.0, 2.0)
    except ValueError as value_error:
        assert f"{value_error}" == "dx=-1.000 is not positive for 'Negative dx'"
    try:
        Square("Negative dy", 1.0, -2.0)
    except ValueError as value_error:
        assert f"{value_error}" == "dy=-2.000 is not positive for 'Negative dy'"
    try:
        Square("Negative corner_radius", 1.0, 2.0, corner_radius=-1.0)
    except ValueError as value_error:
        assert f"{value_error}" == ("corner_radius=-1.000 must be non-negative for "
                                    "'Negative corner_radius'")
    try:
        Square("Negative corner_count", 1.0, 2.0, corner_radius=0.25, corner_count=-1)
    except ValueError as value_error:
        assert f"{value_error}" == ("corner_count=-1 must be non-negative for "
                                    "'Negative corner_count'")
    try:
        Square("Large corner_radius", 2.0, 2.0, corner_radius=2.0)
    except ValueError as value_error:
        assert f"{value_error}" == ("corner radius=2.000 is larger than "
                                    "half of min(2.000, 2.000)/2.0 for 'Large corner_radius'")
    try:
        Square("Circular Square", 2.0, 2.0, corner_radius=1.0)
    except ValueError as value_error:
        assert f"{value_error}" == ("dx/2=1.000, dy/2=1.000, "
                                    "corner_radius=1.000; use Circle instead!")

    # Now valid that the *Square*.*scad_lines_append*() method works:

    # Make sure that *origin_square* looks right:
    scad_lines: List[str] = []
    origin_square.scad_lines_append(scad_lines, "")
    scad_writer(origin_square, scad_lines)
    assert len(scad_lines) == 2
    assert scad_lines[0] == ("// Square 'Origin Square' dx=2.000 dy=3.000 center=P2D(0.000,0.000) "
                             "corner_radius=0.000 corner_count=3"), "Index 0 failed"
    assert scad_lines[1] == "square([2.000, 3.000], center = true);", "Index 1 failed"

    # Make sure that *offset_square* looks right:
    scad_lines = []
    offset_square.scad_lines_append(scad_lines, "")
    scad_writer(offset_square, scad_lines)
    assert len(scad_lines) == 3
    assert scad_lines[0] == ("// Square 'Offset Square' dx=2.500 dy=3.500 center=P2D(3.000,2.000) "
                             "corner_radius=0.000 corner_count=3"), "Index 0 failed"
    assert scad_lines[1] == "translate([3.000, 2.000])"
    assert scad_lines[2] == " square([2.500, 3.500], center = true);", "Index 2 failed"

    # Make sure that *rotated_square* looks right:
    scad_lines = []
    rotated_square.scad_lines_append(scad_lines, "")
    scad_writer(rotated_square, scad_lines)
    assert len(scad_lines) == 3
    assert scad_lines[0] == ("// Square 'Rotated Square' dx=2.000 dy=3.000 center=P2D(0.000,0.000) "
                             "corner_radius=0.000 corner_count=3"), "Index 0 failed"
    assert scad_lines[1] == "rotate(a = [0, 0, 90.000])", "Index 1 failed"
    assert scad_lines[2] == " square([2.000, 3.000], center = true);", "Index 2 failed"

    # Make sure that *slotted_square* looks right:
    scad_lines = []
    slot_square.scad_lines_append(scad_lines, "")
    scad_writer(slot_square, scad_lines)
    assert len(scad_lines) == 14
    assert scad_lines[0] == ("// Square 'Slot Square' dx=2.000 dy=4.000 center=P2D(0.000,0.000) "
                             "corner_radius=1.000 corner_count=3"), "[0!] failed"
    assert scad_lines[1] == ("polygon(points = [  "
                             "// Begin Square 'Slot Square' 0:17"), "[1]! failed"
    assert scad_lines[2] == (" // Square 'Slot Square' 0-17"), "[2]! failed"
    assert scad_lines[3] == ("  [1.000, 1.000], [0.924, 1.383], [0.707, 1.707], "
                             "[0.383, 1.924],  // 0-4"), "[3]!"
    assert scad_lines[4] == ("  [0.000, 2.000], [-0.383, 1.924], [-0.707, 1.707], "
                             "[-0.924, 1.383],  // 4-8"), "[4]!"
    assert scad_lines[5] == ("  [-1.000, 1.000], [-1.000, -1.000], [-0.924, -1.383], "
                             "[-0.707, -1.707],  // 8-12"), "[5]!"
    assert scad_lines[6] == ("  [-0.383, -1.924], [0.000, -2.000], [0.383, -1.924], "
                             "[0.707, -1.707],  // 12-16"), "[6]!"
    assert scad_lines[7] == ("  [0.924, -1.383], [1.000, -1.000]  // 16-18"), "[7]!"
    assert scad_lines[8] == (" ], paths = ["), "[8]!"
    assert scad_lines[9] == ("  // Square 'Slot Square' 0-17"), "[9]!"
    assert scad_lines[10] == "  [0, 1, 2, 3, 4, 5, 6, 7, 8, 9,", "[10]!"
    assert scad_lines[11] == "   10, 11, 12, 13, 14, 15, 16, 17]", "[11]!"
    assert scad_lines[12] == (" ], convexity=4);  "
                              "// End Square 'Slot Square' 0:17"), "[12]!"

    # Make sure that *slotted_square2* looks right:
    scad_lines = []
    slot_square2.scad_lines_append(scad_lines, "")
    scad_writer(slot_square2, scad_lines)
    assert len(scad_lines) == 14
    assert scad_lines[0] == ("// Square 'Slot Square2' dx=4.000 dy=2.000 center=P2D(0.000,0.000) "
                             "corner_radius=1.000 corner_count=3"), "[0]!"
    assert scad_lines[1] == ("polygon(points = [  "
                             "// Begin Square 'Slot Square2' 0:17"), "[1]!"
    assert scad_lines[2] == (" // Square 'Slot Square2' 0-17"), "[2]!"
    assert scad_lines[3] == ("  [1.000, -1.000], [1.383, -0.924], [1.707, -0.707], "
                             "[1.924, -0.383],  // 0-4"), "[3]!"
    assert scad_lines[4] == ("  [2.000, 0.000], [1.924, 0.383], [1.707, 0.707], "
                             "[1.383, 0.924],  // 4-8"), "[4]!"
    assert scad_lines[5] == ("  [1.000, 1.000], [-1.000, 1.000], [-1.383, 0.924], "
                             "[-1.707, 0.707],  // 8-12"), "[5]!"
    assert scad_lines[6] == ("  [-1.924, 0.383], [-2.000, 0.000], [-1.924, -0.383], "
                             "[-1.707, -0.707],  // 12-16"), "[6]!"
    assert scad_lines[7] == ("  [-1.383, -0.924], [-1.000, -1.000]  // 16-18"), "[7]!"
    assert scad_lines[8] == (" ], paths = ["), "[8]!"
    assert scad_lines[9] == ("  // Square 'Slot Square2' 0-17"), "Index 9 failed"
    assert scad_lines[10] == "  [0, 1, 2, 3, 4, 5, 6, 7, 8, 9,", "[10]!"
    assert scad_lines[11] == "   10, 11, 12, 13, 14, 15, 16, 17]", "[11]!"
    assert scad_lines[12] == (" ], convexity=4);  "
                              "// End Square 'Slot Square2' 0:17"), "[12]!"

    # Make sure that *rounded_square* looks right:
    scad_lines = []
    rounded_square.scad_lines_append(scad_lines, "")
    scad_writer(rounded_square, scad_lines)
    assert len(scad_lines) == 13
    assert scad_lines[0] == ("// Square 'Rounded Square' dx=2.000 dy=4.000 "
                             "center=P2D(0.000,0.000) corner_radius=0.500 corner_count=2"), "[0]!"
    assert scad_lines[1] == ("polygon(points = [  // Begin Square 'Rounded Square' 0:15"), "[1]!"
    assert scad_lines[2] == (" // Square 'Rounded Square' 0-15"), "[2]!"
    assert scad_lines[3] == ("  [1.000, 1.500], [0.933, 1.750], [0.750, "
                             "1.933], [0.500, 2.000],  // 0-4"), "[3]!"
    assert scad_lines[4] == ("  [-0.500, 2.000], [-0.750, 1.933], [-0.933, 1.750], "
                             "[-1.000, 1.500],  // 4-8"), "[4]!"
    assert scad_lines[5] == ("  [-1.000, -1.500], [-0.933, -1.750], [-0.750, -1.933], "
                             "[-0.500, -2.000],  // 8-12"), "[5]!"
    assert scad_lines[6] == ("  [0.500, -2.000], [0.750, -1.933], [0.933, -1.750], "
                             "[1.000, -1.500]  // 12-16"), "[6]!"
    assert scad_lines[7] == (" ], paths = ["), "[7]!"
    assert scad_lines[8] == ("  // Square 'Rounded Square' 0-15"), "[8]!"
    assert scad_lines[9] == "  [0, 1, 2, 3, 4, 5, 6, 7, 8, 9,", "[9]!"
    assert scad_lines[10] == "   10, 11, 12, 13, 14, 15]", "[10]!"
    assert scad_lines[11] == (" ], convexity=4);  // End Square 'Rounded Square' 0:15"), "[11]!"

    # Test *Square*.*key*() method:
    assert rounded_square.key() == ("Square", "Rounded Square", 0.0, 0.0, 2.0,
                                    4.0, 0.0, 0.5, 2), "key failed"


def test_union() -> None:
    """Test Union class."""
    square0: Square = Square("Square 0", 2.0, 2.0)
    square1: Square = Square("Square 1", 2.0, 2.0, center=P2D(1.0, 1.0))
    square2: Square = Square("Square 2", 2.0, 2.0, center=P2D(-1.0, -1.0))
    union: scad.Union = scad.Union("Squares Union", [square0, square1, square2])
    scad_lines: List[str] = []
    union.scad_lines_append(scad_lines, "")
    scad_writer(union, scad_lines)

    cube0: LinearExtrude = LinearExtrude("Cube", square0, 2.0)
    try:
        Union("Bogus Union", [cube0, square0])
    except ValueError as value_error:
        assert f"{value_error}" == ("Index 0 of Union is class 'LinearExtrude,' "
                                    "but index 1 is class 'Square'")


if __name__ == "__main__":  # pragma: no cover
    test_polygon()
