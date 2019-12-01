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
import romi_model.scad
from romi_model.scad import P, Polygon, Scad, ScadPolygon

def test_point():
    """Test the point class."""
    origin: P = P()
    assert f"{origin}" == "P(0.000, 0.000, 0.000)"
    p111: P = P(1.0, 1.0, 1.0)
    assert f"{p111}" == "P(1.000, 1.000, 1.000)"
    p123: P = P(1.0, 2.0, 3.0)
    assert f"{p123}" == "P(1.000, 2.000, 3.000)"

    # Addition:
    assert f"{origin + p111}" == "P(1.000, 1.000, 1.000)"

    # Subtraction:
    assert f"{p123 - p111}" == "P(0.000, 1.000, 2.000)"

    # Left/Right Multiplication:
    assert f"{p123 * 2.0}" == "P(2.000, 4.000, 6.000)"
    assert f"{2.0 * p123}" == "P(2.000, 4.000, 6.000)"    

    # Test distance:
    p100: P = P(1.0, 0.0, 0.0)
    assert f"{origin.distance(p100)}" == "1.0"
    p010: P = P(0.0, 1.0, 0.0)
    assert f"{origin.distance(p010)}" == "1.0"
    p001: P = P(0.0, 0.0, 1.0)
    assert f"{origin.distance(p001)}" == "1.0"

    # Division scaling:
    assert f"{p123 / 2.0}" == "P(0.500, 1.000, 1.500)"
    
    # P.rotate2d:
    assert f"{p100.rotate2d(pi/2.0)}" == "P(0.000, 1.000, 0.000)"

    # P.y_mirror:
    assert f"{p123.y_mirror()}" == "P(-1.000, 2.000, -3.000)"

def test_polygon():
    # Test *empty_polygon*:
    empty_polygon: Polygon = Polygon("Empty")
    assert f"{empty_polygon}" == "Polygon('Empty', [])"

    # Test both *Polygon.*__str__*() and the *Polygon*.size_get*() methods:
    p11: P = P(1.0, 1.0)
    p22: P = P(2.0, 2.0)
    p33: P = P(3.0, 3.0)
    no_points: List[P] = []
    one_point: List[P] = [p11]
    two_points: List[P] = [p11, p22]
    three_points: List[P] = [p11, p22, p33]
    polygon0: Polygon = Polygon("No Points")
    assert len(polygon0) == 0
    assert f"{polygon0}" == "Polygon('No Points', [])"
    polygon1: Polygon = Polygon("One Point", [p11])
    assert len(polygon1) == 1
    assert f"{polygon1}" == "Polygon('One Point', [P[1.000, 1.000]])"
    polygon2: Polygon = Polygon("Two Points", [p11, p22])
    assert len(polygon2) == 2
    assert f"{polygon2}" == "Polygon('Two Points', [P[1.000, 1.000], P[2.000, 2.000]])"
    polygon3: Polygon = Polygon("Three Points", [p11, p22, p33])
    assert len(polygon3) == 3
    assert f"{polygon3}" == "Polygon('Three Points', [P[1.000, 1.000], ..., P[3.000, 3.000]])"
    
    # Now test the *Polygon*.*__getitem__*() method:
    assert f"{polygon1[0]}" == "P(1.000, 1.000, 0.000)"
    assert f"{polygon2[0]}" == "P(1.000, 1.000, 0.000)"
    assert f"{polygon2[1]}" == "P(2.000, 2.000, 0.000)"
    assert f"{polygon3[0]}" == "P(1.000, 1.000, 0.000)"
    assert f"{polygon3[1]}" == "P(2.000, 2.000, 0.000)"
    assert f"{polygon3[2]}" == "P(3.000, 3.000, 0.000)"

    # Test Polygon.arc_append() and Polygon.point_append:
    slot: Polygon = Polygon("Slot")
    slot.arc_append(P(1.0, 0.0), 1.0, -pi/2, pi/2, 3)
    slot.point_append(P(0.0, 2.0))
    slot.arc_append(P(-1.0, 0.0), 1.0, pi/2, 3.0 * pi/2, 3)
    slot.point_append(P(0.0, -2.0))
    assert len(slot) == 8
    assert f"{slot[0]}" == "P(1.000, -1.000, 0.000)"
    assert f"{slot[1]}" == "P(2.000, 0.000, 0.000)"
    assert f"{slot[2]}" == "P(1.000, 1.000, 0.000)"
    assert f"{slot[3]}" == "P(0.000, 2.000, 0.000)"
    assert f"{slot[4]}" == "P(-1.000, 1.000, 0.000)"
    assert f"{slot[5]}" == "P(-2.000, 0.000, 0.000)"
    assert f"{slot[6]}" == "P(-1.000, -1.000, 0.000)"
    assert f"{slot[7]}" == "P(0.000, -2.000, 0.000)"
    
    # Test Polygon.circle():
    hole: Polygon = Polygon("Circle")
    center: P = P(1.0, 0.0)
    hole.circle_append(center, 2.0, 4)
    # assert False, [f"{point}" for point in hole.points]

    assert len(hole) == 4
    assert f"{hole[0]}" == "P(2.000, 0.000, 0.000)"
    assert f"{hole[1]}" == "P(1.000, 1.000, 0.000)"
    assert f"{hole[2]}" == "P(0.000, 0.000, 0.000)"
    assert f"{hole[3]}" == "P(1.000, -1.000, 0.000)"

    # Test Polygon.rotated_rectangle_append():
    rotated_rectangle: Polygon = Polygon("Rotated Rectangle")
    rectangle_center: P = P(2.0, 2.0)
    rotated_rectangle.rotated_rectangle_append(P(2.0, 3.0), 2.0, 4.0, pi/2)
    assert f"{rotated_rectangle[0]}" == "P(0.000, 4.000, 0.000)", "Index 0 failed"
    assert f"{rotated_rectangle[1]}" == "P(4.000, 4.000, 0.000)", "Index 1 failed"
    assert f"{rotated_rectangle[2]}" == "P(4.000, 2.000, 0.000)", "Index 2 failed"
    assert f"{rotated_rectangle[3]}" == "P(0.000, 2.000, 0.000)", "Index 3 failed"
    
    # Test Polygon.slot_append():
    slot: Polygon = Polygon("Slot")
    slot.slot_append(P(1.0, 1.0), P(3.0, 1.0), 4.0, 2.0, 3)
    assert f"{slot[0]}" == "P(0.000, 2.000, 0.000)", "Index 0 failed"
    assert f"{slot[1]}" == "P(-1.000, 1.000, 0.000)", "Index 1 failed"
    assert f"{slot[2]}" == "P(0.000, 0.000, 0.000)", "Index 2 failed"
    assert f"{slot[3]}" == "P(4.000, 0.000, 0.000)", "Index 3 failed"
    assert f"{slot[4]}" == "P(5.000, 1.000, 0.000)", "Index 4 failed"
    assert f"{slot[5]}" == "P(4.000, 2.000, 0.000)", "Index 5 failed"

    # Test Polygon.points_scad_lines_append():
    scad_lines: List[str] = []
    start_index: int = 3
    end_index: int = slot.points_scad_lines_append(scad_lines, " ", start_index)
    assert end_index == start_index + len(slot)
    assert len(scad_lines) == 3
    assert scad_lines[0] == "  // Polygon 'Slot' 3:8", "Index 0 failed"
    assert scad_lines[1] == ("   [0.000, 2.000], [-1.000, 1.000], "
                             "[0.000, 0.000], [4.000, 0.000], // 3:6"), "Index 1 failed"
    assert scad_lines[2] == ("   [5.000, 1.000], [4.000, 2.000], // 7:8"), "Index 2 failed"

    # Test Polygon.indices_scad_lines_append():
    scad_lines = []
    end_index: int = slot.indices_scad_lines_append(scad_lines, " ", start_index)
    assert end_index == start_index + len(slot)
    assert len(scad_lines) == 2
    assert scad_lines[0] == "  // Polygon 'Slot' 3:8"
    assert scad_lines[1] == "   [3, 4, 5, 6, 7, 8],"


def test_scad_polygon():
    # Define the four corners of a square:
    print("Entered test_scad_polygon")
    upper_right: P = P(2.0, 2.0)
    lower_right: P = P(2.0, -2.0)
    lower_left: P = P(-2.0, -2.0)
    upper_left: P = P(-2.0, 2.0)

    # Convert the four corners into an *outer_polygon*:
    square_polygon: Polygon = Polygon("Square Polygon",
                                      [upper_right, lower_right, lower_left, upper_left])

    # Create *square_scad_polygon*:
    square_scad_polygon: ScadPolygon = ScadPolygon("Square SCADPolygon", [square_polygon])
    scad_lines: List[str] = []
    square_scad_polygon.scad_lines_append(scad_lines, " ")
    assert scad_lines[0] == " // ScadPolygon 'Square SCADPolygon [0-0]'"
    assert scad_lines[1] == " polygon(points = ["
    assert scad_lines[2] == "   // Polygon 'Square Polygon' 0:3"
    assert scad_lines[3] == ("    [2.000, 2.000], [2.000, -2.000], "
                             "[-2.000, -2.000], [-2.000, 2.000], // 0:3")
    assert scad_lines[4] == "  ], paths = ["
    assert scad_lines[5] == "   // Polygon 'Square Polygon' 0:3"
    assert scad_lines[6] == "    [0, 1, 2, 3],"
    assert scad_lines[7] == "  ]); // End ScadPolygon 'Square SCADPolygon' [0-0]"

    # Test Scad.file_write():
    scad_file: IO[Any]
    # with open("/tmp/test_scad.scad", "w") as scad_file:
    #     square_scad_polygon.file_write(scad_file)
    with io.StringIO("") as scad_file:
        square_scad_polygon.file_write(scad_file)
        input_scad_text: str = scad_file.getvalue()
        input_scad_lines: List[str] = input_scad_text.split('\n')
        assert len(input_scad_lines) == 10
        assert input_scad_lines[0] == "// 'Square SCADPolygon' File", "Index 0 failed"
        assert input_scad_lines[1] == "// ScadPolygon 'Square SCADPolygon [0-0]'", "Index 1 failed"
        assert input_scad_lines[2] == "polygon(points = [", "Index 2 failed"
        assert input_scad_lines[3] ==  "  // Polygon 'Square Polygon' 0:3", "Index 3 failed"
        assert input_scad_lines[4] == ("   [2.000, 2.000], [2.000, -2.000], [-2.000, -2.000], "
                                       "[-2.000, 2.000], // 0:3"), "Index 4 failed"
        assert input_scad_lines[5] == " ], paths = [", "Index 5 failed"
        assert input_scad_lines[6] == "  // Polygon 'Square Polygon' 0:3", "Index 6 failed"
        assert input_scad_lines[7] == "   [0, 1, 2, 3],", "Index 7 failed"
        assert input_scad_lines[8] == (" ]); // End ScadPolygon 'Square SCADPolygon' "
                                       "[0-0]"), "Index 8 failed"
        assert input_scad_lines[9] == "", "Index 9 failed"

    # Test the *ScadPolygon*.*__getitem__*() and *ScadPolygon*.*__size__*() methods:
    new_square_scad_polygon: ScadPolygon = ScadPolygon("New Square ScadPolygon", [])
    assert len(new_square_scad_polygon) == 0
    new_square_scad_polygon.append(square_polygon)
    assert len(new_square_scad_polygon) == 1
    assert new_square_scad_polygon[0] is square_polygon

    # Now Test the *ScadPolygon*.*append*() and *ScadPolygon*.*extend*() methods:
    hole_polygons: List[Polygon] = list()
    diameter: float = 0.25
    x: float
    for x in (-1.0, 0.0, 1.0):
        y: float
        for y in (-1.0, 0.0, 1.0):
            center: P = P(x, y)
            hole_polygon: Polygon = Polygon(f"Hole[{x, y}]")
            hole_polygon.circle_append(center, diameter, 8)
            hole_polygons.append(hole_polygon)
    new_square_scad_polygon.extend(hole_polygons)
    assert len(new_square_scad_polygon) == 10

    # Test ScadPolygon.append() and ScadPolygon.extend():


if __name__ == "__main__":
    test_scad_polygon()
