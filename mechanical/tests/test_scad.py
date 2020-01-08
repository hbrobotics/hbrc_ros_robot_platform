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
from math import pi, sqrt
from scad_models.scad import (Circle, Color, CornerCube, Cube, Cylinder, If2D, If3D, LinearExtrude,
                              Module2D, Module3D, P2D, P3D, Polygon, Rotate3D, Scad3D, ScadProgram,
                              SimplePolygon, Square, Translate3D, Union3D, UseModule2D,
                              UseModule3D, Variable2D)
import scad_models.scad as scad
from typing import Any, IO, List, Tuple


# def scad_writer(scad: Scad, scad_lines: List[str]) -> None:
#     """Write out a Square to a `.scad` file."""
#     scad_file: IO[Any]
#     name: str = scad.name
#     with open(name.replace(' ', '_') + ".scad", "w") as scad_file:
#         scad_text: str = '\n'.join(scad_lines)
#         scad_file.write(scad_text + "\n")


# All `test_*` functions are alphabetized, since oder of execution does not really matter:


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

    # Try out the *Circle*.*copy*() method:
    small_circle: Circle = Circle("Small Circle", 2.0, 8, center)
    assert f"{small_circle}" == "Circle('Small Circle',2.0,8,P2D(2.000,3.000),4)"
    new_center: P2D = P2D(4.0, 5.0)
    big_circle: Circle = small_circle.copy("Small", diameter=8.0,
                                           points_count=16, center=new_center, replace="Big")
    assert f"{big_circle}" == "Circle('Big Circle',8.0,16,P2D(4.000,5.000),4)"

    # Validate *Circle*.*key*() method:
    key: Tuple[Any] = circle.key()
    assert key == ("Circle", "Circle", 2.0, 3.0, 2.0, 2.0, 0.0), f"Bad key: {key}"

    # Validate *Circle*.*x_mirror*()
    x_mirrored_circle: Circle = circle.x_mirror("X Mirrored Circle")
    x_mirrored_center: P2D = x_mirrored_circle.center
    assert x_mirrored_center.x == 2.0
    assert x_mirrored_center.y == -3.0

    # Validate *Circle*.*y_mirror*()
    y_mirrored_circle: Circle = circle.y_mirror("Y Mirrored Circle")
    y_mirrored_center: P2D = y_mirrored_circle.center
    assert y_mirrored_center.x == -2.0
    assert y_mirrored_center.y == 3.0


def test_color() -> None:
    """Test Color class."""
    cube1: Cube = Cube("Cube 1", 1.0, 2.0, 3.0)
    colored_cube1: Color = Color("Blue Cube", cube1, "blue", alpha=0.5)
    assert str(colored_cube1) == ("Color('Blue Cube',Cube('Cube 1',1.000,2.000,3.000,"
                                  "center=P3D(0.000,0.000,0.000)),'blue',alpha=0.50)")
    scad_lines: List[str] = []
    colored_cube1.scad_lines_append(scad_lines, "")
    assert len(scad_lines) == 3
    assert scad_lines[0] == ('color("blue", a = 0.50) {  '
                             "// Color: 'Blue Cube'"), "[0]!"
    assert scad_lines[1] == (" cube(size = [1.000, 2.000, 3.000], center = true);  "
                             "// Cube: 'Cube 1'"), "[1]!"
    assert scad_lines[2] == "}", "[2]!"

    # Now verify that argument checking works:
    try:
        Color("Ugly Color", cube1, "ugly")
        assert False, "This line should not be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "Color 'ugly is not a recognized color."
    try:
        Color("Bad Aplha", cube1, "Blue", alpha=-1.0)
        assert False, "This line should not be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "Alpha (-1.0) must be between 0.0 and 1.0 inclusive."


def test_cube() -> None:
    """Test Cube class."""
    # Create a *centered_cube*:
    centered_cube: Cube = Cube("Centered Cube", 1.0, 2.0, 3.0)
    assert str(centered_cube) == ("Cube('Centered Cube',"
                                  "1.000,2.000,3.000,center=P3D(0.000,0.000,0.000))")

    # Output to *scad_lines*:
    scad_lines: List[str] = []
    centered_cube.scad_lines_append(scad_lines, "")
    assert len(scad_lines) == 1
    assert scad_lines[0] == ("cube(size = [1.000, 2.000, 3.000], center = true);  "
                             "// Cube: 'Centered Cube'")

    # Create a *non_centered_cube*:
    non_centered_cube: Cube = Cube("Non-Centered Cube", 1.0, 2.0, 3.0, center=P3D(1.5, 2.5, 3.5))
    scad_lines = []
    non_centered_cube.scad_lines_append(scad_lines, "")
    assert len(scad_lines) == 3
    assert scad_lines[0] == "translate(v = [1.500, 2.500, 3.500]) {", "[0]!"
    assert scad_lines[1] == (" cube(size = [1.000, 2.000, 3.000], center = true);  "
                             "// Cube: 'Non-Centered Cube'"), "[1]!"
    assert scad_lines[2] == "}", "[2]!"

    # Verify that we fail for non-positive values of dx, dy, and dz:
    try:
        Cube("Cube DX=0", 0.0, 1.0, 2.0)
        assert False, "We should not reach this line"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "Cube 'Cube DX=0' dx=0.0 is not positive"
    try:
        Cube("Cube DY=0", 1.0, 0.0, 2.0)
        assert False, "We should not reach this line"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "Cube 'Cube DY=0' dy=0.0 is not positive"
    try:
        Cube("Cube DZ=0", 1.0, 2.0, 0.0)
        assert False, "We should not reach this line"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "Cube 'Cube DZ=0' dz=0.0 is not positive"


def test_corner_cube() -> None:
    """Test CornerCube class."""
    # Create a *corner_cube*:
    corner_cube1: CornerCube = CornerCube("CornerCube 1", P3D(1.0, 2.0, 3.0), P3D(4.0, 5.0, 6.0))
    assert str(corner_cube1) == ("Cube('CornerCube 1',"
                                 "3.000,3.000,3.000,center=P3D(2.500,3.500,4.500))")

    # Output to *scad_lines*:
    scad_lines: List[str] = []
    corner_cube1.scad_lines_append(scad_lines, "")
    assert len(scad_lines) == 3
    assert scad_lines[0] == "translate(v = [2.500, 3.500, 4.500]) {", "[0]!"
    assert scad_lines[1] == (" cube(size = [3.000, 3.000, 3.000], "
                             "center = true);  // Cube: 'CornerCube 1'"), "[1]!"
    assert scad_lines[2] == "}", "[2]!"

    # Verify that we detect zero volume errors:
    try:
        CornerCube("CornerCube DX=0", P3D(0.0, 0.0, 0.0), P3D(0.0, 1.0, 1.0))
        assert False, "This line should never be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "CornerCube 'CornerCube DX=0' has dx of 0.0"
    try:
        CornerCube("CornerCube DY=0", P3D(0.0, 0.0, 0.0), P3D(1.0, 0.0, 1.0))
        assert False, "This line should never be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "CornerCube 'CornerCube DY=0' has dy of 0.0"
    try:
        CornerCube("CornerCube DZ=0", P3D(0.0, 0.0, 0.0), P3D(1.0, 1.0, 0.0))
        assert False, "This line should never be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "CornerCube 'CornerCube DZ=0' has dz of 0.0"


def test_cylinder() -> None:
    """Test Cylinder class."""
    # Do the simplest cylinder command possible:
    one_above: P3D = P3D(0.0, 0.0, 1.0)
    one_below: P3D = P3D(0.0, 0.0, -1.0)
    centered_cylinder: Cylinder = Cylinder("Centered Cylinder", 1.0, one_below, one_above, 16)
    assert f"{centered_cylinder}" == ("Cylinder('Centered Cylinder',1.0,"
                                      "P3D(0.000,0.000,-1.000),P3D(0.000,0.000,1.000),16)")
    scad_lines: List[str] = []
    centered_cylinder.scad_lines_append(scad_lines, "")
    assert len(scad_lines) == 1
    assert scad_lines[0] == ("  cylinder(h = 2.000, d = 1.000, $fn = 16, center = true);  "
                             "// Cylinder: 'Centered Cylinder'"), "[0]!"

    # Now do a translated cylinder:
    origin: P3D = P3D(0.0, 0.0, 0.0)
    origin_start_cylinder: Cylinder = Cylinder("Origin Start Cylinder", 1.0, origin, one_above, 16)
    scad_lines = []
    origin_start_cylinder.scad_lines_append(scad_lines, "")
    assert len(scad_lines) == 2
    assert scad_lines[0] == "translate(v = [0.000, 0.000, 0.500])", "[0]!"
    assert scad_lines[1] == ("  cylinder(h = 1.000, d = 1.000, $fn = 16, center = true);  "
                             "// Cylinder: 'Origin Start Cylinder'"), "[1]!"

    # Now do x_axis cylinder:
    x_axis: P3D = P3D(1.0, 0.0, 0.0)
    x_axis_cylinder: Cylinder = Cylinder("X Axis Cylinder", 1.0, -x_axis, x_axis, 16)
    scad_lines = []
    x_axis_cylinder.scad_lines_append(scad_lines, "")
    assert len(scad_lines) == 2
    assert scad_lines[0] == " rotate(a = 90.0, v = [0.000, 2.000, 0.000])", "[0]!"
    assert scad_lines[1] == ("  cylinder(h = 2.000, d = 1.000, $fn = 16, center = true);  "
                             "// Cylinder: 'X Axis Cylinder'"), "[1]!"

    # Make sure we detect zero height cylinder:
    try:
        Cylinder("Zero Height Cylinder", 1.0, origin, origin, 16)
        assert False, "This line should never be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "Cylinder 'Zero Height Cylinder' does not have positive height."


def test_if2d() -> None:
    """Test If2D class."""
    # Create some circles:
    circle1: Circle = Circle("Circle 1", 1.0, 8)
    circle2: Circle = Circle("Circle 2", 2.0, 12)
    circle3: Circle = Circle("Circle 3", 3.0, 16)

    # Create first *if2d1* and fill it in:
    if2d1: If2D = If2D("If2D 1", "n == 1", [circle1])
    assert str(if2d1) == "If2D('If2D 1',...,lock=False)"
    if2d1.then_append("n == 2", [circle2])
    if2d1.else_set([circle3])

    # Verify that attempting to set the else clause more than onces fails:
    try:
        if2d1.else_set([circle1])
        assert False, "This should never be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "If2D('If2D 1)' else clause is already set."

    # Now verify that we can not do a *scad_lines_append* until after it is locked:
    scad_lines: List[str] = []
    try:
        if2d1.scad_lines_append(scad_lines, "")
        assert False, "This should never be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "If2D 'If2D 1' is not locked."

    # Now lock *if2d1* and verify that then and else clauses can not be appended:
    if2d1.lock()
    try:
        if2d1.then_append("bogus", [circle2])
        assert False, "This line should never be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "If2D 'If2D 1' is and locked can not accept another then clause"
    try:
        if2d1.else_set([circle3])
        assert False, "This line should never be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "If2D('If2D 1)' is locked and can not have an else clause set."

    # Now generate the *scad_lines*:
    if2d1.scad_lines_append(scad_lines, "")
    assert len(scad_lines) == 7
    assert scad_lines[0] == "if (n == 1) {  // If2D 'If2D 1'", "[0]!"
    assert scad_lines[1] == " circle(d=1.000, $fn=8);  // Circle 'Circle 1'", "[1]!"
    assert scad_lines[2] == "} else if (n == 2) {", "[2]!"
    assert scad_lines[3] == " circle(d=2.000, $fn=12);  // Circle 'Circle 2'", "[3]!"
    assert scad_lines[4] == "} else {", "[4]!"
    assert scad_lines[5] == " circle(d=3.000, $fn=16);  // Circle 'Circle 3'", "[5]!"
    assert scad_lines[6] == "}  // End If2D 'If2D 1'", "[6]!"

    # Do another on with only one then clause and no else clause:
    if2d2: If2D = If2D("If2D 2", "n == 0", [circle1])
    if2d2.lock()

    # Ensure that we can not set the else clause when it is locked:
    try:
        if2d2.else_set([circle2])
        assert False, "This line should never be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "If2D('If2D 2)' is locked and can not have an else clause set."

    # Now make sure that hte *scad_lines* are OK:
    scad_lines = []
    if2d2.scad_lines_append(scad_lines, "")
    assert len(scad_lines) == 3
    assert scad_lines[0] == "if (n == 0) {  // If2D 'If2D 2'", "[0]!"
    assert scad_lines[1] == " circle(d=1.000, $fn=8);  // Circle 'Circle 1'", "[1]!"
    assert scad_lines[2] == "}  // End If2D 'If2D 2'", "[2]!"

    # Do a quick test of the *name_match_append* method:
    empty_module2d: Module2D = Module2D("Empty Module", [])
    if2d_name_match: If2D = If2D("Name Match IF2D", 'name == ""', [])
    if2d_name_match.name_match_append("random_name", empty_module2d, ["Line 1", "Line2"])
    if2d_name_match.lock()
    scad_lines = []
    if2d_name_match.scad_lines_append(scad_lines, "")
    assert len(scad_lines) == 4
    assert scad_lines[0] == "if (name == \"\") {  // If2D 'Name Match IF2D'", "[0]!"
    assert scad_lines[1] == '} else if (name == "random_name") {', "[1]!"
    assert scad_lines[2] == " Empty_Module(); // UseModule2D('random_name Use Module')", "[2]!"
    assert scad_lines[3] == "}  // End If2D 'Name Match IF2D'", "[3]!"


def test_if3d() -> None:
    """Test If3D class."""
    # Create some circles:
    circle1: Circle = Circle("Circle 1", 1.0, 8)
    extruded_circle1: Scad3D = LinearExtrude("Extuded Circle 1", circle1, 3.0)
    circle2: Circle = Circle("Circle 2", 2.0, 12)
    extruded_circle2: Scad3D = LinearExtrude("Extuded Circle 2", circle2, 2.0)
    circle3: Circle = Circle("Circle 3", 3.0, 16)
    extruded_circle3: Scad3D = LinearExtrude("Extuded Circle 3", circle3, 1.0)

    # Create first *if3d1* and fill it in:
    if3d1: If3D = If3D("If3D 1", "n == 1", [extruded_circle1])
    assert str(if3d1) == "If3D('If3D 1',...,lock=False)"
    if3d1.then_append("n == 2", [extruded_circle2])
    if3d1.else_set([extruded_circle3])

    # Verify that attempting to set the else clause more than onces fails:
    try:
        if3d1.else_set([extruded_circle1])
        assert False, "This should never be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "If3D('If3D 1)' else clause is already set."

    # Now verify that we can not do a *scad_lines_append* until after it is locked:
    scad_lines: List[str] = []
    try:
        if3d1.scad_lines_append(scad_lines, "")
        assert False, "This should never be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "If3D 'If3D 1' is not locked."

    # Now lock *if3d1* and verify that then and else clauses can not be appended:
    if3d1.lock()
    try:
        if3d1.then_append("bogus", [extruded_circle2])
        assert False, "This line should never be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "If3D 'If3D 1' is and locked can not accept another then clause"
    try:
        if3d1.else_set([extruded_circle3])
        assert False, "This line should never be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "If3D('If3D 1)' is locked and can not have an else clause set."

    # Now generate the *scad_lines*:
    if3d1.scad_lines_append(scad_lines, "")
    assert len(scad_lines) == 16
    assert scad_lines[0] == "if (n == 1) {  // If3D 'If3D 1'", "[0]!"
    assert scad_lines[1] == " // Begin LinearExtrude 'Extuded Circle 1'", "[1]!"
    assert scad_lines[2] == (" linear_extrude(height=3.0, center=false, "
                             "convexity=10, twist=0.0)"), "[2]!"
    assert scad_lines[3] == "  circle(d=1.000, $fn=8);  // Circle 'Circle 1'", "[3]!"
    assert scad_lines[4] == " // End LinearExtrude 'Extuded Circle 1'", "[4]!"
    assert scad_lines[5] == "} else if (n == 2) {", "[5]!"
    assert scad_lines[6] == " // Begin LinearExtrude 'Extuded Circle 2'", "[6]!"
    assert scad_lines[7] == (" linear_extrude(height=2.0, center=false, "
                             "convexity=10, twist=0.0)"), "[7]!"
    assert scad_lines[8] == "  circle(d=2.000, $fn=12);  // Circle 'Circle 2'", "[8]!"
    assert scad_lines[9] == " // End LinearExtrude 'Extuded Circle 2'", "[9]!"
    assert scad_lines[10] == "} else {", "[10]!"
    assert scad_lines[11] == " // Begin LinearExtrude 'Extuded Circle 3'", "[11]!"
    assert scad_lines[12] == (" linear_extrude(height=1.0, center=false, "
                              "convexity=10, twist=0.0)"), "[12]!"
    assert scad_lines[13] == "  circle(d=3.000, $fn=16);  // Circle 'Circle 3'", "[13]!"
    assert scad_lines[14] == " // End LinearExtrude 'Extuded Circle 3'", "[14]!"
    assert scad_lines[15] == "}  // End If3D 'If3D 1'", "[15]!"

    # Do another on with only one then clause and no else clause:
    if3d2: If3D = If3D("If3D 2", "n == 0", [extruded_circle1])
    if3d2.lock()

    # Ensure that we can not set the else clause when it is locked:
    try:
        if3d2.else_set([extruded_circle2])
        assert False, "This line should never be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "If3D('If3D 2)' is locked and can not have an else clause set."

    # Now make sure that the *scad_lines* are OK:
    scad_lines = []
    if3d2.scad_lines_append(scad_lines, "")
    # scad_writer(if3d2, scad_lines)
    assert len(scad_lines) == 6
    assert scad_lines[0] == "if (n == 0) {  // If3D 'If3D 2'", "[0]!"
    assert scad_lines[1] == " // Begin LinearExtrude 'Extuded Circle 1'", "[1]!"
    assert scad_lines[2] == (" linear_extrude(height=3.0, center=false, "
                             "convexity=10, twist=0.0)"), "[2]!"
    assert scad_lines[3] == "  circle(d=1.000, $fn=8);  // Circle 'Circle 1'", "[3]!"
    assert scad_lines[4] == " // End LinearExtrude 'Extuded Circle 1'", "[4]!"
    assert scad_lines[5] == "}  // End If3D 'If3D 2'", "[5]!"

    # Do a quick test of the *name_match_append* method:
    empty_module3d: Module3D = Module3D("Empty Module", [])
    if3d_name_match: If3D = If3D("Name Match IF3D", 'name == ""', [])
    if3d_name_match.name_match_append("random_name", empty_module3d, ["Line 1", "Line2"])
    if3d_name_match.lock()
    scad_lines = []
    if3d_name_match.scad_lines_append(scad_lines, "")
    assert len(scad_lines) == 4
    assert scad_lines[0] == "if (name == \"\") {  // If3D 'Name Match IF3D'", "[0]!"
    assert scad_lines[1] == '} else if (name == "random_name") {', "[1]!"
    assert scad_lines[2] == " Empty_Module(); // UseModule3D('random_name Use Module')", "[2]!"
    assert scad_lines[3] == "}  // End If3D 'Name Match IF3D'", "[3]!"


def test_linear_extrude() -> None:
    """Test LinearExtrude class."""
    unit_square: Square = Square("Unit Square", 1.0, 1.0)
    linear_extrude: LinearExtrude = LinearExtrude("Linear Extrude", unit_square, 1.0)
    scad_lines: List[str] = []
    linear_extrude.scad_lines_append(scad_lines, "")
    # scad_writer(linear_extrude, scad_lines)
    assert len(scad_lines) == 5
    assert scad_lines[0] == ("// Begin LinearExtrude 'Linear Extrude'"), "[0]!"
    assert scad_lines[1] == ("linear_extrude(height=1.0, center=false, "
                             "convexity=10, twist=0.0)"), "[1]!"
    assert scad_lines[2] == (" // Square 'Unit Square' dx=1.000 dy=1.000 center=P2D(0.000,0.000) "
                             "corner_radius=0.000 corner_count=3"), "[2]!"
    assert scad_lines[3] == (" square([1.000, 1.000], center = true);"), "[3]!"
    assert scad_lines[4] == ("// End LinearExtrude 'Linear Extrude'"), "[4]!"


def test_module2d() -> None:
    """Test Module2D class."""
    # Create some *Scad2D* objects to play with:
    circle1: Circle = Circle("Circle 1", 10.0, 16)
    square1: Square = Square("Sqaare 1", 10.0, 10.0)
    square2: Square = Square("Square 2", 20.0, 20.0)

    # Work on len(), str()
    module2d1: Module2D = Module2D("Module2D 1", [], lock=False)
    assert str(module2d1) == "Module2D('Module2D 1',[...],is_operator=False,lock=False)"
    assert len(module2d1) == 0
    module2d1.append(circle1)
    assert len(module2d1) == 1
    module2d1.extend([square1, square2])
    assert len(module2d1) == 3

    # Attempt to perform *scad_lines_append*() in unlocked state:
    scad_lines: List[str] = []
    try:
        module2d1.scad_lines_append(scad_lines, "")
        assert False, "scad_lines_append() should have failed"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "Module2D 'Module2D 1' is not locked yet."

    # Now lock *module2d1* and verify that *append* and *extend*() do not work:
    module2d1.lock()
    try:
        module2d1.append(circle1)
        assert False, "append() should have failed"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "Can not append to Module2D 'Module2D 1' because is locked"
    try:
        module2d1.extend([circle1])
        assert False, "extend() should have failed"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "Can not extend Module2D 'Module2D 1' because is locked"

    # Now test *__getitem__* method:
    assert module2d1[0] == circle1
    assert module2d1[1] == square1
    assert module2d1[2] == square2
    try:
        module2d1[3]
        assert False, "__getiteme__ should have failed."  # pragma: no cover
    except IndexError as index_error:
        assert f"{index_error}" == "Index 3 exceeds 3 objects in Module2D 'Module2D 1'"

    # Now verify that *scad_lines_append*() works:
    module2d1.scad_lines_append(scad_lines, "")
    assert len(scad_lines) == 7
    assert scad_lines[0] == "module Module2D_1() {", "[0]!"
    assert scad_lines[1] == " circle(d=10.000, $fn=16);  // Circle 'Circle 1'", "[1]!"
    assert scad_lines[2] == (" // Square 'Sqaare 1' dx=10.000 dy=10.000 center=P2D(0.000,0.000) "
                             "corner_radius=0.000 corner_count=3"), "[2]!"
    assert scad_lines[3] == " square([10.000, 10.000], center = true);", "[3]!"
    assert scad_lines[4] == (" // Square 'Square 2' dx=20.000 dy=20.000 center=P2D(0.000,0.000) "
                             "corner_radius=0.000 corner_count=3"), "[4]!"
    assert scad_lines[5] == " square([20.000, 20.000], center = true);", "[5]!"
    assert scad_lines[6] == "}", "[6]!"

    # Verify that *is_operator* attribute can be set:
    module2d2: Module2D = Module2D("Module2D 2", [], is_operator=True)
    assert f"{module2d2}" == "Module2D('Module2D 2',[...],is_operator=True,lock=True)"


def test_module3d() -> None:
    """Test Module3D class."""
    # Create some *Scad3D* objects to play with:
    cube1: Cube = Cube("Cube 1", 1.0, 1.0, 1.0)
    cube2: Cube = Cube("Cube 2", 2.0, 2.0, 2.0)
    cube3: Cube = Cube("Cube 3", 3.0, 3.0, 3.0)

    # Work on len(), str()
    module3d1: Module3D = Module3D("Module3D 1", [], lock=False)
    assert str(module3d1) == "Module3D('Module3D 1',[...],is_operator=False,lock=False)"
    assert len(module3d1) == 0
    module3d1.append(cube1)
    assert len(module3d1) == 1
    module3d1.extend([cube2, cube3])
    assert len(module3d1) == 3

    # Attempt to perform *scad_lines_append*() in unlocked state:
    scad_lines: List[str] = []
    try:
        module3d1.scad_lines_append(scad_lines, "")
        assert False, "scad_lines_append() should have failed"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "Module3D 'Module3D 1' is not locked yet."

    # Now lock *module3d1* and verify that *append* and *extend*() do not work:
    module3d1.lock()
    try:
        module3d1.append(cube1)
        assert False, "append() should have failed"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "Can not append to Module3D 'Module3D 1' because is locked"
    try:
        module3d1.extend([cube1])
        assert False, "extend() should have failed"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "Can not extend Module3D 'Module3D 1' because is locked"

    # Now test *__getitem__* method:
    assert module3d1[0] == cube1
    assert module3d1[1] == cube2
    assert module3d1[2] == cube3
    try:
        module3d1[3]
        assert False, "__getiteme__ should have failed."  # pragma: no cover
    except IndexError as index_error:
        assert f"{index_error}" == "Index 3 exceeds 3 objects in Module3D 'Module3D 1'"

    # Now verify that *scad_lines_append*() works:
    module3d1.scad_lines_append(scad_lines, "")
    assert len(scad_lines) == 5
    assert scad_lines[0] == "module Module3D_1() {", "[0]!"
    assert scad_lines[1] == (" cube(size = [1.000, 1.000, 1.000], center = true);  "
                             "// Cube: 'Cube 1'"), "[1]!"
    assert scad_lines[2] == (" cube(size = [2.000, 2.000, 2.000], center = true);  "
                             "// Cube: 'Cube 2'"), "[2]!"
    assert scad_lines[3] == (" cube(size = [3.000, 3.000, 3.000], center = true);  "
                             "// Cube: 'Cube 3'"), "[3]!"
    assert scad_lines[4] == "}", "[4]!"

    # Verify that *is_operator* attribute can be set:
    module3d2: Module3D = Module3D("Module3D 2", [], is_operator=True)
    assert f"{module3d2}" == "Module3D('Module3D 2',[...],is_operator=True,lock=True)"


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

    # Verify that dot product works:
    assert p123.dot(p123) == 1.0 + 4.0 + 9.0

    # Verify that unitary X cross unitary Y yields unitary Z:
    unitary_x: P3D = P3D(1.0, 0.0, 0.0)
    unitary_y: P3D = P3D(0.0, 1.0, 0.0)
    unitary_z: P3D = unitary_x.cross(unitary_y)
    assert unitary_z.x == 0.0
    assert unitary_z.y == 0.0
    assert unitary_z.z == 1.0

    # Verify that length works:
    assert p123.length() == sqrt(14.0)


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
    # scad_writer(x_square_polygon, scad_lines)

    # Now validate that we got the right values written into *scad*_lines*:
    assert len(scad_lines) == 7
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
        x_square_polygon.scad_file_write(scad_file)
    with io.StringIO("") as scad_file:
        x_square_polygon.scad_file_write(scad_file)
        input_scad_text: str = scad_file.getvalue()
        input_scad_lines: List[str] = input_scad_text.split('\n')
        assert len(input_scad_lines) == 9
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
    new_square_polygon: Polygon = Polygon("New Square Polygon", [], lock=False)
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

    # Test *Polygon*.*simple_polygons_get*()
    assert new_square_polygon.simple_polygons_get() == [square_simple_polygon] + hole_polygons

    # Test *Polygon*.*__get__item__*() method:
    hole_index: int
    for hole_index, hole_polygon in enumerate(hole_polygons):
        assert new_square_polygon[hole_index + 1] is hole_polygons[hole_index]
    try:
        new_square_polygon[10]
    except IndexError as index_error:
        assert f"{index_error}" == "index=10 and it is not in range 0:9"

    # Verify lock detection:
    try:
        unlocked_simple_polygon: SimplePolygon = SimplePolygon("Unlocked SimplePolygon", lock=False)
        Polygon("Unlocked Simple Polygon", [unlocked_simple_polygon])
        assert False, "This line should not be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == ("SimplePolygon ('Unlocked SimplePolygon') "
                                    "at index 0 is not locked.")
    locked_polygon: Polygon = Polygon("Locked Polygon", [], lock=True)
    try:
        locked_polygon.append(hole_polygon)
        assert False, "This line should not be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "Polygon 'Locked Polygon' is locked and can not be appended to."
    try:
        locked_polygon.extend([hole_polygon])
        assert False, "This line should not be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "Polygon 'Locked Polygon' is locked and can not be extended."


def test_rotate3d() -> None:
    """Test Rotate3D class."""
    cube1: Cube = Cube("Cube 1", 1.0, 2.0, 3.0)
    rotate1: Rotate3D = Rotate3D("Rotate3D 1", cube1, pi / 4.0, axis=P3D(0.0, 0.0, 1.0))
    assert str(rotate1) == ("Rotate('Rotate3D 1',"
                            "Cube('Cube 1',1.000,2.000,3.000,center=P3D(0.000,0.000,0.000)),"
                            "P3D(0.000,0.000,1.000),"
                            "45.000deg)")
    scad_lines: List[str] = []
    rotate1.scad_lines_append(scad_lines, "")
    assert len(scad_lines) == 3
    assert scad_lines[0] == ("rotate(a = 45.000000, v=[0.000, 0.000, 1.000]) {  "
                             "// Rotate3D: 'Rotate3D 1'"), "[0]!"
    assert scad_lines[1] == (" cube(size = [1.000, 2.000, 3.000], center = true);  "
                             "// Cube: 'Cube 1'"), "[1]!"
    assert scad_lines[2] == "}", "[2]!"

    # Verify that we detect an axis of zero length:
    origin: P3D = P3D(0.0, 0.0, 0.0)
    try:
        Rotate3D("Origin Axis Translate", cube1, 0.0, origin)
        assert False, "This line should never be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "Rotate axis has no direction."


def test_scad_keys_csv_file_write() -> None:
    """Test Scad.keys_csv_file_write()."""
    circle_key: Tuple[Any, ...] = ("Circle", "Circle1", 1.0, 2.0, 1.0, 1.0, 0.0)
    square_key: Tuple[Any, ...] = ("Square", "Square1", 3.0, 4.0, 2.0, 4.0, 45.0, 0.5, 3)
    keys: List[Tuple[Any, ...]] = [circle_key, square_key]
    csv_file: IO[Any]
    with io.StringIO("") as csv_file:
        scad.Scad.keys_csv_file_write(keys, csv_file)
        csv_file_text: str = csv_file.getvalue()
        csv_lines: List[str] = csv_file_text.split('\n')
        assert len(csv_lines) == 4
        assert csv_lines[0] == 'Type,Name,X,Y,DX,DY,Angle,Corner Radius,Corner Count', "[0]!"
        assert csv_lines[1] == '"Circle","Circle1",1.000,2.000,1.000,1.000,0.000', "[1]!"
        assert csv_lines[2] == '"Square","Square1",3.000,4.000,2.000,4.000,45.000,0.500,3', "[2]!"
        assert csv_lines[3] == '', "[3]!"


def test_scad_keys_html_file_write() -> None:
    """Test Scad.keys_csv_file_write()."""
    circle_key: Tuple[Any, ...] = ("Circle", "Circle1", 1.0, 2.0, 1.0, 1.0, 0.0)
    square_key: Tuple[Any, ...] = ("Square", "Square1", 3.0, 4.0, 2.0, 4.0, 45.0, 0.5, 3)
    keys: List[Tuple[Any, ...]] = [circle_key, square_key]
    tmp_html_file: IO[Any]
    with open("/tmp/scad_keys.html", "w") as tmp_html_file:
        scad.Scad.keys_html_file_write(keys, tmp_html_file, "HTML File Write Test")
    csv_file: IO[Any]
    with io.StringIO("") as html_file:
        scad.Scad.keys_html_file_write(keys, html_file, "HTML File Write Test")
        html_file_text: str = html_file.getvalue()
        html_lines: List[str] = html_file_text.split('\n')
        assert len(html_lines) == 45
        assert html_lines[0] == '<HTML>'
        assert html_lines[1] == ' <Head>'
        assert html_lines[2] == '  <Title>HTML File Write Test</Title>'
        assert html_lines[3] == ' </Head>'
        assert html_lines[4] == ' <Body>'
        assert html_lines[5] == '  <H1>HTML File Write Test</H1>'
        assert html_lines[6] == '  <Table>'
        assert html_lines[7] == '   <TR>'
        assert html_lines[8] == '    <TH align="left">Index</TH>'
        assert html_lines[9] == '    <TH align="left">Type</TH>'
        assert html_lines[10] == '    <TH align="left">Name</TH>'
        assert html_lines[11] == '    <TH align="left">X</TH>'
        assert html_lines[12] == '    <TH align="left">Y</TH>'
        assert html_lines[13] == '    <TH align="left">DX</TH>'
        assert html_lines[14] == '    <TH align="left">DY</TH>'
        assert html_lines[15] == '    <TH align="left">Angle</TH>'
        assert html_lines[16] == '    <TH align="left">Corner Radius</TH>'
        assert html_lines[17] == '    <TH align="left">Corner Count</TH>'
        assert html_lines[18] == '   </TR>'
        assert html_lines[19] == '   <TR>'
        assert html_lines[20] == '    <TD align="left">0</TD>'
        assert html_lines[21] == '    <TD align="left">Circle</TD>'
        assert html_lines[22] == '    <TD align="left">Circle1</TD>'
        assert html_lines[23] == '    <TD align="left">1.000</TD>'
        assert html_lines[24] == '    <TD align="left">2.000</TD>'
        assert html_lines[25] == '    <TD align="left">1.000</TD>'
        assert html_lines[26] == '    <TD align="left">1.000</TD>'
        assert html_lines[27] == '    <TD align="left">0.000</TD>'
        assert html_lines[28] == '   </TR>'
        assert html_lines[29] == '   <TR>'
        assert html_lines[30] == '    <TD align="left">1</TD>'
        assert html_lines[31] == '    <TD align="left">Square</TD>'
        assert html_lines[32] == '    <TD align="left">Square1</TD>'
        assert html_lines[33] == '    <TD align="left">3.000</TD>'
        assert html_lines[34] == '    <TD align="left">4.000</TD>'
        assert html_lines[35] == '    <TD align="left">2.000</TD>'
        assert html_lines[36] == '    <TD align="left">4.000</TD>'
        assert html_lines[37] == '    <TD align="left">45.000</TD>'
        assert html_lines[38] == '    <TD align="left">0.500</TD>'
        assert html_lines[39] == '    <TD align="left">3</TD>'
        assert html_lines[40] == '   </TR>'
        assert html_lines[41] == '  </Table>'
        assert html_lines[42] == ' </Body>'
        assert html_lines[43] == '</HTML>'


def test_scad_program() -> None:
    """Test ScadProgram class."""
    scad_program: ScadProgram = ScadProgram("ScadProgram 1")
    assert str(scad_program) == "ScadProgram('ScadProgram 1')"
    circle1: Circle = Circle('Circle 1', 1.0, 8)
    scad_program.append(circle1)
    scad_lines: List[str] = []
    scad_program.scad_lines_append(scad_lines, "")
    assert len(scad_lines) == 7
    assert scad_lines[0] == "// Begin ScadProgram('ScadProgram 1')", "[0]!"
    assert scad_lines[1] == "circle(d=1.000, $fn=8);  // Circle 'Circle 1'", "[1]!"
    assert scad_lines[2] == "if (false) {  // If2D 'Name If2D'", "[2]!"
    assert scad_lines[3] == "}  // End If2D 'Name If2D'", "[3]!"
    assert scad_lines[4] == "if (false) {  // If3D 'Name If3D'", "[4]!"
    assert scad_lines[5] == "}  // End If3D 'Name If3D'", "[5]!"
    assert scad_lines[6] == "// End ScadProgram('ScadProgram 1')", "[7]!"


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

    # Test SimplePolygon.key():
    p1: P2D = P2D(2.0, 2.0)
    p2: P2D = P2D(2.0, 1.0)
    p3: P2D = P2D(1.0, 1.0)
    p4: P2D = P2D(1.0, 2.0)
    simple_polygon: SimplePolygon = SimplePolygon("SimplePolygon1", [p1, p2, p3, p4], lock=True)
    assert simple_polygon.key() == ("SimplePolygon", "SimplePolygon1", 1.5, 1.5, 1.0, 1.0, 0.0)

    # Test SimplePolygon.scad_lines_append():
    scad_lines = []
    simple_polygon.scad_lines_append(scad_lines, "")
    assert len(scad_lines) == 7
    assert scad_lines[0] == ("polygon(points = [  "
                             "// Begin SimplePolygon 'SimplePolygon1' 0:3"), "[0]!"
    assert scad_lines[1] == " // SimplePolygon 'SimplePolygon1' 0-3", "[1]!"
    assert scad_lines[2] == ("  [2.000, 2.000], [2.000, 1.000], "
                             "[1.000, 1.000], [1.000, 2.000]  // 0-4"), "[2]!"
    assert scad_lines[3] == " ], paths = [", "[3]!"
    assert scad_lines[4] == "  // SimplePolygon 'SimplePolygon1' 0-3", "[4]!"
    assert scad_lines[5] == "  [0, 1, 2, 3]", "[5]!"
    assert scad_lines[6] == " ], convexity=4);  // End SimplePolygon 'SimplePolygon1' 0:3", "[6]!"

    # Test SimpplePolygon.x_mirror():
    mirrored_x_polygon: SimplePolygon = simple_polygon.x_mirror("X Mirrored SimplePolygon1")
    xy0: P2D = mirrored_x_polygon[0]
    assert xy0.x == 2.0 and xy0.y == -2.0
    xy1: P2D = mirrored_x_polygon[1]
    assert xy1.x == 2.0 and xy1.y == -1.0
    xy2: P2D = mirrored_x_polygon[2]
    assert xy2.x == 1.0 and xy2.y == -1.0
    xy3: P2D = mirrored_x_polygon[3]
    assert xy3.x == 1.0 and xy3.y == -2.0

    # Test *SimplePolygon*.*y_mirror*() method:
    mirrored_y_polygon: SimplePolygon = simple_polygon.y_mirror("X Mirrored SimplePolygon1")
    xy0 = mirrored_y_polygon[0]
    assert xy0.x == -2.0 and xy0.y == 2.0
    xy1 = mirrored_y_polygon[1]
    assert xy1.x == -2.0 and xy1.y == 1.0
    xy2 = mirrored_y_polygon[2]
    assert xy2.x == -1.0 and xy2.y == 1.0
    xy3 = mirrored_y_polygon[3]
    assert xy3.x == -1.0 and xy3.y == 2.0

    # Test *SimplePolygon*.*rotate_points*() method.
    origin: P2D = P2D(0.0, 0.0)
    simple_polygon4: SimplePolygon = SimplePolygon("SimplePolygon4", [p1, p2, p3, p4], lock=False)
    simple_polygon4.points_rotate(pi, origin)
    xy0 = simple_polygon4[0]
    assert round(xy0.x, 3) == -2.000 and round(xy0.y, 3) == -2.000
    xy1 = simple_polygon4[1]
    assert round(xy1.x, 3) == -2.000 and round(xy1.y, 3) == -1.000
    xy2 = simple_polygon4[2]
    assert round(xy2.x, 3) == -1.000 and round(xy2.y, 3) == -1.000
    xy3 = simple_polygon4[3]
    assert round(xy3.x, 3) == -1.000 and round(xy3.y, 3) == -2.000
    simple_polygon4.lock()
    try:
        simple_polygon4.points_rotate(pi, origin)
    except ValueError as value_error:
        assert f"{value_error}" == "'SimplePolygon4' is locked"


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
    # scad_writer(origin_square, scad_lines)
    assert len(scad_lines) == 2
    assert scad_lines[0] == ("// Square 'Origin Square' dx=2.000 dy=3.000 center=P2D(0.000,0.000) "
                             "corner_radius=0.000 corner_count=3"), "Index 0 failed"
    assert scad_lines[1] == "square([2.000, 3.000], center = true);", "Index 1 failed"

    # Make sure that *offset_square* looks right:
    scad_lines = []
    offset_square.scad_lines_append(scad_lines, "")
    # scad_writer(offset_square, scad_lines)
    assert len(scad_lines) == 3
    assert scad_lines[0] == ("// Square 'Offset Square' dx=2.500 dy=3.500 center=P2D(3.000,2.000) "
                             "corner_radius=0.000 corner_count=3"), "Index 0 failed"
    assert scad_lines[1] == "translate([3.000, 2.000])"
    assert scad_lines[2] == " square([2.500, 3.500], center = true);", "Index 2 failed"

    # Make sure that *rotated_square* looks right:
    scad_lines = []
    rotated_square.scad_lines_append(scad_lines, "")
    # scad_writer(rotated_square, scad_lines)
    assert len(scad_lines) == 3
    assert scad_lines[0] == ("// Square 'Rotated Square' dx=2.000 dy=3.000 center=P2D(0.000,0.000) "
                             "corner_radius=0.000 corner_count=3"), "Index 0 failed"
    assert scad_lines[1] == "rotate(a = [0, 0, 90.000])", "Index 1 failed"
    assert scad_lines[2] == " square([2.000, 3.000], center = true);", "Index 2 failed"

    # Make sure that *slotted_square* looks right:
    scad_lines = []
    slot_square.scad_lines_append(scad_lines, "")
    # scad_writer(slot_square, scad_lines)
    assert len(scad_lines) == 13
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
    # scad_writer(slot_square2, scad_lines)
    assert len(scad_lines) == 13
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
    # scad_writer(rounded_square, scad_lines)
    assert len(scad_lines) == 12
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

    # Try out the *Square*.*copy*() method:
    small_center: P2D = P2D(2.0, 3.0)
    small_rotate: float = pi/2.0
    small_corner_radius: float = 0.5
    small_corner_count: int = 4
    small_square: Square = Square("Small Square", 1.0, 2.0, small_center,
                                  small_rotate, small_corner_radius, small_corner_count)
    assert f"{small_square}" == ("Square('Small Square',1.000,2.000,center=P2D(2.000,3.000),"
                                 "rotate=90.000deg,corner_radius=0.500,corner_count=4)")
    big_center: P2D = P2D(4.0, 5.0)
    big_rotate: float = -pi/2.0
    big_corner_radius: float = 0.75
    big_corner_count: int = 5
    big_square: Square = small_square.copy("Small", dx=2.0, dy=4.0,
                                           rotate=big_rotate,
                                           center=big_center,
                                           corner_radius=big_corner_radius,
                                           corner_count=big_corner_count,
                                           replace="Big")
    assert f"{big_square}" == ("Square('Big Square',2.000,4.000,center=P2D(4.000,5.000),"
                               "rotate=-90.000deg,corner_radius=0.750,corner_count=5)")

    # Test *Square*.*key*() method:
    assert rounded_square.key() == ("Square", "Rounded Square", 0.0, 0.0, 2.0,
                                    4.0, 0.0, 0.5, 2), "key failed"


def test_translate3d() -> None:
    """Test Translate3D class."""
    cube1: Cube = Cube("Cube 1", 1.0, 2.0, 3.0)
    translated_cube1 = Translate3D("Translated Cube 1", cube1, P3D(4.0, 5.0, 6.0))
    assert str(translated_cube1) == ("Translate3D('Translated Cube 1',"
                                     "Cube('Cube 1',1.000,2.000,3.000,"
                                     "center=P3D(0.000,0.000,0.000)),P3D(4.000,5.000,6.000))")

    scad_lines: List[str] = []
    translated_cube1.scad_lines_append(scad_lines, "")
    assert len(scad_lines) == 3
    assert scad_lines[0] == ("translate(v = [4.000, 5.000, 6.000]) {  "
                             "// Translate 'Translated Cube 1'"), "[0]!"
    assert scad_lines[1] == (" cube(size = [1.000, 2.000, 3.000], center = true);  "
                             "// Cube: 'Cube 1'"), "[1]!"
    assert scad_lines[2] == "}", "[2]!"


def test_union3d() -> None:
    """Test Union class."""
    # Create the *union3d* object:
    cube1: Cube = Cube("Cube 1", 1.0, 2.0, 3.0)
    cube2: Cube = Cube("Cube 2", 4.0, 5.0, 6.0)
    cube3: Cube = Cube("Cube 3", 7.0, 8.0, 9.0)
    cube4: Cube = Cube("Cube 4", 10.0, 11.0, 12.0)
    cubes_union: Union3D = Union3D("Cubes Union", [cube1], lock=False)
    assert len(cubes_union) == 1
    cubes_union.append(cube2)
    assert len(cubes_union) == 2
    cubes_union.extend([cube3, cube4])

    # Verify that we get an exception trying to write out a unlocked union:
    scad_lines: List[str] = []
    try:
        cubes_union.scad_lines_append(scad_lines, "")
        assert False, "This line should not be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "Union3D 'Cubes Union' is not locked yet."

    # Now lock *cubes_union* and verify that the correct OpenSCAD output is generated:
    cubes_union.lock()
    cubes_union.scad_lines_append(scad_lines, "")
    # scad_writer(cubes_union, scad_lines)
    assert len(scad_lines) == 6
    assert scad_lines[0] == "union() {  // Union3D 'Cubes Union'", "[1]!"
    assert scad_lines[1] == (" cube(size = [1.000, 2.000, 3.000], center = true);  "
                             "// Cube: 'Cube 1'"), "[1]!"
    assert scad_lines[2] == (" cube(size = [4.000, 5.000, 6.000], center = true);  "
                             "// Cube: 'Cube 2'"), "[2]!"
    assert scad_lines[3] == (" cube(size = [7.000, 8.000, 9.000], center = true);  "
                             "// Cube: 'Cube 3'"), "[3]!"
    assert scad_lines[4] == (" cube(size = [10.000, 11.000, 12.000], center = true);  "
                             "// Cube: 'Cube 4'"), "[4]!"
    assert scad_lines[5] == "}  // End Union3D 'Cubes Union'", "[5]!"

    # Now verify that *Union3D* *lock*'s work:
    try:
        cubes_union.append(cube1)
        assert "This line should not be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "Can not append to Union3D 'Cubes Union' because is locked"
    try:
        cubes_union.extend([cube2, cube3])
        assert "This line should not be reached"  # pragma: no cover
    except ValueError as value_error:
        assert f"{value_error}" == "Can not extend Union3D 'Cubes Union' because is locked"


def test_use_module2d() -> None:
    """Test UseModule2D class."""
    circle1: Circle = Circle("Circle 1", 2.0, 16)
    module2d1: Module2D = Module2D("Module2D 1", [circle1])
    use_module2d1: UseModule2D = UseModule2D("UseModule2D 1", module2d1)
    assert str(use_module2d1) == ("UseModule2D('UseModule2D 1',"
                                  "Module2D('Module2D 1',[...],is_operator=False,lock=True))")

    # Verify that *scad_lines_append* method works:
    scad_lines: List[str] = []
    module2d1.scad_lines_append(scad_lines, "")
    use_module2d1.scad_lines_append(scad_lines, "")
    assert len(scad_lines) == 4
    assert scad_lines[0] == "module Module2D_1() {", "[0]!"
    assert scad_lines[1] == " circle(d=2.000, $fn=16);  // Circle 'Circle 1'", "[1]!"
    assert scad_lines[2] == "}", "[2]!"
    assert scad_lines[3] == "Module2D_1(); // UseModule2D('UseModule2D 1')", "[3]!"


def test_use_module3d() -> None:
    """Test UseModule3D class."""
    cube1: Cube = Cube("Cube 1", 1.0, 2.0, 3.0)
    module3d1: Module3D = Module3D("Module3D 1", [cube1])
    use_module3d1: UseModule3D = UseModule3D("UseModule3D 1", module3d1)
    assert str(use_module3d1) == ("UseModule3D('UseModule3D 1',"
                                  "Module3D('Module3D 1',[...],is_operator=False,lock=True))")

    # Verify that *scad_lines_append* method works:
    scad_lines: List[str] = []
    module3d1.scad_lines_append(scad_lines, "")
    use_module3d1.scad_lines_append(scad_lines, "")
    assert len(scad_lines) == 4
    assert scad_lines[0] == "module Module3D_1() {", "[0]!"
    assert scad_lines[1] == (" cube(size = [1.000, 2.000, 3.000], center = true);  "
                             "// Cube: 'Cube 1'"), "[1]!"
    assert scad_lines[2] == "}", "[2]!"
    assert scad_lines[3] == "Module3D_1(); // UseModule3D('UseModule3D 1')", "[3]!"


def test_variable2d() -> None:
    """Test Variable2d class."""
    variable2d1: Variable2D = Variable2D("Variable2D 1", "sum", "2 + 3")
    assert str(variable2d1) == "Variable2D('sum=2 + 3')"
    scad_lines: List[str] = []
    variable2d1.scad_lines_append(scad_lines, "")
    assert len(scad_lines) == 1
    assert scad_lines[0] == "sum = 2 + 3;"


if __name__ == "__main__":  # pragma: no cover
    test_polygon()
