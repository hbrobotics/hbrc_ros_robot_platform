"""test_romi_mode: Unit tests for the Romi stuff."""

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

from scad_models.scad_models import OtherPi, RaspberryPi3, Romi
from scad_models.scad import Polygon


# test_raspberry_pi3():
def test_respberry_pi3():
    """Test RaspberryPi3 class."""
    raspberry_pi3: RaspberryPi3 = RaspberryPi3()
    raspberry_pi3.pcb_polygon_get()


# test_romi():
def test_romi():
    """Run the various Romi methods."""
    romi: Romi = Romi()
    romi.debugging = False
    romi_base_polygon: Polygon = romi.base_scad_polygon_generate()
    romi_base_polygon = romi_base_polygon

    expansion_polygon: Polygon = romi.expansion_polygon_get()
    expansion_polygon.lock()
    expansion_polygon = expansion_polygon


# test_other_pi():
def test_other_pi() -> None:
    """Test OtherPi class."""
    other_pi: OtherPi = OtherPi()
    other_pi_pcb_polygon: Polygon = other_pi.pcb_polygon_get()
    other_pi_pcb_polygon = other_pi_pcb_polygon
