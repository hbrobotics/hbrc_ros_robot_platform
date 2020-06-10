# This file is licensed using the "MIT License" below:
#
# ##################################################################################################
#
# MIT License
#
# Copyright 2019-2020 Home Brew Robotics Club
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this
# software and associated documentation files (the "Software"), to deal in the Software
# without restriction, including without limitation the rights to use, copy, modify,
# merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to the following
# conditions:
#
# The above copyright notice and this permission notice shall be included in all copies
# or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
# FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
# OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.
#
# ##################################################################################################
# <======================================= 100 characters =======================================> #

"""KiCad.

This KiCAd module provides an interface to the KiCad file formats.  In
particular it supports:

* Footprints:
  A KiCad footprint has a suffix of `.kicad_mod` and contains the
  locations of the various pads with their names (both through hole and
  surface mount), mounting holes, additional PCB cut outs and both top
  and bottom artwork.
* Printed Circuit Boards:
  A Kicad printed circuit board has a suffix of `.kicad_pcb` and
  contains everything needed for the PCB.  Of interest to this module
  is the ability to insert cut-lines, artwork, footprints.  Each
  footprint has a name, side (front or back), position (X, Y), and
  and an orientation (0-360 degrees).

"""

from typing import Any, Dict, IO, List, Tuple
from pathlib import Path
from scad_models.scad import P2D, SimplePolygon
import time


# Footprint:
class Footprint:
    """Footprint represents a KiCad footprint file."""

    # Footprint.__init__():
    def __init__(self, library_name: str, footprint_name: str) -> None:
        """Initialize a footprint."""
        # Note to print the current time in seconds from 1970 use the following command:
        #     date +%s
        # The *time.time()* method returns the time in seconds as a float.
        # if time_stamp == 0:
        #     time_stamp = int(time.time())
        # Create *lines* and start with the module line:
        timestamp: int = int(time.time())
        lines: List[str] = [f"(module {library_name}:{footprint_name} "
                            f"(layer F.Cu) (tedit {timestamp:08X})"]
        # print(f"lines={lines}")

        # Save *name* and *lines* into *footprint* (i.e. *self*):
        # footprint: FootPrint = self
        self.lines: List[str] = lines
        self.library_name: str = library_name
        self.footprint_name: str = footprint_name

    def header_match(self, previous_header: str) -> bool:
        """See whether a previous header matches the current one."""
        footprint: Footprint = self
        lines: List[str] = footprint.lines
        assert len(lines) >= 1
        header: str = lines[0]
        # The tail end of the header looks like "... (tedit XXXXXXXX)", where "XXXXXXXX"
        # is an eight digit upper case hex number.  We want to identically match everything
        # *except* the hex number:
        # print(f"len(header):{len(header)}")
        # print(f"len(previoius_header):{len(previous_header)}")
        # print(f"last characters match:{header[-1:] == previous_header[-1:]}")
        # print(f"front characters match:{header[:-9] == previous_header[:-9]}")
        match: bool = (len(header) == len(previous_header) and
                       header[-1:] == previous_header[-1:] and
                       header[:-9] == previous_header[:-9])
        # print(f"match:{match}")
        return match

    # Footprint.hole():
    def hole(self, name: str, center: P2D, pad_dx: float, pad_dy: float, drill: float,
             tracing: str = "") -> None:
        """Append a hole to a footprint."""
        # Perform any requested *tracing*:
        if tracing:
            print(f"{tracing}=>Footprint.hole(*, '{name}', {center}, {pad_dx}, {pad_dy}, {drill})")

        # Unpack some values from *footprint* (i.e. *self*):
        footprint: Footprint = self
        lines: List[str] = footprint.lines

        # Construct *line* based on the argument values:
        line: str
        if pad_dx == 0.0 or pad_dy == 0.0:
            line = (f"  (pad \"{name}\" np_thru_hole circle "
                    f"(at {center.x:.4f} {-center.y:.4f}) "
                    f"(size {drill:.1f} {drill:.1f}) "
                    f"(drill {drill:.1f}) "
                    "(layers *.Cu *.Mask))")
        else:
            small_distance: float = 0.0000001
            if abs(pad_dx - pad_dy) < small_distance:
                # Do a simple circular pad:
                line = (f"  (pad \"{name}\" thru_hole circle "
                        f"(at {center.x:.4f} {-center.y:.4f}) "
                        f"(size {pad_dx:.1f} {pad_dy:.1f}) "
                        f"(drill {drill:.1f}) "
                        "(layers *.Cu *.Mask))")
            else:
                # Do an oval pad:
                pad_dx_dy_minimum: float = min(pad_dx, pad_dy)
                pad_extra: float = pad_dx_dy_minimum - drill
                drill_dx: float = pad_dx - pad_extra
                drill_dy: float = pad_dy - pad_extra
                line = (f"  (pad \"{name}\" thru_hole oval "
                        f"(at {center.x:.4f} {-center.y:.4f}) "
                        f"(size {pad_dx:.1f} {pad_dy:.1f}) "
                        f"(drill oval {drill_dx:.1f} {drill_dy:.1f}) "
                        "(layers *.Cu *.Mask))")
        if tracing:
            print(f"{tracing}Footprint.hole:'{line}'")
        lines.append(line)

        # Wrap up any requested *tracing*:
        if tracing:
            print(f"{tracing}<=Footprint.hole(*, '{name}', {center}, {pad_dx}, {pad_dy}, {drill})")

    # Footprint.library_get():
    def librarty_get(self) -> str:
        """Return the name of the library."""
        # Use *footprint* instead of *self*:
        footprint: Footprint = self
        return footprint.library_name

    # Footprint.line():
    def line(self, point1: P2D, point2: P2D, layer: str, width: float, tracing: str = "") -> None:
        """Append a line to a footprint."""
        # Perform any requested *tracing*:
        if tracing:
            print(f"{tracing}=>Footprint.line({point1}, {point2}, '{layer}', {width}")

        # Extract some values from *footprint* (i.e. *self*)
        footprint: Footprint = self
        lines: List[str] = footprint.lines
        x1: float = point1.x
        y1: float = point1.y
        x2: float = point2.x
        y2: float = point2.y

        # Create the *line* and append to *lines*:
        line: str = (f"  (fp_line (start {x1:.4f} {-y1:.4f}) (end {x2:.4f} {-y2:.4f}) "
                     f"(layer {layer}) (width {width:.1f}))")
        lines.append(line)

        # Wrap up any requested *tracing*:
        if tracing:
            print(f"{tracing}<=Footprint.line({point1}, {point2}, '{layer}', {width}")

    # Footprint.name_get():
    def name_get(self) -> str:
        """Return the name of the footprint."""
        # Use *footprint* instead of *self*:
        footprint: Footprint = self
        return footprint.footprint_name

    # Footprint.rectangle():
    def rectangle(self, point1: P2D, point2: P2D, layer: str, width: float) -> None:
        """Append a rectangle to a footprint."""
        footprint: Footprint = self
        x1: float = point1.x
        y1: float = point1.y
        x2: float = point2.x
        y2: float = point2.y
        point3: P2D = P2D(x1, y2)
        point4: P2D = P2D(x2, y1)
        footprint.line(point1, point3, layer, width)
        footprint.line(point3, point2, layer, width)
        footprint.line(point2, point4, layer, width)
        footprint.line(point4, point1, layer, width)

    def reference(self, center: P2D) -> None:
        """Append a reference to a footprint."""
        footprint: Footprint = self
        lines: List[str] = footprint.lines
        x: float = center.x
        y: float = center.y
        lines.append(f"  (fp_text reference REF** (at {x:.2f} {-y:.2f}) (layer F.SilkS)")
        lines.append("    (effects (font (size 1 1) (thickness 0.2)))")
        lines.append("  )")
        # If value is not specified, KiCad currently defaults to putting it into the F.SilkS layer.
        # KiCad completely ignores whatever text that is supplied.  The solution seems to be to put
        # it into the F.Fab layer and you can disable seeing that layer.  Kind of silly, but
        # it works.  Ship it!:
        lines.append(f"  (fp_text value VALUE (at {x:.2f} {-y-2.0:.2f}) (layer F.Fab)")
        lines.append("    (effects (font (size 1 1) (thickness 0.2)))")
        lines.append("  )")
        # lines.append(f"  (fp_text user USER (at {x:.2f} {-y+2.0+2.0:.2f}) (layer B.SilkS)")
        # lines.append("    (effects (font (size 1 1) (thickness 0.2)))")
        # lines.append("  )")

    # KiCadFootprint.save():
    def save(self, footprint_path: Path, flags: str) -> None:
        """Save a footprint to a file.

        Args:
            * *footprint_path* (*Path*):
              The path to the file to write.  Any specified directores
              must already be present.
            * *flags* (*str*):
              Single character flags that control behavior:
              * 'w':
                Aways overwrite the previous file with a new embedded
                KiCad timestamp.
              * 't':
                Always the update file system timestamp for the file.
                If the new content matches the previous content except
                excluding the `tedit` field, the previous content is
                used so that the `tedit` field does not change.
                This is useful for files that are tracked by `git` which
                always matches the file content, but not file system
                timestamp.  The 'w' option takes precedence over this
                option.

        """
        # Grab the *lines* from *footprint* (i.e. *self*):
        footprint: Footprint = self
        lines: List[str] = footprint.lines

        # Figure out if there the *previous_file_matches*:
        previous_text: str
        previous_file_matches: bool = False
        footprint_file: Any[IO]
        if footprint_path.exists():
            # print("file_exists")
            with open(footprint_path, "r") as footprint_file:
                previous_text = footprint_file.read()
                previous_lines: List[str] = previous_text.split('\n')
                # print(f"len(previous_lines):{len(previous_lines)}")
                # print(f"len(lines):{len(lines)}")
                # The +/- 2is because the last two lines have not been tacked onto *lines* yet:
                line_lengths_match: bool = (len(previous_lines) - 2 == len(lines))
                contents_match: bool = (previous_lines[1:-2] == lines[1:])
                header_match: bool = footprint.header_match(previous_lines[0])
                # print(f"line_lengths_match:{line_lengths_match}")
                # print(f"header_match:{header_match}")
                # print(f"contents_match:{contents_match}")
                previous_file_matches = (line_lengths_match and contents_match and header_match)

        #   write    matches  touch  operation
        #   0        0        0      write new
        #   0        0        1      write new
        #   0        1        0      do nothing
        #   0        1        1      write previous
        #   1        0        0      write new
        #   1        0        1      write new
        #   1        1        0      write new
        #   1        1        1      write new
        #
        #   1        x        x      write new
        #   x        0        x      write new
        #   0        1        1      write previous
        #   0        1        0      do nothing

        write_required: bool = 'w' in flags
        touch_required: bool = 't' in flags
        # print(f"write:{write_required} matches:{previous_file_matches} touch:{touch_required}")
        # print(f"footprint_path={footprint_path}")
        if write_required or not previous_file_matches:
            # We need to write out *footprint* to *footprint_path* with any new content
            # and a new `tedit` timestamp.
            #
            # Convert *lines* into *footprint_text*.  We do this by tacking on a closing
            # parenthesis and an empty line to ensure we get the final new-line character
            # at the end of the file:
            footprint_text: str = '\n'.join(lines + [")", ""])
            with open(footprint_path, "w") as footprint_file:
                footprint_file.write(footprint_text)
            # print("write out new")
        elif touch_required:
            # Write out the *previous_text* to force the file system timestamp to be updated.
            with open(footprint_path, "w") as footprint_file:
                footprint_file.write(previous_text)
            # print("write out previous")
        else:
            # print("do nothing")
            pass

    # Footprint.simple_polygon_append():
    def simple_polygon(self, simple_polygon: "SimplePolygon", layer: str, width: float,
                       tracing: str = "") -> None:
        """Append a SimpelePolygon to a Footprint."""
        # Use *footprint* instead of *self*:
        footprint: Footprint = self

        # Perform any requested *tracing*:
        next_tracing: str = tracing + " " if tracing else ""
        if tracing:
            print(f"{tracing}=>Footprint.simplepolygon(*, *, '{layer}', {width}")

        # Extract adjacent point pair from *simple_polygon* and draw a line:
        point1: P2D
        point2: P2D
        simple_polygon_size: int = len(simple_polygon)
        if 1 <= simple_polygon_size <= 2:
            # Draw circle or a line:
            point1 = simple_polygon[0]
            # Trigger a circle of diameter *width* by drawing a really, really short line:
            point2 = simple_polygon[1] if simple_polygon_size == 2 else point1 + P2D(0.0, 0.001)
            footprint.line(point1, point2, layer, width, tracing=next_tracing)
        else:
            # Iterate through *simplepoly* drawing a close loop from point-to-point:
            index: int
            for index in range(simple_polygon_size):
                point1 = simple_polygon[index]
                point2 = simple_polygon[(index + 1) % simple_polygon_size]
                footprint.line(point1, point2, layer, width, tracing=next_tracing)

        # Wrap up any requested *tracing*:
        if tracing:
            print(f"{tracing}=>Footprint.simplepolygon(*, *, '{layer}', {width}")

    # Footprint.thru_hole_pad():
    def thru_hole_pad(self, name: str, center: P2D,
                      pad_diameter: float, drill_diameter: float) -> None:
        """Append a circular through hole pad to a Footprint."""
        footprint: Footprint = self
        lines: List[str] = footprint.lines
        line: str = (f"  (pad {name} thru_hole circle (at {center.x:.4f} {-center.y:.4f}) "
                     f" (size {pad_diameter:.4f} {pad_diameter:.4f}) "
                     f"(drill {drill_diameter:.3f}) (layers *.Cu *.Mask))")
        lines.append(line)

    # Footprint.value():
    def value(self, center: P2D, hide: bool) -> None:
        """Append a value to a Footprint."""
        # Unpack some values from *footprint* (i.e. *self*):
        footprint: Footprint = self
        lines: List[str] = footprint.lines
        footprint_name: str = footprint.footprint_name

        # Now append some lines to *lines*:
        x: float = center.x
        y: float = center.y
        hide_text: str = " hide" if hide else ""
        lines.append(f"  (fp_text value {footprint_name} (at {x:.2f} {-y:.2f}) "
                     f"(layer F.Fab) {hide_text}")
        lines.append("    (effects (font (size 1 1) (thickness 0.2)))")
        lines.append("  )")


# KicadPCB:
class KicadPCB:
    """Represents a KiCAD PCB."""

    # KicadPcb.__init__():
    def __init__(self, file_name: Path, offset: P2D) -> None:
        """Bind to a .kicad_pcb file."""
        # Read in *file_name* and split into *lines*:
        kicad_pcb_file: IO[Any]
        lines: List[str]
        with open(file_name, "r") as kicad_pcb_file:
            kicad_pcb_text: str = kicad_pcb_file.read()
            lines = kicad_pcb_text.split('\n')
            # print(f"KicadPcb.__init__('{file_name}') read in {len(lines)} lines")

        # Sweep through *lines* and strip out cut lines:
        removed_cut_lines: List[str] = []
        line: str
        for line in lines:
            if line.startswith("  (gr_line") and line.find("(layer Edge.Cuts)") > 0:
                # Have a cut line that will *NOT* be copied over.
                pass
            else:
                # We do *NOT* have a cut line and we copy it over:
                removed_cut_lines.append(line)
        lines = removed_cut_lines

        # Save values into *kicad_pcb* (i.e. *self*):
        # kicad_pcb: KicadPcb = self
        self.file_name: Path = file_name
        self.lines: List[str] = lines
        self.offset: P2D = offset

    # KicadPCB.cuts_update():
    def cuts_update(self, cuts: List[SimplePolygon], tracing: str = ""):
        """Update the cuts in in a KicadPCB."""
        # Perform any request *tracing*:
        if tracing:
            print(f"{tracing}=>KicadPCB.cuts_update(len(cuts)={len(cuts)})")

        # Grab some values from *kicad_pcb* (i.e. *self*):
        kicad_pcb: KicadPCB = self
        offset: P2D = kicad_pcb.offset
        lines: List[str] = kicad_pcb.lines

        # Construct *cut_lines* from *cuts*:
        offset_x = offset.x
        offset_y = offset.y
        width: float = 0.05
        layer: str = "Edge.Cuts"
        cut_lines: List[str] = []
        cut: SimplePolygon
        for cut in cuts:
            cut_size: int = len(cut)
            index: int
            for index in range(cut_size):
                point1: P2D = cut[index]
                point2: P2D = cut[(index + 1) % cut_size]
                cut_line: str = ("  (gr_line "
                                 f"(start {offset_x + point1.x:.6f} {offset_y - point1.y:.5f}) "
                                 f"(end {offset_x + point2.x:.6f} {offset_y - point2.y:.5f}) "
                                 f"(layer {layer}) "
                                 f"(width {width:1.2f}))")
                cut_lines.append(cut_line)

        # Now insert *cut_lines* into *lines* 2 lines back:
        assert len(lines) >= 2
        lines[-2:-2] = cut_lines

        # Wrap up any requested *tracing*:
        if tracing:
            print(f"{tracing}<=KicadPCB.cuts_update(len(cuts)={len(cuts)})")

    # # KicadPcb.line_append():
    # def line_append(self, point1: P2D, point2: P2D, layer: str, width:
    #                 float) -> None:
    #     """Append a line cut."""
    #     # Grab some values from *kicad_pcb* (i.e. *self*):
    #     kicad_pcb: KicadPCB = self
    #     cut_lines_insert_index: int = kicad_pcb.cut_lines_insert_index
    #     lines: List[str] = kicad_pcb.lines
    #     offset: P2D = kicad_pcb.offset
    #
    #     # Make sure we know where to insert cut lines:
    #     if cut_lines_insert_index < 0:
    #         line: str
    #         index: int
    #         net_class_default_found: bool = False
    #         for index, line in enumerate(lines):
    #             # print(f"[{index}]:'{line}'")
    #             if line.startswith("  (net_class Default"):
    #                 net_class_default_found = True
    #             if net_class_default_found and line.startswith("  )"):
    #                 cut_lines_insert_index = index + 2
    #                 kicad_pcb.cut_lines_insert_index = cut_lines_insert_index
    #
    #     # Create the *cut_line_text* into *lines*:
    #     cut_line_text: str = (
    #         "  (gr_line (start {0:.6f} {1:.6f}) (end {2:.6f} {3:.6f}) "
    #         "(layer {4}) (width {5:1.2f}))").format(
    #             offset.x + point1.x, offset.y - point1.y,
    #             offset.x + point2.x, offset.y - point2.y, layer, width)
    #     lines.insert(cut_lines_insert_index, cut_line_text)

    # KicadPcb.layer_remove():a
    def layer_remove(self, layer: str) -> None:
        """Remove the a layer."""
        # Grab some values from *kicad_pcb* (i.e. *self*):
        kicad_pcb: KicadPCB = self
        lines: List[str] = kicad_pcb.lines
        new_lines: List[str] = []
        pattern: str = f"(layer {layer})"
        line: str
        for line in lines:
            if line.find(pattern) < 0:
                # Not a layer we care about, copy it over:
                new_lines.append(line)

        # Save *new_lines* back into *kicad*:
        kicad_pcb.lines = new_lines

    # KicadPcb.modules_update():
    def modules_update(self, references: List[Tuple[str, bool, P2D, float]],
                       tracing: str = "") -> None:
        """Update the module positions."""
        # Perform any requested *tracing*:
        # next_tracing: str = tracing + " " if tracing else ""
        assert isinstance(tracing, str)
        if tracing:
            print("********************************************************")
            print(f"{tracing}=>KicadPCB.modules_update()")

        # Grab some values from *kicad_pcb*:
        kicad_pcb: KicadPCB = self
        lines: List[str] = kicad_pcb.lines
        offset: P2D = kicad_pcb.offset

        # Define some variables and constants:
        line_number: int
        line: str
        at_line_index: int
        reference_line_index: int
        reference_prefix: str = "    (fp_text reference "
        reference_prefix_size: int = len(reference_prefix)

        # Construct *references_table* from *referneces*:
        references_table: Dict[str, Tuple[str, bool, P2D, float]] = {}
        reference_name: str
        reference: Tuple[str, bool, P2D, float]
        for reference in references:
            reference_name = reference[0]
            references_table[reference_name] = reference

        # Sweep through *lines* looking for modules:
        references_found_table: Dict[str, bool] = {}
        in_module: bool = False
        for line_index, line in enumerate(lines):
            if line.startswith("  (module "):
                # Beginning of mounting hole:
                if tracing:
                    print(f"{tracing}Module start found:{line_index}")
                in_module = True
                at_line_index = -1
                reference_line_index = -1
            elif line.startswith("  )") and in_module:
                # We have reached the module end:
                if tracing:
                    print(f"{tracing}Module End found:{line_index}")
                if at_line_index < 0 or reference_line_index < 0:
                    # The module is not well formatted:
                    print(f"Incomplete module: at:{at_line_index} "
                          f"reference:{reference_line_index}")  # pragma: no cover
                else:
                    # Grab the hole label from label_line:
                    reference_line: str = lines[reference_line_index]
                    at_index: int = reference_line.find(" (at ")
                    assert at_index > reference_prefix_size
                    reference_name = reference_line[reference_prefix_size:at_index]
                    # print(f"Module Referenec: '{reference}'")

                    # Look up the *reference* from *references_table* and stuff updated location
                    # back into *lines* at *position_index*:
                    if reference_name in references_table:
                        # Unpack the *reference* information from *references_table*:
                        name: str
                        side: bool
                        position: P2D
                        orientation: float
                        name, side, position, orientation = references_table[reference_name]
                        # FIXME: Add orientation!!!
                        at_text: str = (f"    (at "
                                        f"{offset.x + position.x:0.6f} "
                                        f"{offset.y - position.y:0.6f})")  # KiCad inverts Y axis!
                        if tracing:
                            print(f"{tracing}Found '{reference_name}' in table.")
                            print(f"{tracing}Old: '{lines[at_line_index]}'")
                            print(f"{tracing}New: '{at_text}'")
                        lines[at_line_index] = at_text
                        references_found_table[reference_name] = True

                # Reset all of the values:
                in_module = False
                at_line_index = -1
                reference_line_index = -1
                if tracing:
                    print(f"{tracing}Module Pocessed; line indices reset")
            elif in_module:
                if line.startswith("    (at "):
                    at_line_index = line_index
                    if tracing:
                        print(f"{tracing}at_line_index={at_line_index}")
                elif line.startswith("    (fp_text reference "):
                    reference_line_index = line_index
                    if tracing:
                        print(f"{tracing}reference_line_index={reference_line_index}")

        # Iterate through *references_table* looking for modules that have not been placed:
        for reference_name in references_table.keys():
            if reference_name not in references_found_table:
                print(f"Reference '{reference}' is not currently placed in the PCB.")

        # Wrap up any requested *tracing*:
        if tracing:
            print(f"{tracing}=>KicadPCB.modules_update()")

    # KicadPcb.simple_polygon_append():
    def simple_polygon_append(self, simple_polygon: "SimplePolygon",
                              layer: str, width: float) -> None:
        """Append a SimpelePolygon to a PCB."""
        # kicad_pcb: KicadPCB = self
        # simple_polygon_size: int = len(simple_polygon)
        # index: int
        # for index in range(simple_polygon_size):
        #     point1: P2D = simple_polygon[index]
        #     point2: P2D = simple_polygon[(index + 1) % simple_polygon_size]
        #     kicad_pcb.line_append(point1, point2, layer, width)
        pass

    # KicadPcb.save():
    def save(self):
        """Save contents back file."""
        # Grab some values from *kicad_pcb* (i.e. *self*):
        kicad_pcb: KicadPCB = self
        file_name: str = kicad_pcb.file_name
        lines: List[str] = kicad_pcb.lines

        # Delete blank lines from the end of the file:
        while len(lines) > 0 and lines[-1] == "":
            del lines[-1]

        # Write *kicad_lines* out to *kicad_file_name*:
        kicad_pcb_text: str = '\n'.join(lines) + '\n'
        kicad_pcb_file: IO[Any]
        with open(file_name, "w") as kicad_pcb_file:
            kicad_pcb_file.write(kicad_pcb_text)
