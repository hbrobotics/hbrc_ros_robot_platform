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

"""KiCad."""

from typing import Any, IO, List
from pathlib import Path
from scad_models.scad import P2D
import time


# Footprint:
class Footprint:
    """Footprint represents a KiCad footprint file."""

    # Footprint.__init__():
    def __init__(self, name: str) -> None:
        """Initialize a footprint."""
        # Note to print the current time in seconds from 1970 use the following command:
        #     date +%s
        # The *time.time()* method returns the time in seconds as a float.
        # if time_stamp == 0:
        #     time_stamp = int(time.time())
        # Create *lines* and start with the module line:
        timestamp: int = int(time.time())
        lines: List[str] = [f"(module {name} (layer F.Cu) (tedit {timestamp:08X})"]

        # Save *name* and *lines* into *footprint* (i.e. *self*):
        # footprint: FootPrint = self
        self.lines: List[str] = lines
        self.name: str = name

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
    def hole(self, name: str, center: P2D, pad_dx: float, pad_dy: float, drill: float) -> None:
        """Append a hole to a footprint."""
        footprint: Footprint = self
        lines: List[str] = footprint.lines
        line: str
        if pad_dx == 0.0 or pad_dy == 0.0:
            line = (f"  (pad \"{name}\" np_thru_hole circle "
                    f"(at {center.x:.4f} {-center.y:.4f}) "
                    f"(size {drill:.1f} {drill:.1f}) "
                    f"(drill {drill:.1f}) "
                    "(layers *.Cu *.Mask))")
            lines.append(line)
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
            lines.append(line)

    # Footprint.line():
    def line(self, point1: P2D, point2: P2D, layer: str, width: float) -> None:
        """Append a line to a footprint."""
        pass
        footprint: Footprint = self
        lines: List[str] = footprint.lines
        x1: float = point1.x
        y1: float = point1.y
        x2: float = point2.x
        y2: float = point2.y
        line: str = (f"  (fp_line (start {x1:.4f} {-y1:.4f}) (end {x2:.4f} {-y2:.4f}) "
                     f"(layer {layer}) (width {width:.1f}))")
        lines.append(line)

    # Footprint.name_get():
    def name_get(self) -> str:
        """Return the name of the footprint."""
        footprint: Footprint = self
        return footprint.name

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
            print("write out new")
        elif touch_required:
            # Write out the *previous_text* to force the file system timestamp to be updated.
            with open(footprint_path, "w") as footprint_file:
                footprint_file.write(previous_text)
            print("write out previous")
        else:
            print("do nothing")

    # KiCadFootprint.thru_hole_pad():
    def thru_hole_pad(self, name: str, center: P2D,
                      pad_diameter: float, drill_diameter: float) -> None:
        """Append a circular through hole pad to a footprint."""
        footprint: Footprint = self
        lines: List[str] = footprint.lines
        line: str = (f"  (pad {name} thru_hole circle (at {center.x:.4f} {-center.y:.4f}) "
                     f" (size {pad_diameter:.4f} {pad_diameter:.4f}) "
                     f"(drill {drill_diameter:.3f}) (layers *.Cu *.Mask))")
        lines.append(line)

    # KiCadFootprint.value():
    def value(self, center: P2D, hide: bool) -> None:
        """Append a value to a footprint."""
        footprint: Footprint = self
        lines: List[str] = footprint.lines
        name: str = footprint.name
        x: float = center.x
        y: float = center.y
        hide_text: str = " hide" if hide else ""
        lines.append(f"  (fp_text value {name} (at {x:.2f} {-y:.2f}) (layer F.SilkS) {hide_text}")
        lines.append("    (effects (font (size 1 1) (thickness 0.2)))")
        lines.append("  )")
