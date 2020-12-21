# MIT License
#
# Copyright 2020 Home Brew Robotics Club
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

# <--------------------------------------- 100 characters ---------------------------------------> #
"""BOM generator for JLCPCB and Digi-Key."""

import csv
from pathlib import Path
import os
import re
from typing import Dict, IO, List, Set, Tuple
from collections import namedtuple as NamedTuple

REPatterns = Tuple[str, str, str, str]
Row = Tuple[str, ...]
Rows = Tuple[Row, ...]
PriceBreak = NamedTuple("PriceBreak", ["min_quantity", "price", "max_quantity"])
PriceBreaks = Tuple[PriceBreak, ...]
Match = NamedTuple("Match", [
    "library_type",
    "price_breaks",
    "name",
    "description",
    "package",
    "lcsc",
    "stock",
    "manufacturer_part",
])
JlcpcbRow = NamedTuple("JlcpcbRow", [
    "lcsc",
    "first_category",
    "second_category",
    "manufacturer_part",
    "package",
    "solder_joints",
    "manufacturer",
    "library_type",
    "description",
    "datasheet",
    "price",
    "stock",
])


match_indices: Dict[str, int] = {
    "library_type": 0,
    "price_breaks": 1,
    "name": 2,
    "description": 3,
    "package": 4,
    "lcsc": 5,
    "stock": 6,
    "manufacturer_part": 7,
}


def main() -> int:
    """Generate files for a JLCPCB PCB order."""
    error: str = ""
    errors: int = 0

    # Compute the root *hr2_dir*:
    hr2_env_var: str = "HR2_DIRECTORY"
    assert hr2_env_var in os.environ
    hr2_dir = Path(os.environ[hr2_env_var])
    # print(f"{hr2_dir=}")
    hr2_dir = hr2_dir

    # Only search the JLCPCB parts library once:
    matches: Dict[str, List[Match]]
    error, matches = jlcpcb_parts_search(hr2_dir)
    if error:
        errors += 1
    else:
        # Iterate over all of the *bom_paths*:
        bom_paths: Tuple[Tuple[Path, str, int], ...] = (
            (hr2_dir / "electrical" / "master_board" / "rev_a", "master_board", 5),
            (hr2_dir / "electrical" / "encoder" / "rev_a", "encoder", 10),
        )
        path: Path
        prefix: str
        pcb_count: int
        for path, prefix, pcb_count in bom_paths:
            print(f"{path}, {prefix}, {pcb_count}:")
            jlcpcb_bom: Dict[str, Row]
            other_bom: Dict[str, Row]
            bom_indices: Dict[str, int]
            kicad_bom_path: Path = path / (prefix + "_kicad_bom.csv")
            bom_indices, jlcpcb_bom, other_bom, error = bom_read(kicad_bom_path)
            if not error:
                jlcpcb_bom_path: Path = path / (prefix + "_jlcpcb_bom.csv")
                error = jlcpcb_parts_csv_generate(jlcpcb_bom_path, bom_indices,
                                                  jlcpcb_bom, matches, match_indices, pcb_count)
                if not error:
                    kicad_pos_path: Path = path / (prefix + "-all-pos.csv")
                    jlcpcb_pos_path: Path = path / (prefix + "_jlcpcb_pos.csv")

                    error = jlcpcb_pos_csv_generate(kicad_pos_path, jlcpcb_pos_path)
            if error:
                print(error)
                errors += 1
            print("----------------------------------------------------------------")
            print("")
    return min(1, errors)


PositionRow = NamedTuple("PositionRow",
                         ["reference", "value", "package", "x", "y", "rotation", "side"])


def jlcpcb_pos_csv_generate(kicad_pos_path: Path, jlcpcb_pos_path) -> str:
    """Generate the position .csv file for JLCPCB."""
    error: str = ""
    kicad_pos_csv_file: IO[str]
    if not kicad_pos_path.exists():
        print(f"**************** {kicad_pos_path} does not exist")
    else:
        with open(kicad_pos_path, encoding="utf8", errors="ignore") as kicad_pos_csv_file:
            desired_header: List[str] = ["Ref", "Val", "Package", "PosX", "PosY", "Rot", "Side"]
            pos_reader = csv.reader(kicad_pos_csv_file)
            position_row: PositionRow
            position_rows: List[PositionRow] = []
            for row_index, row in enumerate(pos_reader):
                if row_index == 0:
                    if row != desired_header:
                        error = f"Header mismatch actual {row} != desired {desired_header}"
                        break
                else:
                    position_row = PositionRow(*row)
                    position_rows.append(position_row)

            lines: List[str] = ['"Designator","Mid X","Mid Y","Layer","Rotation"']
            for position_row in position_rows:
                side_text: str = "T" if position_row.side == "top" else "B"
                lines.append(
                    f'"{position_row.reference}",'
                    f'"{position_row.x}mm",'
                    f'"{position_row.y}mm",'
                    f'"{side_text}",'
                    f'"{int(float(position_row.rotation))}"'
                )
            lines.append("")
            position_text: str = '\n'.join(lines)
            jlcpcb_pos_csv_file: IO[str]
            with open(jlcpcb_pos_path, "w") as jlcpcb_pos_csv_file:
                jlcpcb_pos_csv_file.write(position_text)
    return error


def jlcpcb_parts_csv_generate(jlcpcb_bom_path: Path, bom_indices: Dict[str, int],
                              jlcpcb_bom: Dict[str, Row], matches: Dict[str, List[Match]],
                              match_indices: Dict[str, int], pcb_count) -> str:
    """Generate the parts .csv for JLCPCB."""
    error: str = ""
    unmatched_values: List[str] = []
    duplicate_matches: List[str] = []
    unfound_values: List[str] = []
    value_matches: List[Match]
    jlcpcb_regular_expressions: Dict[str, REPatterns] = jlcpcb_regular_expressions_get()

    match_index: int
    value: str
    for match_index, value in enumerate(sorted(matches.keys())):
        value_matches = matches[value]
        value_matches_size: int = len(value_matches)
        if value_matches_size > 1:
            # Sort so that Basic comes before Extended, next get lowest price:
            value_matches.sort()
            if True or value_matches[0][0] != "Basic":
                value_matches.sort()
                print(f"Match[{match_index+1}]: {value} {len(value_matches)} matches:")
                sub_match: Match
                sub_match_index: int
                for sub_match_index, sub_match in enumerate(value_matches[:22]):
                    price_breaks: PriceBreaks = sub_match.price_breaks
                    if price_breaks:
                        print(f"  SubMatch[{sub_match_index+1}]: "
                              f"{'B' if sub_match.library_type == 'Basic' else 'E'}",
                              f"{sub_match.stock}, "
                              f"{sub_match.lcsc}, "
                              f"{sub_match.package}, "
                              f"{sub_match.manufacturer_part}, ",
                              f"{sub_match.description}, "
                              f"${price_breaks[0].price:.2f}, ")
                    if sub_match.lcsc == "C127509":
                        print(f"---->${price_breaks[0].price:.2f} {price_breaks}")
                print("")
            del value_matches[1:]
            assert len(value_matches) == 1

    for value in sorted(jlcpcb_regular_expressions.keys()):
        if value in matches:
            matches_size: int = len(matches[value])
            if matches_size == 0:
                unmatched_values.append(value)
            elif matches_size > 1:
                duplicate_matches.append(value)
        else:
            unfound_values.append(value)

    # Generate any error messages:
    index: int
    if unmatched_values:
        error = f"**************** {len(unmatched_values)} Unmatched JLCPCB parts"
        print(error)
        for index, value in enumerate(unmatched_values):
            print(f"[{index+1}]:{value}")
        print("")
    elif duplicate_matches:
        error = f"**************** {len(unmatched_values)} Duplicate JLCPCB parts"
        print(error)
        for index, value in enumerate(unmatched_values):
            print(f"[{index+1}]:{value}")
        print("")
    elif unfound_values:
        error = f"**************** {len(unfound_values)} JLCPCB parts were not found"
        print(error)
        for index, value in enumerate(unfound_values):
            print(f"[{index+1}]:{value}")
        print("")
    print(f"{len(jlcpcb_bom)} matched JLCPCB parts")

    # Construct the *matches* list:
    value_index: int = bom_indices["Value"]
    references_index: int = bom_indices["Ref"]
    # lcsc_index: int = match_indices["lcsc"]
    # manufacturer_part_index: int = match_indices["manufacturer_part"]
    # package_index: int = match_indices["package"]
    extended_count: int = 0
    parts_cost: float = 0.0
    lines: List[str] = ['"Comment","Designator","Footprint","LCSC Part #","Mfg Part No."']
    row: Row
    sorted_rows: List[Row] = sorted(jlcpcb_bom.values())
    for row in sorted_rows:
        value = row[value_index]
        colon_index: int = value.find(";")
        trimmed_value: str = value[:colon_index] if False and colon_index > 0 else value
        if value in matches:
            # Grab some values from the first *match* in *matches_list*:
            matches_list: List[Match] = matches[value]
            if not matches_list:
                print(f"=====================>Matches list for '{value}' is empty")
            else:
                match: Match = matches_list[0]
                price_break: PriceBreak = match.price_breaks[0]
                library_type: str = str(match.library_type)
                manufacturer_part: str = str(match.manufacturer_part)
                package: str = str(match.package)
                lcsc_part: str = str(match.lcsc)

                # Except for LED's, strip everything after the '_" off the *package":
                if not package.startswith("LED"):
                    underscore_index: int = package.find('_')
                    if underscore_index >= 0:
                        package = package[:underscore_index]

                price: float = float(price_break.price)
                references: str = row[references_index]
                references_count: int = len(references.split(references))
                part_cost: float = references_count * price
                if library_type == "Extended":
                    extended_count += 1
                parts_cost += part_cost
                line: str = (f'"{trimmed_value}","{references}","{package}","{lcsc_part}",'
                             f'"{manufacturer_part}",${pcb_count*part_cost:.2f},{library_type}')
                lines.append(line)
    # Write out `*_jlcpcb_bom.csv` file.
    lines.append("")
    jlcpcb_bom_file: IO[str]
    with open(jlcpcb_bom_path, "w") as jlcpcb_bom_file:
        bom_text: str = '\n'.join(lines)
        jlcpcb_bom_file.write(bom_text)

    # Write out summary information:
    print(f"Per board part cost: ${parts_cost:.2f}")
    setup_cost: float = extended_count * 3.00
    print(f"Setup cost: {extended_count} x $3 = ${setup_cost:.2f}")
    total_cost: float = pcb_count * parts_cost + setup_cost
    print(f"Total Parts Cost: {pcb_count} * ${parts_cost:.2f} + "
          f"{setup_cost:.2f} = {total_cost:.2f}")
    average_cost: float = total_cost / float(pcb_count)
    print(f"Average Board Parts cost: ${total_cost:.2f}/{pcb_count} = ${average_cost:.2f}")

    return error


def jlcpcb_parts_search(hr2_dir: Path) -> Tuple[str, Dict[str, List[Match]]]:
    """Process the parts file."""
    # Define some stuff:
    error: str = ""
    desired_jlcpcb_header: Tuple[str, ...] = (
        "LCSC Part",
        "First Category",
        "Second Category",
        "MFR.Part",
        "Package",
        "Solder Joint",
        "Manufacturer",
        "Library Type",
        "Description",
        "Datasheet",
        "Price",
        "Stock",
    )
    parts_path: Path = (hr2_dir /
                        "electrical" / "orders" / "order1" / "jlcpcb_smt_parts_library.csv")
    # print(f"{parts_path=}")

    # Define a helper function to parts a part pricing string:
    def price_breaks_parse(price_breaks_text: str) -> Tuple[PriceBreak, ...]:
        """Parse a price string into a PriceBreak."""
        # The format of a price break is "LOW_QUANTITY-HIGH_QUANTITY:PRICE".
        # The last one is of the form "LOW_QUANTITY-:PRICE".
        price_breaks: List[PriceBreak] = []
        price_break_texts: List[str] = price_breaks_text.split(",")
        price_break_text: str
        for price_break_text in price_break_texts:
            # print(f"{price_break_text=}")
            if price_break_text:
                hyphen_index: int = price_break_text.find('-')
                colon_index: int = price_break_text.find(':')
                low_quantity: int = int(price_break_text[:hyphen_index])
                high_quantity_text: str = price_break_text[hyphen_index+1:colon_index]
                high_quantity: int = 999999
                if high_quantity_text.isdecimal():
                    high_quantity = int(high_quantity_text)
                    price: float = float(price_break_text[colon_index+1:])
                    price_break: PriceBreak = PriceBreak(low_quantity,
                                                         price, high_quantity)
                    price_breaks.append(price_break)
        # Always ensure that *price_breaks* is non-empty and has a min-quanitity of 1 value:
        price_breaks.sort()
        if not price_breaks or price_breaks[0].min_quantity > 10:
            price_breaks.append(PriceBreak(1, 999999.0, 999999999))
            price_breaks.sort()
        assert price_breaks[0].min_quantity <= 10, price_breaks
        return tuple(sorted(price_breaks))

    # Create JLCPCB indices:
    jlcpcb_indices: Dict[str, int] = {}
    index: int
    heading: str
    for index, heading in enumerate(desired_jlcpcb_header):
        jlcpcb_indices[heading] = index

    # Sweep through acceptable values to build up the various regular expressions:
    jlcpcb_regular_expressions: Dict[str, REPatterns] = jlcpcb_regular_expressions_get()
    value_re_list: List[str] = []
    # package_re_list: List[str] = []
    # lcsc_re_list: List[str] = []
    re_patterns_table: Dict[str, Tuple[re.Pattern, re.Pattern, re.Pattern, re.Pattern]] = {}
    name: str
    re_patterns: REPatterns
    for name, re_patterns in jlcpcb_regular_expressions.items():
        assert isinstance(name, str)
        # Unpack the value and pattern Regular Expressions:
        description_re: str
        package_re: str
        manufacturer_part_re: str
        lcsc_re: str
        value_re, package_re, manufacturer_part_re, lcsc_re = re_patterns
        if value_re and package_re and lcsc_re and manufacturer_part_re:
            value_re_pattern: re.Pattern = re.compile(value_re)
            package_re_pattern: re.Pattern = re.compile(package_re)
            lcsc_re_pattern: re.Pattern = re.compile(lcsc_re)
            manufacturer_part_re_pattern: re.Pattern = re.compile(manufacturer_part_re)
            re_patterns_table[name] = (value_re_pattern, package_re_pattern,
                                       manufacturer_part_re_pattern, lcsc_re_pattern)
            value_re_list.append(value_re)
            # package_re_list.append(package_re)
            # lcsc_re_list.append(lcsc_re)

    # Create an RE that matches all part patterns:
    all_values_pattern: str = '|'.join(value_re_list)
    all_values_re: re.Pattern = re.compile(all_values_pattern)
    # print(f"{all_values_pattern=}")
    matches: Dict[str, List[Match]] = {}

    # Choke if no regular expressions are set or the file does not exist:
    if not re_patterns_table:
        error = "No regular expression values are set"
    elif not parts_path.exists():
        error = f"'{parts_path}' does not exist"
    else:
        # Open the parts CSV file and start a CSV file reader:
        parts_file: IO[str]
        with open(parts_path, encoding="utf8", errors="ignore") as parts_file:
            parts_file = parts_file
            parts_reader = csv.reader(parts_file, delimiter=",")
            row: List[str]
            row_index: int
            header: Tuple[str, ...]

            # Sweep through the file.  Process the header first and then all of the entries:
            part_file: IO[str]
            for row_index, row in enumerate(parts_reader):
                if row_index == 0:
                    # Make sure the header matches:
                    header = tuple(row)
                    if header != desired_jlcpcb_header:
                        error = f"Part headers is {header} not {desired_jlcpcb_header}"
                        break
                else:
                    # Convert *row* into *jlcpcb_row* and try to match *description*:
                    jlcpcb_row: JlcpcbRow = JlcpcbRow(*row)
                    description: str = str(jlcpcb_row.description)
                    if all_values_re.search(description):
                        # We have a match opportunity, now see if it matches across the board:
                        # print(f"[{row_index}]: '{description}' '{package}'")
                        compiled_re_patterns: Tuple[re.Pattern, re.Pattern, re.Pattern, re.Pattern]
                        for name, compiled_re_patterns in re_patterns_table.items():
                            # Unpack *description_package_patterns*:
                            description_pattern: re.Pattern
                            footprint_pattern: re.Pattern
                            lcsc_pattern: re.Pattern
                            manufacturer_part_pattern: re.Pattern
                            description_pattern, package_pattern, manufacturer_part_pattern, \
                                lcsc_pattern = compiled_re_patterns

                            # Determine if there is a solid match:
                            package: str = str(jlcpcb_row.package)
                            lcsc: str = str(jlcpcb_row.lcsc)
                            manufacturer_part: str = jlcpcb_row.manufacturer_part

                            description_match: bool = bool(description_pattern.search(description))
                            package_match: bool = bool(package_pattern.search(package))
                            manufacturer_part_match: bool = bool(
                                manufacturer_part_pattern.search(manufacturer_part))
                            lcsc_match: bool = bool(lcsc_pattern.search(lcsc))
                            patterns_match: bool = (description_match and package_match and
                                                    lcsc_match and manufacturer_part_match)
                            if patterns_match:
                                # We have a solid *match*. Save it into *matches*:
                                match: Match = Match(
                                    jlcpcb_row.library_type,
                                    price_breaks_parse(jlcpcb_row.price),
                                    name,
                                    description,
                                    package,
                                    lcsc,
                                    jlcpcb_row.stock,
                                    manufacturer_part)
                                if name not in matches:
                                    matches[name] = []
                                if int(match.stock):
                                    matches[name].append(match)
                                # print(f"match[{row_index}]: {match}")
    return error, matches  # , jlcpcb_indices


def bom_read(bom_path: Path) -> Tuple[Dict[str, int], Dict[str, Row], Dict[str, Row], str]:
    """Read in the BOM and perform santity checks."""
    bom_indices: Dict[str, int] = {}
    jlcpcb_bom: Dict[str, Row] = {}
    other_bom: Dict[str, Row] = {}
    error: str = ""

    # print(f"{bom_path=}")
    actual_generator: str = ""
    if not bom_path.exists():
        error = f"{bom_path} does not exist"
    else:
        # Read in *bom_path*:
        rows: List[Row] = []
        bom_file: IO[str]
        with open(bom_path, encoding="utf8", errors="ignore") as bom_file:
            bom_reader = csv.reader(bom_file, delimiter=",")
            row: List[str]
            for row in bom_reader:
                # Skip over non BOM entries:
                if len(row) == 2:
                    if row[0] == "Generator:":
                        actual_generator = row[1]
                elif len(row) > 2:
                    rows.append(tuple(row))

        # Extract the header and tuplize the bom rows:
        if not rows:
            error = f"No data in {bom_path}"
        else:
            header: Row = rows.pop(0)
            bom_rows = tuple(rows)

            # Fill in bom_indices:
            index: int
            heading: str
            for index, heading in enumerate(header):
                bom_indices[heading] = index

            # Do some sanity checking:
            desired_generator: str = "bom_csv_grouped_by_value_with_fp"
            full_desired_generator: str = f"/usr/share/kicad/plugins/{desired_generator}.py"
            if actual_generator != full_desired_generator:
                error = (f"BOM needs to be generated by '{desired_generator}': "
                         f"not '{actual_generator}'!")
            elif len(bom_rows) < 2:
                error = "No data found in {bom_path}"
            else:
                desired_header: Tuple[str, ...] = (
                    "Ref", "Qnty", "Value", "Cmp name", "Footprint", "Description", "Vendor")
                if header != desired_header:
                    error = (f"{header=} != {desired_header=}!"
                             " Use '{desired_generator}' KiCad BOM generator.")
                elif not error:
                    error, jlcpcb_bom, other_bom = bom_split(bom_indices, bom_rows)
                    if not error:
                        error = bom_process(bom_indices, bom_rows)

    return bom_indices, jlcpcb_bom, other_bom, error


def bom_split(bom_indices: Dict[str, int],
              bom_rows: Rows) -> Tuple[str, Dict[str, Row], Dict[str, Row]]:
    """Split the BOM into a JLCPCB BOM and an other BOM."""
    jlcpcb_regular_expressions: Dict[str, REPatterns] = jlcpcb_regular_expressions_get()
    other_values: Set[str] = other_values_get()
    value_index: int = bom_indices["Value"]

    error: str = ""
    jlcpcb_bom: Dict[str, Row] = {}
    other_bom: Dict[str, Row] = {}

    unmatched_values: List[str] = []
    bom_row: Row
    for bom_row in bom_rows:
        bom_value: str = bom_row[value_index]
        if bom_value in other_values:
            other_bom[bom_value] = bom_row
        elif bom_value in jlcpcb_regular_expressions:
            jlcpcb_bom[bom_value] = bom_row
        else:
            unmatched_values.append(bom_value)

    if unmatched_values:
        error = f"**************** {len(unmatched_values)} Unmatched values found"
        print(error)
        unmatched_value: str
        index: int
        for index, unmatched_value in enumerate(unmatched_values):
            print(f"Unmatched[{index+1}]: {unmatched_value}")

    return error, jlcpcb_bom, other_bom


def bom_process(column_indices: Dict[str, int], bom_rows: Rows) -> str:
    """Process the BOM."""
    error: str = ""
    # Do some sanity checks:
    # print(f"{header=}")

    # Use logical names rather than hard code indices in code below.
    # This makes it easier to change BOM generators:
    value_index: int = column_indices["Value"]
    footprint_index: int = column_indices["Footprint"]

    error = unrecognized_footprints_check(bom_rows, footprint_index)
    if not error:
        error = unrecognized_values_check(bom_rows, value_index)
    return error


def unrecognized_footprints_check(bom_rows: Rows, footprint_index: int) -> str:
    """Return an error string if unrecognized footprints are detected."""
    # Sweep through looking for acceptable vs unacceptable footprints:
    unrecognized_footprints: Set[str] = set()
    jlcpcb_footprints: Set[str] = jlcpcb_footprints_get()
    other_footprints: Set[str] = other_footprints_get()
    bom_index: int
    bom_row: Tuple[str, ...]
    footprint: str
    for bom_row in bom_rows:
        footprint = bom_row[footprint_index]
        if footprint not in jlcpcb_footprints and footprint not in other_footprints:
            unrecognized_footprints.add(footprint)
    error: str = ""
    if unrecognized_footprints:
        error = "Unrecognized footprints"
        print(f"{len(unrecognized_footprints)} unrecognized footprints")
        for footprint in sorted(unrecognized_footprints):
            print(f"'{footprint}'")
        print("")
    return error


def jlcpcb_footprints_get() -> Set[str]:
    """Return a set of acceptable footprints for JLCPCB."""
    return {
        "Capacitor_SMD:C_0402_1005Metric",
        "Capacitor_SMD:C_0603_1608Metric",
        "Crystal:Crystal_SMD_3215-2Pin_3.2x1.5mm",
        "HR2:BUTTON_6x6",
        "Button_Switch_SMD:SW_Push_1P1T_NO_6x6mm_H9.5mm",
        "HR2:TE1376164-1_COINHOLDER6.8",
        "LED_SMD:LED_0603_1608Metric",
        "32.768kHz9pF;3.2x1.5",
        "MCP2542;TDFN8EP3x2",
        "Package_SO:HTSSOP-16-1EP_4.4x5mm_P0.65mm_EP3.4x5mm_Mask3x3mm_ThermalVias",
        "Package_DFN_QFN:TDFN-8-1EP_3x2mm_P0.5mm_EP1.80x1.65mm",
        "Package_SO:HTSSOP-16-1EP_4.4x5mm_P0.65mm_EP3.4x5mm",
        "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm",
        "Package_SO:SOP-4_3.8x4.1mm_P2.54mm",
        "Package_SO:TSOP-6_1.65x3.05mm_P0.95mm",
        "Package_SO:TSSOP-16_4.4x5mm_P0.65mm",
        "Package_SO:TSSOP-8_3x3mm_P0.65mm",
        "Package_TO_SOT_SMD:SOT-223-3_TabPin2",
        "Package_TO_SOT_SMD:SOT-23",
        "Package_TO_SOT_SMD:TSOT-23-6",
        "Resistor_SMD:R_0402_1005Metric",
        "Resistor_SMD:R_0603_1608Metric",
        "Resistor_SMD:R_0805_2012Metric",
        "Resistor_SMD:R_1206_3216Metric",
    }


def other_footprints_get() -> Set[str]:
    """Return a set of footprints that do not work for JLCPCB.

    Currently all through hole parts are not suitable for JLCPCB.
    """
    return {
        "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm",
        "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
        "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical",
        "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical",
        "Connector_USB:USB_C_Receptacle_JAE_DX07S024WJ3R400",
        "HR2:ENCODER_2xF1x3",
        "HR2:ENCODER_2xM1x3RA",
        "HR2:GROVE20x20",
        "HR2:HCSR04_F1x4",
        "HR2:HCSR04_F1x4H",
        "HR2:HCSR04_F1x4LP",
        "HR2:LED_GRNRA",
        "HR2:LIDAR_ADAPTER_2xF1x4_F1x3",
        "HR2:NUCLEO_F767ZI_2xF2x35",
        "HR2:PinHeader_2x06_P2.54mm_Vertical_Shrouded",
        "HR2:RASPI_F2X20",
        "HR2:SC59",
        "HR2:STADAPTER_F2x4",
        "HR2:U3V70xMATE_F1x4+F1x5",
        "MountingHole:MountingHole_2.7mm_M2.5",
        "TestPoint:TestPoint_THTPad_1.5x1.5mm_Drill0.7mm",
    }


def unrecognized_values_check(bom_rows: Rows, value_index: int) -> str:
    """Return an error string if unrecognized values are detected."""
    # Sweep through looking for acceptable vs unacceptable values:
    unrecognized_values: Set[str] = set()
    jlcpcb_regular_expressions: Dict[str, REPatterns] = jlcpcb_regular_expressions_get()
    other_values: Set[str] = other_values_get()
    bom_index: int
    bom_row: Tuple[str, ...]
    value: str
    for bom_row in bom_rows:
        value = bom_row[value_index]
        is_jlcpcb: bool = value in jlcpcb_regular_expressions
        is_other: bool = value in other_values
        if not is_jlcpcb and not is_other:
            unrecognized_values.add(value)
    error: str = ""
    if unrecognized_values:
        error = "Unrecognized values"
        print(f"{len(unrecognized_values)} unrecognized values")
        for value in sorted(unrecognized_values):
            print(f"'{value}'")
        print("")
    return error


def jlcpcb_regular_expressions_get() -> Dict[str, REPatterns]:
    """Return map from JLCPCB value regular expression pair."""
    regular_expressions: Dict[str, Tuple[str, str, str, str]] = {
        ".16Ω.25W;2012": (r"0\.16Ohm", "0805$", ".*", ".*"),
        "0.01µF;1005": (" 10nF", "0402$", ".*", ".*"),
        "0.1µF;1005": (" 100nF", "0402$", ".*", "C1525$"),
        "0Ω;1608": (" 0Ohm", "0603$", ".*", ".*"),
        "0Ω;1005": (" 0Ohm", "0402$", ".*", ".*"),
        "0Ω;3216": (" 0Ohm", "1206$", ".*", ".*"),
        # "100Ω;1005": (" 100Ohms", "0402$", ".*", ".*"),
        "100KΩ;1005": (" 100KOhms", "0402$", ".*", "C25741$"),
        "100KΩ;1608": (" 100KOhms", "0603$", ".*", "C25803$"),
        "10KΩ;1608": (" 10KOhms", "0603$", ".*", "C25804$"),
        "10µF;1608": (" 10uF", "0603$", ".*", "C19702$"),
        "120Ω;1005": (" 120Ohms", "0402$", ".*", ".*"),
        # "120Ω.25W;1608": (" 120Ohms", "0603$", ".*", "C22787$"),
        "120Ω.25W;3216": (" 120Ohms", "1206$", ".*", "C17909$"),
        # "16pF;1608": (" 16pF", "0603$", ".*", "C1646$"),  # Only in 0603
        "15pF;1608": (" 15pF", ".*", ".*", "C1644$"),
        "150Ω;1005": (" 150Ohms", "0402$", ".*", ".*"),
        "220Ω;1005": (" 220Ohms", "0402$", ".*", ".*"),
        "1KΩ;1005": (" 1KOhms 1%", "0402$", ".*", "C11702$"),
        "1KΩ;1608": (" 1KOhms 1%", "0603$", ".*", "C21190$"),
        "2.2µF6.3V;1608": (" 2.2uF", "0603$", ".*", "C23630$"),
        "270Ω;1608": (" 270Ohms", "0603$", ".*", ".*"),
        "2N7002;SOT23": ("MOSFET N Trench.*", ".*", "2N7002", "C8545$"),
        "3.9KΩ;1005": (r" 3\.9KOhms", "0402$", ".*", ".*"),
        "32.768kHz9pF;3.2x1.5": ("32.768", ".*", ".*", "C32346$"),
        "330Ω;1608": (" 330Ohms 1%", "0603$", ".*", ".*"),
        "4.7KΩ;1005": (r" 4\.7KOhms 1%", "0402$", ".*", ".*"),
        "4.7µF;1608": (" 4.7uF", "0603$", ".*", "C19666$"),
        "470Ω.25W;1608": (" 470Ohms 1%", "0603$", ".*", ".*"),
        "470Ω;1608": (" 470Ohms 1%", "0603$", ".*", ".*"),
        "560Ω;1608": (" 560Ohms", "0603$", ".*", ".*"),
        "ACS711;SOIC8": ("Current Sensors SOIC-8_150mil RoHS", ".*",  ".*", "C10681$"),
        "AH1806;SC59": (r"Magnetic Sensors SOT-23", ".*", ".*", "C126664"),
        "AP2114HA-3.3TRG1_1A;SOT223": (r"Low Dropout Regulators\(LDO\) SOT-223",
                                       ".*",  ".*", "C166063$"),
        "BUTTON;6x6": ("SPST.*Tactile Switches", "6.*x6", ".*", "C127509$"),
        "CAT24C32;SOIC8": (r"EEPROM 32Kb", "SOIC-8", "CAT24C32", ".*"),
        "CD4504B;TTSOP16": ("Level Translators", ".*", "4504", "C233582$"),
        "CPC1017N;SOP4W3.8L4.1": ("Solid State Relays SOP-4_P2.54 RoHS", ".*", ".*", "C261926$"),
        "DRV8833PWPR;HTSSOP16EP3.4x5": ("Motor Drivers HTSSOP-16 RoHS",
                                        "HTSSOP-16", ".*", "^C50506$"),
        "LEDGRN;1608": ("LED.*Green", "LED_0603", ".*", "C72043$"),
        "LM5050-1;TSOT-23-6": ("PMIC - Power Distribution Switches SOT-23-6 RoHS",
                               ".*", ".*", "C55266$"),
        "MCP2542;TDFN8EP3x2": ("CAN", ".*", "MCP2542", ".*"),
        "MCP7940;SOIC8": ("Real-time Clocks Clock/Calendar I2C", "SOIC-8", ".*", "C7440$"),
        "NFET_3A_GSD;SOT23": (r"MOSFET N Trench.*V [3-7]\.[0-9]A", "SOT-23-3", ".*", "C20917$"),
        "PFET_6A_GSD;SOT23": (r"MOSFET P Trench.*V [6-7]\.[0-9]A", "SOT-23-3", ".*", "C141546$"),
        "REDLED;1608": ("LED.*Red", "LED_0603", ".*", "C2286$"),
        "SN74HC165;HTSSOP16EP3.4x5": ("74 Series Shift Register.*TSSOP-16",
                                      "HTSSOP-16", "165", "C201716$"),
        # "SN74HC595;TTSOP16": ("74 Series TSSOP-16 RoHS", ".*", "595", ".*"),  # "C5948$"),
        "SN74HC595;HTSSOP16EP3.4x5": ("74 Series.*TSSOP-16 RoHS", ".*", "595", "C5948$"),
    }
    return regular_expressions


def other_values_get() -> Set[str]:
    """Return a set of KiCAD values that will not work with JLCPCB.

    Currently all through hole parts are not suitable for JLCPCB.
    """
    return {
        "1000µF@10Vmin;D10P5H13max",
        "3.3V",
        "470µF,25Vmin;D10P5H13max",
        "5V",
        "74LVC1G74;TSSOP8",
        "74x11G1;TSOP6",
        "9V",
        "COIN_HOLDER6.8R;TE1376164-1",
        "EXT_ESTOP;M1x2",
        "GND",
        "LED_EN;M1x3",
        "LIDAR_ADAPTER;2xF1x4_F1x3",
        "LED;GRNRA",
        "HR2_ENCODER;2xF1x3",
        "JUMPER;M1x2",
        "HCSR04LP;F1X4",
        "HOLE;M2.5",
        "GROVE;20x20",
        "NUCLEO-F767ZI;2xF2x35",
        "P5V",
        "POLOLU_U3V70F9;F1x4+F1x5",
        "RASPI;F2X20",
        "RPI_DISP_EN;M1x3",
        "RPI_DSP_PWR;M1x2",
        "SBC_RX;M1x2",
        "SENSE;M1x3",
        "SERVO;M1x3",
        "SERVO;M1x4",
        "SHUNT;M1x2",
        "SPC_TX;M1x2",
        "STADAPTER;F2x4",
        "WOW_OUT;M2x6S",
        "WOW_RX;M1x1",
        "WOW_TX;M1x1",
        "USB-C;USB-C",
        "USB-C;USB-C,POGND",
        "USB5V",
        "~nESTOP_SET",
        "~nWOW_ESTOP;M1x1",
        # For encoder/rev_a:
        "ENCODER;2xM1x3RA",
    }


if __name__ == "__main__":
    main()