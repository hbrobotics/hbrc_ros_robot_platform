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
from typing import Any, Dict, IO, List, Set, Tuple

Row = Tuple[str, ...]
Rows = Tuple[Row, ...]
PriceBreak = Tuple[int, float, int]
Match = Tuple[str, Tuple[PriceBreak, ...], str, str, str, str, str, str]

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
    jlcpcb_indices: Dict[str, int]
    error, matches, jlcpcb_indices = jlcpcb_parts_search(hr2_dir)
    print(f":Matches size:{len(matches)}")
    if error:
        errors += 1
    else:
        # Iterate over all of the *bom_paths*:
        bom_paths: Tuple[Path, ...] = (
            hr2_dir / "electrical" / "master_board" / "rev_a" / "master_board_kicad_bom.csv",
            hr2_dir / "electrical" / "encoder" / "rev_a" / "encoder_kicad_bom.csv",
        )
        for bom_path in bom_paths:
            print(f"{bom_path}:")
            jlcpcb_bom: Dict[str, Row]
            other_bom: Dict[str, Row]
            bom_indices: Dict[str, int]
            bom_indices, jlcpcb_bom, other_bom, error = bom_read(bom_path)
            if not error:
                error = jlcpcb_parts_csv_generate(hr2_dir, bom_indices,
                                                  jlcpcb_bom, matches, match_indices)
            if error:
                print(bom_path)
                print(error)
                errors += 1
    return min(1, errors)


def jlcpcb_parts_csv_generate(hr2_dir: Path, bom_indices: Dict[str, int],
                              jlcpcb_bom: Dict[str, Row], matches: Dict[str, List[Match]],
                              match_indices: Dict[str, int]) -> str:
    """Generate the parts .csv for JLCPCB."""
    error: str = ""
    unmatched_values: List[str] = []
    duplicate_matches: List[str] = []
    unfound_values: List[str] = []
    value_matches: List[Match]
    jlcpcb_regular_expressions: Dict[str, Tuple[str, str, str]] = jlcpcb_regular_expressions_get()

    match_index: int
    value: str
    for match_index, value in enumerate(sorted(matches.keys())):
        value_matches = matches[value]
        value_matches_size: int = len(value_matches)
        if value_matches_size > 1:
            # Sort so that Basic comes before Extended, next get lowest price:
            value_matches.sort()
            if True or value_matches[0][0] != "Basic":
                print(f"Match[{match_index+1}]: {value} {len(value_matches)} matches:")
                sub_match: Match
                sub_match_index: int
                for sub_match_index, sub_match in enumerate(value_matches[:3]):
                    print(f"  SubMatch[{sub_match_index+1}]: {sub_match}")
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
        error = f"{len(unmatched_values)} Unmatched JLCPCB parts"
        print(error)
        for index, value in enumerate(unmatched_values):
            print(f"[{index+1}]:{value}")
        print("")
    elif duplicate_matches:
        error = f"{len(unmatched_values)} Duplicate JLCPCB parts"
        print(error)
        for index, value in enumerate(unmatched_values):
            print(f"[{index+1}]:{value}")
        print("")
    elif unfound_values:
        error = f"{len(unfound_values)} JLCPCB that were not found"
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
    lines: List[str] = ['"Comment","Designator","Footprint","LCSC Part #","Mfg Part No."']
    row: Row
    sorted_rows: List[Row] = sorted(jlcpcb_bom.values())
    for row in sorted_rows:
        value = row[value_index]
        colon_index: int = value.find(";")
        trimmed_value: str = value[:colon_index] if False and colon_index > 0 else value
        if value in matches:
            # We have some matches:
            matches_list: List[Match] = matches[value]
            match: Match = matches_list[0]

            key_to_text: Dict[str, str] = {}
            key: str
            key_index: int
            for key, key_index in match_indices.items():
                text_any: Any = match[key_index]
                if isinstance(text_any, str):
                    key_to_text[key] = text_any

            manufacturer_part: str = key_to_text["manufacturer_part"]
            package: str = key_to_text["package"]
            if not package.startswith("LED"):
                underscore_index: int = package.find('_')
                if underscore_index >= 0:
                    package = package[:underscore_index]
            lcsc_part: str = key_to_text["lcsc"]
            references: str = row[references_index]
            line: str = (f'"{trimmed_value}","{references}","{package}","{lcsc_part}"'
                         f',"{manufacturer_part}"')
            lines.append(line)
    lines.append("")
    text: str = "\n".join(lines)
    print("JLCPCB Data:")
    print(text)

    return error


def jlcpcb_parts_search(hr2_dir: Path) -> Tuple[str, Dict[str, List[Match]], Dict[str, int]]:
    """Process the parts file."""
    parts_path: Path = (hr2_dir /
                        "electrical" / "orders" / "order1" / "jlcpcb_smt_parts_library.csv")

    # print(f"{parts_path=}")
    error: str = ""

    # This is the desired header and only care about a few of the entries:
    desired_header: Tuple[str, ...] = (
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

    # Create JLCPCB indices:
    jlcpcb_indices: Dict[str, int] = {}
    index: int
    heading: str
    for index, heading in enumerate(desired_header):
        jlcpcb_indices[heading] = index

    package_index: int = desired_header.index("Package")
    description_index: int = desired_header.index("Description")
    lcsc_index: int = desired_header.index("LCSC Part")
    library_type_index: int = desired_header.index("Library Type")
    price_index: int = desired_header.index("Price")
    stock_index: int = desired_header.index("Stock")
    manufacturer_part_index: int = desired_header.index("MFR.Part")

    # Sweep through acceptable values to build up the various regular expressions:
    jlcpcb_regular_expressions: Dict[str, Tuple[str, str, str]] = jlcpcb_regular_expressions_get()
    value_re_list: List[str] = []
    package_re_list: List[str] = []
    lcsc_re_list: List[str] = []
    re_patterns: Dict[str, Tuple[re.Pattern, re.Pattern, re.Pattern]] = {}
    name: str
    description_package_triple: Tuple[str, str, str]
    for name, description_package_triple in jlcpcb_regular_expressions.items():
        assert isinstance(name, str)
        # Unpack the value and pattern RE's:
        description_re: str
        package_re: str
        lcsc_re: str
        value_re, package_re, lcsc_re = description_package_triple
        if value_re and package_re and lcsc_re:
            value_re_pattern: re.Pattern = re.compile(value_re)
            package_re_pattern: re.Pattern = re.compile(package_re)
            lcsc_re_pattern: re.Pattern = re.compile(lcsc_re)
            re_patterns[name] = (value_re_pattern, package_re_pattern, lcsc_re_pattern)
            value_re_list.append(value_re)
            package_re_list.append(package_re)
            lcsc_re_list.append(lcsc_re)

    # Create an RE that matches all part patterns:
    all_values_pattern: str = '|'.join(value_re_list)
    all_values_re: re.Pattern = re.compile(all_values_pattern)
    # print(f"{all_values_pattern=}")
    matches: Dict[str, List[Match]] = {}

    # Choke if no regular expressions are set or the file does not exist:
    if not re_patterns:
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
                    if header != desired_header:
                        error = f"Part headers is {header} not {desired_header}"
                        break
                else:
                    # Perform a coarse match first:
                    description: str = row[description_index]
                    package: str = row[package_index]
                    lcsc: str = row[lcsc_index]
                    manufacturer_part: str = row[manufacturer_part_index]
                    if all_values_re.search(description):
                        # We have a match opportunity, now see if it matches across the board:
                        # print(f"[{row_index}]: '{description}' '{package}'")
                        description_package_patterns: Tuple[re.Pattern, re.Pattern, re.Pattern]
                        for name, description_package_patterns in re_patterns.items():
                            description_pattern: re.Pattern
                            footprint_pattern: re.Pattern
                            lcsc_pattern: re.Pattern
                            description_pattern, package_pattern, lcsc_pattern = (
                                description_package_patterns)
                            description_match: bool = bool(description_pattern.search(description))
                            package_match: bool = bool(package_pattern.search(package))
                            lcsc_match: bool = bool(lcsc_pattern.search(lcsc))
                            if description_match and package_match and lcsc_match:
                                # We have a solid match.  Save it into matches:
                                # Parse the price_breaks:
                                library_type: str = row[library_type_index]
                                price_text: str = row[price_index]
                                price_breaks: List[Tuple[int, float, int]] = []
                                price_break_texts: List[str] = price_text.split(",")
                                price_break_text: str
                                for price_break_text in price_break_texts:
                                    # print(f"{price_break_text=}")
                                    if price_break_text:
                                        hyphen_index: int = price_break_text.find('-')
                                        colon_index: int = price_break_text.find(':')
                                        low_quantity: int = int(price_break_text[:hyphen_index])
                                        high_quantity_text: str = (
                                            price_break_text[hyphen_index+1:colon_index])
                                        high_quantity: int = 999999
                                        if high_quantity_text.isdecimal():
                                            high_quantity = int(high_quantity_text)
                                            price: float = float(price_break_text[colon_index+1:])
                                        price_break: PriceBreak = (
                                            low_quantity, price, high_quantity)
                                        price_breaks.append(price_break)
                                # print(f"{price_breaks=}")
                                stock: str = row[stock_index]
                                match: Match = (
                                    library_type,
                                    tuple(sorted(price_breaks)),
                                    name,
                                    description,
                                    package,
                                    lcsc,
                                    stock,
                                    manufacturer_part)
                                if name not in matches:
                                    matches[name] = []
                                matches[name].append(match)
                                # print(f"match[{row_index}]: {match}")
    print(f"Matches size:{len(matches)}")
    return error, matches, jlcpcb_indices


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
    jlcpcb_regular_expressions: Dict[str, Tuple[str, str, str]] = jlcpcb_regular_expressions_get()
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
        error = f"{len(unmatched_values)} Unmatched values found"
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
        "Crystal:Crystal_SMD_5032-2Pin_5.0x3.2mm",
        "HR2:BUTTON_6x3.5",
        "HR2:TE1376164-1_COINHOLDER6.8",
        "LED_SMD:LED_0603_1608Metric",
        "Package_SO:HTSSOP-16-1EP_4.4x5mm_P0.65mm_EP3.4x5mm_Mask3x3mm_ThermalVias",
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
    jlcpcb_regular_expressions: Dict[str, Tuple[str, str, str]] = jlcpcb_regular_expressions_get()
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


def jlcpcb_regular_expressions_get() -> Dict[str, Tuple[str, str, str]]:
    """Return map from JLCPCB value regular expression pair."""
    regular_expressions: Dict[str, Tuple[str, str, str]] = {
        ".16Ω.25W;2012": (r"0\.16Ohm", "0805$", ".*"),
        "0.01µF;1005": (" 10nF", "0402$", ".*"),
        "0.1µF;1005": (" 100nF", "0402$", "C1525$"),
        "0Ω;1608": (" 0Ohm", "0603$", ".*"),
        "100Ω;1005": (" 100Ohms", "0402$", ".*"),
        "100KΩ;1005": (" 100KOhms", "0402$", "C25741$"),
        "100KΩ;1608": (" 100KOhms", "0603$", "C25803$"),
        "10KΩ;1608": (" 10KOhms", "0603$", "C25804$"),
        "10µF;1608": (" 10uF", "0603$", "C19702$"),
        "120Ω.25W;1608": (" 120Ohms", "0603$", "C22787$"),
        "16pF;1005": (" 16pF", "0402$", ".*"),  # Only in 0603
        "1KΩ;1005": (" 1KOhms 1%", "0402$", "C11702$"),
        "1KΩ;1608": (" 1KOhms 1%", "0603$", "C21190$"),
        "2.2µF6.3V;1608": (" 2.2uF", "0603$", "C24630$"),
        "2N7000;SOT23": ("", ".*", ".*"),
        "3.9KΩ;1005": (r" 3\.9KOhms", "0402$", ".*"),
        "32.768kHz9pF;3.2x1.5": ("32.768", ".*", ".*"),
        "330Ω;1608": (" 330Ohms 1%", "0603$", ".*"),
        "3A_PWR_NMOS_DSG;SOT23": ("", ".*", ".*"),
        "4.7KΩ;1005": (r" 4\.7KOhms 1%", "0402$", ".*"),
        "4.7µF;1608": (" 4.7uF", "0603$", "19666$"),
        "470Ω.25W;1608": (" 470Ohms 1%", "0603$", ".*"),
        "470Ω;1608": (" 470Ohms 1%", "0603$", ".*"),
        "74LVC1G74;TSSOP8": ("", ".*", ".*"),
        "74x11G1;TSOP6": ("", ".*", ".*"),
        "ACS711;SOIC8": ("Current Sensors SOIC-8_150mil RoHS", ".*", "C10681$"),
        "AH1806;SC59": (r"Magnetic Sensors SOT-23\(SOT-23-3\) RoHS", ".*", ".*"),
        "AP2114HA-3.3TRG1_1A;SOT223": (r"Low Dropout Regulators\(LDO\) SOT-223", ".*", "C166063$"),
        "BUTTON;6x3.5": ("", ".*", ".*"),
        "CAT24C32;SOIC8": (r"EEPROM EEPROM 32Kb \(4K x 8\) I2C SOIC-8_150mil RoHS", ".*", "C6654$"),
        "CD4504B;TTSOP16": ("Level Translators,  Shifters SOIC-16_150mil RoHS", ".*", "C151885$"),
        "COIN_HOLDER6.8R;TE1376164-1": ("", ".*", ".*"),
        "CPC1017N;SOP4W3.8L4.1": ("Solid State Relays SOP-4_P2.54 RoHS", ".*", "C261926$"),
        "DRV8833PWPR;16HTSSOP": ("Motor Drivers HTSSOP-16 RoHS", "HTSSOP-16", "^C50506$"),
        # "GRNLED;1608": ("LED.*Green", "LED_0603", ".*"),
        "LEDGRN;1608": ("LED.*Green", "LED_0603", "C72043$"),
        "LM5050-1;TSOT-23-6": ("PMIC - Power Distribution Switches SOT-23-6 RoHS", ".*", "C55266$"),
        "MCP2542;SOIC8": ("CAN 1/1 8Mbps", "SOIC-8", "C124016"),
        "MCP7940;SOIC8": ("Real-time Clocks Clock/Calendar I2C", "SOIC-8", "C7440$"),
        "PFET_6A;SOT23": ("", ".*", ".*"),
        "REDLED;1608": ("LED.*Red", "LED_0603", "C2286$"),
        "SN74HC165;TTSOP16": ("74 Series Shift Register", ".*", "C5613$"),
        "SN74HC595;TTSOP16": ("74 Series TSSOP-16 RoHS", ".*", "C5948$"),
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
        "9V",
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
