"""Program to compute spacer stacks."""

# <--------------------------------------- 100characters ----------------------------------------> #


import csv
import itertools
from typing import Callable, Dict, IO, List, Tuple, Union

Immutable = Union[float, str, int]

Spacer = Tuple[Immutable, ...]
Spacers = Tuple[Spacer, ...]
Stack = Tuple[float, float, float, Spacers]
Stacks = Tuple[Stack, ...]
StackKey = Tuple[str, ...]
Selector = Tuple[str, Callable[[str], Immutable]]
Selectors = Tuple[Selector, ...]


def main() -> None:
    """Compute the spacer stacks for the HR2."""
    # Extract *washers* from the associated `.csv` file:
    washer_headings = (
        "Datasheet",
        "Image",
        "DK Part #",
        "Mfr Part #",
        "Mfr",
        "Supplier",
        "Description",
        "Stock",
        "Price",
        "@ qty",
        "Min Qty",
        "Package",
        "Series",
        "Part Status",
        "Type",
        "Thread/Screw/Hole Size",
        "Diameter - Inside",
        "Diameter - Outside",
        "Thickness",
        "Plating",
        "Material",
    )

    washer_selectors: Selectors = (
        ("Thickness", mm_height_select),  # [0]
        ("Type", string_select),  # [1]
        ("Description", string_select),  # [2]
        ("DK Part #", string_select),  # [1] [-2]
        ("Price", float_select),  # [2] [-1]
    )
    washers: Spacers = csv_read("washers_#2.csv", washer_headings, washer_selectors)
    washers_list: List[Spacer] = []
    washer: Spacer
    for washer in washers:
        if washer[1] == "Flat":
            washers_list.append(washer)
    washers = tuple(washers_list)

    # Extract *spacers_standoffs* from the associated `.csv` file:
    spacer_headings = (
        "Datasheet",
        "Image",
        "DK Part #",
        "Mfr Part #",
        "Mfr",
        "Supplier",
        "Description",
        "Stock",
        "Price",
        "@ qty",
        "Min Qty",
        "Package",
        "Series",
        "Part Status",
        "Type",
        "Threaded/Unthreaded",
        "Gender",
        "Screw, Thread Size",
        "Diameter - Inside",
        "Diameter - Outside",
        "Between Board Height",
        "Length - Overall",
        "Features",
        "Material",
        "Plating",
        "Color",
    )
    spacer_selectors: Selectors = (
        ("Between Board Height", mm_height_select),  # [0]
        ("Type", string_select),  # [1]
        ("Threaded/Unthreaded", string_select),  # [2]
        ("Gender", string_select),  # [3]
        ("DK Part #", string_select),  # [4] [-2]
        ("Price", float_select),  # [5] [-1]
    )
    spacers_standoffs: Spacers = csv_read(
        "spacers_standoffs_#2.csv", spacer_headings, spacer_selectors)

    # Extract *spacers* and *standoffs*:
    standoff: Spacer
    standoffs_list: List[Spacer] = []
    for standoff in spacers_standoffs:
        if standoff[3] == "Male, Female" and standoff[2] == "Threaded":
            standoffs_list.append(standoff)
    standoffs: Spacers = tuple(sorted(standoffs_list))

    spacer: Spacer
    spacers_list: List[Spacer] = []
    for spacer in spacers_standoffs:
        if spacer[3] == "Female, Female" and spacer[2] == "Threaded":
            spacers_list.append(spacer)
    spacers: Spacers = tuple(sorted(spacers_list))

    hr2_heights: Dict[str, float] = {
        "master_arm_height": 28.40,  # mm (printed arm_spacer_dz)
        "master_nucleo_height": 13.00,  # mm (from class Nucleo144)
        "base_master_height": 26.05,  # mm
        "battery_master_height": 45.55,  # mm
        "base_pi_height": 15.80,  # mm
        "battery_pi_height": 5.80,  # mm
    }

    stack_name: str
    desired_height: float
    for stack_name, desired_height in hr2_heights.items():
        distance: float
        stacks_list: List[Stack] = \
            stacks_search(desired_height, (washers,)) + \
            stacks_search(desired_height, (standoffs,)) + \
            stacks_search(desired_height, (spacers,)) + \
            stacks_search(desired_height, (standoffs, spacers)) + \
            stacks_search(desired_height, (standoffs, standoffs, spacers)) + \
            stacks_search(desired_height, (washers, standoffs)) + \
            stacks_search(desired_height, (washers, standoffs, standoffs, spacers))
        #  stacks_search(desired_height, (washers, spacers)) + \
        # stacks_search(desired_height, (washers, standoffs, spacers)) + \

        print(f"{stack_name}: {desired_height:.2f}mm")
        stacks: Stacks = tuple(sorted(stacks_list))
        match_index: int
        for match_index, match in enumerate(stacks[:3]):
            print(f"  {stack_name}[{match[0]:.3f}, {match[2]:.2f}, {match[1]:.2f}]:")
            for index, spacer in enumerate(match[-1]):
                print(f"    {stack_name}[{match_index}, {index}]: {spacer}")


def csv_read(csv_file_name: str, headings: Tuple[str, ...], selectors: Selectors) -> Spacers:
    """Return spacers extracted from a .csv file."""
    # Create *indices_table*:
    indices_table: Dict[str, int] = {}
    heading: str
    heading_index: int
    for heading_index, heading in enumerate(headings):
        indices_table[heading] = heading_index

    # Read in CSV file
    csv_file: IO[str]
    spacers_list: List[Spacer] = []
    with open(csv_file_name, encoding="utf-8-sig") as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',', quotechar='"')
        row: List[str]
        row_index: int
        for row_index, row in enumerate(csv_reader):
            if row_index == 0:
                # Verify that the first row of the `.csv` file match *headings*:
                # print(f"{row=}")
                for index, heading in enumerate(row):
                    assert heading == headings[index], (
                        f"[{index}]: '{heading}' != '{headings[index]}'")
                assert len(row) == len(headings)
            else:
                selector_index: int
                immutables: List[Immutable] = []
                for selector_index, selector in enumerate(selectors):
                    select: Callable[[str], Immutable]
                    heading, select = selector
                    immutable: Immutable = select(row[indices_table[heading]])
                    immutables.append(immutable)
                spacer: Spacer = tuple(immutables)
                spacers_list.append(spacer)
    return tuple(spacers_list)


def stacks_search(desired_height: float, search_spacers: Tuple[Spacers, ...]) -> List[Stack]:
    """Return spacer stacks for a desired height."""
    stacks_table: Dict[StackKey, Stack] = {}
    spacers: Spacers
    for spacers in itertools.product(*search_spacers):
        spacers_height: float = 0.0
        prices_total: float = 0.0
        parts: List[str] = []
        for spacer in spacers:
            # 0 is the spacer height:
            height: Immutable = spacer[0]
            assert isinstance(height, float), spacer
            spacers_height += float(height)
            # -1 is the spacer price:
            price: Immutable = spacer[-1]
            assert isinstance(price, float), spacer
            prices_total += float(price)
            # -2 is the Digi-Key part number:
            part: Immutable = spacer[-2]
            assert isinstance(part, str), spacer
            parts += str(part)
        height_offset: float = desired_height - spacers_height
        stack: Stack = (abs(height_offset), height_offset, prices_total, spacers)
        stack_key: StackKey = tuple(sorted(parts))
        stacks_table[stack_key] = stack
    return list(stacks_table.values())


def heights_select(heights_text: str) -> Tuple[float, float, str]:
    """Extract the heights from an inch/mm/fractional string."""
    mm_height: float = 1000000.0
    inch_height: float = mm_height / 25.4
    fractional_height: str = ""
    if len(heights_text) >= 2:
        heights: Tuple[str, ...] = tuple(heights_text.split(' '))
        assert len(heights) <= 3, heights_text
        inch_text: str = heights[0].strip('"').strip('"')
        inch_height = float(inch_text)
        mm_height = float(heights[1].strip(')mm').strip('('))
        fractional_height = "" if len(heights) < 3 else heights[2]
    return (mm_height, inch_height, fractional_height)


def mm_height_select(heights_text: str) -> float:
    """Select the millimeter heights."""
    return heights_select(heights_text)[0]


def string_select(text: str) -> str:
    """Select the entire string."""
    return text


def float_select(text: str) -> float:
    """Select a floating point number."""
    return float(text)


if __name__ == "__main__":
    main()
