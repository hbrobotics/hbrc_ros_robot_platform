"""Program to compute spacer stacks."""

# <--------------------------------------- 100characters ----------------------------------------> #


import csv
import itertools
from typing import Callable, Dict, IO, List, Tuple, Union
from dataclasses import dataclass

Immutable = Union[float, str, int]
Spacer = Tuple[Immutable, ...]
Spacers = Tuple[Spacer, ...]


@dataclass(order=True)
class Stack:
    """A class that represents a stack of washers, standoffs, and spacers."""

    # Order matters.  Minimize error first, error_offset, unit_price.
    error: float  # The absolute value of the actual height vs the desired height
    unit_price: float  # The sum of the spacer unit prices
    error_offset: float  # Signed error (negative means below desired and positive means above.)
    desired_height: float  # Desired stack height
    spacers: Spacers  # The spacers that make up th stack


Stacks = Tuple[Stack, ...]


@dataclass
class StackRequest:
    """Record of information about a Stack request."""

    name: str  # The name of the stack.
    desired_height: float  # The desired stack height in millimeters.
    count: int  # Number required per robot.
    selected_index: int  # The index of the selected stack.
    selected_stack: Stack  # The selected stack (initialized to empty and filled in later).


StackRequests = Tuple[StackRequest, ...]

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

    empty_stack: Stack = Stack(-1.0, -1.0, -1.0, -1.0, ())
    stack_requests: StackRequests = (
        StackRequest("base_pi_height", 15.80, 2, 0, empty_stack),
        StackRequest("base_master_height", 26.05, 2, 0, empty_stack),
        StackRequest("battery_pi_height", 5.80, 2, 0, empty_stack),
        StackRequest("battery_master_height", 45.55, 2, 0, empty_stack),
        StackRequest("master_nucleo_height", 13.00, 4, 1, empty_stack),  # (from class Nucleo144)
        StackRequest("master_arm_height", 28.40, 4, 2, empty_stack),  # mm arm_spacer_dz
    )

    # Fill in the *stack_requests*:
    stack_index: int
    stack_request: StackRequest
    for stack_request in stack_requests:
        desired_height: float = stack_request.desired_height
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

        print(f"{stack_request.name}: {desired_height:.2f}mm")
        stacks: Stacks = tuple(sorted(stacks_list))
        match_index: int
        for stack_index, stack in enumerate(stacks[:3]):
            print(f"  {stack_request.name}[{stack.error:.3f}, "
                  f"{stack.unit_price:.2f}, {stack.error_offset:.2f}]:")
            for index, spacer in enumerate(stack.spacers):
                print(f"    {stack_request.name}[{stack_index}, {index}]: {spacer}")

    # Popluate *digikey_parts* table which maps each Digi-Key part to a list of *StackRequest*'s
    # that use the part:
    digikey_part: str
    digikey_spacers: Dict[str, Spacer] = {}
    digikey_parts: Dict[str, StackRequests] = {}
    for stack_index, stack_request in enumerate(stack_requests):
        print(f"Stack['{stack_request.name}']:")
        for spacer in stack_request.selected_stack.spacers:
            digikey_part = str(spacer[-2])
            assert isinstance(digikey_part, str)
            if digikey_part not in digikey_parts:
                digikey_parts[digikey_part] = ()
            digikey_parts[digikey_part] += (stack_request,)
            digikey_spacers[digikey_part] = spacer

    for digikey_part, stack_requests in digikey_parts.items():
        total_count = 0
        stack_references: List[Tuple[str, int]] = []
        for stack_request in stack_requests:
            count: int = stack_request.count
            total_count += count
            stack_references.append((stack_request.name, count))
        print(f"{digikey_part}: {total_count}")
        print(f"  {stack_references}")
        print(f"  {digikey_spacers[digikey_part]}")


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
        stack: Stack = Stack(abs(height_offset), prices_total, height_offset,
                             desired_height, spacers)
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
