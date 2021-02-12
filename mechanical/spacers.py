"""Program to compute spacer stacks."""

# <--------------------------------------- 100characters ----------------------------------------> #


import csv
import itertools
from dataclasses import dataclass
from typing import Callable, Dict, IO, List, Tuple, Union

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

    robots_count: int = 5
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
    spacer_standoff_headings = (
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
    spacer_standoff_selectors: Selectors = (
        ("Between Board Height", mm_height_select),  # [0]
        ("Type", string_select),  # [1]
        ("Threaded/Unthreaded", string_select),  # [2]
        ("Gender", string_select),  # [3]
        ("DK Part #", string_select),  # [4] [-2]
        ("Price", float_select),  # [5] [-1]
    )
    spacers: Spacers = csv_read(
        "spacers_#2.csv", spacer_standoff_headings, spacer_standoff_selectors)
    standoffs: Spacers = csv_read(
        "standoffs_#2.csv", spacer_standoff_headings, spacer_standoff_selectors)

    drawers_compute(spacers, standoffs)

    # Cull duplicate spacers with higher prices:
    washers = spacers_cull(washers, "Washers")
    spacers = spacers_cull(spacers, "Spacers")
    standoffs = spacers_cull(standoffs, "Standoffs")

    empty_stack: Stack = Stack(-1.0, -1.0, -1.0, -1.0, ())
    stack_requests: StackRequests = (
        StackRequest("base_pi_height", 15.80, 2, 0, empty_stack),
        StackRequest("base_master_height", 26.05, 2, 0, empty_stack),
        StackRequest("battery_pi_height", 5.80, 2, 0, empty_stack),
        StackRequest("battery_master_height", 45.55, 2, 0, empty_stack),
        StackRequest("master_nucleo_height", 13.00, 4, 0, empty_stack),  # (from class Nucleo144)
        StackRequest("master_arm_height", 28.40, 4, 0, empty_stack),  # mm arm_spacer_dz
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
            stacks_search(desired_height, (washers, spacers)) + \
            stacks_search(desired_height, (washers, standoffs, spacers)) + \
            stacks_search(desired_height, (washers, standoffs, standoffs, spacers))

        print("")
        print(f"{stack_request.name}: {desired_height:.2f}mm")
        stacks: Stacks = stacks_cull(tuple(stacks_list))
        selected_index: int = stack_request.selected_index
        for stack_index, stack in enumerate(stacks[:8]):
            flag: str = "*" if stack_index == selected_index else " "
            print(f"{flag} {stack_request.name}[{stack.error:.2f}, "
                  f"{stack.unit_price:.2f}, {stack.error_offset:.2f}]:")
            for index, spacer in enumerate(stack.spacers):
                print(f"    {stack_request.name}[{stack_index}, {index}]: {spacer}")
        print("")

        # Select the requested stack:
        if selected_index >= 0:
            stack_request.selected_stack = stacks[selected_index]

    # Popluate *digikey_parts* table which maps each Digi-Key part to a list of *StackRequest*'s
    # that use the part:
    references_count: int = 0
    digikey_part: str
    digikey_spacers: Dict[str, Spacer] = {}
    digikey_parts: Dict[str, StackRequests] = {}
    for stack_index, stack_request in enumerate(stack_requests):
        for spacer in stack_request.selected_stack.spacers:
            references_count += 1
            digikey_part = str(spacer[-2])
            assert isinstance(digikey_part, str)
            if digikey_part not in digikey_parts:
                digikey_parts[digikey_part] = ()
            digikey_parts[digikey_part] += (stack_request,)
            digikey_spacers[digikey_part] = spacer
    print(f"References Count:{references_count}")

    total_unit_cost: float = 0.0
    for digikey_part, stack_requests in digikey_parts.items():
        total_count = 0
        stack_references: List[Tuple[str, int]] = []
        for stack_request in stack_requests:
            count: int = stack_request.count
            total_count += count
            stack_references.append((stack_request.name, count))
        spacer = digikey_spacers[digikey_part]
        spacer_unit_price: Immutable = spacer[-1]
        assert isinstance(spacer_unit_price, float)
        total_unit_cost += total_count * spacer_unit_price
        print(f"{digikey_part}: {total_count * robots_count}")
        print(f"  {stack_references}")
        print(f"  {spacer}")
    print(f"Total_unit_cost: 1 Robot: ${total_unit_cost:.2f}")
    print(f"Total_unit_cost: {robots_count} Robots: ${total_unit_cost*robots_count:.2f}")


def spacers_cull(spacers: Spacers, label: str) -> Spacers:
    """Only use the least expensive spacer of the same height."""
    spacers_table: Dict[float, Spacer] = {}
    spacer: Spacer
    culled: int = 0
    for spacer in spacers:
        current_height: Immutable = float(spacer[0])  # Height
        assert isinstance(current_height, float)
        current_price: Immutable = spacer[-1]  # Price
        assert isinstance(current_price, float)
        if current_height in spacers_table:
            previous_spacer: Spacer = spacers_table[current_height]
            previous_price: Immutable = previous_spacer[-1]
            assert isinstance(previous_price, float)
            if current_price < previous_price:
                spacers_table[current_price] = spacer
            else:
                culled += 1
        else:
            spacers_table[current_price] = spacer
    culled_spacers: Spacers = tuple(spacers_table.values())
    print(f"{label}: {len(spacers)} culled down to {len(culled_spacers)}")
    return culled_spacers


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
                height: Immutable = immutables[0]
                assert isinstance(height, float)
                if height > 0:
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
        assert stack.error == abs(height_offset)
        assert stack.unit_price == prices_total
        assert stack.error_offset == height_offset
        assert stack.desired_height == desired_height
        assert stack.spacers == spacers

        stack_key: StackKey = tuple(sorted(parts))
        stacks_table[stack_key] = stack
    return list(stacks_table.values())


def stacks_cull(stacks: Stacks) -> Stacks:
    """Return stacks that have duplicate heights culled."""
    stacks_table: Dict[str, Stack] = {}
    stack: Stack
    actual_height: float
    for stack in stacks:
        error_offset: str = f"{stack.error_offset:.2f}"
        if error_offset in stacks_table:
            unit_price: float = stack.unit_price
            previous_stack: Stack = stacks_table[error_offset]
            previous_unit_price: float = previous_stack.unit_price
            if unit_price < previous_unit_price:
                stacks_table[error_offset] = stack
        else:
            stacks_table[error_offset] = stack
    culled_stacks_list: List[Stack] = sorted(stacks_table.values())

    # *culled_stacks* is sorted by error.
    # We are only interested in errors less that .3mm.
    culled_stacks_list = [stack for stack in culled_stacks_list if stack.error <= 0.3]
    culled_stacks_list = sorted(culled_stacks_list, key=lambda stack: stack.unit_price)
    culled_stacks: Stacks = tuple(culled_stacks_list)
    return culled_stacks


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


def fractional_height_select(heights_text: str) -> float:
    """Select the fractional heights."""
    fractional_height: str
    _, _, fractional_height = heights_select(heights_text)
    height: float = 0.0
    if fractional_height:
        if '/' in fractional_height:
            fractional_splits: List[str] = fractional_height.split('/')
            if len(fractional_splits) == 2:
                numerator: float = float(fractional_splits[0])
                denominator: float = float(fractional_splits[1])
                height = numerator / denominator
    return height


def mm_height_select(heights_text: str) -> float:
    """Select the millimeter heights."""
    return heights_select(heights_text)[0]


def string_select(text: str) -> str:
    """Select the entire string."""
    return text


def float_select(text: str) -> float:
    """Select a floating point number."""
    return float(text)


def closest_fractional(mm_height: float) -> Tuple[int, str, float, str]:
    """Find the closest fractional inch."""
    inch_height = mm_height / 25.4
    adjust: float = 1.0 / 128.
    sixty_forths_numerator: int = int((inch_height + adjust) * 64.0)
    numerator: int = sixty_forths_numerator
    denominator: int = 64
    whole_inches: int = 0
    while numerator >= 64:
        numerator -= 64
        whole_inches += 1
    while numerator % 2 == 0 and denominator % 2 == 0:
        numerator >>= 1
        denominator >>= 1
    fractional: str = f"{numerator}/{denominator}"
    drawer_id: str = "hw2ss"  # ss = Spacers/Standoffs
    if whole_inches:
        if numerator:
            fractional = f"{whole_inches}-{fractional}"
        else:
            fractional = f"{whole_inches}"
        drawer_id += f"{whole_inches}"
    sixty_forths: int = whole_inches + sixty_forths_numerator
    error = 64 * whole_inches * 64 + float(numerator) / float(denominator) - inch_height
    if numerator == 0:
        denominator = 0
    drawer_id += f"{numerator:02d}{denominator:02d}"
    return (sixty_forths, fractional, error, drawer_id)


def drawers_compute(spacers: Spacers, standoffs: Spacers) -> None:
    """Extract spacer and standoff part numbers."""
    # All tables indexed by the spacer/standoff height in 1/64 inch:
    fractional_table: Dict[int, Tuple[str, str]] = {}
    drawers_table: Dict[int, Tuple[List[Spacer], List[Spacer]]] = {}  # List[Spacers, Standoffs]

    height: Immutable
    sixty_forths: int
    fractional: str
    price: Immutable
    part_number: Immutable
    drawer_id: str

    spacer: Spacer
    for spacer in spacers:
        height = spacer[0]
        assert isinstance(height, float)
        price = spacer[-1]
        assert isinstance(price, float)
        part_number = spacer[-2]
        assert isinstance(part_number, str)

        sixty_forths, fractional, error, drawer_id = closest_fractional(height)
        fractional_table[sixty_forths] = (fractional, drawer_id)
        if sixty_forths not in drawers_table:
            drawers_table[sixty_forths] = ([], [])
        drawers_table[sixty_forths][0].append((price, part_number))

    standoff: Spacer
    for standoff in standoffs:
        height = standoff[0]
        assert isinstance(height, float)
        price = standoff[-1]
        assert isinstance(price, float)
        part_number = standoff[-2]
        assert isinstance(part_number, str)

        sixty_forths, fractional, error, drawer_id = closest_fractional(height)
        fractional_table[sixty_forths] = (fractional, drawer_id)
        if sixty_forths not in drawers_table:
            drawers_table[sixty_forths] = ([], [])
        drawers_table[sixty_forths][1].append((price, part_number))

    lines: List[str] = []
    for sixty_forths in sorted(fractional_table.keys()):
        fractional_drawer_id: Tuple[str, str] = fractional_table[sixty_forths]
        fractional, drawer_id = fractional_drawer_id
        drawer: Tuple[List[Spacer], List[Spacer]] = drawers_table[sixty_forths]

        spacers_list: List[Spacer] = sorted(drawer[0])
        spacer_part_number: str = f"{spacers_list[0][-1]} (SP)" if spacers_list else ""

        standoffs_list: List[Spacer] = sorted(drawer[1])
        standoff_part_number: str = f"{standoffs_list[0][-1]} (SO)" if standoffs_list else ""

        lines.append(
            f'    o.drawer("{drawer_id}", ["#2-56 {fractional} Inch", "Spacer/Standoff"],')
        lines.append(
            f'             ["D:{spacer_part_number}", "D:{standoff_part_number}"])')
    lines.append("")
    code: str = "\n".join(lines)
    code_file: IO[str]
    with open("/tmp/drawer_code.py", "w") as code_file:
        code_file.write(code)


if __name__ == "__main__":
    main()
