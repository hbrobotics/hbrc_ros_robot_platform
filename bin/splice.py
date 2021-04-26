#!/usr/bin/env python3

#<---------------------------------------- 100 characters ---------------------------------------->#

import sys
from dataclasses import dataclass

from typing import Dict, List, Set, TextIO, Tuple


@dataclass
class Patch:
    """A patch to some code."""
    name: str
    befores: List[str]
    afters: List[str]
    replaces: List[Tuple[str, str]]


def main() -> None:
    # Process command line arguments:
    arguments: Tuple[str, ...] = tuple(sys.argv)
    assert len(arguments) == 2, "Usage: splice.py FILE_NAME"
    file_name: str = arguments[1]

    # Peform a splice:
    includes_patch: Patch = Patch(
        name="Includes",
        befores=["// Before Includes"],
        afters=["// After Includes"],
        replaces=[]
    )
    init_patch: Patch = Patch(
        name="Init",
        befores=["// Before Init"],
        afters=["// After Init"],
        replaces=[]
    )
    patches: List[Patch] = [includes_patch, init_patch]
    
    splice(file_name, file_name)
    # Testing code:
    # splice(file_name, "/tmp/a.c", [])  # Converts from CLRF to LF end-of-lines.
    # splice("/tmp/a.c", "/tmp/b.c", patches)
    # splice("/tmp/b.c", "/tmp/c.c", patches)
    # splice("/tmp/c.c", "/tmp/d.c", [])
    # splice("/tmp/d.c", "/tmp/e.c", [])


def splice(input_file_name: str, output_file_name: str, patches: List[Patch]) -> None:
    """Scan a file for code fences and insert/delete additional code.

    Args:
      input_file_name: The file name to read from.
      output_file_name: The file name to write the output to.
        (If empty, the input_file_name is used.)
      patches: A Tuple of named Patch objects.

    Generally, a patch is used to modify a code fence in a file.  A code fence looks like:

         /* USER CODE BEGIN name */
         ...
         /* USER CODE END name */

    Where `name` is the unique code fence name and `...` is some user supplied code.
    The MxCube program is perfectly willing to take an updated `.ioc` file and generate
    all of the initialization and and glue code.  The MXCube program leave the code inside
    the code fences alone.

    This program bascially is used to insert additional code into one of the code fences:
    The updated code fence looks as follows:

         /* USER CODE BEGIN name */
         /* USER CODE BEGIN:BEGIN name */
         ... // Before patch code
         /* USER CODE BEGIN:END name */
         ... // Pre-existing code in code fence.
         /* USER CODE END:BEGIN name */
         ... // After patch code
         /* USER CODE END:END name */
         /* USER CODE BEGIN name */
        
    A patch looks as follows:

    patch: = Patch(
      name="name",
      befores=("Before line 1", ..., "Before line N"),
      afters=("After Line 1", ..., "After line N"),
      replaces=(
        ("exact_pattern1", "exact_replace1"),
        ...,
        ("exact_patternN", "exact_replaceN"),
      ))

    For now, only the `name`, `befores`, and `after` fields are discuseed and the `replaces`
    field is discussed a bit further below.  After applying the patch above (ignoring
    the `replaces` field): the resulting code looks like:

         /* USER CODE BEGIN name */
         /* USER CODE BEGIN:BEGIN name */
         Before line 1
         ...
         Before line N
         /* USER CODE BEGIN:END name */
         ... // Pre-existing code in code fence.
         /* USER CODE END:BEGIN name */
         After line 1
         ...
         After Line N
         /* USER CODE END:END name */
         /* USER CODE BEGIN name */
    
    The `replaces` field is specifies a 2-tuple of strings which has an old to new pattern.
    These replace patterns are performed on the pre-existing user code.

    """
    if output_file_name == "":
        output_file_name = input_file_name

    pattern_flags: Tuple[Tuple[str, str], ...] = (
        ("/* USER CODE BEGIN ", "B"),
        ("/* USER CODE BEGIN:BEGIN ", "D"),
        ("/* USER CODE BEGIN:END ", "K"),
        ("/* USER CODE END:BEGIN ", "D"),
        ("/* USER CODE END:END ", "K"),
        ("/* USER CODE END ", "E"),
    )

    patch: Patch
    patches_table: Dict[str, Patch] = {patch.name: patch for patch in patches}

    # Read in *lines* from the file named *file_name*:
    original_lines: List[str] = []
    file: TextIO
    with open(input_file_name) as file:
        original_lines = file.read().split("\n")

    # Scan through *original_lines* looking for user code fences:
    replaces_table: Dict[str, str] = {}
    dones: Set[str] = set()
    updated_lines: List[str] = []
    keep_mode: bool = True
    line: str = ""
    for line in original_lines:
        # Search for a matching *pattern* and associated *flags*:
        indent: str = ""
        name: str = ""
        for pattern, flags in pattern_flags:
            offset: int = line.find(pattern)
            if offset >= 0:
                indent = line[:offset]
                name = line[offset + len(pattern):-3]
                break

        # Deal with a pattern match:
        if name != "":
            # Dispatch based on flags:
            if "B" in flags:
                # Insert a begin chunk of code:
                updated_lines.append(line)
                if name in patches_table:
                    patch = patches_table[name]
                    replaces: List[Tuple[str, str]] = patch.replaces
                    replace: Tuple[str, str]
                    replaces_table = {replace[0]: replace[1] for replace in replaces}

                    befores: List[str] = patch.befores
                    updated_lines.append(f"{indent}/* USER CODE BEGIN:BEGIN {name} */")
                    updated_lines.extend([indent + insert_line for insert_line in befores])
                    updated_lines.append(f"{indent}/* USER CODE BEGIN:END {name} */")
                    keep_mode = True
            elif "E" in flags:
                # Insert an end chunk of code:
                if name in patches_table:
                    afters: List[str] = patch.afters
                    updated_lines.append(f"{indent}/* USER CODE END:BEGIN {name} */")
                    updated_lines.extend([indent + insert_line for insert_line in afters])
                    updated_lines.append(f"{indent}/* USER CODE END:END {name} */")
                    assert name not in dones, f"Duplicate code fence {name}"
                    dones.add(name)
                    replaces_table = {}  # Diable further replacements.
                    keep_mode = True
                updated_lines.append(line)
            elif "K" in flags:
                # Skip this line but start keeping the following lines:
                keep_mode = True
            elif "D" in flags:
                # Skip this line and keep deleting lines until told otherwise:
                keep_mode = False
        elif keep_mode:
            # Keep the lines until told otherwise:
            # Peform any replacements first:
            old: str
            new: str
            for old, new in replaces_table.items():
                line = line.replace(old, new)
            updated_lines.append(line)
        # else we are in in skip mode and the line is not kept:

    # Check for mistyped fence names:
    requests = set(patches_table.keys())
    assert dones == requests, (
      f"dones={dones}, requests={requests}")

    # Write out the updated contents to *file_name*.
    if len(updated_lines) >= 0 and updated_lines[-1] != "":
        updated_lines.append("")
    with open(output_file_name, "w") as file:
        file.write("\n".join(updated_lines))


if __name__ == "__main__":
    main()

