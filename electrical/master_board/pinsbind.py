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

# This program is run using the corresponding `Makefile` which does the following:
#
#        mypy pins_bind.py                           # Do static type checking
#        flake8 --max-line-length=100 pins_bind.py   # Search for misc. errors (unsed variables,...)
#        pydocstyle pins_bind.py                     # Make sure the doc strings are consistent.
#        python pins_bind.py                         # Run the program

"""Figure out pin binding for Morpho/Zio connectors."""

# Use type hints for mypy program.
from typing import Dict, IO, List, Text, Tuple
import csv  # .csv file parser (CSV => Comma Separated Values)

# Define TextTuple as an abreviation.  In general, Tuples are needed to support Python sorting.
TextTuple = Tuple[Text, ...]  # Helper type to make the code easier to read. Immutable.
# A Binding is a ("SIGNAL_NAME", "ANNOTATED_PIN_NAME", "SCHMEATIC_NAME") tuple.  Immutable.
Binding = Tuple[Text, Text, Text]
Row = TextTuple  # A helper type for a row from the alternate functions table. Immutable
Signals = Tuple[TextTuple, ...]  # Signal lists are organized a tuple of text tuples. Immutable.

# A PinBind represnt a request to bind an internal signal to a schematics symbol with contraints
# using the following tuple format:
#    ("SIGNAL_NAME", "SCHEMATIC_NAME", SIGNALS, "FORCE_PIN_NAME")
# where,
#    "SIGNAL_NAME" is the internal signal name.
#    "SCHEMATIC_NAME" is the schemtatic name.
#    "SIGNALS" is one of the signals subset (e.g. Arduino, Zio, or Morpho)
#    "FORCE_PIN_NAME" force the signal to be bound to a particular pin.
PinBind = Tuple[Text, Text, Tuple[TextTuple, ...], Text]


def main() -> None:
    """Figure out pin binding for Morpho/ZIO connectors."""
    # Construct the *zios_table*:
    zios_table: Dict[Text, Text] = zios_table_create()

    # Read in *all_singals and subset into *arduino_signals*, *morpho_signals* and *zio_signals*:
    all_signals: Signals = extract_all_signals(zios_table)
    arduino_signals: Signals = signals_subset(all_signals, '@', "Arduino")
    morpho_signals: Signals = signals_subset(all_signals, '-', "Morpho")
    zio_signals: Signals = signals_subset(all_signals, '+', "Zio")
    arduino_signals = arduino_signals
    morpho_signals = morpho_signals
    zio_signals = zio_signals

    # See the *PinBind* definition above for an explanation of the entries below:
    pin_binds: List[PinBind] = [
        # The servos work best with 32-bit counters (i.e. TIM2/5).  It turns out that
        # there are exactly 4 Zio availble pins to TIM2/5, so that are used for the servos.
        ("TIM5_CH1", "SERVO1", zio_signals, ""),
        ("TIM2_CH2", "SERVO2", zio_signals, ""),
        ("TIM2_CH3", "SERVO3", zio_signals, ""),
        ("TIM2_CH4", "SERVO4", zio_signals, ""),

        # The encoders are a real challenge.  TIM8 has CH1/2 available on the Zio so the
        # get assigned to the RENCODER.  The none of the other encoder enabled timers
        # (TIM1/2/3/4/5/6/8) have both CH1/2 available.  This is fixed by shorting a Morpho
        # pin to LPTIM1_IN2.  This allows TIM1 to be used for LENCODER.
        ("LPTIM1_IN1", "LENCODER_A", zio_signals, ""),
        ("LPTIM1_IN2", "LENCODER_B", morpho_signals, ""),
        ("TIM8_CH1", "RENCODER_A", zio_signals, ""),
        ("TIM8_CH2", "RENCODER_B", zio_signals, ""),

        # It is a requirement that each motor have both its +/- outputs on the same timer.
        # There is a real dirth of timers with two measly channels on the same counter.
        # TIM9 meets the requirement and gets RMOTOR.  This is fixed by shorting a Morpho
        # pin to TIM_CH3.
        ("TIM1_CH3", "LMOTOR+", morpho_signals, ""),
        ("TIM1_CH4", "LMOTOR-", zio_signals, ""),
        ("TIM9_CH1", "RMOTOR+", zio_signals, ""),
        ("TIM9_CH2", "RMOTOR-", zio_signals, ""),

        # The LED's just need an internal timer to trigger the DMA peripheral.
        # TIM6 is used by HAL, so TIM7 will have to do.

        # The Lidar just needs one dedicated timer.
        # There are plenty to choose from, so TIM4_CH2 is used.
        ("TIM4_CH2", "LIDAR_PWM", zio_signals, ""),

        # UARTS's come next.
        ("UART5_TX", "D0_TX", arduino_signals, ""),
        ("UART5_RX", "D1_RX", arduino_signals, ""),
        ("USART3_RX", "STLINK_RX", morpho_signals, ""),  # Nucleo-144 manual says it Must be USART3
        ("USART3_TX", "STLINK_TX", morpho_signals, ""),
        ("USART1_RX", "SBC_RX", morpho_signals, ""),
        ("USART1_TX", "SBC_TX", morpho_signals, ""),
        ("USART1_RX", "SBC_RX", morpho_signals, ""),
        ("USART1_TX", "SBC_TX", morpho_signals, ""),
    ]
    signal_bindings_table, pin_bindings_table = pins_bind(pin_binds)

    Quad = Tuple[Text, int, Text, Text]

    def binding_extract(binding: Binding) -> Quad:
        signal_name: Text
        pin_name_with_af: Text
        schematic_name: Text
        signal_name, pin_name_with_af, schematic_name = binding
        pin_name: Text
        af: Text
        pin_name, af = tuple(pin_name_with_af.split(':'))
        port_name: Text = pin_name[:2]
        digits: Text = pin_name[2:]
        assert digits.isdigit(), f"'pnwa:'{pin_name_with_af}' pn:'{pin_name}' digits='{digits}'"
        pin_number: int = int(digits)
        return (port_name, pin_number, signal_name, schematic_name)

    print("Pin Bindings:")
    pin_bindings: List[Quad] = [binding_extract(binding)
                                for binding in pin_bindings_table.values()]
    pin_bindings = sorted(pin_bindings)
    quad: Quad
    for quad in pin_bindings:
        print(f"{quad}")
    print("")

    print("Signal Bindings:")
    signal_binding: Binding
    signal_bindings: List[Binding] = [signal_binding
                                      for signal_binding in signal_bindings_table.values()]
    signal_bindings = sorted(signal_bindings)
    for signal_binding in signal_bindings:
        print(f"{signal_binding}")
    print("")

    # signals_print(zio_signals, "Zio")
    # signals_print(morpho_signals, "Morpho")


# The functions after main() are listed alphabetically to make them easier to find.

def alternate_functions_csv_read() -> Tuple[Row, Tuple[Row, ...]]:
    """Read the alternate functions table."""
    # Read in *rows* from "stm32f7676_af.csv" file:
    rows: List[Tuple[Text, ...]] = []
    csv_file: IO[Text]
    with open("stm32f767_af.csv", "r") as csv_file:
        af_reader = csv.reader(csv_file, delimiter=',', quotechar='"')
        for row_list in af_reader:
            rows.append(tuple(row_list))

    # The first row is the column *headings* and the remaining rows are the *data_rows*:
    headings: Row = rows[0]
    data_rows: Tuple[Row, ...] = tuple(rows[1:])
    return headings, data_rows


def extract_all_signals(zios_table: Dict[Text, Text]) -> Tuple[TextTuple, ...]:
    """Return the extracted signals as a sorted tuple.

    The signals table maps an internal signals table to tuple of the form:

        ("SIGNAL_NAME", "ANNOTATED_PIN1", ..., "ANNOTATED_PINn")

    where,
        "SIGNAL_NAME" is the internal signal name used by the processor
            spec. sheet.
        "ANNOTATEDPINi" is an annotated PIN of the form:
            "{FLAG}{PIN_NAME}:{ALTERNATE_FUNCTION_NAME}"
        where,
            {FLAG} is:
                '@' pin is an Arduino pin (by definition a Zio pin)
                '+' pin is a Zio Pin, but not an Arduino pin
                '-' pin is a Morpho pin, but not a Zio pin
            {PIN_NAME} is "P{PORT_LETTER}{PIN_NUMBER}:
                PORT_LETTER is a letter between 'A'and 'G'.
                PIN_NUMBER is a letter between 0 and 15.
    """
    # Read in the alternate functions table:
    headings: Row
    data_rows: Tuple[Row, ...]
    headings, data_rows = alternate_functions_csv_read()

    # Find *pin_number_index*, *pin_name_index* and *af_indices* from the *headings*:
    heading: Text
    pin_number_index: int = -1
    pin_name_index: int = -1
    af_indices: Dict[int, int] = {}
    index: int
    for index, heading in enumerate(headings):
        # print(f"Heading[{index}]:'{heading}'")
        if heading == "Position":
            pin_number_index = index
        elif heading == "Name":
            pin_name_index = index
        elif heading.startswith("AF") and heading[2:].isdigit():
            af_number: int = int(heading[2:])
            # print(f"af_number:{af_number}")
            af_indices[af_number] = index
    assert pin_name_index >= 0
    assert pin_number_index >= 0
    assert len(af_indices) == 15

    # Now scan through the *data_rows* building up useful tables.  The *pins_table* builds
    # up informat about signals that can be connected to a given pin name.
    # The *signals_table* builds up a list of pin names that can be connected to each signal.
    pins_table: Dict[Text, TextTuple] = {}
    signals_table: Dict[Text, List[Text]] = {}

    # Some shared variables used both inside and outside of the loops:
    signal_pins: List[Text]
    data_row: Row
    signal_name: Text
    af0_index: int = af_indices[0]
    # print(f"af0_index={af0_index}")
    data_row_index: int
    for data_row_index, data_row in enumerate(data_rows):
        # Grab the *pin_number*
        pin_number: int
        try:
            pin_number = int(data_row[pin_number_index])
        except ValueError:
            assert False, "Bad pin number"
        pin_number = pin_number  # *pin_number is not currently used.

        # Grab the *pin_name* and only look at pins of the form "P[A-G]#", where '#" is a number.
        pin_name: Text = data_row[pin_name_index]
        pin_name_fields: List[Text] = pin_name.split('/')
        if len(pin_name_fields) >= 1:
            pin_name = pin_name_fields[0]
        if (
                len(pin_name) >= 3 and pin_name[0] == 'P' and
                pin_name[1] in "ABCDEFGH" and pin_name[2:].isdigit()):
            # print(f"Pin[{pin_number}]:'{pin_name}'")

            # The altenate data function bundings are at the end starting at *af0_index*.
            # Their should be 15 of them.
            afs: TextTuple = data_row[af0_index:]
            assert len(afs) == 15
            pins_table[pin_name] = afs

            # Now fill in the *signals_table*:
            af_index: int
            for af_index, signal_name in enumerate(afs):
                # For now ignore signals that are not needed by HR2 right now:
                if signal_name_ignore(signal_name):
                    continue

                # Append xxx to *signal_pins*:
                if signal_name:
                    if signal_name not in signals_table:
                        signals_table[signal_name] = [signal_name]
                    signal_pins = signals_table[signal_name]

                    # For now only keep track of pins that are "free" for use on the ZIO connectors:
                    zios_flag: Text = zios_table[pin_name]
                    if zios_flag in "+@-":
                        annotated_pin: Text = f"{zios_flag}{pin_name}:AF{af_index}"
                        signal_pins.append(annotated_pin)
                    # print(f"  '{signal_name}':{signal_pins}")

    # Sort the final *signals* and return them.
    signals: List[TextTuple] = []
    for signal_pins in signals_table.values():
        signals.append(tuple(signal_pins))
    return tuple(sorted(signals))


def pins_bind(pin_binds: List[PinBind]) -> Tuple[Dict[Text, Binding], Dict[Text, Binding]]:
    """Bind needed signals to specific pins.

    Args:
        pin_binds: An ordered list of PinBind's.
    Returns:
        signal_bindings_table (Dict[Text, Binding]):
            A table of Binding's keyed by the signal name.
        pin_bindings_table (Dict[Text, Binding]):
            A table of Binding's keyed by the pin name.

    After each pin is bound, it is inserted into both
    signal_bindings_table and pin_bindings_table.
    An error is generated no binding occurs.
    Both signal_bindings_table and pin_bindings_table are returned.
    """
    # Initialize empty *pin_bindings_table* and *signal_bindings_table*:
    pin_bindings_table: Dict[Text, Binding] = {}
    signal_bindings_table: Dict[Text, Binding] = {}

    # This step is pretty obscure.  We need to partion the *pin_binds* by it *signal*
    # (type *Signals*) component, which is one of *arduino_signals*, *zio_signals*,
    # or *morpho_signals*.  These three values are large but immutable structures,
    # so they can be used a dictionary key.  So, we can build *signals_table* which
    # is keyed by off of these three values to build lists of *PinBind*'s that are
    # isolated to a single *Signals* instance.  The *PinBind*'s order must be preserved.
    pin_bind: PinBind
    signal_name: Text
    schematic_name: Text
    signals: Signals
    force_pin_name: Text
    signals_table: Dict[Signals, List[PinBind]] = {}
    for pin_bind in pin_binds:
        # Unpack *pin_bind*:
        signal_name, schematic_name, signals, force_pin_name = pin_bind
        if signals not in signals_table:
            signals_table[signals] = []
        signals_table[signals].append(pin_bind)

    # Now we can sweep through the *signals_table* and find try to allocate the associated
    # *PinBind*'s.  The order that the *signals* comes out of the iterator is semi-random,
    # but it does not matter since no pins are shared between them.
    index: int = 0
    pin_binds_list: List[PinBind]
    for signals, pin_binds_list in signals_table.items():
        index += 1
        # signals_print(signals, f"######################### Signals table {index} being processed")
        # Now sweep through *signals* which has been presorted so that the hardest to bind
        # signals (i.e. the ones with the fewest available pins come first):
        duplicate_signals: Signals
        for signal in signals:
            # print(f"Signal {signal} looking for any takers")
            desired_signal_name: Text = signal[0]
            annotated_pin_names: TextTuple = signal[1:]
            for signal_name, schematic_name, duplicate_signals, force_pin_name in pin_binds_list:
                # print(f"    Attempting to bind '{signal_name}'")
                assert signals == duplicate_signals, "Something is very confused!"
                match: bool = False
                if signal_name == desired_signal_name:
                    # We have a signal match; attempt to do a binding:
                    # print(f"    Found '{signal_name}' signal={signal}")
                    annotated_pin_name: Text
                    pin_name: Text
                    for annotated_pin_name in annotated_pin_names:
                        pin_name = pin_name_deannotate(annotated_pin_name)
                        if (
                                pin_name not in pin_bindings_table and
                                signal_name not in signal_bindings_table):
                            # We have a match; perform the binding:
                            binding = (signal_name, annotated_pin_name[1:], schematic_name)
                            signal_bindings_table[signal_name] = binding
                            pin_bindings_table[pin_name] = binding
                            # print(f"    >>> Bind: '{pin_name}' signal={signal} binding={binding}")
                            match = True
                            break
                    else:
                        # print(f"    No bind:: signal={signal}")
                        for annotated_pin_name in annotated_pin_names:
                            pin_name = pin_name_deannotate(annotated_pin_name)
                            assert pin_name in pin_bindings_table, f"Pin {pin_name} not bound????"
                            previous_binding: Binding = pin_bindings_table[pin_name]
                            print(f"    {pin_name} is already bound to {previous_binding}")
                            # signals_print(signals, "Bind: All Pins Previously Bound")
                        assert False, f"Unable to bind {signal_name} to {signal}"
                    if match:
                        break
            else:
                # print(f"Signal {signal} no takers found")
                pass

    # Now look for pins that did not get bound:
    unbound_pins: int = 0
    for pin_bind in pin_binds:
        signal_name, schematic_name, signals, force_pin_name = pin_bind
        if signal_name not in signal_bindings_table:
            print(f"{signal_name} did not get assigned to a pin")
            unbound_pins += 1
    assert unbound_pins == 0, f"{unbound_pins} pin(s) did not get bound."

    return signal_bindings_table, pin_bindings_table


def pin_name_deannotate(annotated_pin_name: Text) -> Text:
    """Return only the pin name from an annotated pin name."""
    pair: List[Text] = annotated_pin_name.split(':')
    return pair[0][1:]


def signal_name_ignore(signal_name: Text) -> bool:
    """Return True if signal name is not useful."""
    return (
        signal_name.startswith("SYS") or
        signal_name.startswith("USB") or
        signal_name.startswith("SDMMC") or
        signal_name.startswith("SAI") or
        signal_name.startswith("I2S") or
        signal_name.startswith("FMC") or
        signal_name.startswith("ETH") or
        signal_name.startswith("DFSDM") or
        signal_name.startswith("DCMI") or
        signal_name.startswith("CAN") or
        signal_name.startswith("LTDC") or
        # signal_name.startswith("LPTIM") or
        signal_name.startswith("SPDIFRX") or
        signal_name.startswith("RCC") or
        signal_name.startswith("CEC") or
        signal_name.startswith("MDIOS") or
        signal_name.startswith("QUADSPI") or
        signal_name.startswith("RTC") or
        signal_name.endswith("_CTS") or
        signal_name.endswith("_RTS") or
        signal_name.endswith("_CK") or signal_name.endswith("_ETR")
    )


def signals_print(signals: Tuple[TextTuple, ...], title: Text) -> None:
    """Print out the signals with a title."""
    print(f"{title}:")
    signal_pins_tuple: TextTuple
    for signal_pins_tuple in signals:
        print(f"{signal_pins_tuple}")
    print("")


def signals_subset(
        signals: Tuple[TextTuple, ...], pattern: Text, title: Text) -> Tuple[TextTuple, ...]:
    """Return a subset of signals based on a pattern."""
    culled_signals: List[TextTuple] = []
    signal_pins: TextTuple
    for signal_pins in signals:
        signal_name: Text = signal_pins[0]
        annotated_pin_names: TextTuple = signal_pins[1:]
        culled_signal_pins: List[Text] = [signal_name]
        annotated_pin_name: Text
        for annotated_pin_name in annotated_pin_names:
            flag: Text = annotated_pin_name[0]
            assert flag in "@+-", f"Flag '{flag}' must be one of '@', '+', or '-'"
            if flag in pattern:
                culled_signal_pins.append(annotated_pin_name)

        # Only keep result that have at least one annotated pin name:
        if len(culled_signal_pins) > 1:
            culled_signals.append(tuple(culled_signal_pins))

    # Uncomment for debugging:
    # signals_print(tuple(culled_signals), title)

    # There is some serious trickiness here needed for the pin_bind() function.
    # It is extremely important that we prioritize binding to minimize chances of not binding.
    # This is done by sorting the attempted pin binding order so that the shortest tuples
    # occur first.  The shortest tuples should be bound before the longer tuples since the
    # shorter tuples have fewer chances of success.
    final_culled_signals: Tuple[TextTuple, ...] = (
        tuple(sorted(culled_signals, key=lambda signal_pins: len(signal_pins))))

    # Uncomment for debugging:
    # signals_print(final_culled_signals, title)
    return final_culled_signals


def zios_table_create() -> Dict[Text, Text]:
    """Return a port pin name to pin class table.

    The pin names are labled P{LETTER}{NUMBER} where
    LETTER is in [A-G] and NUMBER is 0=15.
    The returned value is:
    * '@' it is a Zio pin allocated for arduino shield pin.
    * '+' it is a Zio pin that is not allocated for an arduino shield
    * '-' it is a Morpho pin that is not connected to a Zio pin.
    """
    # Create *zio_lines* which has a 16-bit pin mapping for each port.
    zio_lines: TextTuple = (
        "+--@+@@@-------+",  # PA
        "+++++++-@@++@@-+",  # PB
        "@-+@--+++++++---",  # PC
        "++++++++---+++@@",  # PD
        "+-+*+++++@+@+@++",  # PE
        "+++@+@-+++@-@@@@",  # PF
        "++++-----@----@-",  # PG
    )

    # Create and fill in the *zios_table*:
    zios_table: Dict[Text, Text] = {}
    zio_line: Text
    port_index: int
    for port_index, zio_line in enumerate(zio_lines):
        port_name: Text = "P" + chr(ord('A') + port_index)
        # print(f"Zio: port_name='{port_name}'")
        bit_index: int
        for bit_index, flag in enumerate(zio_line):
            pin_name: Text = f"{port_name}{bit_index}"
            zios_table[pin_name] = flag
    return zios_table


if __name__ == "__main__":
    main()
