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
#
# Note: that this file is formated to stay with 100 characters as enforced by flake8.

"""Figure out pin binding for Morpho/Zio connectors."""

# Use type hints for mypy program.
from typing import Dict, IO, List, Set, Text, Tuple
import csv  # .csv file parser (CSV => Comma Separated Values)
import itertools

# Some type definitions:

# An ANNOTATED_PIN_NAME has the form ".P[A-F][0-9]+:AF[0-9]+" (using regular expression format).
# The first character is '@' for an Arduino pin, '+' for a daughter pin (i.e. a Zio pin that
# that is not an Arduino pin), and '-' for a Morpho pin that is not connected to the Zio connectors.
# There is a single letter after the 'P' that is the a port name (e.g. 'PA' - 'PF').
# Lastly there is an alternate function designator which is 'AF' followed by by a number between
# 0 and 14.  For example "@PD14:AF11" is an arduino ('@') pin connected to pin name 'PD14',
# using alternate function "AF11".

# Define TextTuple as an abreviation.  In general, Tuples are needed to support Python sorting.
TextTuple = Tuple[Text, ...]  # Helper type to make the code easier to read.

# A Binding is a ("SIGNAL_NAME", "ANNOTATED_PIN_NAME", "SCHEMATIC_NAME") tuple.
Binding = Tuple[Text, Text, Text]
Row = TextTuple  # A helper type for a row read from a `.csv` file.

# A Signal is a ("SIGNAL_NAME", "ANNOTATED_PIN_NAME1", ..., "ANNOTATED_PIN_NAMEn")
# There is always a signal name and 0, 1 or more annotated pin names.
Signal = TextTuple
Signals = Tuple[TextTuple, ...]  # Signal lists are organized a tuple of text tuples.

# A PinBind represnt a request to bind an internal signal to a schematics symbol with contraints
# using the following tuple format:
#    ("SIGNAL_NAME", "SCHEMATIC_NAME", SIGNALS, "FORCE_PIN_NAME")
# where,
#    "SIGNAL_NAME" is the internal signal name.
#    "SCHEMATIC_NAME" is the schemtatic name.
#    "SIGNALS" is one of the signals subset (e.g. Arduino, Daughter, or Morpho)
#    "FORCE_PIN_NAME" When non-empty, force a signal binding to a specific pin.
#                     The same signal can be bound to more that one pin using this feature.
PinBind = Tuple[Text, Text, Signals, Text]


def main() -> None:
    """Figure out pin binding for Morpho/ZIO connectors."""
    # Extract the Arduino/Daughter/Morpho Signals/Sets.  See the README.md for what these
    # 3 sets are all about:
    arduino_signals: Signals
    daughter_signals: Signals
    morpho_signals: Signals
    arduino_set: Set[Text]
    daughter_set: Set[Text]
    morpho_set: Set[Text]
    pin_numbers_table: Dict[Text, int]
    arduino_signals, daughter_signals, morpho_signals, \
        arduino_set, daughter_set, morpho_set, pin_numbers_table = pin_signals_sets_get()

    # Create *periperal_permutations* list that lists all the possible permutations of
    # each peripheral class.  The number of I2C's, SPI's, and UART's are specified:
    i2cs_want: int = 1  # One manaully pre-bound to Arduino and one for everything else.
    spis_want: int = 2  # + SPI1 is already bound to Arduino
    uarts_want: int = 5  # + UART6 is already bound to Arduino
    peripheral_permutations: Tuple[TextTuple, ...] = peripheral_permutations_get(
        i2cs_want, spis_want, uarts_want)
    print(f"len(peripheral_permutations)={len(peripheral_permutations)}")

    # Iterate across *peripheral_permutations* searching for the solutions with no unbound signal
    # names.  The *permutation_scores* list shows the results of the search and can be sorted to
    # find solutions with the fewest (hopefully 0) unbound pins.  A *permutation_score* has the
    # format of (len(unbounded_signals), unbounded_signals, permutation):
    permutation_scores: List[Tuple[int, TextTuple, TextTuple]] = []
    peripheral_permutation: TextTuple
    for peripheral_permutation in peripheral_permutations:
        # Create the *pin_binds* list from a single *peripheral_permuatation*.
        pin_binds: List[PinBind] = pin_binds_get(peripheral_permutation, arduino_signals,
                                                 daughter_signals, morpho_signals)

        # Perform the pin bindings.  Ignore the the resulting pin/signals tables, but keep
        # the *unbound_signals* and build up a *permutation_score*:
        signal_bindings_table: Dict[Text, Tuple[Binding, ...]]
        pin_bindings_table: Dict[Text, Binding]
        unbound_signals: TextTuple
        signal_bindings_table, pin_bindings_table, unbound_signals = pins_bind(pin_binds)
        permutation_score: Tuple[int, TextTuple, TextTuple] = (
            len(unbound_signals),
            tuple(unbound_signals),
            peripheral_permutation,
            )
        permutation_scores.append(permutation_score)

    # Sort *permutation_scores* so that the best permutations sort to the front:
    permutation_scores = sorted(permutation_scores)

    # Now unpack the first entry in *permutation scores* to select one that has the lowest
    # number of unbound signals:
    initial_unbound_signals_size: int
    permutation: TextTuple
    initial_unbound_signals_size, unbound_signals, permutation = permutation_scores[0]
    if initial_unbound_signals_size == 0:
        # We have a winner, so we can show a more detailed summary of the solution:
        summary_show(permutation,
                     arduino_signals, daughter_signals, morpho_signals,
                     arduino_set, daughter_set, morpho_set, pin_numbers_table)
    else:
        # Nope, we are asking for too much and ran out of pins.  So now we list all of the
        # permutations that have the same number of unbound signals as the first one.
        permutation_record: Tuple[int, TextTuple, TextTuple]
        print("Unbound siginals:")
        unbound_signals_size: int
        for unbound_signals_size, unbound_signals, permutation in permutation_scores:
            if unbound_signals_size > initial_unbound_signals_size:
                # We do not need to look as solutions that event more unbound signals than
                # the first one.
                break
            print(f"{unbound_signals}: {permutation}")
        print("")


# The functions after main() are listed alphabetically to make them easier to find.

def all_signals_extract(zios_table: Dict[Text, Text]) -> Tuple[Tuple[Signal, ...], Dict[Text, int]]:
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
            {ALTERNATE_FUNCTION_NAME} is "AF" follow by a number between 0 and 14.
    """
    # Read in the alternate functions table from the .csv file for the Nucleo144-767ZI:
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
    # up information about signals that can be connected to a given pin name.
    # The *signals_table* builds up a list of pin names that can be connected to each signal.
    pins_table: Dict[Text, TextTuple] = {}
    pin_numbers_table: Dict[Text, int] = {}
    signals_table: Dict[Text, List[Text]] = {}

    # Some shared variables used both inside and outside of the loops:
    signal_pins: List[Text]
    data_row: Row
    signal_name: Text
    af0_index: int = af_indices[0]
    # print(f"af0_index={af0_index}")

    # Iterate over *data_rows*:
    data_row_index: int
    for data_row_index, data_row in enumerate(data_rows):
        # Grab the *pin_number*
        pin_number: int
        try:
            pin_number = int(data_row[pin_number_index])
        except ValueError:
            assert False, "Bad pin number"

        # Grab the *pin_name* and only look at pins of the form "P[A-G]#", where '#" is a number.
        pin_name: Text = data_row[pin_name_index]
        pin_name_fields: List[Text] = pin_name.split('/')
        if len(pin_name_fields) >= 1:
            pin_name = pin_name_fields[0]
        if (
                len(pin_name) >= 3 and pin_name[0] == 'P' and
                pin_name[1] in "ABCDEFGH" and pin_name[2:].isdigit()):
            # print(f"Pin[{pin_number}]:'{pin_name}'")
            pin_numbers_table[pin_name] = pin_number

            # The altenate data function bundings are at the end starting at *af0_index*.
            # Their should be 15 of them.
            afs: TextTuple = data_row[af0_index:]
            assert len(afs) == 15
            pins_table[pin_name] = afs

            # Now fill in the *signals_table*:
            af_index: int
            signal_names: Text
            for af_index, signal_names in enumerate(afs):
                # For now ignore signals that are not needed by HR2 right now:
                for signal_name in signal_names.split('/'):
                    # Ignore uninteresting signals that will not be bound to:
                    if signal_name and not signal_name_ignore(signal_name):
                        # Ensure *signal_name* is in *signals_table* and get *signal_pins* list:
                        if signal_name not in signals_table:
                            signals_table[signal_name] = [signal_name]
                        signal_pins = signals_table[signal_name]

                        # Only keep track of pins that are "free" for use on the ZIO connectors:
                        # The *zios_table* has a '@', '+', '-' for Arduino/Daughter/Morpho.
                        zios_flag: Text = zios_table[pin_name]
                        if zios_flag in "+@-":
                            annotated_pin: Text = f"{zios_flag}{pin_name}:AF{af_index}"
                            signal_pins.append(annotated_pin)
                        # print(f"  '{signal_name}':{signal_pins}")

    # Create the *signals_stuple* list from the *signals_table*, using tuples instead of lists.
    # It is really important that the *signals* be a Tuple so it can be used as a dictionary key
    # later on in the program:
    signals: List[Signal] = []
    for signal_pins in signals_table.values():
        signals.append(tuple(signal_pins))
    signals_tuple: Tuple[Signal, ...] = tuple(sorted(signals))

    # For reference/debugging perposes, write *signals_tuple* out to `/tmp/signals.txt`:
    signals_file: IO[Text]
    with open("/tmp/signals.txt", "w") as signals_file:
        for signal in signals_tuple:
            signals_file.write(f"{str(signal)}\n")
    return signals_tuple, pin_numbers_table


def alternate_functions_csv_read() -> Tuple[Row, Tuple[Row, ...]]:
    """Read the alternate functions table."""
    # Read in *rows* from "stm32f7676_af.csv" file:
    rows: List[TextTuple] = []
    csv_file: IO[Text]
    with open("stm32f767_af.csv", "r") as csv_file:
        af_reader = csv.reader(csv_file, delimiter=',', quotechar='"')
        for row_list in af_reader:
            rows.append(tuple(row_list))

    # The first row is the column *headings* and the remaining rows are the *data_rows*:
    headings: Row = rows[0]
    data_rows: Tuple[Row, ...] = tuple(rows[1:])
    return headings, data_rows


def morpho_set_extract() -> Set[Text]:
    """Return the Morph pin names read from .csv file."""
    # Read in *rows* from "morpho_.csv" file:
    rows: List[TextTuple] = []
    csv_file: IO[Text]
    with open("morpho_pins.csv", "r") as csv_file:
        af_reader = csv.reader(csv_file, delimiter=',', quotechar='"')
        for row_list in af_reader:
            rows.append(tuple(row_list))

    # The first row is the column *headings* and the remaining rows are the *data_rows*:
    data_rows: Tuple[Row, ...] = tuple(rows[1:])

    morpho_set: Set[Text] = set()
    data_row: TextTuple
    for data_row in data_rows:
        if len(data_row) >= 4:
            pin_name = data_row[2]
            if pin_name.startswith('@'):
                pin_name = pin_name[1:]
            if (
                    len(pin_name) >= 3 and pin_name[0] == 'P' and
                    pin_name[1] in "ABCDEFG" and pin_name[2:].isdigit()):
                morpho_set |= set([pin_name])
    return morpho_set


def nucleo_set_create() -> Set[Text]:
    """Return the processor pins used by the Nucleo-144."""
    pin_number_name_uses: List[Tuple[int, Text, Text]] = [
        (7, "PC13", "USER_Btn"),
        (8, "PC14", "RCC_OSC32_IN"),
        (9, "PC15", "RCC_OSC32_OUT"),
        (23, "PH0", "OSC_IN"),
        (24, "PH1", "OSC_OUT"),
        (27, "PC1", "RMII_MDC"),
        (35, "PA1", "RMII_REF_CLK"),
        (36, "PA2", "RMII_MDIO"),
        (43, "PA7", "RMII_CRS_DV"),  # ZIO overlap @D11
        (44, "PC4", "RMII_RXD0"),
        (45, "PC5", "RMII_RXD1"),
        (46, "PB0", "LD1"),  # ZIO overlap CN10-31
        (91, "PG6", "USB_PowerSwitchOn"),
        (92, "PG7", "USB_OverCurrent"),
        (100, "PA8", "USB_SOF"),
        (101, "PA9", "USB_VBUS"),
        (102, "PA10", "USB_ID"),
        (103, "PA11", "USB_DM"),
        (104, "PA12", "USB_DP"),
        (105, "PA13", "TMS"),
        (109, "PA14", "TCK"),
        (126, "PG11", "RMII_TX_EN"),
        (128, "PG13", "RMII_TXD0"),
        (133, "PB3", "SWO"),  # CN7-15
        (137, "PB7", "LD2 (Blue)"),  # Correct
    ]
    pin_number_name_use: Tuple[int, Text, Text]
    nucleo_set: Set[Text] = {pin_number_name_use[1]
                             for pin_number_name_use in pin_number_name_uses}
    return nucleo_set


def ordered_permutations(peripherals: TextTuple, count: int) -> TextTuple:
    """Return a sorted list of unique permutations."""
    # The output of the *permutations*() is an unordered list of permutations.
    # Sort them and stuff them into a set to to cull duplicates.
    permuation: List[Text]
    permutations_set: Set[Text] = set()
    for permutation in itertools.permutations(peripherals, count):
        sorted_permutation: Text = ':'.join(sorted(permutation))
        permutations_set.add(sorted_permutation)

    # Extract the final *ordered_permutations* from the set and return them:
    ordered_permutations: TextTuple = tuple(sorted(list(permutations_set)))
    return ordered_permutations


def peripheral_permutations_get(
        i2cs_count: int, spis_count: int, uarts_count: int) -> Tuple[TextTuple, ...]:
    """Return permuation list for I2C, SPI, and UART peripherals."""
    # These are the list of peripherals to permuate:
    i2cs: TextTuple = ("I2C1", "I2C2", "I2C4")  # "I2C3" is manually shorted to arduino pins
    spis: TextTuple = ("SPI2", "SPI3", "SPI4", "SPI5", "SPI6")  # SPI1 is prebound to arduino.
    # UART6 is already connected to Arduino pins already
    # UART8 Conflicts with LPTIM_IN1 on PE1 and there is no work around.  UART8 is not avaiable.
    uarts: TextTuple = ("USART1", "USART2", "USART3", "USART4", "UART5", "UART7")

    # Create the permutations for the I2C's, SPI'2, and UART's:
    i2c_permutations: TextTuple = ordered_permutations(i2cs, i2cs_count)
    spi_permutations: TextTuple = ordered_permutations(spis, spis_count)
    uart_permutations: TextTuple = ordered_permutations(uarts, uarts_count)

    # Just iterate over all possible permutations of each peripheral type:
    peripheral_permutations: List[TextTuple] = []
    i2c_permutation: Text
    for i2c_permutation in i2c_permutations:
        spi_permutation: Text
        for spi_permutation in spi_permutations:
            uart_permutation: Text
            for uart_permutation in uart_permutations:
                # Join all three permutaions into a single *peripheral_permutation:
                peripheral_permutation: TextTuple = tuple(sorted(
                    uart_permutation.split(':') +
                    i2c_permutation.split(':') +
                    spi_permutation.split(':')
                ))
                # print(f"Peripheral Permutation: {peripheral_permutation}")
                peripheral_permutations.append(peripheral_permutation)
    return tuple(peripheral_permutations)


def pin_binds_get(peripheral_permutation: TextTuple, arduino_signals: Signals,
                  daughter_signals: Signals, morpho_signals: Signals) -> List[PinBind]:
    """Return a list of pin pindings based on a permutation list."""
    # See the *PinBind* definition at the being of this program.
    # This is an ordered set of attempts to bind a pin.  The pins are bound in order
    # of importance (e.g. Arduino 1st, Nucleo 2nd, timers 3rd, etc.
    peripheral_set: Set[Text] = set(peripheral_permutation)
    pin_binds: List[PinBind] = []

    # Bind the arduino pins using the force pin name (4th field.)
    pin_binds.extend([
        ("UART6_RX", "D0_RX", arduino_signals, "PG9"),
        ("UART6_TX", "D1_TX", arduino_signals, "PG14"),
        (":D2", "D2", arduino_signals, "PF15"),
        ("TIM1_CH3", "D3_PWM", arduino_signals, "PE13"),        # ~TIM1_CH3  (~ means PWM)
        (":D4", "D4", arduino_signals, "PF14"),
        ("TIM1_CH2", "D5_PWM", arduino_signals, "PE11"),        # ~TIM1_CH2
        ("TIM1_CH1", "D6_PWM", arduino_signals, "PE9"),         # ~TIM1_CH1
        (":D7", "D7", arduino_signals, "PF13"),
        (":D8", "D8", arduino_signals, "PF12"),
        ("TIM4_CH4", "D9_PWM", arduino_signals, "PD15"),        # ~TIM4_CH4
        ("TIM4_CH3", "D10_PWM_NSS", arduino_signals, "PD14"),   # ~TIM4_CH3 (SPI[1|6]_NSS)
        ("SPI1_MOSI", "D11_PWM_MOSI", arduino_signals, "PA7"),  # ~SPI1/6_MOSI, SPI1/6_MOSI,
        #                                                       # TIM3_CH2, TIM14_CH1, RMII_CRS_DV
        ("SPI1_MISO", "D12_MISO", arduino_signals, "PA6"),      # SPI1_MISO, SPI6_MISO
        ("SPI1_SCK", "D13_SCK", arduino_signals, "PA5"),        # SPI1_SCK, SPI6_SCK
        (":D14", "D14", arduino_signals, "PB9"),
        (":D15", "D15", arduino_signals, "PB8"),
        ("ADC123_IN3", "A0", arduino_signals, "PA3"),           # PA3 ADC123_IN3
        ("ADC123_IN10", "A1", arduino_signals, "PC0"),          # PC0 ADD123_IN10
        ("ADC123_IN13", "A2", arduino_signals, "PC3"),          # PC3 ADC123_IN13
        ("ADC3_IN9", "A3", arduino_signals, "PF3"),             # PF3 ADC3_IN9

        ("I2C3_SDA", "A4_SDA", daughter_signals, "PC9"),        # Short together
        ("ADC3_IN15", "A4_SDA", arduino_signals, "PF5"),        # PF5 ADC3_IN15

        ("I2C3_SCL", "A5_SCL", morpho_signals, "PA8"),          # Short together
        ("ADC3_IN8", "A5_SCL", arduino_signals, "PF10"),        # PF10 ADC3_IN8
    ])

    # The Nucleo Pins should be bound next, since they tend not too overlap with the Zio pins
    # (with the 1 big exception of PA7 which everybody wants to use.)  Over time some of these
    # pins can be sniped for other purposes.  For now they can be left allocated.
    pin_binds.extend([
        ("USER_BTN", ":USER_SW1", morpho_signals, "PC13"),
        ("RCC_OSC32_IN", ":RCC_OSC_IN", morpho_signals, "PC14"),
        ("RCC_OSC32_OUT", ":RCC_OSC_OUT", morpho_signals, "PC15"),
        # ("OSC_IN", ":OCS_IN", morpho_signals, "PH0"),    # Do not bother with PH0
        # ("OSC_OUT", ":OSC_OUT", morpho_signals, "PH1"),  # Do not bother with PH1
        ("RMII_MDC", ":RMII_MDC", morpho_signals, "PC1"),
        ("RMII_REF_CLK", ":RMI_REF_CLK", morpho_signals, "PA1"),
        ("RMII_MDIO", ":RMII_MDIO", morpho_signals, "PA2"),
        # ("RMII_CRS_DV", ":RMII_CRS_DV", morpho_signals, "PA7"),  # ZIO overlap @D11
        ("RMII_RXD0", ":RMII_RXD0", morpho_signals, "PC4"),
        ("RMII_RXD1", ":RMII_RXD1", morpho_signals, "PC5"),
        ("LD1", ":LD1", morpho_signals, "PB0"),  # ZIO overlap CN10-31
        ("USB_POWERSWITCHON", ":USB_PWR_SW", morpho_signals, "PG6"),
        ("USB_OVERCURRENT", ":USB_OVERDRAW", morpho_signals, "PG7"),
        # ("USB_SOF", ":USB_SOF", morpho_signals, "PA8"),
        ("USB_VBUS", ":USB_VBUS:", morpho_signals, "PA9"),
        ("USB_ID", ":USB_ID", morpho_signals, "PA10"),
        ("USB_DM", ":USB_DM", morpho_signals, "PA11"),
        ("USB_DP", ":USB_DP", morpho_signals, "PA12"),
        ("SYS_JTMS-SWDIO", "SWDIO", morpho_signals, "PA13"),  # ST_LINK
        ("SYS_JTCK-SWCLK", "SWCLK", morpho_signals, "PA14"),  # ST_LINK
        ("RMII_TX_EN", "RMII_TX_EN", morpho_signals, "PG11"),
        ("RMII_TXD0", "RMII_TXD0", morpho_signals, "PG13"),
        # ("SWO", "SW0", morpho_signals, "PB3"),  # CN7-15 Not needed because SWO is unimplemented.
        ("LD2 (Blue)", ":LED_BLUE", morpho_signals, "PB7"),
    ])

    # Do the timers:
    pin_binds.extend([
        # The servos work best with 32-bit counters (i.e. TIM2/5).  It turns out that
        # there are exactly 4 Daughter availble pins to TIM2/5, so that are used for the servos.
        ("TIM5_CH1", "SERVO1", daughter_signals, ""),
        ("TIM2_CH2", "SERVO2", daughter_signals, ""),
        ("TIM2_CH3", "SERVO3", daughter_signals, ""),
        ("TIM2_CH4", "SERVO4", daughter_signals, ""),

        # The encoders are a real challenge.  TIM8 has CH1/2 available on the Daugher so the
        # get assigned to the RENCODER.  The none of the other encoder enabled timers
        # (TIM1/2/3/4/5/6/8) have both CH1/2 available.  This is fixed by shorting a Morpho
        # pin to LPTIM1_IN2.  This allows TIM1 to be used for LENCODER.
        ("LPTIM1_IN1", "LQUAD_A", daughter_signals, ""),
        ("LPTIM1_IN2", "LQUAD_B", morpho_signals, "PE1"),    # LPTIM1_IN2 only on PE1.
        ("LPTIM1_IN2", "LQUAD_B", daughter_signals, "PA4"),  # Short these two lines together
        ("TIM8_CH1", "RQUAD_A", daughter_signals, ""),
        ("TIM8_CH2", "RQUAD_B", daughter_signals, ""),

        # It is a requirement that each motor have both its +/- outputs on the same timer.
        # TIM3 fits the bill:
        ("TIM3_CH1", "LMOTOR_CTL1", daughter_signals, ""),
        ("TIM3_CH2", "LMOTOR_CTL2", daughter_signals, ""),
        ("TIM3_CH3", "RMOTOR_CTL1", daughter_signals, ""),
        ("TIM3_CH4", "RMOTOR_CTL2", daughter_signals, ""),

        # The Lidar just needs one dedicated timer.
        # There are plenty to choose from, so TIM4_CH2 is used.
        ("TIM4_CH2", "LIDAR_PWM", daughter_signals, ""),

        # The LED's just need an internal timer to trigger the DMA peripheral.
        # TIM6 is used by HAL, so TIM7 will have to do.
    ])

    if False and "SPI1" in peripheral_set:    # SPI1 is bound to the arduino pins:
        pin_binds.extend([
            ("SPI1_MISO", ":SPI1_MISO", daughter_signals, ""),
            ("SPI1_MOSI", ":SPI1_MOSI", daughter_signals, ""),
            ("SPI1_SCK", ":SPI1_SCK", daughter_signals, ""),
            ("SPI1_NSS", ":SPI1_NSS", daughter_signals, ""),
        ])

    if "SPI2" in peripheral_set:
        pin_binds.extend([
            ("SPI2_MISO", ":SPI2_MISO", daughter_signals, ""),
            ("SPI2_MOSI", ":SPI2_MOSI", daughter_signals, ""),
            ("SPI2_SCK", ":SPI2_SCK", daughter_signals, ""),
            ("SPI2_NSS", ":SPI2_NSS", daughter_signals, ""),
        ])
    if "SPI3" in peripheral_set:
        pin_binds.extend([
            ("SPI3_MISO", "LEDS_MISO", daughter_signals, ""),  # Use these for the LED shift regs.
            ("SPI3_MOSI", "LEDS_MOSI", daughter_signals, ""),
            ("SPI3_SCK", "LEDS_SCK", daughter_signals, ""),
            ("SPI3_NSS", "LEDS_NSS", daughter_signals, ""),
        ])
    if "SPI4" in peripheral_set:
        pin_binds.extend([
            ("SPI4_MISO", "DIO_MISO", daughter_signals, ""),  # Use these for the DIO shift regs.
            ("SPI4_MOSI", "DIO_MOSI", daughter_signals, ""),
            ("SPI4_SCK", "DIO_SCK", daughter_signals, ""),
            ("SPI4_NSS", "DIO_NSS", daughter_signals, ""),
        ])
    if "SPI5" in peripheral_set:
        # SPI5 needs to use a Morpho pin:
        pin_binds.extend([
            ("SPI5_MISO", ":SPI5_MISO", daughter_signals, ""),
            ("SPI5_MOSI", ":SPI5_MOSI", daughter_signals, ""),
            ("SPI5_SCK", ":SPI5_SCK", daughter_signals, ""),
            ("SPI5_NSS", ":SPI5_NSS", morpho_signals, ""),  # Note: SPI5 needs a Morpho pin!
        ])

    if False and "SPI6" in peripheral_set:
        # SPI6 interferes with TIM6 which must be used by the servos:
        pin_binds.extend([
            ("SPI6_MISO", ":SPI6_MISO", daughter_signals, ""),
            ("SPI6_MOSI", ":SPI6_MOSI", daughter_signals, ""),
            ("SPI6_SCK", ":SPI6_SCK", daughter_signals, ""),  # Note: TIM2_CH2 needs this pin.
            ("SPI6_NSS", ":SPI6_NSS", daughter_signals, ""),
        ])

    if "I2C1" in peripheral_set:
        pin_binds.extend([
            ("I2C1_SCL", ":I2C1SL", daughter_signals, ""),
            ("I2C1_SDA", ":I2C1DA", daughter_signals, ""),
        ])

    if "I2C2" in peripheral_set:
        # This pin name are selected after the permutation is fixed.
        pin_binds.extend([
            ("I2C2_SCL", "MISC_SCL", daughter_signals, ""),  # This is the miscellaneours I2C.
            ("I2C2_SDA", "MISC_SDA", daughter_signals, ""),
        ])

    if "I2C3" in peripheral_set:
        pin_binds.extend([
            ("I2C3_SCL", ":I2C3SL", morpho_signals, ""),
            ("I2C3_SDA", ":I2C3DA", daughter_signals, ""),
        ])

    if "I2C4" in peripheral_set:
        pin_binds.extend([
            ("I2C4_SCL", ":I2C4SL", daughter_signals, ""),
            ("I2C4_SDA", ":I2C4DA", daughter_signals, ""),
        ])

    # UARTS's come next.  Note that the schematic signal names swapped from the peripheral names:
    if "USART1" in peripheral_set:
        pin_binds.extend([
            ("USART1_RX", "SBC_TX", daughter_signals, ""),
            ("USART1_TX", "SBC_RX", daughter_signals, ""),
        ])
    if "USART2" in peripheral_set:
        pin_binds.extend([
            ("USART2_RX", "WOW_TX", daughter_signals, ""),
            ("USART2_TX", "WOW_RX", daughter_signals, ""),
        ])
    if "USART3" in peripheral_set:
        pin_binds.extend([
            ("USART3_RX", "STL_TX", morpho_signals, ""),  # Nucleo manual expects USART3
            ("USART3_TX", "STL_RX", morpho_signals, ""),
        ])
    if "USART4" in peripheral_set:
        pin_binds.extend([
            ("USART4_RX", "U4_TX", daughter_signals, ""),
            ("USART4_TX", "U4_RX", daughter_signals, ""),
        ])
    if "UART5" in peripheral_set:
        pin_binds.extend([
            ("UART5_RX", "LIDAR_TX", daughter_signals, ""),
            ("UART5_TX", "LIDAR_RX", daughter_signals, ""),
        ])
    if "UART6" in peripheral_set:  # UART6 already been bound with the arduino pins.
        assert False, "UART6 is already connected to Arduino pins already"
        pin_binds.extend([
            ("UART6_RX", "D0_TX", arduino_signals, "PG9"),
            ("UART6_TX", "D1_RX", arduino_signals, "PG14"),
        ])
    if "UART7" in peripheral_set:
        pin_binds.extend([
            ("UART7_RX", "FPGA_TX", daughter_signals, ""),
            ("UART7_TX", "FPGA_RX", daughter_signals, ""),
        ])

    if "UART8" in peripheral_set:
        # UART8 is not avaiable.
        assert False, "UART8 Conflicts with LPTIM_IN1 on PE1 and there is no work around."
        pin_binds.extend([
            ("UART8_RX", "U8_RT", morpho_signals, ""),
            ("UART8_TX", "U8_RX", morpho_signals, ""),
        ])

    # This is all done manually after the final periperal permutation is selected:
    # Bind pins to the sonars first:
    if True:
        # Each port bit number must be different in order to support external interrupts.
        # '>' means digital input:
        pin_binds.extend([
            (">PE0", "ECHO1", daughter_signals, "PE0"),
            (">PE8", "ECHO2", daughter_signals, "PE8"),
            (">PE10", "ECHO3", daughter_signals, "PE10"),
            (">PE12", "ECHO4", daughter_signals, "PE12"),
            (">PE14", "ECHO5", daughter_signals, "PE14"),
            (">PE15", "ECHO6", daughter_signals, "PE15"),
            (">PF9", "ECHO7", daughter_signals, "PF9"),
        ])

    return pin_binds


def pin_name_deannotate(annotated_pin_name: Text) -> Text:
    """Return only the pin name from an annotated pin name."""
    pair: List[Text] = annotated_pin_name.split(':')
    return pair[0][1:]


def pin_signals_sets_get() -> Tuple[Signals, Signals, Signals,
                                    Set[Text], Set[Text], Set[Text], Dict[Text, int]]:
    """Return the Arduino/Daughter/Morpho Signals/Sets."""
    # Construct the *zios_table*:
    zios_table: Dict[Text, Text] = zios_table_create()
    all_signals: Signals
    pin_numbers_table: Dict[Text, int]
    all_signals, pin_numbers_table = all_signals_extract(zios_table)

    # The "arduino" pins are the Zio connector signal pins that map to the
    # Arduino A0-A5 and D0-D15 pins.
    # Extract *arduino_set* and *arduino_signals* from *all_signals*:
    arduino_set: Set[Text]
    arduino_signals: Signals
    arduino_signals, arduino_set = signals_subset(all_signals, '@', "Arduino")
    print(f"arduino_set={len(arduino_set)}:{sorted(list(arduino_set))}")

    # The daughter pins are the Zio connector signal pins that do not map to Arduino pins.
    # Extract *daughter_set* and *daughter_signals* from *all_signals*:
    daughter_set: Set[Text]
    daugther_signals: Signals
    daughter_signals, daughter_set = signals_subset(all_signals, '+', "Daughter")
    print(f"daughter_set={len(daughter_set)}:{sorted(list(daughter_set))}")

    # The morpho pins are the morpho connector signal pins that do not map to any of the Zio
    # connector pins.
    # Extract the *morpho_set* and the *morpho_signals* from *all_signals*:
    morpho_set: Set[Text]
    morpho_signals: Signals
    morpho_signals, morpho_set = signals_subset(all_signals, '-', "Morpho")
    print(f"morpho_set={len(morpho_set)}:{sorted(list(morpho_set))}")

    # Nucleo pins are pins that are pre-wired by the Nucleo board:
    nucleo_set: Set[Text] = nucleo_set_create()
    print(f"nucleo_set={len(nucleo_set)}:{sorted(list(nucleo_set))}")

    # Look for pins that are used are prebound by Nucleo:
    print(f"nucleo_arduino_pins={sorted(list(nucleo_set & arduino_set))}")
    print(f"nucleo_daughter_pins={sorted(list(nucleo_set & daughter_set))}")
    print(f"nucleo_morpho_pins={sorted(list(nucleo_set & morpho_set))}")

    # Compute some sets to check for issues (yes at least one was found and fixed):
    connector_set: Set[Text] = arduino_set | daughter_set | morpho_set
    non_connector_nucleo_set: Set[Text] = nucleo_set - connector_set
    print(f"non_connector_nucleo_set={sorted(list(non_connector_nucleo_set))}")

    return (arduino_signals, daughter_signals, morpho_signals,
            arduino_set, daughter_set, morpho_set, pin_numbers_table)


def pins_bind(pin_binds: List[PinBind]) -> Tuple[
        Dict[Text, Tuple[Binding, ...]], Dict[Text, Binding], TextTuple]:
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
    signal_bindings_table: Dict[Text, Tuple[Binding, ...]] = {}

    # This step is pretty obscure.  We need to partition the *pin_binds* by it *signal*
    # (type *Signals*) component, which is one of *arduino_signals*, *daughter_signals*,
    # or *morpho_signals*.  These three values are large but immutable structures,
    # so they can be used a dictionary key.  So, we can build *signals_table* which
    # is keyed by off of these three values to build lists of *PinBind*'s that are
    # isolated to a single *Signals* instance.  The *PinBind*'s order must be preserved,
    # since the *Signal* with each *Signals* object are sorted so that the shortest tuples
    # occur first.  This is encourage binding to pins that have the fewest options first.
    binding: Binding
    pin_bind: PinBind
    signal_name: Text
    schematic_name: Text
    signals: Signals
    force_pin_name: Text
    shorts_count: int = 0
    signals_table: Dict[Signals, List[PinBind]] = {}
    for pin_bind in pin_binds:
        # Unpack *pin_bind*:
        signal_name, schematic_name, signals, force_pin_name = pin_bind
        if force_pin_name != "":
            binding = (signal_name, f"{force_pin_name}:AF15", schematic_name)
            # print(f"signal_name={signal_name} schematic_name={schematic_name} "
            #       f"force_pin_name={force_pin_name} binding={binding}")
            if force_pin_name not in pin_bindings_table:
                pin_bindings_table[force_pin_name] = binding
            else:
                print("        !!!!!!!!!!!! Should not happen")
            if signal_name in signal_bindings_table:
                # This is a shorted pin and it is treated specially:
                previous_binding_tuple: Tuple[Binding, ...] = signal_bindings_table[signal_name]
                new_binding_tuple: Tuple[Binding, ...] = previous_binding_tuple + (binding,)
                signal_bindings_table[signal_name] = new_binding_tuple
                shorts_count += 1
            else:
                signal_bindings_table[signal_name] = (binding,)
        else:
            if signals not in signals_table:
                signals_table[signals] = []
            signals_table[signals].append(pin_bind)
    assert len(signal_bindings_table) + shorts_count == len(pin_bindings_table), (
        f"Early binding failed {binding}")

    # Now we can sweep through the *signals_table* and find try to allocate the associated
    # *PinBind*'s.  The order that the *signals* comes out of the iterator is semi-random,
    # but it does not matter since no pins are shared between them.
    index: int = 0
    pin_binds_list: List[PinBind]
    for signals, pin_binds_list in signals_table.items():
        index += 1
        # Now sweep through *signals* which has been presorted so that the hardest to bind
        # signals (i.e. the ones with the fewest available pins come first):
        duplicate_signals: Signals
        for signal in signals:
            # print(f"Signal {signal} looking for any takers")
            desired_signal_name: Text = signal[0]
            annotated_pin_names: TextTuple = signal[1:]
            for signal_name, schematic_name, duplicate_signals, force_pin_name in pin_binds_list:
                if force_pin_name == "":
                    # print(f"    Attempting to bind '{signal_name}'")
                    assert signals == duplicate_signals, "Something is very confused!"
                    match: bool = False
                    if signal_name == desired_signal_name:
                        # We have a signal match; attempt to do a binding:
                        # print(f"    Found '{signal_name}' signal={signal}")
                        annotated_pin_name: Text
                        pin_name: Text
                        # assert len(signal_bindings_table) == len(pin_bindings_table), "Not OK"
                        for annotated_pin_name in annotated_pin_names:
                            pin_name = pin_name_deannotate(annotated_pin_name)
                            if (
                                    pin_name not in pin_bindings_table and
                                    signal_name not in signal_bindings_table):
                                # We have a match; perform the binding:
                                binding = (signal_name, annotated_pin_name[1:], schematic_name)
                                signal_bindings_table[signal_name] = (binding,)
                                pin_bindings_table[pin_name] = binding
                                # print(f"    >>> Bind: '{pin_name}' "
                                #       f"signal={signal} binding={binding}")
                                match = True
                                break
                        else:
                            for annotated_pin_name in annotated_pin_names:
                                pin_name = pin_name_deannotate(annotated_pin_name)
                                assert pin_name in pin_bindings_table, f"Pin {pin_name} not bound?"
                                # previous_binding: Binding = pin_bindings_table[pin_name]
                                # signals_print(signals, "Bind: All Pins Previously Bound")
                            # assert False, f"Unable to bind {signal_name} to {signal}"
                        if match:
                            break
            else:
                # print(f"Signal {signal} no takers found")
                pass

    # Now look for pins that did not get bound:
    unbound_names: List[Text] = []
    for pin_bind in pin_binds:
        signal_name, schematic_name, signals, force_pin_name = pin_bind
        if signal_name not in signal_bindings_table:
            unbound_names.append(signal_name)

    # Sweep through pin and signal bindings table looking for any last issues:
    bindings: Tuple[Binding, ...]
    for signal_name, bindings in signal_bindings_table.items():
        for binding in bindings:
            assert signal_name == binding[0]
    for pin_name, binding in pin_bindings_table.items():
        assert pin_name == binding[1].split(':')[0]

    # One last sanity check on the pin and signal bindings table before returning them:
    signal_bindings_size = len(signal_bindings_table)
    pin_bindings_size = len(pin_bindings_table)
    if signal_bindings_size + shorts_count != pin_bindings_size:
        print(f"pinsbind: pin_bindings_size={pin_bindings_size}")
        print(f"pinsbind: signal_bindings_size={signal_bindings_size}")
        print(f"pinsbind: shorts_count={shorts_count}")
        assert False, "Inconsistency between pins and signal bindings table"
    return signal_bindings_table, pin_bindings_table, tuple(unbound_names)


def signal_name_ignore(signal_name: Text) -> bool:
    """Return True if signal name is not useful."""
    return (
        # Ignore these prefixes:
        signal_name.startswith("CAN") or
        signal_name.startswith("CEC") or
        signal_name.startswith("DCMI") or
        signal_name.startswith("DFSDM") or
        signal_name.startswith("ETH") or
        signal_name.startswith("FMC") or
        signal_name.startswith("I2S") or
        signal_name.startswith("LTDC") or
        signal_name.startswith("MDIOS") or
        signal_name.startswith("QUADSPI") or
        signal_name.startswith("RCC") or
        signal_name.startswith("RTC") or
        signal_name.startswith("SAI") or
        signal_name.startswith("SDMMC") or
        signal_name.startswith("SPDIFRX") or
        signal_name.startswith("SYS") or
        signal_name.startswith("USB") or
        # Ignore these suffixes:
        signal_name.endswith("_CK") or
        signal_name.endswith("_CTS") or
        signal_name.endswith("_DE") or
        signal_name.endswith("_ETR") or
        signal_name.endswith("_RTS") or
        signal_name.endswith("_SMBA")
    )


def signals_print(signals: Tuple[TextTuple, ...], title: Text) -> None:
    """Print out the signals with a title."""
    print(f"{title}:")
    signal_pins_tuple: TextTuple
    for signal_pins_tuple in signals:
        print(f"{signal_pins_tuple}")
    print("")


def signals_subset(signals: Signals, pattern: Text, title: Text) -> Tuple[Signals, Set[Text]]:
    """Return a subset of signals based on a pattern."""
    culled_signals: List[TextTuple] = []
    pin_names_set: Set[Text] = set()
    signal_pins: TextTuple
    for signal_pins in signals:
        signal_name: Text = signal_pins[0]
        annotated_pin_names: TextTuple = signal_pins[1:]
        culled_signal_pins: List[Text] = [signal_name]
        annotated_pin_name: Text
        for annotated_pin_name in annotated_pin_names:
            flag_pin_name_af: List[Text] = annotated_pin_name.split(':')
            assert len(flag_pin_name_af) == 2, f"Bad signal '{flag_pin_name_af}'"
            flag_pin_name: Text = flag_pin_name_af[0]
            # af: Text = flag_pin_name_af[1]  # No used!
            flag: Text = flag_pin_name[0]
            assert flag in "@+-", f"Flag '{flag}' must be one of '@', '+', or '-'"
            pin_name: Text = flag_pin_name[1:]
            assert (len(pin_name) >= 3 and pin_name[0] == 'P' and pin_name[1] in "ABCDEFG" and
                    pin_name[2:].isdigit()), f"Bad pin name from '{pin_name}'"
            if flag in pattern:
                culled_signal_pins.append(annotated_pin_name)
                pin_names_set |= set([pin_name])  # Need to enclose in list to avoid set of letters.

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
    return final_culled_signals, pin_names_set


def summary_show(peripheral_permutation: TextTuple,
                 arduino_signals: Signals, daughter_signals: Signals, morpho_signals: Signals,
                 arduino_set: Set[Text], daughter_set: Set[Text], morpho_set: Set[Text],
                 pin_numbers_table: Dict[Text, int]) -> None:
    """Show a summary of what worked."""
    # print(f"len(signal_bindings_table)={len(signal_bindings_table)}")
    # print(f"len(pin_bindings_table)={len(pin_bindings_table)}")

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

    # Recompute the various tables from *peripheral_permuation*:
    pin_binds: List[PinBind] = pin_binds_get(peripheral_permutation,
                                             arduino_signals, daughter_signals, morpho_signals)
    signal_bindings_table: Dict[Text, Tuple[Binding, ...]]
    pin_bindings_table: Dict[Text, Binding]
    unbound_signals: TextTuple
    signal_bindings_table, pin_bindings_table, unbound_signals = pins_bind(pin_binds)

    # Print out the Pin Name bindings:
    print("Pin Name Bindings:")
    pin_bindings: List[Quad] = [binding_extract(binding)
                                for binding in pin_bindings_table.values()]
    pin_bindings = sorted(pin_bindings)
    quad: Quad
    for quad in pin_bindings:
        print(f"{quad}")

    pin_numbers_bindings: List[Tuple[int, Quad]] = []
    pin_number: int
    for quad in pin_bindings:
        pin_number = pin_numbers_table[f"{quad[0]}{quad[1]}"]
        pin_numbers_bindings.append((pin_number, quad))
    print("")

    # Write the pin number bindings out to a temporary file:
    pin_number_file: IO[Text]
    with open("/tmp/pin_order.txt", "w") as pin_number_file:
        pin_numbers_bindings = sorted(pin_numbers_bindings)
        pin_number_binding: Tuple[int, Quad]
        for pin_number, quad in pin_numbers_bindings:
            pin_number_file.write(f"{pin_number}: {quad}\n")

    print(f"len(signal_bindings_table)={len(signal_bindings_table)}")
    schematic_bindings: List[Binding] = []
    signal_bindings: List[Binding] = []
    signal_binding: Binding
    schematic_binding: Binding
    bindings_tuple: Tuple[Binding, ...]
    shorts: List[Tuple[Binding, ...]] = []
    for bindings_tuple in signal_bindings_table.values():
        if len(bindings_tuple) > 1:
            shorts.append(bindings_tuple)
        for signal_binding in bindings_tuple:
            signal_bindings.append(signal_binding)
            schematic_binding = (signal_binding[2], signal_binding[0], signal_binding[1])
            schematic_bindings.append(schematic_binding)
    signal_bindings = sorted(signal_bindings)
    schematic_bindings = sorted(schematic_bindings)

    print("Signal Bindings:")
    for signal_binding in signal_bindings:
        print(f"{signal_binding}")
    print("")

    print("Schematic Bindings:")
    for schematic_binding in schematic_bindings:
        print(f"{schematic_binding}")
    print("")

    # Print out the shorts list.  All the effort to support shorts in the pin binding
    # process went to waste because ADC pins are not Alternate function pins.  It is easier
    # just manually solve the problem below.  In hindsight, the correct thing to have
    # done was to add ADC bindings as alternate functions, but the amount of effort to
    # extract that data from the spec. sheet was going to be enormous. Hence, the hack below:
    print("Shorts:")
    # The two I2C3 pins shorted into in the arduino pins bindings section.
    shorts.append((('I2C3_SDA', 'PC9:AF4', 'A4_SDA'), ('ADC3_IN15', 'PF5:ADC', 'A4_SDA')))
    shorts.append((('I2C3_SCL', 'PA8:AF4', 'A4_SCL'), ('ADC3_IN8', 'PF10:ADC', 'A4_SCL')))
    shorts = sorted(shorts)
    for bindings_tuple in shorts:
        print(f"{bindings_tuple}")
    print("")

    # pin_bindings_size = len(pin_bindings)
    # signal_bindings_size = len(signal_bindings)
    # assert pin_bindings_size == signal_bindings_size, (
    #     f"pin_bindings_size={pin_bindings_size} != signal_bindings_size={signal_bindings_size}")

    unused_set: Set[Text] = set(list(pin_bindings_table.keys()))
    unused_arduino_set: Set[Text] = arduino_set - unused_set
    unused_daughter_set: Set[Text] = daughter_set - unused_set
    unused_morpho_set: Set[Text] = morpho_set - unused_set
    print(f"unused_arduino_set= {len(unused_arduino_set)}: {sorted(list(unused_arduino_set))}")
    print(f"unused_morpho_set: {len(unused_morpho_set)}: {sorted(list(unused_morpho_set))}")
    print(f"unused_daughter_set: {len(unused_daughter_set)}: {sorted(list(unused_daughter_set))}")
    print(f"Peripherals: {peripheral_permutation}")
    print("")

    # signals_print(daughter_signals, "Daughter")
    # signals_print(morpho_signals, "Morpho")


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
        "+++++++-@@++++-+",  # PB
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
