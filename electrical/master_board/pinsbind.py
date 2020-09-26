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
#    "SIGNALS" is one of the signals subset (e.g. Arduino, Zio, or Morpho)
#    "FORCE_PIN_NAME" When non-empty, force a signal binding to a specific pin.
#                     The same signal can be bound to more that one pin using this feature.
PinBind = Tuple[Text, Text, Signals, Text]


def main() -> None:
    """Figure out pin binding for Morpho/ZIO connectors."""
    # Construct the *zios_table*:
    zios_table: Dict[Text, Text] = zios_table_create()
    all_signals: Signals = all_signals_extract(zios_table)

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
    daughter_signals, daughter_set = signals_subset(all_signals, '+', "Zio")
    print(f"daughter_set={len(daughter_set)}:{sorted(list(daughter_set))}")

    # The morpho pins are the morpho connector signal pins that do not map to any of the zioc
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

    # See the *PinBind* definition above for an explanation of the entries below.
    # This is an ordered set of attempts to bind a pin.  The pins are bound in order
    # of importance (e.g. Arduino 1st, Nucleo 2nd, timers 3rd, etc.
    pin_binds: List[PinBind] = [
        # ("SPI2_MISO", ":SPI2_MISO", daughter_signals, ""),
        # ("SPI2_MOSI", ":SPI2_MOSI", daughter_signals, ""),
        # ("SPI2_SCK", ":SPI2_SCK", daughter_signals, ""),
        # ("SPI2_NSS", ":SPI2_NSS", daughter_signals, ""),

        # ("SPI3_MISO", ":SPI3_MISO", daughter_signals, ""),
        # ("SPI3_MOSI", ":SPI3_MOSI", daughter_signals, ""),
        # ("SPI3_SCK", ":SPI3_SCK", daughter_signals, ""),
        # ("SPI3_NSS", ":SPI3_NSS", daughter_signals, ""),

        # ("SPI4_MISO", ":SPI4_MISO", daughter_signals, ""),
        # ("SPI4_MOSI", ":SPI4_MOSI", daughter_signals, ""),
        # ("SPI4_SCK", ":SPI4_SCK", daughter_signals, ""),
        # ("SPI4_NSS", ":SPI4_NSS", daughter_signals, ""),

        # SPI5 needs to use a Morpho pin:
        # ("SPI5_MISO", ":SPI5_MISO", daughter_signals, ""),
        # ("SPI5_MOSI", ":SPI5_MOSI", daughter_signals, ""),
        # ("SPI5_SCK", ":SPI5_SCK", daughter_signals, ""),
        # ("SPI5_NSS", ":SPI5_NSS", morpho_signals, ""),  # Note: SPI5 appears to require a Morpho!

        # SPI6 interferes with TIM6 which must be used by the servos:
        # ("SPI6_MISO", ":SPI6_MISO", daughter_signals, ""),
        # ("SPI6_MOSI", ":SPI6_MOSI", daughter_signals, ""),
        # ("SPI6_SCK", ":SPI6_SCK", daughter_signals, ""),  # Note: TIM2_CH2 needs this pin.
        # ("SPI6_NSS", ":SPI6_NSS", daughter_signals, ""),

        # ("I2C1_SCL", ":I2C1SL", daughter_signals, ""),
        # ("I2C1_SDA", ":I2C1DA", daughter_signals, ""),

        # ("I2C2_SCL", ":I2C2SL", daughter_signals, ""),
        # ("I2C2_SDA", ":I2C2DA", daughter_signals, ""),

        # ("I2C3_SCL", ":I2C3SL", morpho_signals, ""),
        # ("I2C3_SDA", ":I2C3DA", daughter_signals, ""),

        # ("I2C4_SCL", ":I2C4SL", daughter_signals, ""),
        # ("I2C4_SDA", ":I2C4DA", daughter_signals, ""),

        # Bind the arduino pins using the force pin name (4th field.)
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
        ("ADC3_IN15", "A4_SDA", arduino_signals, "PF5"),        # PF5 ADC3_IN15
        ("ADC3_IN8", "A5_SCL", arduino_signals, "PF10"),        # PF10 ADC3_IN8

        # The Nucleo Pins should be bound next, since they tend not too overlap with the Zio pins
        # (with the 1 big exception of PA7 which everybody wants to use.)  Over time some of these
        # pins can be sniped for other purposes.  For now they can be left allocated.
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
        ("USB_SOF", ":USB_SOF", morpho_signals, "PA8"),
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

        # The servos work best with 32-bit counters (i.e. TIM2/5).  It turns out that
        # there are exactly 4 Zio availble pins to TIM2/5, so that are used for the servos.
        ("TIM5_CH1", "SERVO1", daughter_signals, ""),
        ("TIM2_CH2", "SERVO2", daughter_signals, ""),
        ("TIM2_CH3", "SERVO3", daughter_signals, ""),
        ("TIM2_CH4", "SERVO4", daughter_signals, ""),

        # The encoders are a real challenge.  TIM8 has CH1/2 available on the Zio so the
        # get assigned to the RENCODER.  The none of the other encoder enabled timers
        # (TIM1/2/3/4/5/6/8) have both CH1/2 available.  This is fixed by shorting a Morpho
        # pin to LPTIM1_IN2.  This allows TIM1 to be used for LENCODER.
        ("LPTIM1_IN1", "LENCODER_A", daughter_signals, ""),
        ("LPTIM1_IN2", "LENCODER_B", morpho_signals, "PE1"),    # LPTIM1_IN2 only on PE1.
        ("LPTIM1_IN2", "LENCODER_B", daughter_signals, "PA4"),  # Short these two lines together
        ("TIM8_CH1", "RENCODER_A", daughter_signals, ""),
        ("TIM8_CH2", "RENCODER_B", daughter_signals, ""),

        # It is a requirement that each motor have both its +/- outputs on the same timer.
        # There is a real dirth of timers with two measly channels on the same counter.
        # TIM9 meets the requirement and gets RMOTOR.  This is fixed by shorting a Morpho
        # pin to TIM_CH3.
        ("TIM3_CH1", "LMOTOR+", daughter_signals, ""),
        ("TIM3_CH2", "LMOTOR-", daughter_signals, ""),
        ("TIM3_CH3", "RMOTOR+", daughter_signals, ""),
        ("TIM3_CH4", "RMOTOR-", daughter_signals, ""),

        # The Lidar just needs one dedicated timer.
        # There are plenty to choose from, so TIM4_CH2 is used.
        ("TIM4_CH2", "LIDAR_PWM", daughter_signals, ""),

        # The LED's just need an internal timer to trigger the DMA peripheral.
        # TIM6 is used by HAL, so TIM7 will have to do.

        # UARTS's come next.  Note that the signal names swapped from the peripheral names:
        # # ("USART1_RX", "U1_TX", daughter_signals, ""),
        # # ("USART1_TX", "U1_RX", daughter_signals, ""),
        # # ("USART2_RX", "U2_RX", daughter_signals, ""),
        # # ("USART2_TX", "U2_TX", daughter_signals, ""),
        # # ("USART3_RX", "STLINK_TX", morpho_signals, ""),  # Nucleo manual expects USART3
        # # ("USART3_TX", "STLINK_RX", morpho_signals, ""),
        # # ("UART4_RX", "U4_TX", daughter_signals, ""),
        # # ("UART4_TX", "U4_RX", daughter_signals, ""),
        # # ("UART5_RX", "U5_TX", daughter_signals, ""),
        # # ("UART5_TX", "U5_RX", daughter_signals, ""),
        # ## ("UART6_RX", "D0_TX", arduino_signals, "PG9"),  # Already done with Arduino
        # ## ("UART6_TX", "D1_RX", arduino_signals, "PG14"),
        # # ("UART7_RX", "U7_TX", daughter_signals, ""),  # Works
        # # ("UART7_TX", "U7_RX", daughter_signals, ""),
        # ("UART8_RX", "U8_RT", morpho_signals, ""),  # Conflicts with LPTIM_IN1 on PE1
        # ("UART8_TX", "U8_RX", morpho_signals, ""),  # No work around. UART8 is not avaiable.

        # # (":SPI1_MOSI", "D11_PWM_MOSI+", arduino_signals, "PA15"),  # ~SPI1/6_MOSI, SPI1/6_MOSI,

    ]

    # Perform the pin bindings and get the resulting pin and signals tables:
    signal_bindings_table: Dict[Text, Tuple[Binding, ...]]
    pin_bindings_table: Dict[Text, Binding]
    signal_bindings_table, pin_bindings_table = pins_bind(pin_binds)

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

    print("Pin Bindings:")
    pin_bindings: List[Quad] = [binding_extract(binding)
                                for binding in pin_bindings_table.values()]

    pin_bindings = sorted(pin_bindings)
    quad: Quad
    for quad in pin_bindings:
        print(f"{quad}")
    print("")

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

    print("Shorts:")
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

    # signals_print(daughter_signals, "Zio")
    # signals_print(morpho_signals, "Morpho")


# The functions after main() are listed alphabetically to make them easier to find.

def all_signals_extract(zios_table: Dict[Text, Text]) -> Tuple[TextTuple, ...]:
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
            signal_names: Text
            for af_index, signal_names in enumerate(afs):
                # For now ignore signals that are not needed by HR2 right now:
                for signal_name in signal_names.split('/'):
                    if signal_name_ignore(signal_name):
                        continue

                    # Append xxx to *signal_pins*:
                    if signal_name:
                        if signal_name not in signals_table:
                            signals_table[signal_name] = [signal_name]
                        signal_pins = signals_table[signal_name]

                        # Only keep track of pins that are "free" for use on the ZIO connectors:
                        zios_flag: Text = zios_table[pin_name]
                        if zios_flag in "+@-":
                            annotated_pin: Text = f"{zios_flag}{pin_name}:AF{af_index}"
                            signal_pins.append(annotated_pin)
                        # print(f"  '{signal_name}':{signal_pins}")

    # Sort the final *signals* and return them.
    signals: List[TextTuple] = []
    for signal_pins in signals_table.values():
        signals.append(tuple(signal_pins))

    # For reference, write *signals* out to `/tmp/signals.txt`:
    signals_file: IO[Text]
    with open("/tmp/signals.txt", "w") as signals_file:
        for signal in sorted(signals):
            signals_file.write(f"{str(signal)}\n")

    return tuple(signals)


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
    """Return the Pins used by the Nucleo-144."""
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


def pins_bind(pin_binds: List[PinBind]) -> Tuple[
        Dict[Text, Tuple[Binding, ...]], Dict[Text, Binding]]:
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

    # This step is pretty obscure.  We need to partion the *pin_binds* by it *signal*
    # (type *Signals*) component, which is one of *arduino_signals*, *daughter_signals*,
    # or *morpho_signals*.  These three values are large but immutable structures,
    # so they can be used a dictionary key.  So, we can build *signals_table* which
    # is keyed by off of these three values to build lists of *PinBind*'s that are
    # isolated to a single *Signals* instance.  The *PinBind*'s order must be preserved,
    # since the *Signal* with each *Signals* object are sort so that the shortest tuples
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
                            print(f"    {signal_name} wants to bind to {annotated_pin_names}")
                            for annotated_pin_name in annotated_pin_names:
                                pin_name = pin_name_deannotate(annotated_pin_name)
                                assert pin_name in pin_bindings_table, f"Pin {pin_name} not bound?"
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
    unbound_names: List[Text] = []
    for pin_bind in pin_binds:
        signal_name, schematic_name, signals, force_pin_name = pin_bind
        if signal_name not in signal_bindings_table:
            unbound_names.append(signal_name)
    if len(unbound_names) > 0:
        signal_names = sorted(unbound_names)
        raise RuntimeError(f"Unbound pins: {signal_names}")

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
    return signal_bindings_table, pin_bindings_table


def pin_name_deannotate(annotated_pin_name: Text) -> Text:
    """Return only the pin name from an annotated pin name."""
    pair: List[Text] = annotated_pin_name.split(':')
    return pair[0][1:]


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
