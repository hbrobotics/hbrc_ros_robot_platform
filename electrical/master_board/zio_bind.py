#!/usr/bin/env python

"""Figure out pin binding for ZIO connectors."""

from typing import Dict, IO, List, Text, Tuple
import csv


def main() -> None:
    """Figure out pin binding for ZIO connectors."""
    # Create a *zio_lines* table that consist of 7 rows (one for each port) of 16 character per
    # line, where '+'=>available; '-'=>not connected; '@'=>Arduino Pin, '*'=>needs solder bridge:
    zio_lines: Tuple[Text, ...] = (
        "+--@+@@@-------+",  # PA
        "+++++++-@@++@@-+",  # PB
        "@-+@--+++++++---",  # PC
        "++++++++---+++@@",  # PD
        "+-+*+++++@+@+@++",  # PE
        "+++@+@-+++@-@@@@",  # PF
        "++++-----@----@-",  # PG
    )

    # Construct the *zios_table* which is filled with "P?#', where '?' is 'A' to 'G' and '#'
    # is a bit number between 0 and 15.  This table returns *True* if the pin is availble
    # for use on the Zio connector and does not interfere with the Arduino pins:
    zios_table: Dict[Text, Text] = {}
    zio_line: Text
    port_index: int
    for port_index, zio_line in enumerate(zio_lines):
        port_name: Text = "P" + chr(ord('A') + port_index)
        # print(f"Zio: port_name='{port_name}'")
        bit_index: int
        for bit_index, flag in enumerate(zio_line):
            zios_table[f"{port_name}{bit_index}"] = flag

    # Read in *rows* from "af.csv":
    rows: List[Tuple[Text, ...]] = []
    csv_file: IO[Text]
    with open("stm32f767_af.csv", "r") as csv_file:
        af_reader = csv.reader(csv_file, delimiter=',', quotechar='"')
        for row_list in af_reader:
            rows.append(tuple(row_list))

    # The first row is the column *headings* and the remaing rows are the *data_rows*:
    headings: Tuple[Text, ...] = rows[0]
    data_rows: List[Tuple[Text, ...]] = rows[1:]

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
    pins_table: Dict[Text, Tuple[Text, ...]] = {}
    signals_table: Dict[Text, List[Text]] = {}

    # Some shared variables used both inside and outside of the loops:
    signal_pins: List[Text]
    data_row: Tuple[Text, ...]
    signal_name: Text
    af0_index: int = af_indices[0]
    print(f"af0_index={af0_index}")
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
            afs: Tuple[Text, ...] = data_row[af0_index:]
            assert len(afs) == 15
            pins_table[pin_name] = afs

            # Now fill in the *signals_table*:
            af_index: int
            for af_index, signal_name in enumerate(afs):
                # For now ignore signals that are not needed by HR2 right now:
                if (
                        signal_name.startswith("SYS") or signal_name.startswith("USB") or
                        signal_name.startswith("SDMMC") or signal_name.startswith("SAI") or
                        signal_name.startswith("I2S") or signal_name.startswith("FMC") or
                        signal_name.startswith("ETH") or signal_name.startswith("DFSDM") or
                        signal_name.startswith("DCMI") or signal_name.startswith("CAN") or
                        signal_name.startswith("LTDC") or  # signal_name.startswith("LPTIM") or
                        signal_name.startswith("SPDIFRX") or signal_name.startswith("RCC") or
                        signal_name.startswith("CEC") or signal_name.startswith("MDIOS") or
                        signal_name.startswith("QUADSPI") or signal_name.startswith("RTC") or
                        signal_name.endswith("_CTS") or signal_name.endswith("_RTS") or
                        signal_name.endswith("_CK") or signal_name.endswith("_ETR")):
                    continue

                # Append xxx to *signal_pins*:
                if signal_name:
                    if signal_name not in signals_table:
                        signals_table[signal_name] = [signal_name]
                    signal_pins = signals_table[signal_name]

                    # For now only keep track of pins that are "free" for use on the ZIO connectors:
                    zios_flag: Text = zios_table[pin_name]
                    if zios_flag in "+@-":
                        signal_pins.append(f"{zios_flag}{pin_name}:AF{af_index}")
                    # print(f"  '{signal_name}':{signal_pins}")

    # Now convert *signals_table* into a sorted *all_signals_list*:
    all_signals_list: List[Tuple[Text, ...]] = []
    for signal_name, signal_pins in signals_table.items():
        all_signals_list.append(tuple(signal_pins))
    all_signals_list = sorted(all_signals_list)

    def cull_signals_list(
            all_signals_list: List[Tuple[Text, ...]], pattern: Text) -> List[Tuple[Text, ...]]:
        culled_signals_list: List[Tuple[Text, ...]] = []
        signal_pins_tuple: Tuple[Text, ...]
        for signal_pins_tuple in all_signals_list:
            culled_signal_pins: List[Text] = [signal_pins_tuple[0]]
            annotated_pin_name: Text
            for annotated_pin_name in signal_pins_tuple[1:]:
                if annotated_pin_name[0] in pattern:
                    culled_signal_pins.append(annotated_pin_name)
            if len(culled_signal_pins) > 1:
                culled_signals_list.append(tuple(culled_signal_pins))
        return culled_signals_list

    def show_signals_list(signals_list: List[Tuple[Text, ...]], title: Text) -> None:
        print(f"{title}:")
        for signal_pins_tuple in signals_list:
            print(f"{signal_pins_tuple}")
        print("")

    arduino_signals_list: List[Tuple[Text, ...]] = cull_signals_list(all_signals_list, '@')
    show_signals_list(arduino_signals_list, "Arduino")

    morpho_signals_list: List[Tuple[Text, ...]] = cull_signals_list(all_signals_list, '-')
    show_signals_list(morpho_signals_list, "Morpho")

    zio_signals_list: List[Tuple[Text, ...]] = cull_signals_list(all_signals_list, '+')
    show_signals_list(zio_signals_list, "Zio")

    def cleanup(annotated_pin_name: Text) -> Text:
        pair: List[Text] = annotated_pin_name.split(':')
        return pair[0]

    # A Binding is tuple consisting of a signal name, annotated pin name, and schematic name.
    Binding = Tuple[Text, Text, Text]
    signal_bindings_table: Dict[Text, Binding] = {}
    pin_bindings_table: Dict[Text, Binding] = {}

    def bind(signal_bindings_table: Dict[Text, Binding], pin_bindings_table: Dict[Text, Binding],
             needed_signals: Dict[Text, Text], signals_list: List[Tuple[Text, ...]]) -> None:
        """Bind needed signals to specific pins.

        Args:
            signal_bindings_table (Dict[Text, Binding]):
                A table of Binding's keyed by the signal name.
            pin_bindings_table (Dict[Text, Binding]):
                A table of Binding's keyed by the pin name.
            needed_signals (Dict[Text, Text]):
                A table of signals that need to be bound to a schematic
                symbol name.  The key is the signal name and the value
                is the schematic name.
            signals_list (List[Tuple[Text, ...]]):
                A list of available signal bindings:

        After each needed signal is bound, it is inserted into both
        signal_bindings_table and pin_bindings_table.  An error is
         generated of any of the needed_signals do not find a match.
        """
        # There is some trickiness here.  It is extremely important that we prioritize binding
        # to minimize chances of not binding.  This is done by sorting the attempted pin binding
        # order so that the shortest tuples occur first.  The shortest tuples should be bound
        # before the longer tuples since the shorter tuples have fewer chances of success.
        signal_tuple: Tuple[Text, ...]
        signals_list = sorted(signals_list, key=lambda signal_tuple: len(signal_tuple))

        # Now sweep through the prioritized *signals_list* trying to get a successful binding:
        binding: Binding
        length: int
        for signal_pins_tuple in signals_list:
            signal_name = signal_pins_tuple[0]
            # print(f"{signal_name}:")
            if signal_name in needed_signals:
                schematic_name: Text = needed_signals[signal_name]
                annotated_pin_name: Text
                for annotated_pin_name in signal_pins_tuple[1:]:
                    clean_pin_name: Text = cleanup(annotated_pin_name)
                    # print(f"    '{clean_pin_name}'")
                    if (
                            clean_pin_name not in pin_bindings_table and
                            signal_name not in signal_bindings_table):
                        # We have a match; perform the binding:
                        binding = (signal_name, annotated_pin_name[1:], schematic_name)
                        signal_bindings_table[signal_name] = binding
                        pin_bindings_table[clean_pin_name] = binding

        # Verify that all *needed_signals* got bound to a specific pin:
        needed_signal: Text
        for needed_signal in needed_signals.keys():
            if needed_signal not in signal_bindings_table:
                print(f"Unable to bind '{needed_signal} to a pin'")

    zio_needed_signals: Dict[Text, Text] = {
        "TIM2_CH2": "LMOTOR+",
        "TIM2_CH3": "LMOTOR-",
        # "TIM2_CH4": "LMOTOR",
        "TIM3_CH1": "LENCODER_A",
        "TIM3_CH2": "LENCODER_B",
        "TIM4_CH1": "RENCODER_A",
        "TIM4_CH2":
        "RENCODER_B",
        "TIM8_CH1": "SERVO1",
        "TIM8_CH2": "SERVO2",
        "TIM8_CH3": "SERVO3",
        "TIM8_CH4": "SERVO4",
        "TIM9_CH1": "RMOTOR+",
        "TIM9_CH2": "RMOTOR-",
    }
    bind(signal_bindings_table, pin_bindings_table, zio_needed_signals, zio_signals_list)

    arduino_needed_signals: Dict[Text, Text] = {
        "UART5_TX": "D0_TX",
        "UART5_RX": "D1_RX",
    }
    bind(signal_bindings_table, pin_bindings_table, arduino_needed_signals, arduino_signals_list)

    morpho_needed_signals: Dict[Text, Text] = {
        "USART3_RX": "STLINK_RX",  # Must be USART3
        "USART3_TX": "STLINK_TX",
        "USART1_RX": "SBC_RX",
        "USART1_TX": "SBC_TX",
        "USART1_RX": "SBC_RX",
        "USART1_TX": "SBC_TX",
    }
    bind(signal_bindings_table, pin_bindings_table, morpho_needed_signals, morpho_signals_list)

    Quad = Tuple[Text, int, Text, Text]

    def binding_extract(binding: Binding) -> Quad:
        clean_pin_name: Text = cleanup(binding[1])
        return (clean_pin_name[:2], int(clean_pin_name[2:]), binding[0], binding[2])

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


if __name__ == "__main__":
    main()
