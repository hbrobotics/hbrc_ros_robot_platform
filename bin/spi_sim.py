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

"""Simulate SPI logic for HR2.

This is a basic logic simulator.  Everything is broken into Devices
and Wires.  A Wire is used to interconnect two or more Device pins.

The process of running the simulator is:
1. Instantiate all of the Devices.
2. Instantiate all of the Wires.
3. Bind the Wires to Devices.
4. Run the simulator.
5. Look at the output.

Each Device has a step method that implements the Device logic.
The step method reads the current input values at time T and
generates the next output values for time T+1.  Each time increment
can currently be thought of as one gate delay.  For a given time T,
the simulator invokes the step metheds for all devices.  The the
order that the step methods are called in does not matter, since
the only inputs at time T are read and outputs for time T+1 are
generated.

The basic types are:
* Wire:
  Wires interconnect Device pins.  The keep a signal trace for the
  wire.  In addition, the there are methods for detecting when the
  wire has a raising or falling edge.
* Device:
  The Device class is a base class.  Each actual device is a
  sub-class of Device.  Each Device implements the following methods:
  * __init__():  Intialize the device.
  * bind(): Bind wires to devices.
  * step(): Computes the next step of the device.
* Circuit:
  The Circuit class is a bass class that us used to encapsulate a
  circuit.  For each circuit the user wants to simulate they can
  sub class from Circuit.  This allows multiple Circuit's to coexit
  in the same file.  You provide an __init__ method, and the base
  class provides the run method.

Have fun!
"""

from typing import List, Text, Tuple


# Wire:
class Wire:
    """Simulate a digital wire."""

    # Wire.__init__():
    def __init__(self, name: Text) -> None:
        """Initialize a Wire."""
        # wire: Wire = self
        self.name: Text = name
        self.source_name: Text = ""
        self.values: List[bool] = [False]

    # Wire.__getitem__():
    def __getitem__(self, time: int):
        """Return the Wire value from a given time."""
        wire: Wire = self
        values: List[bool] = wire.values
        assert time < len(values), f"Wire '{wire.name}' is inactive at time {time}."
        return self.values[time]

    # Wire.__set_item__():
    def __setitem__(self, time: int, value: bool) -> None:
        """Set the Wire value for a given time."""
        wire: Wire = self
        values: List[bool] = wire.values
        if time == len(values):
            values.append(value)
        else:
            assert False, f"Wire '{wire.name}' is broken (multiple outputs)"

    # Wire.bind():
    def xbind(self, device: "Device", name: Text) -> None:
        """Bind to a Wire as a signal source."""
        wire: Wire = self
        assert not wire.source_name, "Wire is already bound to '{wire.source}'"
        source_name: Text = f"{device.name}.{name}"
        wire.source_name = source_name

    # Wire.is_falling():
    def is_falling(self, time: int) -> bool:
        """Return True if wire switched from high to low."""
        wire: Wire = self
        values: List[bool] = wire.values
        return values[time - 1] and not values[time]

    # Wire.is_raising():
    def is_raising(self, time: int) -> bool:
        """Return True if wire switched from low to high."""
        wire: Wire = self
        values: List[bool] = wire.values
        return not values[time - 1] and values[time]


Wires = Tuple[Wire, ...]


# Device:
class Device:
    """Device base class."""

    def __init__(self, name: Text) -> None:
        """Initialize Device base class."""
        # device: Device = self
        assert name, f"Device {name=} must be empty."
        self.name: Text = name
        self.inputs: Wires = ()
        self.outputs: Wires = ()

    def inputs_outputs_bind(self, inputs: Wires, outputs: Wires) -> None:
        """Bind wires to the device."""
        device: Device = self
        assert not device.inputs and not device.outputs, f"Device '{device.name}' is already bound"
        assert len(inputs) + len(outputs) > 0, f"Device '{device.name}' has not inputs or outputs"
        device.inputs = inputs
        device.outputs = outputs

    def step(self, time: int) -> None:
        """Run one step of the simulation."""
        assert False


# HighSource:
class HighSource(Device):
    """Represents a signal tied high."""

    # HighSource.__init__():
    def __init__(self) -> None:
        """Initialize the High device."""
        super().__init__("High")
        self.wire: Wire = Wire("")

    # HighSource.bind():
    def bind(self, wire: Wire) -> None:
        """Bind a Wire to High."""
        super().inputs_outputs_bind((), (wire,))
        self.wire = wire

    # HighSource.step():
    def step(self, time: int) -> None:
        """Perform a step on a High."""
        self.wire[time + 1] = True


# Inverter:
class Inverter(Device):
    """Represents an inverter."""

    # Inverter.__init__():
    def __init__(self, name: Text) -> None:
        """Initialize an Inverter."""
        super().__init__(name)
        self.in_wire: Wire = Wire("")
        self.out_wire: Wire = Wire("")

    # Inverter.bind():
    def bind(self, in_wire: Wire, out_wire: Wire) -> None:
        """Bind wires to an Inverter."""
        inverter: Inverter = self
        inverter.in_wire = in_wire
        inverter.out_wire = out_wire
        print(f"Inverter.bind('{inverter.name}', '{in_wire.name}', '{out_wire.name}')")

    # Inverter.Step():
    def step(self, time: int) -> None:
        """Performa a step on an Inverter."""
        inverter: Inverter = self
        in_wire: Wire = inverter.in_wire
        out_wire: Wire = inverter.out_wire
        out_wire[time + 1] = not in_wire[time]


# LowSource:
class LowSource(Device):
    """Represents a signal tied low."""

    # LowSource.__init__():
    def __init__(self) -> None:
        """Initialize the Low device."""
        super().__init__("Low")
        self.wire: Wire = Wire("")

    # LowSource.bind():
    def bind(self, wire: Wire) -> None:
        """Bind a Wire to Low."""
        super().inputs_outputs_bind((), (wire,))
        self.wire = wire

    # LowSource.step():
    def step(self, time: int) -> None:
        """Perform a step on a Low."""
        self.wire[time + 1] = False


# X595:
class X595(Device):
    """Simulate an x595 serial in/parallel out shift register."""

    # X595.__init__():
    def __init__(self, name: Text) -> None:
        """Initailize an X595 device."""
        super().__init__(name)

        # x595: X595 = self
        # Input Wires:
        zilch: Wire = Wire("")
        self.ser_in: Wire = zilch
        self.sr_clk: Wire = zilch
        self.r_clk: Wire = zilch
        self.sr_clr: Wire = zilch
        self.oe: Wire = zilch

        # Ouput wires:
        self.qa: Wire = zilch
        self.qb: Wire = zilch
        self.qc: Wire = zilch
        self.qd: Wire = zilch
        self.qe: Wire = zilch
        self.qf: Wire = zilch
        self.qg: Wire = zilch
        self.qh: Wire = zilch
        self.ser_out: Wire = zilch

        # Shift register:
        self.shift_a: bool = False
        self.shift_b: bool = False
        self.shift_c: bool = False
        self.shift_d: bool = False
        self.shift_e: bool = False
        self.shift_f: bool = False
        self.shift_g: bool = False
        self.shift_h: bool = False

        # Ouput register:
        self.latch_a: bool = False
        self.latch_b: bool = False
        self.latch_c: bool = False
        self.latch_d: bool = False
        self.latch_e: bool = False
        self.latch_f: bool = False
        self.latch_g: bool = False
        self.latch_h: bool = False

    # X595.bind():
    def bind(self,
             ser_in: Wire, sr_clk: Wire, r_clk: Wire, sr_clr: Wire, oe: Wire,
             qa: Wire, qb: Wire, qc: Wire, qd: Wire, qe: Wire, qf: Wire, qg: Wire, qh: Wire,
             ser_out: Wire) -> None:
        """Bind wires to X595."""
        inputs: Wires = (ser_in, sr_clk, r_clk, sr_clr, oe)
        outputs: Wires = (qa, qb, qc, qd, qe, qf, qg, qh)
        super().inputs_outputs_bind(inputs, outputs)
        x595: X595 = self

        # Bind inputs.
        x595.ser_in = ser_in
        x595.sr_clk = sr_clk
        x595.r_clk = r_clk
        x595.sr_clr = sr_clr
        x595.oe = oe

        # Ouput wires:
        x595.qa = qa
        x595.qb = qb
        x595.qc = qc
        x595.qd = qd
        x595.qe = qe
        x595.qf = qf
        x595.qg = qg
        x595.qh = qh
        x595.ser_out = ser_out

    # X595.step():
    def step(self, time: int):
        """Perform one step of simulation."""
        # Emulates one step.
        x595: X595 = self
        races: int = 0
        clear: bool = False
        shift: bool = False
        latch: bool = False
        if not x595.sr_clr[time]:
            # Clear:
            x595.shift_a = False
            x595.shift_b = False
            x595.shift_c = False
            x595.shift_d = False
            x595.shift_e = False
            x595.shift_f = False
            x595.shift_g = False
            x595.shift_h = False
            x595.latch_a = False
            x595.latch_b = False
            x595.latch_c = False
            x595.latch_d = False
            x595.latch_e = False
            x595.latch_f = False
            x595.latch_g = False
            x595.latch_h = False
            clear = True
            races += 1
        if x595.sr_clk.is_raising(time):
            # Shift:
            # print(f"{time}: sr_clk is raising; before shift=0x{x595.shift_get():02x}")
            x595.shift_h = x595.shift_g
            x595.shift_g = x595.shift_f
            x595.shift_f = x595.shift_e
            x595.shift_e = x595.shift_d
            x595.shift_d = x595.shift_c
            x595.shift_c = x595.shift_b
            x595.shift_b = x595.shift_a
            x595.shift_a = x595.ser_in[time]
            # print(f"{time}: sr_clk is raising; after shift=0x{x595.shift_get():02x}")
            shift = True
            races += 1
        if x595.r_clk.is_raising(time):
            # Latch data:
            x595.latch_a = x595.shift_a
            x595.latch_b = x595.shift_b
            x595.latch_c = x595.shift_c
            x595.latch_d = x595.shift_d
            x595.latch_e = x595.shift_e
            x595.latch_f = x595.shift_f
            x595.latch_g = x595.shift_g
            x595.latch_h = x595.shift_h
            latch = True
            print(f"{time}: X565({x595.name}): r_clk is raising; latch=0x{x595.latch_get():02x}")
            races += 1
        if races > 1 and time != 0:
            print(f"{time}: X595('{x595.name}'): Races: "
                  f"{races=} {clear=} {shift=} {latch=}")

        # Update the output wires:
        next: int = time + 1
        x595.qa[next] = x595.latch_a
        x595.qb[next] = x595.latch_b
        x595.qc[next] = x595.latch_c
        x595.qd[next] = x595.latch_d
        x595.qe[next] = x595.latch_e
        x595.qf[next] = x595.latch_f
        x595.qg[next] = x595.latch_g
        x595.qh[next] = x595.latch_h
        x595.ser_out[next] = x595.shift_h

    # X595.shift_get()
    def shift_get(self) -> int:
        """Return the value of the shift register."""
        x595: X595 = self
        shift: int = (
            (bool(x595.shift_a) << 0) |
            (bool(x595.shift_b) << 1) |
            (bool(x595.shift_c) << 2) |
            (bool(x595.shift_d) << 3) |
            (bool(x595.shift_e) << 4) |
            (bool(x595.shift_f) << 5) |
            (bool(x595.shift_g) << 6) |
            (bool(x595.shift_h) << 7)
        )
        return shift

    # X595.latch_get()
    def latch_get(self) -> int:
        """Return the value of the latch register."""
        x595: X595 = self
        latch: int = (
            (bool(x595.latch_a) << 0) |
            (bool(x595.latch_b) << 1) |
            (bool(x595.latch_c) << 2) |
            (bool(x595.latch_d) << 3) |
            (bool(x595.latch_e) << 4) |
            (bool(x595.latch_f) << 5) |
            (bool(x595.latch_g) << 6) |
            (bool(x595.latch_h) << 7)
        )
        return latch


# X165:
class X165(Device):
    """Implements an X165 8-bit shift register."""

    # X165.__init__():
    def __init__(self, name: Text) -> None:
        """Initialize an X165 shift register."""
        # Control inputs:
        super().__init__(name)
        # x165: X165 = self
        self.pl: Wire = Wire("")
        self.ds: Wire = Wire("")
        self.cp1: Wire = Wire("")
        self.cp2: Wire = Wire("")

        # Data Inputs:
        self.p0: Wire = Wire("")
        self.p1: Wire = Wire("")
        self.p2: Wire = Wire("")
        self.p3: Wire = Wire("")
        self.p4: Wire = Wire("")
        self.p5: Wire = Wire("")
        self.p6: Wire = Wire("")
        self.p7: Wire = Wire("")

        # Data Output:
        self.q7_out: Wire = Wire("")
        self.not_q7_out: Wire = Wire("")

        # Data bits:
        self.q0: bool = False
        self.q1: bool = False
        self.q2: bool = False
        self.q3: bool = False
        self.q4: bool = False
        self.q5: bool = False
        self.q6: bool = False
        self.q7: bool = False

    # X165.bind():
    def bind(self, pl: Wire, ds: Wire, cp1: Wire, cp2: Wire,
             p0: Wire, p1: Wire, p2: Wire, p3: Wire, p4: Wire, p5: Wire, p6: Wire, p7: Wire,
             q7_out: Wire, not_q7_out: Wire) -> None:
        """Bind wires to an X165."""
        # Control inputs:
        x165: X165 = self
        x165.pl = pl
        x165.ds = ds
        x165.cp1 = cp1
        x165.cp2 = cp2

        # Data Inputs:
        x165.p0 = p0
        x165.p1 = p1
        x165.p2 = p2
        x165.p3 = p3
        x165.p4 = p4
        x165.p5 = p5
        x165.p6 = p6
        x165.p7 = p7

        # Data Outputs:
        x165.q7_out = q7_out
        x165.not_q7_out = not_q7_out

    # X165.step():
    def step(self, time: int) -> None:
        """Perform one step of the X165."""
        x165: X165 = self

        # Compute the *cp* the parallel shift signal.
        cp1_is_raising: bool = x165.cp1.is_raising(time)
        cp2_is_raising: bool = x165.cp2.is_raising(time)

        # The asynchronous parallel load take precedence of the synchronous shift *cp*:
        # if x165.pl.is_raising(time):
        #    print(f"{time}: X165: latched={x165.shift_get()}")
        races: int = 0
        if not x165.pl[time]:
            # An asynchronous parallel load occurs.  No shifting can occur:
            x165.q0 = x165.p0[time]
            x165.q1 = x165.p1[time]
            x165.q2 = x165.p2[time]
            x165.q3 = x165.p3[time]
            x165.q4 = x165.p4[time]
            x165.q5 = x165.p5[time]
            x165.q6 = x165.p6[time]
            x165.q7 = x165.p7[time]
            races += 1
        elif cp1_is_raising or cp2_is_raising:
            assert not (cp1_is_raising and cp2_is_raising), "Race condition for '{x165.name}'"
            # We have a raising clock pulse edge, so perform the shift:
            # print(f"X165.step({time}): shift_before: 0x{x165.shift_get():02x}")
            x165.q7 = x165.q6
            x165.q6 = x165.q5
            x165.q5 = x165.q4
            x165.q4 = x165.q3
            x165.q3 = x165.q2
            x165.q2 = x165.q1
            x165.q1 = x165.q0
            x165.q0 = x165.ds[time]
            races += 1
            print(f"{time}: X165({x165.name}): cp1={x165.cp1.name} cp2={x165.cp2.name} "
                  f"shift_after: 0x{x165.shift_get():02x}")
        # else nothing happens:
        if races >= 2:
            print(f"{time}: X165({x165.name}): Race condition")

        # Update the outputs:
        next: int = time + 1
        q7: bool = x165.q7
        x165.q7_out[next] = q7
        x165.not_q7_out[next] = not q7

    # X165.shift_get()
    def shift_get(self) -> int:
        """Return the shift register value."""
        x165: X165 = self
        shift: int = (
            (bool(x165.q0) << 0) |
            (bool(x165.q1) << 1) |
            (bool(x165.q2) << 2) |
            (bool(x165.q3) << 3) |
            (bool(x165.q4) << 4) |
            (bool(x165.q5) << 5) |
            (bool(x165.q6) << 6) |
            (bool(x165.q7) << 7)
        )
        return shift


# SPIDevice:
class SPIDevice(Device):
    """Implements a SPI Controller."""

    # SPIDevice.__init__():
    def __init__(self, name: Text, transfer_size: int,
                 sck_phase: bool, sck_polarity: bool, sck_width: int,
                 ss_idle: bool, ss_prelude: int, ss_postlude: int, mosi_idle: bool,
                 tracing: Text = "") -> None:
        """Initialize a SPIDevice.

        Args:
            name (Text): The hopefully unique name of the SPIDevice.
            transfer_size (int): The number of bits to transfer (usually 8, 16, or 32.)
            sck_phase (bool): True for clock aligned data and False for data skewed forward.
            sck_polarity (bool): The idle level for the SCK line.
            mosi_idle (bool): The idle level of the MOSI output at when no data is present.
            sck_width (int): The number of clock ticks per clock. Must be even and >= 4.
            ss_idle (bool): The level to idle the ss ouput at when not asserted.
            ss_prelude (int): The number of clock ticks for before the first SCK pulse.
            ss_postlude (int): The number of clock ticks for after the end of the SCK pulse.
            mosi_idle (bool): The MOSI output when no data is present.
        """
        # Initialize *Device* base type:
        if tracing:
            print(f"{tracing}=>SPIDevice(...)")

        super().__init__(name)

        # Do some argument sanity checks:
        assert transfer_size in (8, 16, 32), f"{transfer_size=} must be 8, 16, or 32."
        assert sck_width >= 4 and (sck_width & 1) == 0, f"{sck_width=} must be even and >= 4."
        assert ss_prelude >= 2, f"{ss_prelude=} must be >= 2."
        assert ss_postlude >= 2, f"{ss_prelude=} must be >= 2."

        # Clock pulse is *sck_width* clock ticks long must always be an odd number of clock ticks.
        # Three points are defined for each clock tick -- the beginning, middle and end.
        # Thus, the edge time offset are at index 0, middle at *sck_width* << 1,
        # and end at *sck_width* - 1.

        # A full SPI transfer is broken into three segements:
        # * Prelude:
        #   The prelude is at the beginning and is *ss_prelude* clock ticks wide.
        # * Interlude:
        #   The interlude section consists of *transfer_size* sections that are each
        #   *sck_width* clock ticks wide.
        # * Postlude:
        #   The postlude section is at the end and is *ss_postlude* clock ticks wide.
        # The start and ends times are defined below:
        prelude_start: int = 0
        prelude_end: int = prelude_start + ss_prelude - 1
        interlude_start: int = prelude_end + 1

        # There are *transfer_size* clock ticks.  Each clock tick has begin, center, and end,
        # For each bit, the start, sample, and end times are computed.
        # The *sck_phase* controls when the data is put on the *mosi* line.
        # * *sck_phase* == *True*:
        #   The data is largely aligned with clock.  The exception is that last bit
        #   stays on until the *ss* input is deasserted.
        # * *sck_phase* == *False*:
        #   The data is skewed forward.  Each bit starts half way through the previous
        #   clock cycle.  The first bit starts immediately when the *ss* input is asserted.

        # The clock timess specify exactly when the clock is asserted:
        clock_start_times: List[int] = []
        clock_end_times: List[int] = []

        # The bit times specify when the data starts, ends, and is sampled:
        bit_start_times: List[int] = []
        bit_sample_times: List[int] = []
        bit_end_times: List[int] = []

        # Fill in the bit and clock time tables:
        clock_end: int = sck_width >> 1
        clock_end = clock_end - 1
        for bit_index in range(transfer_size):
            # We compute 4 times for each bit index.  The 4 values are:
            # * *before*: This is in the middle of the previous clock cycle.
            #   The value for computed for *before* is both bogus and unused when *bit_index* is 0:
            # * *start*: This is the first time the clock is asserted
            # * *middle*: This is the last time the clock is asserted.
            #   Note that *middle* + 1 is when the data gets sampled for one of the phases.
            # * *end*: This is at the end of the current clock cycle.
            before: int = interlude_start + (bit_index - 1) * sck_width + clock_end
            start: int = interlude_start + bit_index * sck_width
            middle: int = start + clock_end
            end: int = start + sck_width - 1
            if tracing:
                print(f"{tracing}[{bit_index}]:{before=} {start=} {middle=} {end=}")

            # Clock assertion times are simple:
            clock_start_time: int = start
            clock_end_time: int = middle

            # The bit start/sample/end times depend on the *sck_phase*:
            bit_start_time: int = start if sck_phase else before + 1
            bit_sample_time: int = (middle + 1 if sck_phase else start)
            bit_end_time: int = end if sck_phase else middle

            # Do some sanity checking:
            assert bit_start_time < bit_sample_time < bit_end_time, (
                f"{bit_start_time=} < {bit_sample_time=} < {bit_end_time=} failed")
            assert clock_start_time < clock_end_time, (
                f"{clock_start_time=} < {clock_end_time=}")

            # Stuff values into edges lists:
            clock_start_times.append(clock_start_time)
            clock_end_times.append(clock_end_time)
            bit_start_times.append(bit_start_time)
            bit_sample_times.append(bit_sample_time)
            bit_end_times.append(bit_end_time)

        # Now stuff everything into *spi_device* (i.e. *self*):
        # spi_device: SPI_Device = self
        self.name: Text = name
        self.transfer_size: int = transfer_size
        self.sck_phase: bool = sck_polarity
        self.sck_polarity: bool = sck_polarity
        self.sck_width: int = sck_width
        self.ss_idle: bool = ss_idle
        self.ss_prelude: int = ss_prelude
        self.ss_postlude: int = ss_postlude
        self.mosi_idle: bool = mosi_idle
        self.miso: Wire = Wire("")
        self.mosi: Wire = Wire("")
        self.sck: Wire = Wire("")
        self.ss: Wire = Wire("")
        self.clock_start_times: List[int] = clock_start_times
        self.clock_end_times: List[int] = clock_end_times
        self.bit_start_times: List[int] = bit_start_times
        self.bit_sample_times: List[int] = bit_sample_times
        self.bit_end_times: List[int] = bit_end_times
        self.timed_transfers: Tuple[Tuple[int, int], ...] = ()
        self.timed_transfers_index: int = 0  # The next timed transfer.
        self.transfer_duration: int = ss_prelude + transfer_size * sck_width + ss_postlude
        self.bit_index: int = 0  # The last bit index used
        self.miso_data: int = -1
        self.input_value: int = -1  # Current input value
        self.input_values: List[int] = []  # Collected input values

        if tracing:
            print(f"{tracing}<=SPIDevice(...)")

    # SPIDevice.bind():
    def bind(self, ss: Wire, mosi: Wire, miso: Wire, sck: Wire,
             timed_transfers: Tuple[Tuple[int, int], ...]) -> None:
        """Bind wires to SPIDevice."""
        spi_device: SPIDevice = self
        # Stuff some values into *spi_device* (i.e. *self*):
        # Stuff bind values into *spi_device* (i.e. *self*):
        spi_device.ss = ss
        spi_device.miso = miso
        spi_device.mosi = mosi
        spi_device.sck = sck
        spi_device.timed_transfers = timed_transfers

        # Register *input* and *output* with the *Device* base class:
        inputs: Wires = (miso,)
        outputs: Wires = (ss, mosi, sck)
        super().inputs_outputs_bind(inputs, outputs)

    # SPIDevice.step():
    def step(self, time: int):
        """Step the SPIDevcie."""
        tracing: Text = ""
        if tracing:
            print(f"{tracing}=>SPIDevice.step({time})")

        # Grab some values from *spi_device* (i.e. *self*):
        spi_device: SPIDevice = self
        transfer_size: int = spi_device.transfer_size
        sck_polarity: bool = spi_device.sck_polarity
        ss_idle: bool = spi_device.ss_idle
        mosi_idle: bool = spi_device.mosi_idle
        miso: Wire = spi_device.miso
        mosi: Wire = spi_device.mosi
        sck: Wire = spi_device.sck
        ss: Wire = spi_device.ss
        clock_start_times: List[int] = spi_device.clock_start_times
        clock_end_times: List[int] = spi_device.clock_end_times
        bit_start_times: List[int] = spi_device.bit_start_times
        bit_sample_times: List[int] = spi_device.bit_sample_times
        bit_end_times: List[int] = spi_device.bit_end_times
        timed_transfers: Tuple[Tuple[int, int], ...] = spi_device.timed_transfers
        timed_transfers_index: int = spi_device.timed_transfers_index
        transfer_duration: int = spi_device.transfer_duration
        bit_index: int = spi_device.bit_index

        # Initialize the associated value values with the idle values:
        mosi_value: bool = mosi_idle
        sck_value: bool = sck_polarity
        ss_value: bool = ss_idle

        # Advance to the next timed transfer if the current one is done:
        timed_transfer_start: int = 0xffffffff  # Large time value that will not be reached.
        timed_transfer_data: int = 0
        while timed_transfers_index < len(timed_transfers):
            timed_transfer_start, timed_transfer_data = timed_transfers[timed_transfers_index]
            if time > timed_transfer_start + transfer_duration:
                timed_transfers_index += 1
                spi_device.timed_transfers_index = timed_transfers_index
            else:
                break

        # Determine if we are in an active timed transfer:
        if timed_transfer_start <= time < timed_transfer_start + transfer_duration:
            # We are in an active transfer cycle:
            relative_time: int = time - timed_transfer_start
            if tracing:
                print(f"{tracing}Transfer[{timed_transfers_index}]: "
                      f"{time=} {relative_time=} {timed_transfer_start=} {timed_transfer_data}")

            # First assert *ss_value*, where `not ss_idle` means asserted:
            ss_value = not ss_idle  # Not idle => asserted

            # Make sure the *bit_index* points to the closest bit to *relative_time*:
            assert 0 <= bit_index < transfer_size, "{bit_index=} is bad. {transfer_size=}"
            while bit_index > 0 and relative_time < bit_start_times[bit_index]:
                bit_index -= 1
                spi_device.bit_index = bit_index
            while bit_index + 1 < transfer_size and relative_time > bit_end_times[bit_index]:
                bit_index += 1
                spi_device.bit_index = bit_index
            assert 0 <= bit_index < transfer_size, (
                f"{time}: SPIDevice({spi_device.name}): Bad {bit_index=}")

            # Grab the appropriate clock/bit values:
            clock_start_time: int = clock_start_times[bit_index]
            clock_end_time: int = clock_end_times[bit_index]
            bit_start_time: int = bit_start_times[bit_index]
            bit_sample_time: int = bit_sample_times[bit_index]
            bit_end_time: int = bit_end_times[bit_index]

            # Ensure we are in interlude and not prelude/postlude:
            if bit_start_time <= relative_time <= bit_end_time:
                # Assert the clock if appropriate:
                if clock_start_time <= relative_time <= clock_end_time:
                    # *sck_polarity* specifies the idle value, hence the `not` used for assertion:
                    sck_value = not sck_polarity

                # Specify the output value:
                if bit_start_time <= relative_time <= bit_end_time:
                    mosi_value = bool((timed_transfer_data >> bit_index) & 1)

                # Sample the input value:
                if relative_time == bit_sample_time:
                    input_value: int = 0 if bit_index == 0 else spi_device.input_value
                    miso_value: bool = miso[time]
                    shift: int = bit_index  # transfer_size - 1 - bit_index
                    assert input_value >= 0, (
                        f"{time}: SPIDevice({spi_device.name}): Bad {input_value=} {bit_index=}")
                    print(f"{time}: SPIDevice: MISO[{shift}] = {miso_value}")
                    if miso_value:
                        input_value |= (1 << shift)
                    spi_device.input_value = input_value
                    if bit_index + 1 == transfer_size:
                        spi_device.input_values.append(input_value)
                        spi_device.input_value = -1  # Make bogus until next one comes along.

        # Set the output values:
        next: int = time + 1
        mosi[next] = mosi_value
        sck[next] = sck_value
        ss[next] = ss_value

        if tracing:
            print(f"{tracing}<=SPIDevice.step({time})")


# Circuit:
class Circuit:
    """Represents a Circuit of Devices and Wires."""

    # Circuit.__init__():
    def __init__(self, name: Text, devices: Tuple[Device, ...], traced_wires: Wires) -> None:
        """Initialize a Circuit."""
        self.name: Text = name
        self.devices: Tuple[Device, ...] = devices
        self.traced_wires: Wires = traced_wires

    # Circuit.run():
    def run(self, step_count: int, total_width: int, tracing: Text = "") -> Text:
        """Run a Circuit simulation for a number of steps."""
        if tracing:
            print(f"{tracing}=>Circuit.run(*)")

        # Unpack some values from *circuit* (i.e. *self*):
        circuit: Circuit = self
        devices: Tuple[Device, ...] = circuit.devices
        traced_wires: Wires = circuit.traced_wires

        # Compute the maximum signal name length:
        max_header_width: int = 0
        traced_wire: Wire
        for traced_wire in traced_wires:
            max_header_width = max(max_header_width, len(traced_wire.name))
        max_header_width += 1  # for the ":" on the end
        chunks_per_line: int = int((total_width - max_header_width) / 11)

        # Compute the headers:
        headers: List[Text] = []
        for traced_wire in traced_wires:
            name: Text = traced_wire.name
            name_size: int = len(name)
            padding: Text = ' ' * (max_header_width - name_size)
            header: Text = f"{padding}{name}:"
            headers.append(header)

        # Time marker lines:
        headers.append(max_header_width * ' ' + '|')
        headers.append(max_header_width * ' ' + '|')
        headers.append(max_header_width * ' ' + '|')

        # Steps are done in chunks of 10 steps, with a space inserted.
        # So each 10 steps generates 11 characters of output.
        all_lines: List[Text] = []
        chunk_lines: List[Text] = []
        chunk_index: int = 0
        index: int
        for time in range(step_count):
            # Deal with headers and inserting a space every tenth step.
            if time % 10 == 0:
                if chunk_index % chunks_per_line == 0:
                    # Flush the current *chunk_lines* to *all_lines* and initialize with *headers*:
                    all_lines.extend(chunk_lines)
                    all_lines.append("")  # Put a blank line between chunks
                    chunk_lines = headers[:]  # Copy the headers over.
                chunk_index += 1

                # Every 10th step emit a space in the trace:
                for index in range(len(headers)):
                    chunk_lines[index] += ' '

            # Append all of the *traced_wires* values to *chunk_lines*:
            for index, traced_wire in enumerate(traced_wires):
                chunk_lines[index] += 'X' if traced_wire[time] else '.'
            time_index: int = len(traced_wires)
            chunk_lines[time_index] += f"{int(time/100) % 10}"
            chunk_lines[time_index + 1] += f"{int(time/10) % 10}"
            chunk_lines[time_index + 2] += f"{time % 10}"

            # Now step all of the devices:
            device: Device
            for device in devices:
                device.step(time)

        # Now join all of the lines together:
        all_lines.extend(chunk_lines)
        joined_lines: Text = '\n'.join(all_lines)

        if tracing:
            print(f"{tracing}<=Circuit.run(*)=>*")
        return joined_lines


# SPICircuit:
class SPICircuit1(Circuit):
    """Represents a simple SPI circuit."""

    # SPICircuit1.__init__():
    def __init__(self) -> None:
        """Initialize the SPICircuit1."""
        # Create *devices*:
        high_source: HighSource = HighSource()
        low_source: LowSource = LowSource()
        spi_device: SPIDevice = SPIDevice("SPI", 8, True, True, 4, True, 4, 4, False)
        x595: X595 = X595("X595")
        ss_inverter: Inverter = Inverter("NSS_TO_SS")
        x165: X165 = X165("X165")
        devices: Tuple[Device, ...] = (spi_device, ss_inverter, x595, x165,
                                       high_source, low_source)

        # Create *wires*:
        hi: Wire = Wire("HI")
        lo: Wire = Wire("LO")
        miso: Wire = Wire("MISO")
        mosi: Wire = Wire("MOSI")
        nss: Wire = Wire("NSS")
        ss: Wire = Wire("SS")
        sck: Wire = Wire("SCK")
        qa: Wire = Wire("QA")
        qb: Wire = Wire("QB")
        qc: Wire = Wire("QC")
        qd: Wire = Wire("QD")
        qe: Wire = Wire("QE")
        qf: Wire = Wire("QF")
        qg: Wire = Wire("QG")
        qh: Wire = Wire("QH")
        nc1: Wire = Wire("NC1")
        nc2: Wire = Wire("NC1")
        wires: Wires = (qa, qb, qc, qd, qe, qf, qg, qh, sck, nss, ss, mosi, miso)

        # Bind the *devices* to the *wires*:
        spi_device.bind(nss, mosi, miso, sck, ((9, 0x99),))
        high_source.bind(hi)
        low_source.bind(lo)
        x595.bind(mosi, sck, nss, hi, lo, qa, qb, qc, qd, qe, qf, qg, qh, nc1)
        ss_inverter.bind(nss, ss)
        x165.bind(ss, lo, sck, lo,
                  hi, lo, lo, hi, hi, lo, lo, hi,
                  miso, nc2)

        # Intialize the Circuit base clases:
        super().__init__("SPI Circuit", devices, wires)
        self.spi_device: SPIDevice = spi_device

    # SPICircuit1.wrap_up():
    def wrap_up(self) -> None:
        """Wrap up the simulation."""
        spi_circuit1: SPICircuit1 = self
        spi_device: SPIDevice = spi_circuit1.spi_device
        input_values: List[int] = spi_device.input_values
        index: int
        input_value: int
        for index, input_value in enumerate(input_values):
            print(f"Input Value[{index}]=0x{input_value:02x}")


# SPICircuit2:
class SPICircuit2(Circuit):
    """Represents a more complex SPI circuit."""

    # SPICircuit2.__init__():
    def __init__(self) -> None:
        """Initialize the SPI circuit."""
        # Create *devices*:
        high_source: HighSource = HighSource()
        low_source: LowSource = LowSource()
        spi_device: SPIDevice = SPIDevice("SPI", 16, True, True, 4, True, 4, 4, False)
        x595a: X595 = X595("X595A")
        x595b: X595 = X595("X595B")
        ss_inverter: Inverter = Inverter("NSS_TO_SS")
        x165a: X165 = X165("X165A")
        x165b: X165 = X165("X165A")
        devices: Tuple[Device, ...] = (spi_device, ss_inverter, x595a, x595b, x165a, x165b,
                                       high_source, low_source)

        # Create *wires*:
        hi: Wire = Wire("HI")
        lo: Wire = Wire("LO")
        miso: Wire = Wire("MISO")
        mosi: Wire = Wire("MOSI")
        nss: Wire = Wire("NSS")
        ss: Wire = Wire("SS")
        sck: Wire = Wire("SCK")

        b0: Wire = Wire("B0")
        b1: Wire = Wire("B1")
        b2: Wire = Wire("B2")
        b3: Wire = Wire("B3")
        b4: Wire = Wire("B4")
        b5: Wire = Wire("B5")
        b6: Wire = Wire("B6")
        b7: Wire = Wire("B7")
        b8: Wire = Wire("B8")
        b9: Wire = Wire("B9")
        b10: Wire = Wire("B10")
        b11: Wire = Wire("B11")
        b12: Wire = Wire("B12")
        b13: Wire = Wire("B13")
        b14: Wire = Wire("B14")
        b15: Wire = Wire("B15")
        # nc1: Wire = Wire("NC1")
        nc2: Wire = Wire("NC1")
        nc3: Wire = Wire("NC1")
        nc4: Wire = Wire("NC1")
        bridge165: Wire = Wire("BRIDGE165")
        bridge595: Wire = Wire("BRIDGEX595")
        wires: Wires = (
            sck, nss, ss, mosi, miso,
            b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13, b14, b15,
        )

        # Bind the *devices* to the *wires*:
        high_source.bind(hi)
        low_source.bind(lo)
        spi_device.bind(nss, mosi, miso, sck, ((9, 0x1248), (99, 0)))
        ss_inverter.bind(nss, ss)
        x595a.bind(mosi, sck, nss, hi, lo, b0, b1, b2, b3, b4, b5, b6, b7, bridge595)
        x595b.bind(bridge595, sck, nss, hi, lo, b8, b9, b10, b11, b12, b13, b14, b15, nc2)
        x165a.bind(ss, lo, sck, lo,
                   b0, b1, b2, b3, b4, b5, b6, b7,
                   bridge165, nc3)
        x165b.bind(ss, bridge165, sck, lo,
                   b8, b9, b10, b11, b12, b13, b14, b15,
                   miso, nc4)

        # Intialize the Circuit base clases:
        super().__init__("SPI Circuit", devices, wires)
        self.spi_device: SPIDevice = spi_device

    # SPICircuit2.wrap_up():
    def wrap_up(self) -> None:
        """Wrap up the simulation."""
        spi_circuit2: SPICircuit2 = self
        spi_device: SPIDevice = spi_circuit2.spi_device
        input_values: List[int] = spi_device.input_values
        index: int
        input_value: int
        for index, input_value in enumerate(input_values):
            print(f"Input Value[{index}]=0x{input_value:02x}")


def main():
    """Run the simulator."""
    print("Start simulation")
    # spi_circuit1: SPICircuit1 = SPICircuit1()
    spi_circuit2: SPICircuit2 = SPICircuit2()
    results: Text = spi_circuit2.run(179, 110)
    print(results)
    spi_circuit2.wrap_up()

    print("Simulation Done")


if __name__ == "__main__":
    print("Hello")
    main()
