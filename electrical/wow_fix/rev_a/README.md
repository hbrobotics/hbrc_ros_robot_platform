# LD06 Lidar Adapter

The LD06 is an inexpensive and small LIDAR produced by
[LDROBOT](https://www.ldrobot.com/en).
It is available in the United State via
[Amazon](https://www.amazon.com/innomaker-Omni-Directional-Scanning-Resistance-Ranging/dp/B08GJJX41D/ref=sr_1_54)
for approximately $100 (as of late December 2020.)
There is [techinical documenation](https://www.inno-maker.com/product/lidar-ld06/) available.

The adpater board converts from the generic HR2 daughter board format
to the 4-pin connector used by the LD06.
The host microcontroller is responsible for providing a PWM signal and
a UART to listen for the returned data stream from the LD06.
