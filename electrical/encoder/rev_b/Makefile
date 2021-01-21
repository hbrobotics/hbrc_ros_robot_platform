# This file is licensed using the "MIT License" below:
#
# ##################################################################################################
#
# MIT License
#
# Copyright 2019, 2020 Home Brew Robotics Club
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
#
# ##################################################################################################
# <======================================= 100 characters =======================================> #

.PHONY: all clean everything

ORDER := order2
ORDERS_DIRECTORY := ../../orders/$(ORDER)
KIPARTS_DIRECTORY := $(ORDERS_DIRECTORY)/kiparts

ORDER_CSV := $(KIPARTS_DIRECTORY)/$(ORDER).csv
ORDER_LIB := $(ORDERS_DIRECTORY)/$(ORDER).lib
ORDER_DCM := $(ORDERS_DIRECTORY)/$(ORDER).dcm

KIPART_BASES := \
    ENCODER_2xM1x3RA
KIPART_CSVS := ${KIPART_BASES:%=$(KIPARTS_DIRECTORY)/%.kipart.csv}

all: $(ORDER_LIB)

$(ORDER_LIB): ${KIPART_CSVS}
	(cd $(ORDERS_DIRECTORY) ; $(MAKE) )

foo:
	echo KIPART_CSVS=$(KIPART_CSVS)
	echo ORDER_DIRECTORY=$(ORDERS_DIRECTORY)

clean:
	(cd $(ORDERS_DIRECTORY) ; $(MAKE) clean)

