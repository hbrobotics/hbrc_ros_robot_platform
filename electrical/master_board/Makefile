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

IOC_DIR := rev_a/hr2
IOC_CSV := $(IOC_DIR)/hr2.ioc
F767ZI_DIR := rev_a
F767ZI_CSV := $(F767ZI_DIR)/f767zi.csv
KIPART_DIR := ../orders/order1/kiparts
KIPART_CSV := $(KIPART_DIR)/NUCLEO_F767ZI_2xF2x35.kipart.csv

all:
	mypy pinsbind.py
	flake8 --max-line-length=100 pinsbind.py
	pydocstyle pinsbind.py
	python pinsbind.py
	kicube32 $(IOC_CSV) $(F767ZI_CSV) $(KIPART_CSV)

