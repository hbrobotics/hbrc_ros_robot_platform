.PHONY: all clean everything

KIPARTS_DIRECTORY := kiparts

KICUBE32 := kicube32

KIPART_CSVS := \
    $(KIPARTS_DIRECTORY)/f767zi.kipart_csv

all: $(KIPART_CSVS)

$(KIPARTS_DIRECTORY)/f767zi.kipart_csv: f767zi.ioc f767zi.csv
	echo PWD=`pwd`
	$(KICUBE32) f767zi.ioc f767zi.csv $@

clean:
	rm -f ${KIPART_CSVS}


%.lib: 
