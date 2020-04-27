.PHONY: all clean everything

KICUBE32 := kicube32

KIPARTS_DIRECTORY := kiparts

F767ZI_IOC_CSV := f767zi.ioc f767zi.csv
F767ZI_CSV := $(KIPARTS_DIRECTORY)/f767zi.kipart.csv
MASTER_CSV := $(KIPARTS_DIRECTORY)/master_board.csv
MASTER_LIB := master_board.lib
MASTER_DCM := master_board.dcm

KIPART_BASES := \
    hcsr04	\
    f767zi	\
    st_link_cn2	\
    st_link_cn3
KIPART_CSVS := ${KIPART_BASES:%=$(KIPARTS_DIRECTORY)/%.kipart.csv}

all: $(MASTER_LIB)

$(F767ZI_CSV): ${KIPART_IOC_CSV}
	echo PWD=`pwd`
	$(KICUBE32) ${F767ZI_IOC_CSV} $(F767ZI_CSV)

$(MASTER_LIB): ${KIPART_CSVS}
	cat ${KIPART_CSVS} > $(MASTER_CSV)
	kipart --fill bg_fill --overwrite -o $(MASTER_LIB) $(MASTER_CSV)
	kidocgen $(KIPARTS_DIRECTORY) $(MASTER_DCM)

clean:
	rm -f $(F767ZI_CSV)
	rm -f $(MASTER_CSV)
	rm -f $(MASTER_LIB)
	rm -f $(MASTER_DCM)

