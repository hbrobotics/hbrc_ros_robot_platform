.PHONY: all clean everthinga

KICUBE32_HOME := $(PROJECT_HOME)/kicube32
KICUBE32_PY := $(KICUBE32_HOME)/kicube32/kicube32.py
KICUBE32_HR2 := $(HOME)/.virtualenvs/hr2/bin/kicube32

BOARD_DIRECTORIES := \
    master_board/rev_a

all: $(KICUBE32_HR)
	@echo "****************"
	ls -l $(KICUBE32_PY)
	ls -l $(KICUBE32_HR2)
	@echo "================"
	for d in ${BOARD_DIRECTORIES} ; do (cd $$d ; echo $$d ; $(MAKE) all) ; done

foo:
	touch $(KICUBE32_PY)

everything:
	for d in ${BOARD_DIRECTORIES} ; do (cd $$d ; echo a$(MAKE) everything) ; done

clean:
	for d in ${BOARD_DIRECTORIES} ; do (cd $$d ; $(MAKE) clean) ; done

$(KICUBE32_HR2): $(KICUBE32_PY)
	@echo '>>>>>>>>>>>>>>>>'
	(cd $(KICUBE32_HOME) ; echo "++++" ; $(MAKE) ; echo "----")
	echo '================'
	$(PIP_INSTALL) $(KICUBE32_HOME)
	@echo '<<<<<<<<<<<<<<<<'

