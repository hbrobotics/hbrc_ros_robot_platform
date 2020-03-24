.PHONY: all clean everthinga

BOARD_DIRECTORIES = \
    master_board/rev_a

all:
	for d in ${BOARD_DIRECTORIES} ; do (cd $$d ; $(MAKE) all) ; done

everything:
	for d in ${BOARD_DIRECTORIES} ; do (cd $$d ; $(MAKE) everything) ; done

clean:
	for d in ${BOARD_DIRECTORIES} ; do (cd $$d ; $(MAKE) clean) ; done
