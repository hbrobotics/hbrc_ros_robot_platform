.PHONY: all

BOARD_DIRECTORIES = \
    master_board/rev_a

all:
	@for d in ${BOARD_DIRECTORIES} ; do	\
	    (cd $$d ; make all) ;		\
	done
