.PHONY: all clean everything

SUB_DIRECTORIES :=	\
    electrical		\
    mechanical

all:
	for d in ${SUB_DIRECTORIES} ; do (cd $$d ; $(MAKE) all) ; done

clean:
	for d in ${SUB_DIRECTORIES} ; do (cd $$d ; $(MAKE) clean); done

everything:
	for d in $(SUB_DIRECTORIES) ; do (cd $$d ; $(MAKE) everything) ; done
