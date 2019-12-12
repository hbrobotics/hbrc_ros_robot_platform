SCAD_MODELS_DIRECTORY := scad_models
SCAD_MODELS_PY_FILES :=				\
    $(SCAD_MODELS_DIRECTORY)/__init__.py	\
    $(SCAD_MODELS_DIRECTORY)/scad.py		\
    $(SCAD_MODELS_DIRECTORY)/scad_models.py
SCAD_MODELS_PYL_FILES := ${SCAD_MODELS_PY_FILES:%.py=%.pyl}
SCAD_MODELS_COVER_FILES := ${SCAD_MODELS_PY_FILES:%=%,cover}

TESTS_DIRECTORY := tests
TESTS_PY_FILES :=				\
    $(TESTS_DIRECTORY)/test_scad_models.py	\
    $(TESTS_DIRECTORY)/test_scad.py
TESTS_PYL_FILES := ${TESTS_PY_FILES:%.py=%.pyl}
TESTS_COVER_FILES := ${TESTS_PY_FILES:%=%,cover}

CLEAN_DIRECTORIES :=				\
    .mypy_cache					\
    .pytest_cache				\
    scad_models/__pycache__			\
    tests/__pycache__

ALL_PY_FILES :=					\
    ${SCAD_MODELS_PY_FILES}			\
    ${TESTS_PY_FILES}
ALL_PYL_FILES :=				\
    ${SCAD_MODELS_PYL_FILES}			\
    ${TESTS_PYL_FILES}
ALL_COVER_FILES :=				\
    ${SCAD_MODELS_COVER_FILES}			\
    ${TESTS_COVER_FILES}

.PHONY: all clean test

all: ${ALL_PYL_FILES}
	pip install .
	rm -f ${SCAD_FILES}
	pip install .

clean:
	rm -f ${ALL_PYL_FILES} ${ALL_COVER_FILES} .coverage
	rm -rf ${CLEAN_DIRECTORIES}

ETC_IMAGEMAGIC6_DIRECTORY := 
IM6_DIR := /etc/ImageMagick-6
PDF_FILE_BASES := 				\
    romi-chasis-expansion-plate-dimensions	\
    romi-chasis-expansion-plate-hole-alignment

images:
	if [ ! -f $(IM6_DIR)/policy-old.xml ] ;						\
	   then	if [ ! -f $(IM6_DIR)/policy-old.xml ] ;					\
		   then	echo "We need to install ImageMagik 6." ;			\
		   	echo "Running sudo apt install imagemagick6" ;			\
		   	echo "Please provide root password:" ;				\
			sudo apt install imagemagick6 ;					\
		fi ;									\
		echo "We need to rename the $(IM6_DIR)/policy.xml file." ;		\
		echo "Please provide root password (if requested):" ;			\
                echo "Runnng sudo mv $(IM6_DIR)/polcy.xml $(IM6_DIR)/policy-old.xml" ;	\
		sudo mv $(IM6_DIR)/polcy.xml $(IM6_DIR)/policy-old.xml ;		\
           fi
	for base in ${PDF_FILE_BASES} ; do 						\
		echo "Producing png/$$base.png" ;					\
		convert -density 100 -trim pdf/$$base.pdf -quality 100 png/$$base.png ;	\
	done



# The documentation for `pytest` and `pytest-cov` is a bit weak.  None of this makes
# much sense and was arrived at by tedious trial and error.  There may be some strange
# interaction between `pytest` and Python Virtual Environments that is causing all of
# this strangeness.  Altenatively, this might just be poor integration between the
# overall `pytest` package and the `pytest-cov` plugin for `pytest`.  Extensive web
# searching did not uncover any real hints.
#
# We specify:
# * `-cov=DIR`: for each directory that contain `.py` files to generate coverate for.
# * `--cov-append` makes sure that each test run appends to the overal coverage rather
#   than reseting `.coverage` file after every test.
# * `--cov_report annoate` generates a list of `*.py,cover` files.  Statements that
#   have no coverate will be preceeded by an exclaimation point (`!`).  These lines
#   can be `grep`'ed for to show un-executed statements.  `# pragma: no cover` is attached
#   to debugging and error statements for which no coverage is expected.
# * `*.py`: explicity list each `.py` file that is to be covered.  It is really unclear
#   why `-cov=DIR` does not do this; but it does not.
# * `tests`: The directory containing the `test_*.py` test files.  These files are
#   have individual `test_*()` functions that are exectutedy by `pytest`.
# What a total pain in the tush!
test: all
	rm -f ${ALL_COVER_FILES} .coverage
	pytest --cov=scad_models --cov=tests --cov-append --cov-report=annotate \
	    ${ALL_PY_FILES} tests
	@grep -H -n "^!" ${ALL_COVER_FILES} || true

# Pattern rule for running `mypy`, `flake8`, and `pydocstyle` over a `.py` Python file.
# The `.pyl` suffix is used to remember that the linting has occured:
%.pyl: %.py
	mypy --no-color-output       $<
	flake8 --max-line-length=100 $<
	pydocstyle                   $<
	touch $@



