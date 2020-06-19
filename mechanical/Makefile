# Virtual Environment stuff.  Note that `VIRTUAL_ENV` must be defined as an envrionment
# variable by your virtual environments wrapper:
PYTHON_VERSION := python3.6
VIRTUAL_ENV_SCAD_MODELS_DIRECTORY := $(VIRTUAL_ENV)/lib/$(PYTHON_VERSION)/site-packages/scad_models

# Define the source SCAD Modles `.py` files and their installed location:
SCAD_MODELS_BASE_PY_FILES :=			\
    __init__.py					\
    scad.py					\
    hr2_models.py
SCAD_MODELS_DIRECTORY := scad_models
SCAD_MODELS_PY_FILES := ${SCAD_MODELS_BASE_PY_FILES:%=$(SCAD_MODELS_DIRECTORY)/%}
INSTALLED_SCAD_MODELS_PY_FILES := 						\
    ${SCAD_MODELS_BASE_PY_FILES:%=$(VIRTUAL_ENV_SCAD_MODELS_DIRECTORY)/%}
SCAD_MODELS_PYL_FILES := ${SCAD_MODELS_PY_FILES:%.py=%.pyl}
SCAD_MODELS_COVER_FILES := ${SCAD_MODELS_PY_FILES:%=%,cover}

# The `tests` directory is where all of the Python unit tests live:
TESTS_DIRECTORY := tests
TESTS_PY_FILES :=				\
    $(TESTS_DIRECTORY)/test_hr2_models.py	\
    $(TESTS_DIRECTORY)/test_scad.py
TESTS_PYL_FILES := ${TESTS_PY_FILES:%.py=%.pyl}
TESTS_COVER_FILES := ${TESTS_PY_FILES:%=%,cover}

# All of the Python (`.py`), Python lint (`.pyl`) and test coverage (`,cover`) files:
ALL_PY_FILES :=					\
    ${SCAD_MODELS_PY_FILES}			\
    ${TESTS_PY_FILES}
ALL_PYL_FILES :=				\
    ${SCAD_MODELS_PYL_FILES}			\
    ${TESTS_PYL_FILES}
ALL_COVER_FILES :=				\
    ${SCAD_MODELS_COVER_FILES}			\
    ${TESTS_COVER_FILES}

# These are the various `.dxf` directories and files:
DXF_DIRECTORY := dxf
DXF_FILES :=					\
    $(DXF_DIRECTORY)/encoder_pcb.dxf		\
    $(DXF_DIRECTORY)/expansion_flat.dxf		\
    $(DXF_DIRECTORY)/master_pcb.dxf		\
    $(DXF_DIRECTORY)/nucleo144_pcb.dxf		\
    $(DXF_DIRECTORY)/other_pi_pcb.dxf		\
    $(DXF_DIRECTORY)/raspi3_pcb.dxf		\
    $(DXF_DIRECTORY)/romi_base.dxf		\
    $(DXF_DIRECTORY)/stlink_pcb.dxf

# These are the various `.png` file definitions and flags:
# PNG_CLEAN := # exiftool -all=
PNG_CLEAN := echo
PNG_TOP_FLAGS := --imgsize 1024,1024 --projection ortho --camera=0,0,425,0,0,0
PNG_TOP_THUMB_FLAGS := --imgsize 128,128 --projection ortho --camera=0,0,425,0,0,0
PNG_ANGLE_FLAGS := --imgsize 1024,1024 --projection perspective --camera=300,-300,300,0,0,0
PNG_ANGLE_THUMB_FLAGS := --imgsize 128,128 --projection perspective --camera=300,-300,300,0,0,0
PNG_DIRECTORY := png
PNG_BASE_NAMES :=				\
    hr2_base_assembly				\
    hr2_pi_assembly				\
    hr2_master_assembly				\
    hr2_wheel_assembly				\
    hr2_nucleo_assembly
PNG_FILES :=						\
    ${PNG_BASE_NAMES:%=$(PNG_DIRECTORY)/%.png}		\
    ${PNG_BASE_NAMES:%=$(PNG_DIRECTORY)/thumb_%.png}

# These are the generated `.scad` files that should be removed for the `clean` target:
SCAD_FILES :=					\
    rectangular_connector.scad			\
    hr2_models.scad

# These are the directories that are entirely removed for the `clean` target:
CLEAN_DIRECTORIES :=				\
    .mypy_cache					\
    .pytest_cache				\
    scad_models/__pycache__			\
    tests/__pycache__

.PHONY: all clean everything test dxf_files

# This is the top level target that builds everything.  It uses sub-targets to
# force things to be built in the "correct* order:
all: ${INSTALLED_SCAD_MODELS_PY_FILES} hr2_models.scad # ${DXF_FILES} README.html # ${PNG_FILES}

everything: all

# This is the target the generates the `.dxf` files from the OpenSCAD files:
${DXF_FILES}: ${INSTALLED_SCAD_MODELS_PY_FILES}

# This is the target that forces all of the code to be pushed through the
# Python linting programs `flake8`, `mypy`, and `pydocstyle`.  Each time
# the code succeeds it leaves behing a `.pyl` file to indicate that it succeeded:
${INSTALLED_SCAD_MODELS_PY_FILES}: ${ALL_PYL_FILES}

# This forces all `.py` files in the `scad_models` package to be installed in
# the Python virtual environment:
${INSTALLED_SCAD_MODELS_PY_FILES}: ${SCAD_MODELS_PY_FILES}
	pip install .

# Individual targets for generating `.png` files:
$(PNG_DIRECTORY)/expansion.png: hr2_models.scad
	openscad hr2_models.scad -D 'name="expansion"' $(PNG_TOP_FLAGS) -o $@
	$(PNG_CLEAN) $@
$(PNG_DIRECTORY)/romi_base.png: hr2_models.scad
	openscad hr2_models.scad -D 'name="romi_base"' $(PNG_TOP_FLAGS) -o $@
	$(PNG_CLEAN) $@
$(PNG_DIRECTORY)/hr2_base_assembly.png: hr2_models.scad
	openscad hr2_models.scad -D 'name="hr2_base_assembly"' $(PNG_ANGLE_FLAGS) -o $@
	$(PNG_CLEAN) $@
$(PNG_DIRECTORY)/hr2_pi_assembly.png: hr2_models.scad
	openscad hr2_models.scad -D 'name="hr2_pi_assembly"' $(PNG_ANGLE_FLAGS) -o $@
	$(PNG_CLEAN) $@
$(PNG_DIRECTORY)/hr2_master_assembly.png: hr2_models.scad
	openscad hr2_models.scad -D 'name="hr2_master_assembly"' $(PNG_ANGLE_FLAGS) -o $@
	$(PNG_CLEAN) $@
$(PNG_DIRECTORY)/hr2_wheel_assembly.png: hr2_models.scad
	openscad hr2_models.scad -D 'name="hr2_wheel_assembly"' $(PNG_ANGLE_FLAGS) -o $@
	$(PNG_CLEAN) $@
$(PNG_DIRECTORY)/hr2_nucleo_assembly.png: hr2_models.scad
	openscad hr2_models.scad -D 'name="hr2_nucleo_assembly"' $(PNG_ANGLE_FLAGS) -o $@
	$(PNG_CLEAN) $@

$(PNG_DIRECTORY)/thumb_expansion.png: hr2_models.scad
	openscad hr2_models.scad -D 'name="expansion"' $(PNG_TOP_THUMB_FLAGS) -o $@
	$(PNG_CLEAN) $@
$(PNG_DIRECTORY)/thumb_romi_base.png: hr2_models.scad
	openscad hr2_models.scad -D 'name="romi_base"' $(PNG_TOP_THUMB_FLAGS) -o $@
	$(PNG_CLEAN) $@
$(PNG_DIRECTORY)/thumb_hr2_base_assembly.png: hr2_models.scad
	openscad hr2_models.scad -D 'name="hr2_base_assembly"' $(PNG_ANGLE_THUMB_FLAGS) -o $@
	$(PNG_CLEAN) $@
$(PNG_DIRECTORY)/thumb_hr2_pi_assembly.png: hr2_models.scad
	openscad hr2_models.scad -D 'name="hr2_pi_assembly"' $(PNG_ANGLE_THUMB_FLAGS) -o $@
	$(PNG_CLEAN) $@
$(PNG_DIRECTORY)/thumb_hr2_master_assembly.png: hr2_models.scad
	openscad hr2_models.scad -D 'name="hr2_master_assembly"' $(PNG_ANGLE_THUMB_FLAGS) -o $@
	$(PNG_CLEAN) $@
$(PNG_DIRECTORY)/thumb_hr2_wheel_assembly.png: hr2_models.scad
	openscad hr2_models.scad -D 'name="hr2_wheel_assembly"' $(PNG_ANGLE_THUMB_FLAGS) -o $@
	$(PNG_CLEAN) $@
$(PNG_DIRECTORY)/thumb_hr2_nucleo_assembly.png: hr2_models.scad
	openscad hr2_models.scad -D 'name="hr2_nucleo_assembly"' $(PNG_ANGLE_THUMB_FLAGS) -o $@
	$(PNG_CLEAN) $@

$(DXF_DIRECTORY)/encoder_pcb.dxf: hr2_models.scad
	openscad -o $@ -mm -D 'name="encoder_pcb"' hr2_models.scad
$(DXF_DIRECTORY)/expansion_flat.dxf: hr2_models.scad
	openscad -o $@ -mm -D 'name="expansion_flat"' hr2_models.scad
$(DXF_DIRECTORY)/master_pcb.dxf: hr2_models.scad
	openscad -o $@ -mm -D 'name="master_pcb"' hr2_models.scad
$(DXF_DIRECTORY)/nucleo144_pcb.dxf: hr2_models.scad
	openscad -o $@ -mm -D 'name="nucleo144_pcb"' hr2_models.scad
$(DXF_DIRECTORY)/other_pi_pcb.dxf: hr2_models.scad
	openscad -o $@ -mm -D 'name="other_pi_pcb"' hr2_models.scad
$(DXF_DIRECTORY)/raspi3_pcb.dxf: hr2_models.scad
	openscad -o $@ -mm -D 'name="raspi3_pcb"' hr2_models.scad
$(DXF_DIRECTORY)/romi_base.dxf: hr2_models.scad
	openscad -o $@ -mm -D 'name="romi_base"' hr2_models.scad
$(DXF_DIRECTORY)/stlink_pcb.dxf: hr2_models.scad
	openscad -o $@ -mm -D 'name="stlink_pcb"' hr2_models.scad

# This runs the `hr2_models.py` program to generate `hr2_models.scad`:
hr2_models.scad: ${INSTALLED_SCAD_MODELS_PY_FILES}
	python $(SCAD_MODELS_DIRECTORY)/hr2_models.py

# Construct the `README.html` for local reading:
README.html: README.md
	markdown README.md > README.html

# Cleans everything up:
clean:
	rm -f ${ALL_PYL_FILES} ${ALL_COVER_FILES} ${DXF_FILES} ${PNG_FILES} ${SCAD_FILES}
	rm -f README.html~ README.html setup.py,cover .coverage
	rm -rf ${CLEAN_DIRECTORIES}

checkout:
	git checkout ${PNG_FILES}
	git checkout ../electrical/master_board/rev_a/master_board.kicad_pcb


# The documentation for `pytest` and `pytest-cov` is a bit weak.  None of this makes
# much sense and was arrived at by tedious trial and error.  There may be some strange
# interaction between `pytest` and Python Virtual Environments that is causing all of
# this strangeness.  Alternatively, this might just be poor integration between the
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
# What a total pain in the rear end!
test: all
	rm -f ${ALL_COVER_FILES} .coverage
	pytest --cov=scad_models --cov=tests --cov-append --cov-report=annotate \
	    ${ALL_PY_FILES} tests
	@grep -H -n "^!" ${ALL_COVER_FILES} || true

# Pattern rule for running `mypy`, `flake8`, and `pydocstyle` over a `.py` Python file.
# The `.pyl` suffix is used to remember that the linting has occured:
%.pyl: %.py
	mypy                         $<
	flake8 --max-line-length=100 $<
	pydocstyle                   $<
	touch $@



