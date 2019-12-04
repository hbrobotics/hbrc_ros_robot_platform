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

test: all
	rm -f ${ALL_COVER_FILES} .coverage
	pytest --cov=scad_models scad_models/scad.py tests/test_scad.py  \
            tests/test_scad.py --cov-report=annotate tests
	@grep -H -n "^!" ${ALL_COVER_FILES} || true

foo:
	echo ALL_PYL_FILES=${ALL_PYL_FILES}

#	pytest --cov=scad_models scad_models/scad.py tests/test_scad.py  \
#            tests/test_scad.py --cov-report=annotate tests


# Pattern rule for running `mypy`, `flake8`, and `pydocstyle` over a `.py` Python file.
# The `.pyl` suffix is used to remember that the linting has occured:
%.pyl: %.py
	mypy --no-color-output       $<
	flake8 --max-line-length=100 $<
	pydocstyle                   $<
	touch $@



