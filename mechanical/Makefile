.PHONY: all clean test

PY_FILES :=                      \
    romi_model/__init__.py       \
    romi_model/scad.py           \
    romi_model/romi_model.py
PYCACHE_DIRECTORIES :=           \
    romi_model/__pycache__       \
    tests/__pycache__
PYL_FILES := ${PY_FILES:%.py=%.pyl}
COVER_FILES := ${PY_FILES:%=%,cover}
SCAD_FILES := romi_base.scad

all: ${PYL_FILES}
	pip install .
	rm -f ${SCAD_FILES}
	romi_model
foo:
	echo ${PY_FILES}
	echo ${PYL_FILES}

clean:
	rm -f ${PYL_FILES} ${SCAD_FILES} ${COVER_FILES}
	rm -rf ${PYCACHE_DIRECTORIES}

test: all
	rm -f ${COVER_FILES} .coverage
	pytest --cov=romi_model romi_model/scad.py tests/test_scad.py  \
            tests/test_romi_model.py --cov-report=annotate tests
	@grep -H -n "^!" ${COVER_FILES} || true

# Pattern rule for running `mypy`, `flake8`, and `pydocstyle` over a `.py` Python file.
# The `.pyl` suffix is used to remember that the linting has occured:
%.pyl: %.py
	mypy                         $<
	flake8 --max-line-length=100 $<
	pydocstyle                   $<
	touch $@



