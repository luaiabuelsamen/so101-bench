# ROS installs pytest plugins system-wide (/opt/ros/.../launch_testing) that get
# auto-loaded and fail on an unrelated missing dependency. This project has
# nothing to do with ROS, so plugin autoloading stays off.
PYTEST := PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest

.PHONY: test test-fast lint fmt eval

test:
	$(PYTEST) tests/ -q

test-fast:
	$(PYTEST) tests/ -q -m "not slow"

lint:
	ruff check src tests scripts

fmt:
	ruff format src tests scripts

eval:
	python scripts/evaluate.py --episodes 40
