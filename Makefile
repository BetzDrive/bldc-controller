test: pytest lint

pytest: install_python_packages
	python3 -m pytest tests/

lint: lint_python lint_cpp

lint_python:
	isort --check tests bd_tools
	black --check bd_tools
	black --check tests
	pflake8 bd_tools tests

lint_cpp:
	cpplint --recursive common/ bootloader/ firmware/ drivers/

format: format_python format_bazel

format_python:
	isort tests bd_tools
	black bd_tools
	black tests

format_bazel:
	bazel run //:buildifier

setup: init_submodules install_python_packages

init_submodules:
	git submodule update --recursive --init

install_python_packages:
	python3 -m pip install -r requirements.txt --user
	python3 -m pip install -e bd_tools/ --user
