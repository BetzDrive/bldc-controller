test: pytest lint

pytest: install_bd_tools
	python3 -m pytest tests/

lint: lint_python lint_cpp

lint_python:
	isort --check tests bd_tools
	black --check tests bd_tools
	pflake8 bd_tools tests

lint_cpp:
	cpplint --recursive common/ bootloader/ firmware/

format: format_python format_bazel

format_python:
	isort tests bd_tools
	black tests bd_tools

format_bazel:
	bazel run //:buildifier

setup: init_submodules install_python_packages install_bd_tools

init_submodules:
	git submodule update --recursive --init

install_python_packages:
	python3 -m pip install -r requirements.txt

install_bd_tools:
	python3 -m pip install bd_tools/ --user
