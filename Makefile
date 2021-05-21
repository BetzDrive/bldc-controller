test: pytest lint

install_bd_tools:
	python3 setup.py install --user

pytest: install_bd_tools
	python3 -m pytest tests/

lint: lint_python lint_cpp

lint_python:
	isort --check tests bd_tools
	black --check tests bd_tools
	#flake8 bd_tools tests

lint_cpp:
	cpplint --recursive common/ bootloader/ firmware/

format: format_python

format_python:
	isort tests bd_tools
	black tests bd_tools
