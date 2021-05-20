test: pytest lint

install_bd_tools:
	python3 setup.py install --user

pytest: install_bd_tools
	python3 -m pytest tests/

lint:
	cpplint --recursive --quiet --extensions=hpp,cpp common/ bootloader/ firmware/
