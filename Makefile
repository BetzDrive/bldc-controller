test: pytest lint

pytest:
	python3 -m pytest --pyargs tools tests/

lint:
	cpplint --recursive --quiet --extensions=hpp,cpp common/ bootloader/ firmware/
