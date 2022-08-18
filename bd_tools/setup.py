from setuptools import find_packages, setup

with open("README.md", "r") as fh:
    long_description = fh.read()

setup(
    name="bd_tools",
    version="0.1",
    description="Python API for BetzDrive",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="http://github.com/betzdrive/bldc-controller",
    author="Greg Balke",
    author_email="gbalke@berkeley.edu",
    license="BSD",
    packages=find_packages(),
    install_requires=["crcmod", "matplotlib", "numpy", "scipy", "pyserial"],
    zip_safe=False,
    classifiers=[
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "License :: OSI Approved :: BSD License",
        "Operating System :: OS Independent",
    ],
)
