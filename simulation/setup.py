# setup.py
from setuptools import setup, Extension
import pybind11

ext_modules = [
    Extension(
        "control",  # Must match the name in PYBIND11_MODULE
        ["cpp/bindings.cpp", "cpp/lib/pid/pid.cpp"],
        include_dirs=[pybind11.get_include(), "cpp/lib/pid"],
        language="c++",
    ),
]

setup(
    name="control",
    ext_modules=ext_modules,
)
