#!/usr/bin/env python3
"""Setup script for lilsim Python SDK."""

from setuptools import setup, find_packages

setup(
    name="lilsim",
    version="0.1.0",
    description="Python SDK for lilsim autonomous racing simulator",
    author="lilsim",
    packages=find_packages(),
    install_requires=[
        "pyzmq>=25.0.0",
        "protobuf>=4.21.0",
        "numpy>=1.24.0",
    ],
    python_requires=">=3.8",
)

