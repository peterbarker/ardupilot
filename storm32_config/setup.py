"""
Setup script for SToRM32 Configuration Tool.
"""

from setuptools import setup, find_packages
from pathlib import Path

# Read README for long description
readme_file = Path(__file__).parent / "README.md"
long_description = readme_file.read_text() if readme_file.exists() else ""

setup(
    name="storm32-config",
    version="0.1.0",
    author="ArduPilot Community",
    description="Python/PyQt6 configuration tool for SToRM32 gimbal controllers",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/yourusername/storm32-config",  # TODO: Update with actual URL
    packages=find_packages(),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Topic :: System :: Hardware :: Hardware Drivers",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: Python :: 3.12",
        "Operating System :: POSIX :: Linux",
    ],
    python_requires=">=3.10",
    install_requires=[
        "PyQt6>=6.6.0",
        "pyserial>=3.5",
    ],
    entry_points={
        "console_scripts": [
            "storm32-config=main:main",
        ],
    },
    include_package_data=True,
    zip_safe=False,
)
