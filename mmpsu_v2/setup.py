from setuptools import find_packages, setup
import glob
import os

package_name = "mmpsu_v2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob.glob("launch/*launch.py")),
        (os.path.join("share", package_name, "config"), glob.glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Adam Cordingley",
    maintainer_email="adam@repoweredelectronics.com",
    description="Control of the MMPSU v2",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mmpsu_v2_node = mmpsu_v2.mmpsu_v2_node:main",
            "mmpsu_debug_pane = mmpsu_v2.mmpsu_debug_pane:main",
        ],
    },
)
