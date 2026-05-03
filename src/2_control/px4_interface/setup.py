from setuptools import find_packages, setup

package_name = "px4_interface"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Juan Amaya",
    maintainer_email="jdamayac@outlook.com",
    description="PX4 offboard interface for AeroSAR",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "offboard_hover = px4_interface.offboard_hover_node:main",
            "waypoint_sequencer = px4_interface.waypoint_sequencer:main", 
        ],
    },
)