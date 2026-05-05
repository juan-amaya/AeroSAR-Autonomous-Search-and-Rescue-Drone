from setuptools import find_packages, setup

package_name = "mission_manager"

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
    description="General Mission Maganer ",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "basic_mission = mission_manager.basic_mission_node:main",
        ],
    },
)