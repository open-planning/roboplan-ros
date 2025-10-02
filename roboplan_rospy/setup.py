from setuptools import setup

package_name = "roboplan_rospy"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sebastian Castro, Erik Holum",
    maintainer_email="sebas.a.castro@gmail.com, eholum@gmail.com",
    description="Python bindings for the roboplan motion planning library.",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [],
    },
)
