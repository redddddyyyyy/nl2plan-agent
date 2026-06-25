from setuptools import setup

package_name = "manipulation_node"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rajeev Reddy",
    maintainer_email="rajeevreddy1009@gmail.com",
    description="MoveIt2 wrapper exposing pick and place as ROS2 services.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "manipulation = manipulation_node.manipulation:main",
        ],
    },
)
