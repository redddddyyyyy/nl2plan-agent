from setuptools import setup

package_name = "perception_node"

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
    description="GroundingDINO open-vocabulary detector + RGB-D to 3D pose.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "perception = perception_node.perception:main",
        ],
    },
)
