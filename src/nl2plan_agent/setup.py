from setuptools import setup

package_name = "nl2plan_agent"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, package_name + ".ros_backend"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", []),
    ],
    install_requires=[
        "setuptools",
        "ollama",
        "jsonschema",
        "json-repair",
        "pyyaml",
    ],
    zip_safe=True,
    maintainer="Rajeev Reddy",
    maintainer_email="rajeevreddy1009@gmail.com",
    description="LLM-driven task planner: natural language to robot actions.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "nl2plan = nl2plan_agent.agent:main",
        ],
    },
)
