from setuptools import find_packages, setup

package_name = "nav_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/nav_controller.yaml"]),
        ("share/" + package_name + "/launch", ["launch/nav_controller.launch.py"]),
    ],
    install_requires=["setuptools", "onnxruntime", "numpy"],
    zip_safe=True,
    maintainer="xcj",
    maintainer_email="todo@todo.com",
    description="High-level navigation policy controller for hierarchical RL",
    license="TODO",
    entry_points={
        "console_scripts": [
            "nav_controller_node = nav_controller.nav_controller_node:main",
        ],
    },
)
