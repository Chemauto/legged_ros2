from setuptools import find_packages, setup

package_name = "push_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/push_controller.yaml"]),
        ("share/" + package_name + "/launch", ["launch/push_controller.launch.py"]),
    ],
    install_requires=["setuptools", "onnxruntime", "numpy"],
    zip_safe=True,
    maintainer="xcj",
    maintainer_email="todo@todo.com",
    description="High-level push-box policy controller for hierarchical RL",
    license="TODO",
    entry_points={
        "console_scripts": [
            "push_controller_node = push_controller.push_controller_node:main",
            "push_box_obs_bridge_node = push_controller.push_box_obs_bridge_node:main",
            "push_pose_bridge_node = push_controller.push_pose_bridge_node:main",
        ],
    },
)
