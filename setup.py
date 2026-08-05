from setuptools import find_packages, setup

package_name = "game_controller"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/two_class_threshold.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Paolo",
    maintainer_email="forin.paolo98@gmail.com",
    description="Controller nodes that decide game_bridge commands from the integrated classifier signal.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "two_class_threshold_controller = game_controller.two_class_threshold_controller:main",
            "dummy_keyboard_controller = game_controller.dummy_keyboard_controller:main",
        ],
    },
)
