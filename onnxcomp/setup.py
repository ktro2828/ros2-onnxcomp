import os.path as osp
from glob import glob

from setuptools import find_packages, setup

package_name = "onnxcomp"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (osp.join("share", package_name, "config"), glob("config/*")),
        (osp.join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kotaro Uetake",
    maintainer_email="kotaro.uetake@tier4.jp",
    description="A ROS 2 package for ONNX model compilation",
    license="Apache License 2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [f"{package_name}_node = {package_name}.node:main"],
    },
)
