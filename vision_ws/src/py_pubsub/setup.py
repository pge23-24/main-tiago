from setuptools import setup


package_name = "py_pubsub"

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
    maintainer="pge-2023",
    maintainer_email="pge-2023@todo.todo",
    description="Examples of minimal publisher/subscriber using rclpy",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "display = py_pubsub.display:main",
            "camera_yolo = py_pubsub.camera_yolo:main",
            "simu_ecal = py_pubsub.simu_ecal_image:main",
        ],
    },
)
