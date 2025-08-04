from setuptools import find_packages, setup

package_name = "c5"

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
    maintainer="juwan",
    maintainer_email="dlacksdn352@gmail.com",
    description="ROKEY BOOT CAMP Package",
    license="Apache 2.0 License",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "force_control = c5.basic.force_control:main",
            "move_periodic = c5.basic.move_periodic:main",
            "getting_position = c5.basic.getting_position:main",
            "simple_move=c5.basic.move:main",
            "grip=c5.basic.grip:main",
            "pickNplace=c5.basic.pickNplace:main",
            "sort_block=c5.basic.sortingBlock:main",
            "domino=c5.basic.stack_domino1:main",
            "cup=c5.basic.cup_stacking:main",
        ],
    },
)
