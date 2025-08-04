from setuptools import find_packages, setup

package_name = 'tomjo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaeheyoung',
    maintainer_email='jaeheyoung@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amr = tomjo.amr_ctrl:main',
            'obserber = tomjo.observer_test:main',
            'tracker = tomjo.tracker_test:main',
            'dummy_nav2 = tomjo.dummy_nav2:main',
        ],
    },
)
