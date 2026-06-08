from setuptools import find_packages, setup

package_name = 'turtlebot3_test'

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
    maintainer='baotran',
    maintainer_email='baotran@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["turtlebot3_test1=turtlebot3_test.turtlebot3_test1:main",
                            "turtlebot3_test2=turtlebot3_test.turtlebot3_test2:main",
                            "turtlebot3_move_client=turtlebot3_test.turtlebot3_move_client:main",
                            "turtlebot3_control=turtlebot3_test.turtlebot3_control:main",
                            "control_node_1=turtlebot3_test.control_node_1:main",
                            "control_node_2=turtlebot3_test.control_node_2:main",
                            "turtlebot3_estimate_heading=turtlebot3_test.turtlebot3_estimate_heading:main",
                            "estimate_heading=turtlebot3_test.estimate_heading:main",
        ],
    },
)
