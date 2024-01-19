from setuptools import find_packages, setup

package_name = 'my_moveit_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
        'rclpy',
        'xarm_msgs',
        'geometry_msgs',
        'std_msgs'],
    zip_safe=True,
    maintainer='erl-tianyu',
    maintainer_email='nmarinosraitsevits@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_xarm_planner_node_pose = scripts.test_xarm_planner_node_pose:main',            
            'test_pub = scripts.test_pub:main',            
        ],
    },
)
