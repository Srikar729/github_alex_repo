from setuptools import find_packages, setup
from glob import glob
from os.path import join

# Add new nodes here
nodes = [
    "logic_controller_node",
    "actuated_ph_control",
    "raven_parser",
    "alex_mode_regulator",
    "automated_weight_balance",
    "robot_arm_controller",
]

package_name = 'alex_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (join('share', package_name, "launch"), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gautham Sam',
    maintainer_email='gautham.sam@neuralfoundry.co.uk',
    description='Handles the control of the cobot',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [f"{item} = {package_name}.{item}:main" for item in nodes],
    },
)
