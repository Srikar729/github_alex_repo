from setuptools import find_packages, setup
from glob import glob
from os.path import join

package_name = 'alex_externals'

nodes = [
    "doosan_arm_control",
    "webrtc_client",
    "liquid_level",
    "atlas_liquid_doser",
    "serial_interface_lifecycle",
    "weight_balance",
    "scitek_magnetic_stirrer",
    "neuation_centrifuge",
    "sclean_sonicator",
    "dc_actuator",
    "device_manager",
    "radwag_analytial_balance",
    "atlas_ph_probe",
    "atlas_temperature_probe",
    "stepper_actuator",
    "zed_camera_image",
    "doosan_gripper_control",
    "joanlab_magnetic_stirrer",
    "wiggens_homogenizer"
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (join('share', package_name, "params"), glob('params/*.yaml')),
        (join('share', package_name, "launch"), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='Gautham Sam',
    maintainer_email='gautham.sam@neuralfoundry.co.uk',
    description='Package to connect to all external devices for ros control',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [f"{item} = {package_name}.{item}:main" for item in nodes],
    },
)
