from setuptools import find_packages, setup

package_name = 'alex_lab_objects'

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
    maintainer='Gautham Sam',
    maintainer_email='gautham.sam@neuralfoundry.co.uk',
    description='Contains all the lab equipments objects',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
