from setuptools import find_packages, setup

package_name = 'alex_utilities'

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
    description='Contains all the python importable utility function and tools',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'common_utilities = alex_utilities.common_utilities:main',
            'get_port = alex_utilities.get_port:main',
        ],
    },
)
