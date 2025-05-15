from setuptools import find_packages, setup

package_name = 'alex_intelligence'
nodes = [
    "object_detection"
]
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
    maintainer='root',
    maintainer_email='srikar.yechuri@neuralfoundry.co.uk',
    description='Contains the AI/ML',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [f"{item} = {package_name}.{item}:main" for item in nodes],
    },
)
