from setuptools import find_packages, setup
from pathlib import Path
package_name = 'alex_gazebo'

base_share_path = Path("share")

gazebo_models = []
for model in Path('models').iterdir():
    for item in  model.glob("*"):
        if item.is_dir():
            gazebo_models.append((
                str(base_share_path / package_name / item), 
                [str(item) for item in item.glob("*.stl")] 
            ))
        else:
            gazebo_models.append((
                str(base_share_path / package_name / item.parent), 
                [str(item)] 
            ))

meshes = [
    (
        str(base_share_path / package_name / 'meshes' / mesh.stem), 
        [str(item) for item in mesh.glob("*.stl")]
    )
    for mesh in Path("meshes").iterdir()
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (str(base_share_path / 'ament_index' / 'resource_index' / 'packages'), [ str(Path("resource") / package_name) ]),
        (str(base_share_path / package_name), ['package.xml']),
        (str(base_share_path / package_name / 'params'), [str(item) for item in Path('params').glob("*.yaml")]),
        (str(base_share_path / package_name / 'launch'), [str(item) for item in Path('launch').glob("*.launch.py")]),
        (str(base_share_path / package_name / 'xacro'),  [str(item) for item in Path('xacro').glob("*.xacro")]),
        (str(base_share_path / package_name / 'ros2_control'),  [str(item) for item in Path('ros2_control').glob("*.xacro")]),
        (str(base_share_path / package_name / 'config'),  [str(item) for item in Path('config').glob("*.yaml")]),
    ] + gazebo_models + meshes,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gautham Sam',
    maintainer_email='gautham.sam@neuralfoundry.co.uk',
    description='Contains package related to Gazebo simulation',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
