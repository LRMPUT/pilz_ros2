from setuptools import find_packages, setup
import os, glob

package_name = 'pilz_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob.glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'urdf', 'meshes'), glob.glob('urdf/meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bartlomiej Kulecki',
    maintainer_email='bartlomiej.kulecki@put.poznan.pl',
    description='The pilz_tutorial package - ROS2 Jazzy version',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'example_node = pilz_tutorial.example:main',
            # ADD YOUR NODE HERE, e.g. 'imie_node = pilz_tutorial.ImieApp:main',
        ],
    },
)
