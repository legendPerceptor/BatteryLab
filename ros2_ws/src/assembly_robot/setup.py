from setuptools import find_packages, setup
from glob import glob

package_name = 'assembly_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/yaml', ['yaml/well_positions.yaml']),
        ('share/' + package_name + '/yaml', ['yaml/arm_camera_positions.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuanjian',
    maintainer_email='yuanjian@uchicago.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'assembly_robot = assembly_robot.AssemblyRobot:main',
            'test_yaml_path = assembly_robot.test_yaml_path:main',
            'crimper_robot = assembly_robot.CrimperRobot:main',
            'liquid_robot = assembly_robot.LiquidRobot:main',
            'app = assembly_robot.app:main' 
        ],
    },
)
