from setuptools import find_packages, setup

package_name = 'camera_service'

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
    maintainer='yuanjian',
    maintainer_email='yuanjian@uchicago.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "camera_server = camera_service.camera_server:main",
            "camera_client = camera_service.camera_client:main",
            'camera_subscriber = camera_service.camera_subscriber:main'
        ],
    },
)
