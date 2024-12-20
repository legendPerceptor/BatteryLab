from setuptools import find_packages, setup

package_name = 'sartorius'

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
            "sartorius_client = sartorius.sartorius_client:main",
            "sartorius_server = sartorius.sartorius_server:main"
        ],
    },
)
