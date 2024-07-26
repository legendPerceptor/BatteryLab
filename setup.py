from setuptools import setup, find_packages

setup(
    name='BatteryLab',
    version='0.1.0',
    author="Yuanjian Liu",
    author_email="yuanjian@uchicago.edu",
    packages=find_packages(),  # Find packages in the src directory
    include_package_data=True,
    install_requires=[],
)