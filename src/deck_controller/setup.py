from setuptools import setup
from glob import glob
import os

package_name = 'deck_controller'
pkg = package_name

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{pkg}']),
        (f'share/{pkg}', ['package.xml']),
        (f'share/{pkg}/config', glob('config/*.yaml')),  # Copy YAML config files
        (f'share/{pkg}/launch', glob('launch/*.py')),    # Copy Python launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Josh Cooper',
    maintainer_email='hi@joshc.uk',
    description='Scripts and launch files for controlling ros with a steam deck',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
