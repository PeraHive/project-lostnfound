from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'lostnfound'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
        glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'), 
         glob('scripts/*.sh')),
        (os.path.join('share', package_name, 'scripts'), 
         glob('scripts/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jazz',
    maintainer_email='janithcyapa@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'dummy_node = lostnfound.dummy_node:main',
        ],
    },
)
