from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomous_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), [os.path.join('autonomous_navigation', 'models', 'best.pt')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='timii',
    maintainer_email='timiiowolabi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "navigation = autonomous_navigation.navigation:main",
            "save_data = autonomous_navigation.save_data:main",
            'task1_solution = autonomous_navigation.task1_solution:main'
        ],
    },
)
