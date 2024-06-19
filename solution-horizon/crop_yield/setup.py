from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'crop_yield'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), [os.path.join('crop_yield', 'best.pt')]),
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
            "task2_solution = crop_yield.task2_solution:main",
        ],
    },
)
