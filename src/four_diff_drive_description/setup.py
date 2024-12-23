from setuptools import setup
import os
from glob import glob

package_name = 'four_diff_drive_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        ### changed by Anisha 12/11/2024 from
        (os.path.join('share', package_name, 'models/worlds'), glob('models/worlds/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        #(os.path.join('share', package_name, 'models/worlds/meshes'), glob('models/worlds/meshes/*')),
        #(os.path.join('share', package_name, 'models/four_diff_drive'), glob('models/four_diff_drive/*'))
        ### changed upto
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='The ' + package_name + ' package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_mover = four_diff_drive_description.key_control:main',
        ],
    },
)
