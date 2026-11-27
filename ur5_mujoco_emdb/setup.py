import os
from glob import glob
from setuptools import setup

package_name = 'ur5_mujoco_emdb'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jpperezm',
    maintainer_email='alu0101278919@ull.edu.es',
    description='e-MDB node of the MuJoCo UR5e simulator, with its launch file and scripts',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_ur5 = ur5_mujoco_emdb.sim_ur5_mujoco:main',
        ],
    },
)
