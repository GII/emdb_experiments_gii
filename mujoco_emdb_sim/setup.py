from setuptools import find_packages, setup

package_name = 'mujoco_emdb_sim'

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
    maintainer='jummo',
    maintainer_email='jpmoro0307@gmail.com',
    description='Bridge + perception adapters connecting emdb_simulator '
                '(RoboCasa/robosuite MuJoCo scenes) to the e-MDB architecture.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_bridge = mujoco_emdb_sim.sim_bridge:main',
        ],
    },
)
