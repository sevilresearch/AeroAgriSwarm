from setuptools import find_packages, setup

package_name = 'crazyflie_missions'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['crazyflie_missions', 'crazyflie_missions.*'], exclude=['test', 'tests*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools>=65,<80',  'cflib==0.1.28', 'numpy==2.2.*', 'cfclient==2025.9'],
    zip_safe=True,
    maintainer='Dhyey Patel',
    maintainer_email='dbp15@students.uwf.edu',
    description='Single and multi-agent UAVs waypoint navigation using parallel and agri-swarm algorithm.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'takeoff = crazyflie_missions.waypoint_tests.takeoff:main',
            'crazyflie_bridge = crazyflie_missions.waypoint_tests.crazyflie_bridge:main',
            'crazyflie_bridge_multi = crazyflie_missions.waypoint_tests.crazyflie_bridge_multi:main',
            'crazyflie_bridge_lh = crazyflie_missions.waypoint_tests.crazyflie_bridge_lh:main',
            'waypoint_manager = crazyflie_missions.waypoint_tests.waypoint_manager:main',
            'waypoint_manager_motive = crazyflie_missions.waypoint_tests.waypoint_manager_motive:main',
            'waypoint_manager_multi = crazyflie_missions.waypoint_tests.waypoint_manager_multi:main',
        ],
    },
)
