from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'ff_test_controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='joris997',
    maintainer_email='joris.verhagen@gmail.com',
    description='Simple Controllers for the DISCOWER free-flyer platform',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'python_executable_name = pkg_name.python_executable_name:main',
            
            'waypoint_planner = ff_test_controllers.waypoint_planner:main',
            'thruster = ff_test_controllers.thruster:main',
            'setpoint_PID_controller = ff_test_controllers.setpoint_PID_controller:main',
            'manual_controller = ff_test_controllers.manual_controller:main',

            'ff_tf2_broadcaster = ff_test_controllers.ff_tf2_broadcaster:main'
        ],
    },
)
