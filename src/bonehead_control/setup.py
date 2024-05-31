from setuptools import find_packages, setup

package_name = 'bonehead_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=['bonehead_ik', 'bonehead_gait_scheduler', 'bonehead_motor_controller'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kevin',
    maintainer_email='klewis23278@gmail.com',
    description='Package that handles the control functionality of bonehead',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik_node = bonehead_ik.ik_node:main',
            'gait_scheduler = bonehead_gait_scheduler.gait_scheduler_node:main',
            'motor_controller = bonehead_motor_controller.motor_control_node:main'
        ],
    },
)
