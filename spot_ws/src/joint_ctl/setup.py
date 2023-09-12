from setuptools import find_packages, setup

package_name = 'joint_ctl'

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
    maintainer='Morgan',
    maintainer_email='redfieldm@gmail.com',
    description='Control endpoint for Spot servos. It\'s all hardware after this.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_ctl = joint_ctl.servo_ctl:main'
        ],
    },
)
