from setuptools import find_packages, setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('third_party/depth_anything',
            ['../../../third_party/depth_anything/config.json',
                '../../../third_party/depth_anything/model.safetensors',
                '../../../third_party/depth_anything/preprocessor_config.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Morgan',
    maintainer_email='redfieldm@gmail.com',
    description='Perception for spot micro.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_anything = perception.depth_anything:main'
        ],
    },
)
