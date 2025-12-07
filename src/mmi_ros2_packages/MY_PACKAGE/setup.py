from setuptools import find_packages, setup

package_name = 'MY_PACKAGE'

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
    maintainer='niklas',
    maintainer_email='niklas.burger@tuwien.ac.at',
    description='ROS arduino smile detector with LED indicator',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cam_reader = MY_PACKAGE.cam_reader:main',
            'microcontroller_communicator = MY_PACKAGE.microcontroller_communicator:main',
            'image_processor_v2 = MY_PACKAGE.image_processor_v2:main'
        ],
    },
)
