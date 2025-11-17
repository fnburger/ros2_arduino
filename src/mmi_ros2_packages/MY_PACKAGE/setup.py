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
    maintainer='konsti',
    maintainer_email='konsti@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'publisher = MY_PACKAGE.my_publisher:main',
            'subscriber = MY_PACKAGE.my_subscriber:main',
            'cam_reader = MY_PACKAGE.cam_reader:main',
            'image_processor = MY_PACKAGE.image_processor:main',
            'microcontroller_communicator = MY_PACKAGE.microcontroller_communicator:main',
            'image_processor_v2 = MY_PACKAGE.image_processor_v2:main'
        ],
    },
)
