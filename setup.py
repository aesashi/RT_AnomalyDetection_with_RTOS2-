#!/path/to/correct/python

from setuptools import find_packages, setup

package_name = 'yolo_detection'

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
    maintainer='ae0589',
    maintainer_email='k.for-nba@live.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = yolo_detection.camera_node:main',
            'detection_node = yolo_detection.detection_node:main',
            'demonstration_node = yolo_detection.demonstration_node:main',
        ],
    },
)
