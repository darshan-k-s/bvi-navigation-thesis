from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'master'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name, 'launch'),
   	    glob('launch/*.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
   	 'console_scripts': [
       		 'imu_node = master.imu_node:main',
       		 'obstacle_zone_node = master.obstacle_zone_node:main',
		 'fusion_node = master.fusion_node:main',
		 'audio_feedback_node = master.audio_feedback_node:main',
                 'yolo_node = master.yolo_node:main',
   	],
    },
)
