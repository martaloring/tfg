import os
from glob import glob
from setuptools import setup

package_name = 'chat_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marta',
    maintainer_email='marta@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'asr_node = chat_pkg.asr_node:main',
        	'chat_node = chat_pkg.chat_node:main',
        	'tts_node = chat_pkg.tts_node:main',
        	'social_patrol = chat_pkg.social_patrol:main',
        	'poses_mng = chat_pkg.poses_mng:main',
        ],
    },
)
