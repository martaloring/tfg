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
        	'test_1 = chat_pkg.test_1:main',
        	'test_2 = chat_pkg.test_2:main',
        	'test_3 = chat_pkg.test_3:main',
        	'test_keyword = chat_pkg.test_keyword:main',
        	'test_sub_key = chat_pkg.test_sub_key:main',
        ],
    },
)
