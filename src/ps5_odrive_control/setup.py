from setuptools import find_packages, setup

package_name = 'ps5_odrive_control'

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
    maintainer='trevorperey',
    maintainer_email='trevorperey@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = ps5_odrive_control.test_node:main',
            'ps5_controller_node = ps5_odrive_control.ps5_controller_node:main',
        ],
    },
)
