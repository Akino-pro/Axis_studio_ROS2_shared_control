from setuptools import find_packages, setup

package_name = 'my_robot_viewer'

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
    maintainer='yuchen',
    maintainer_email='yuchen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'viewer_node = my_robot_viewer.viewer_node:main',
            'viewer_node_simulation = my_robot_viewer.viewer_node_simulation:main',
        ],
    },
)
