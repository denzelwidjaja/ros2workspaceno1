from setuptools import find_packages, setup

package_name = 'skyforge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/skyforge']),
        ('share/skyforge', ['package.xml']),
        ('share/skyforge/launch', ['launch/view_robot.launch.py']),
        ('share/skyforge/urdf', ['urdf/skyforge_robot.urdf.xacro']),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dwidjaja',
    maintainer_email='dwidjaja@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test = skyforge.my_first_node:main"
        ],
    },
)
