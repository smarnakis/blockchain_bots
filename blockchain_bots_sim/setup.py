from setuptools import setup

package_name = 'blockchain_bots_sim'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/my_world.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/robot_1.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/robot_2.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/robot_2.urdf.xml']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aris',
    maintainer_email='aris.smarnakis@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_driver = blockchain_bots_sim.my_robot_driver:main',
            'obstacle_avoider = blockchain_bots_sim.obstacle_avoider:main'
        ],
    },
)
