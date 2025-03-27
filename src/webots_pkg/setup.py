from setuptools import find_packages, setup

package_name = 'webots_pkg'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))

data_files.append(('share/' + package_name + '/launch', [
    'launch/epuck_launch.py']))

data_files.append(('share/' + package_name + '/worlds', [
    'worlds/open_arena.wbt']))

data_files.append(('share/' + package_name + '/resource', [
    'resource/webots_epuck.urdf',
    'resource/webots_epuck_predator.urdf',
    'resource/webots_epuck_peer.urdf']))

data_files.append(('share/' + package_name + '/protos', [
    'protos/E-puck.proto',
    'protos/E-puckDistanceSensor.proto',
    'protos/E-puck_predator.proto',
    'protos/E-puck_peer.proto',
    'protos/Apple.proto',
    'protos/WaterBottle.proto']))

data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user.name@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'epuck_agent = webots_pkg.epuck_agent:main',
            'epuck_predator = webots_pkg.epuck_predator:main',
            'epuck_peer = webots_pkg.epuck_peer:main',
            'random_walker = webots_pkg.random_walker:main',
            'supervisor = webots_pkg.supervisor:main',
            'gradients = webots_pkg.gradients:main',
            'allostatic_model = webots_pkg.allostatic_model:main',
            'robot_navigation = webots_pkg.robot_navigation:main',
            'data_gathering = webots_pkg.data_gathering:main',
        ],
    },
)
