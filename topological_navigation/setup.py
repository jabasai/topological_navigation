from setuptools import find_packages
from setuptools import setup
from glob import glob

package_name = 'topological_navigation'

setup(
    name=package_name,
    version='4.0.0',  # Major version bump - ROS1 code removed
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config/', glob('config/*', recursive=True)),
        ('share/' + package_name + '/config/',
            glob('test/fixtures/mixed_actions_map.yaml')),
        ('share/' + package_name + '/launch/', glob('launch/*', recursive=True)),
        ('share/' + package_name + '/rviz/', glob('rviz/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adam Binch',
    maintainer_email='abinch@sagarobotics.com',
    description='ROS2 topological navigation package (ROS1 support removed in v4.0.0)',
    license='MIT',
    tests_require=['pytest', 'launch-pytest'],
    entry_points={
        'console_scripts': [
            # Core ROS2 Navigation Nodes
            'navigation2.py = topological_navigation.scripts.navigation2:main',
            'localisation2.py = topological_navigation.scripts.localisation2:main',
            'map_manager2.py = topological_navigation.scripts.map_manager2:main',
            'get_simple_policy2.py = topological_navigation.scripts.get_simple_policy2:main',

            # Supporting Utilities
            'occupancy_checker.py = topological_navigation.scripts.occupancy_checker:main',
            'topological_transform_publisher.py = topological_navigation.scripts.topological_transform_publisher:main',
            'manual_topomapping.py = topological_navigation.scripts.manual_topomapping:main',
            'validate_map.py = topological_navigation.scripts.validate_map:main',

            # Networkx
            'networkx_utils.py = topological_navigation.scripts.networkx_utils:main',
        ],
    },

)




