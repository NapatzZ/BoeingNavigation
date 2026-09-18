from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'skuba_navigation'


def maps_data_files():
    """Install maps/ preserving its category-subfolder structure.

    maps/ is organized into category folders (00_test, 10_rcj, ...), so a flat
    glob would miss the nested files. Walk the tree and emit one data_files
    entry per directory that contains files.
    """
    entries = []
    for root, _dirs, files in os.walk('maps'):
        files = [f for f in files if not f.startswith('.')]
        if files:
            dest = os.path.join('share', package_name, root)
            entries.append((dest, [os.path.join(root, f) for f in files]))
    return entries

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install whole directories via glob so new files are picked up
        # automatically and nothing silently goes missing from the install.
        ('share/' + package_name, glob('launch/*.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
        ('share/' + package_name + '/behavior_trees', glob('behavior_trees/*.xml')),
    ] + maps_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SKUBA Software Team',
    maintainer_email='napat559977@gmail.com',
    description='Navigation for SKUBA robot software.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_server = skuba_navigation.nodes.navigation_server_node:main',
            'filter_lidar = skuba_navigation.nodes.filter_lidar:main',
            'map_save_cli = skuba_navigation.nodes.map_save_cli:main',
            'costmap_reconfig_server = skuba_navigation.nodes.costmap_reconfig_server:main',
            'door_detector = skuba_navigation.nodes.door_detector:main',
            'following_server = skuba_navigation.nodes.following_server:main',
            'person_cluster_filter = skuba_navigation.nodes.person_cluster_filter:main',
            'save_location = skuba_navigation.nodes.save_location:main',
            'emergency_stop = skuba_navigation.nodes.emergency_stop:main',
        ],
    },
)
