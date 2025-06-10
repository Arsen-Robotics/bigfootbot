from setuptools import find_packages, setup

package_name = 'bfb_road_follower'

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
    maintainer='jevgeni',
    maintainer_email='arseni.kalbin@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'edge_detection_node = bfb_road_follower.edge_detection:main',
            'depth_node = bfb_road_follower.depth:main',
        ],
    },
)
