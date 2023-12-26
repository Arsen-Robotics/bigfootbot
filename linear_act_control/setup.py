from setuptools import find_packages, setup

package_name = 'linear_act_control'

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
    maintainer_email='jevgeni.kalbin@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'linear_act_control_node = linear_act_control.linear_act_control_sub:main',
            'keyboard_publisher_node = linear_act_control.linear_act_control_pub:main',
        ],
    },
)
