from setuptools import find_packages, setup

package_name = 'so101_grasping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/fixed_pick.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huanghao',
    maintainer_email='3487650498@qq.com',
    description='Fixed-position SO-101 MoveIt pick sequence',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fixed_pick_demo = so101_grasping.fixed_pick_demo:main',
            'real_fixed_pick_demo = so101_grasping.real_fixed_pick_demo:main',
        ],
    },
)
