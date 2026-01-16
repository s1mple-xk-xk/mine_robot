from setuptools import find_packages, setup

package_name = 'mine_aplication'

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
    maintainer='xk',
    maintainer_email='xk@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'init_robot_pose = mine_aplication.init_robot_pose:main',
            'get_robot_pose = mine_aplication.get_robot_pose:main',
            'nav_to_pose = mine_aplication.nav_to_pose:main',
        ],
    },
)
