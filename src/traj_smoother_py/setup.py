from setuptools import find_packages, setup

package_name = 'traj_smoother_py'

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
    maintainer='w',
    maintainer_email='w@todo.todo',
    description='Trajectory smoother using csaps',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'smoother_node = traj_smoother_py.smoother_node:main',
        ],
    },
)
