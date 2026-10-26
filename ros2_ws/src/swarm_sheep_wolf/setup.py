from glob import glob

from setuptools import setup

package_name = 'swarm_sheep_wolf'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anghelloerr',
    description='Algoritmo ovejas-lobo para enjambres MicroROS-Pi5',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent_node = swarm_sheep_wolf.agent_node:main',
            'metrics_node = swarm_sheep_wolf.metrics_node:main',
            'odom_test_node = swarm_sheep_wolf.odom_test_node:main',
        ],
    },
)
