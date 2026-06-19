from setuptools import setup, find_packages

package_name = 'arm_ik'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data', ['data/benchmark_poses.yaml']),
        ('share/' + package_name + '/config', []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ARIA Team',
    maintainer_email='aria@aria.dev',
    description='IK solvers and benchmark for ARIA 5-DoF arm',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ik_node = arm_ik.ik_node:main',
            'ik_benchmark_node = arm_ik.ik_benchmark_node:main',
        ],
    },
)
