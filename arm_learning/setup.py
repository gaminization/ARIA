from setuptools import setup, find_packages

package_name = 'arm_learning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/tracking_config.yaml',
        ]),
    ],
    install_requires=[
        'setuptools',
        'pyyaml',
    ],
    zip_safe=True,
    maintainer='ARIA Team',
    maintainer_email='aria@aria.dev',
    description='ARIA learning infrastructure',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bag_recorder_node = arm_learning.bag_recorder_node:main',
            'bag_indexer = arm_learning.bag_indexer:main',
            'bag_to_dataset = arm_learning.bag_to_dataset:main',
        ],
    },
)
