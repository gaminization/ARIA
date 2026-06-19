from setuptools import setup

package_name = 'arm_vla'

setup(
    name=package_name,
    version='3.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ARIA Team',
    maintainer_email='aria@gaminizer.dev',
    description='ARIA VLA model interface and benchmark',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vla_benchmark = arm_vla.vla_benchmark:main',
        ],
    },
)
