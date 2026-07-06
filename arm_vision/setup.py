from setuptools import setup, find_packages

package_name = 'arm_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ARIA Team',
    maintainer_email='aria@aria.dev',
    description='Vision pipeline for ARIA arm',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_node = arm_vision.camera_node:main',
            'detection_node = arm_vision.detection_node:main',
            'depth_node = arm_vision.depth_node:main',
            'grasp_node = arm_vision.grasp_node:main',
            'apriltag_calibration_node = arm_vision.apriltag_calibration_node:main',
            # U2 Advanced Perception
            'sam2_node = arm_vision.sam2_node:main',
            'pose_6d_node = arm_vision.pose_6d_node:main',
            'transparent_object_node = arm_vision.transparent_object_node:main',
            'material_recognition_node = arm_vision.material_recognition_node:main',
            'gaussian_splatting_node = arm_vision.gaussian_splatting_node:main',
            'perception_orchestrator = arm_vision.perception_orchestrator:main',
            'grasp_node_v2 = arm_vision.grasp_node_v2:main',
        ],
    },
)
