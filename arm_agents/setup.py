from setuptools import setup, find_packages

package_name = 'arm_agents'

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
    description='ARIA 15-agent system for autonomous manipulation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vision_agent = arm_agents.vision_agent:main',
            'depth_agent = arm_agents.depth_agent:main',
            'tracking_agent = arm_agents.tracking_agent:main',
            'affordance_agent = arm_agents.affordance_agent:main',
            'planning_agent = arm_agents.planning_agent:main',
            'skill_agent = arm_agents.skill_agent:main',
            'control_agent = arm_agents.control_agent:main',
            'safety_agent = arm_agents.safety_agent:main',
            'memory_agent = arm_agents.memory_agent:main',
            'world_model_agent = arm_agents.world_model_agent:main',
            'learning_agent = arm_agents.learning_agent:main',
            'evaluation_agent = arm_agents.evaluation_agent:main',
            'dialogue_agent = arm_agents.dialogue_agent:main',
            'attention_agent = arm_agents.attention_agent:main',
            'reachability_agent = arm_agents.reachability_agent:main',
        ],
    },
)
