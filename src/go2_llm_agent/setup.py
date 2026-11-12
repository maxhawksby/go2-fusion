from setuptools import find_packages, setup

package_name = 'go2_llm_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # The typed Ollama agent uses the standard library; ROS deps are in package.xml.
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Riley Francis',
    maintainer_email='rileyfrancis01@gmail.com',
    description='LLM agent that turns natural-language commands into Go2 motion.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'agent = go2_llm_agent.agent:main',
            'speech_bridge = go2_llm_agent.speech_bridge:main',
        ],
    },
)
