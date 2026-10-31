from setuptools import find_packages, setup

package_name = 'go2_auto_mapper'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Autonomous frontier exploration for Unitree Go2',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'explorer = go2_auto_mapper.explorer:main',
            'vlm_view = go2_auto_mapper.vlm_view:main',
        ],
    },
)
