from setuptools import find_packages, setup

package_name = 'orchestrator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/orchestrator.launch.py']),
        ('share/' + package_name + '/config', ['config/nav2.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='anchen082016@gmail.com',
    description='Orchestrator node for maze solver',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'orchestrator = orchestrator.orchestrator:main',
        ],
    },
)
