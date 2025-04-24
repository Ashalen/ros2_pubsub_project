from setuptools import setup

package_name = 'pubsub_project'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'publisher_node = pubsub_project.publisher_node:main',
            'subscriber_node = pubsub_project.subscriber_node:main',
        ],
    },
)