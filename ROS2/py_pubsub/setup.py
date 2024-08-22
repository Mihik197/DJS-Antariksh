from setuptools import find_packages, setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "hello_node = py_pubsub.hello_publisher:main",
            "draw_circle = py_pubsub.draw_circle:main",
            "pose_subscriber = py_pubsub.pose_subscriber:main",
            "turtle_controller = py_pubsub.turtle_controller:main",
            "world_node = py_pubsub.world_publisher:main",
            "helloworld_subscriber = py_pubsub.helloworld_subscriber:main",
        ],
    },
)
