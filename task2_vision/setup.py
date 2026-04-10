from setuptools import find_packages, setup

package_name = 'task2_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/task2_touch_demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'camera_view = task2_vision.camera_view:main',
		'detect_colors = task2_vision.detect_colors:main',
        'object_listener = task2_vision.object_listener:main',
        'task2_controller = task2_vision.task2_controller:main',
        'pixel_to_workspace = task2_vision.pixel_to_workspace:main',
        'touch_command_planner = task2_vision.touch_command_planner:main',
        'mg400_executor = task2_vision.mg400_executor:main',
        'visual_servo_planner = task2_vision.visual_servo_planner:main',
        ],
    },
)
