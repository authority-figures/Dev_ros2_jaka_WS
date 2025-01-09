from setuptools import find_packages, setup

package_name = 'joint_state_visualization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','rclpy','sensor_msgs','matplotlib','numpy','my_custom_msgs','time','threading','os','sys'],
    zip_safe=True,
    maintainer='lwh',
    maintainer_email='2029717756@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'visualize_joint_state = joint_state_visualization.visualize_joint_state:main',
            'joints_visualizer = joint_state_visualization.joints_visualizer:main',
            'joint_state_publisher = joint_state_visualization.joint_state_publisher:main',
        ],
    },
)
