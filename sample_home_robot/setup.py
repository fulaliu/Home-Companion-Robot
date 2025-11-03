from setuptools import find_packages, setup

package_name = 'sample_home_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament 索引（必须）
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # 安装 launch 文件
        ('share/' + package_name + '/launch', ['launch/ros_ws_bridge.launch.py']),
        ('share/' + package_name + '/launch', ['launch/ros_ws_bridge_with_cameras.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your name',
    maintainer_email='you@example.com',
    description='Sample home robot package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 下面三行的右侧必须能导入到函数；如果你的文件里函数名不是 main()，请改成实际函数名
            'camera0_ws_node = sample_home_robot.camera0_ws_node:main',
            'camera1_ws_node = sample_home_robot.camera1_ws_node:main',
            'cmd_ws_node     = sample_home_robot.cmd_ws_node:main',
        ],
    },
)

