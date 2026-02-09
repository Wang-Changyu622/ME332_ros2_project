import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'nav2_mppi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        ('share/' + package_name, ['package.xml']),
        
        # 安装 launch 文件
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.launch.py'))),
        
        # 安装参数配置文件
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*.yaml'))),

        # --- 修改点：安装地图文件夹 ---
        # 这会把 maps 目录下所有的 .yaml 和图片文件 (.pgm, .png) 拷贝到安装目录
        (os.path.join('share', package_name, 'maps'), 
            glob(os.path.join('maps', '*.yaml')) + glob(os.path.join('maps', '*.pgm')) + glob(os.path.join('maps', '*.png'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='daniel@example.com',
    description='Nav2 bringup wrapper using MPPI controller',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)