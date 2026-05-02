from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ros2_web_viewer'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # Web assets
        ('share/' + package_name + '/web', glob('web/*.*')),
        ('share/' + package_name + '/web/assets', glob('web/assets/*')),
        ('share/' + package_name + '/web/examples', glob('web/examples/*')),
        # Vendored Three.js (offline use)
        ('share/' + package_name + '/web/vendor/three', glob('web/vendor/three/*.js')),
        ('share/' + package_name + '/web/vendor/three/addons/controls',
         glob('web/vendor/three/addons/controls/*.js')),
        ('share/' + package_name + '/web/vendor/three/addons/loaders',
         glob('web/vendor/three/addons/loaders/*.js')),
    ],
    install_requires=[
        'setuptools',
        'fastapi',
        'uvicorn[standard]',
        'numpy',
    ],
    zip_safe=True,
    maintainer='Marc Hanheide',
    maintainer_email='mhanheide@lincoln.ac.uk',
    description='Web-based 3D robot visualiser for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_web_viewer = ros2_web_viewer.node:main',
            'list_urdf_links = ros2_web_viewer.list_urdf_links:main',
        ],
    },
)
