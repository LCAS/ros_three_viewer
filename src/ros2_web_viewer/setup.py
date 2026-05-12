from setuptools import setup, find_packages
import logging
import os
import sys
from glob import glob

package_name = 'ros2_web_viewer'

# ---------------------------------------------------------------------------
# Download vendored web assets (Three.js, fonts) at build/install time.
# Internet access is required here; at runtime the files are served locally.
# ---------------------------------------------------------------------------
_BUILD_CMDS = {'build', 'install', 'develop', 'egg_info', 'bdist_wheel',
               'bdist_egg', 'install_data', 'build_py'}
if _BUILD_CMDS.intersection(sys.argv):
    logging.basicConfig(level=logging.INFO, format='%(message)s')
    _here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, _here)
    try:
        from download_web_assets import download_web_assets
        download_web_assets()
    except Exception as exc:  # pragma: no cover
        print(
            f'WARNING: Failed to download web assets - viewer will require '
            f'internet connection at runtime: {exc}',
            file=sys.stderr,
        )
    finally:
        sys.path.pop(0)

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
        # Vendored Three.js — downloaded by download_web_assets.py at build time
        ('share/' + package_name + '/web/vendor/three', glob('web/vendor/three/*.js')),
        ('share/' + package_name + '/web/vendor/three/addons/controls',
         glob('web/vendor/three/addons/controls/*.js')),
        ('share/' + package_name + '/web/vendor/three/addons/loaders',
         glob('web/vendor/three/addons/loaders/*.js')),
        # Vendored web fonts — downloaded by download_web_assets.py at build time
        ('share/' + package_name + '/web/vendor/fonts', glob('web/vendor/fonts/*.css')),
        ('share/' + package_name + '/web/vendor/fonts/files',
         glob('web/vendor/fonts/files/*')),
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
            'restart_service_node = ros2_web_viewer.restart_service_node:main',
            'list_urdf_links = ros2_web_viewer.list_urdf_links:main',
        ],
    },
)
