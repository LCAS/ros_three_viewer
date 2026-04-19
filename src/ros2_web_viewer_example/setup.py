from glob import glob

from setuptools import setup

package_name = 'ros2_web_viewer_example'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marc Hanheide',
    maintainer_email='mhanheide@lincoln.ac.uk',
    description='Example package for ros2_web_viewer with a simulated UR3e robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_sim = ros2_web_viewer_example.pointcloud_sim:main',
            'image_sim = ros2_web_viewer_example.image_sim:main',
            'marker_sim = ros2_web_viewer_example.marker_sim:main',
            'html_panel_sim = ros2_web_viewer_example.html_panel_sim:main',
            'trigger_service_sim = ros2_web_viewer_example.trigger_service_sim:main',
        ],
    },
)
