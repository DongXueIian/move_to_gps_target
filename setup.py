from setuptools import find_packages, setup

package_name = 'move_to_gps_target'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+'/launch', ['launch/move_to_gps_target.py']),
        ('share/' + package_name+'/launch', ['launch/apm_controller.py']),
        ('share/' + package_name+'/launch', ['launch/apm_keyborad_controller.py']),
        ('share/' + package_name+'/params', ['params/nav2_params.yaml']),
        ('share/' + package_name+'/maps', ['maps/empty_world.pgm']),
        ('share/' + package_name+'/maps', ['maps/empty_world.yaml']),
        ('share/' + package_name+'/rviz2', ['rviz2/test_nav2_apm.rviz']),
        ('share/' + package_name+'/rviz2', ['rviz2/test_nav2_apm_mppi.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orangepi',
    maintainer_email='orangepi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'regularly_clear_costmap_after_starting_nav2=move_to_gps_target.regularly_clear_costmap_after_starting_nav2_node:main',
            'gztf_filter_not_height=myTestPockage.gztf_filter_not_height:main',
            'my_velocity_controller=move_to_gps_target.my_velocity_controller:main',
            'apm_controller_node=move_to_gps_target.apm_controller_node:main'
        ],
    },
)
