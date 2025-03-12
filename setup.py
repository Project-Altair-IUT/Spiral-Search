from setuptools import setup

package_name = 'spiral_search'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rafid',
    maintainer_email='ahmedrafidx360@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = spiral_search.my_node:main',
            'dummy_gps = spiral_search.dummy_gps:main',
            'move = spiral_search.move:main',
            'spiral_search = spiral_search.spiral_search:main',
            'spiral_motion_sim = spiral_search.spiral_motion_sim:main',
            'gps_nav = spiral_search.gps_nav:main',
            'rover_spiral = spiral_search.rover_spiral:main',
            'move_rover = spiral_search.move_rover:main',
            'circular_spiral_sim = spiral_search.circular_spiral_sim:main',
            
        ],
    },
)
