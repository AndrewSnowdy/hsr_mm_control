from setuptools import setup

package_name = 'hsr_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line ensures your .pt model is actually installed!
        ('share/' + package_name + '/models', ['models/best.pt']), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrewsnowdy',
    maintainer_email='your@email.com',
    description='HSR Vision Node for ADA Button Detection',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This allows you to run: ros2 run hsr_vision hsr_vision_node
            'hsr_vision_node = hsr_vision.hsr_vision_node:main',
        ],
    },
)