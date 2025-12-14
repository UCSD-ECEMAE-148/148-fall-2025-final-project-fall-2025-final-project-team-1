from setuptools import setup

package_name = 'robojeep_arduino_bridge'

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
    maintainer='root',
    maintainer_email='djnighti@ucsd.edu',
    description='Arduino bridge node for steering + throttle control',
    license='TODO: License declaration',
    tests_require=['pytest'],
	entry_points={
		'console_scripts': [
			'arduino_bridge = robojeep_arduino_bridge.bridge_node:main',
			'arduino_bridge_calibrated = robojeep_arduino_bridge.bridge_node_calibrated:main',  # ADD THIS LINE
		],
	},
)


