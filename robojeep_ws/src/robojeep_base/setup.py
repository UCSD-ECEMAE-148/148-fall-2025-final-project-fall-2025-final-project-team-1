from setuptools import setup

package_name = 'robojeep_base'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_node = robojeep_base.base_node:main',                              # ORIGINAL (Ackermann)
            'base_node_differential = robojeep_base.base_node_differential:main',    # NEW (Tank steering)
        ],
    },
)
