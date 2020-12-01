from setuptools import setup

package_name = 'sparkie'

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
    maintainer='Ioannis Broumas',
    maintainer_email='ioabro17@student.hh.se',
    description='DEIS SparkFun Redbot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sparkie = sparkie.sparkie:main',
            'teleop = sparkie.teleop:main',
            'gps = sparkie.gps:main',
            'odometer = sparkie.odometer:main',
            'imu = sparkie.imu:main',
            'teleop_C = sparkie.teleop_C:main',
            'follower = sparkie.follower:main',
            'roof = sparkie.roof:main'
        ],
    },
)
