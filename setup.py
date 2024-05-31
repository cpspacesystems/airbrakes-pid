from setuptools import find_packages, setup

package_name = 'airbrakes_pid'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='cpss@email.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_controller = airbrakes_pid.pid_loop:main',
            'altimeter_pub = airbrakes_pid.altimeter_data_pub:main',
        ],
    },
)
