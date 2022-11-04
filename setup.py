from setuptools import setup

package_name = 'ros_delay_out_center_task'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, 'delay_out_center_task'],
    package_dir={'delay_out_center_task': 'delay_out_center_task/delay_out_center_task'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a.whit',
    maintainer_email='nml@whit.contact',
    description='TODO: Package description',
    license='Mozilla Public License 2.0',
    requires=['delay_out_center_task'],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node = ros_delay_out_center_task.node:main'
        ],
    },
)
