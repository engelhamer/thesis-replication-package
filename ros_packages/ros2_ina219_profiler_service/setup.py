from setuptools import setup

package_name = 'ina219_profiler_service'

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
    maintainer='engel',
    maintainer_email='engelhamer@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "server_node = ina219_profiler_service.ina219_profiler_server:main",
            "client_node = ina219_profiler_service.ina219_profiler_client:main",
        ],
    },
)
