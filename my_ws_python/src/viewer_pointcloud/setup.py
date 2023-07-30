from setuptools import setup

package_name = 'viewer_pointcloud'
submodule_name = 'include'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, submodule_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BigTree777',
    maintainer_email='workInoshita@gmail.com',
    description='This package is a pointcloud publisher',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_pointcloud = viewer_pointcloud.publisher_pointcloud:main',
        ],
    },
)
