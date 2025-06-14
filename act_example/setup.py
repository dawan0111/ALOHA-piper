from setuptools import find_packages, setup

package_name = 'act_example'

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
    maintainer='airo',
    maintainer_email='msd030428@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'act_data_collection = '
            'act_example.act_data_collection.'
            'act_data_collection:main',

            'act_hdf5_viewer = '
            'act_example.act_hdf5_viewer.'
            'act_hdf5_viewer:main',
        ],
    },
)
