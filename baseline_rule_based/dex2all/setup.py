from setuptools import find_packages, setup

package_name = 'dex2all'

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
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'la_dext_to_all = dex2all.la_dext_to_all:main',
            'la_dext_to_all_v2 = dex2all.la_dext_to_all_v2:main',
            'ra_dext_to_all = dex2all.ra_dext_to_all:main',
            'joint_state_pos = dex2all.joint_state_pos:main',
            'video_recorder = dex2all.video_recorder:main',
            'maper_isaac_ros = dex2all.maper_isaac_ros:main',
        ],
    },
)
