from setuptools import find_packages, setup

package_name = 'tanerb_sub'

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
    maintainer='ougy',
    maintainer_email='srongougy@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tanerb_sub = tanerb_sub.tanerb_sub:main',
            'read_pos_DM4340 = tanerb_sub.read_pos_DM4340:main',
            'input_position = tanerb_sub.input_position:main',
            'tanerb_sub_motor = tanerb_sub.tanerb_sub_motor:main',
        ],
    },
)
