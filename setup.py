from setuptools import setup

package_name = 'perception'

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
    maintainer='atakan',
    maintainer_email='GOKMENATAKAN.TURKMEN@bmc.com.tr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['computer_vision_node = perception.computer_vision_node:main',
                            'computer_vision_node2 = perception.computer_vision_node2:main',
                            'computer_vision_node3 = perception.computer_vision_node3:main'
        ],
    },
)
