from setuptools import find_packages, setup

package_name = 'dynamic_control_py'

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
    maintainer='ager',
    maintainer_email='sgerget74@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # extras_require={
    #     'test': [
    #         'pytest',
    #     ],
    # },
    entry_points={
        'console_scripts': [
            'hw_dynamic_control = dynamic_control_py.hw_dynamic_control:main',
            'dynamic_control_node = dynamic_control_py.dynamic_control_node:main',
        ],
    },
)
