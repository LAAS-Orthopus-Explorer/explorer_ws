from setuptools import find_packages, setup
from glob import glob

package_name = 'laas_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laas_hello = laas_controller.main:main',
            'laas_ik = laas_controller.inverse_kinematics_node:main',
            'laas_fk = laas_controller.fk_orthopus_node:main',
            'laas_mpc_orthopus = laas_controller.mpc_orthopus:main',
            'save_urdf = laas_controller.save_urdf:main',
        ],
    },
)
