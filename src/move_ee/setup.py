from setuptools import setup

package_name = 'move_ee'
submodules_1 = 'move_ee/urdf/'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules_1],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abishek',
    maintainer_email='abishek.swaminathan03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_pub = move_ee.joint_state_pub:main',
            'move_program = move_ee.moveit_trial:main',
            'coord_publisher = move_ee.main:main'
        ],
    },
)
