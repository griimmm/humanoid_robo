from setuptools import setup

package_name = 'move_ee'

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
    maintainer='Abishek',
    maintainer_email='abishek.swaminathan03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_publisher = move_ee.joint_state_pub:main',
            'coord_publisher = move_ee.cv_coord_publisher:main'
        ],
    },
)
