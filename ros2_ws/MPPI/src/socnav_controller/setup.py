from setuptools import setup

package_name = 'socnav_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Phani-Teja Singamaneni',
    maintainer_email='phaniteja.sp@gmail.com',
    description='Social navigation controller for Tutorial',
    license='MIT',
    entry_points={
        'console_scripts': [
        ],
    },
)
