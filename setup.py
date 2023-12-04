from setuptools import find_packages, setup
from glob import glob

package_name = 'gesture_recognizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['gesture_recognizer/gesture_recognizer.task']),
        ('share/' + package_name + '/launch', glob('launch/*launch.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='glorie',
    maintainer_email='glorieramazani@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_recognizer_node = gesture_recognizer.gesture_recognizer_node:main'
        ],
    },
)
