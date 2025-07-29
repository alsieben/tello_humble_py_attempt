from setuptools import find_packages, setup

package_name = 'tello_aruco_follower'

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
    maintainer='august',
    maintainer_email='august@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tello_aruco_follower = tello_aruco_follower.tello_aruco_follower:main',
            'tello_calibrator = tello_aruco_follower.tello_calib_img_cap_srv:main',
        ],
    },
)
