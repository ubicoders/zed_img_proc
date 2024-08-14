from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'zed_img_proc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name + '/zedm', glob(os.path.join(package_name, 'zedm', '*.npz'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='anon@ubicoders.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'zed_play = zed_img_proc.main:main',
            'imgproc_node = zed_img_proc.imgproc_node:main',
        ],
    },
)
