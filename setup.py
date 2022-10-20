import os
from glob import glob
from setuptools import setup

package_name = 'robm_tp3'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch','*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vincent Drevelle',
    maintainer_email='vincent.drevelle@univ-rennes1.fr',
    description='Outils et squelettes de code pour le TP3 de ROBM',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_heading = robm_tp3.control_heading:main',
            'move = robm_tp3.move:main',
        ],
    },
)
