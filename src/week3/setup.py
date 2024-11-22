from setuptools import find_packages, setup

package_name = 'week3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaenote',
    maintainer_email='na06219@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'force_control = week3.basic.force_control:main',
            'pallet_move = week3.practice.palet_move:main',
            'random_place = week3.practice.random_place:main',
            'domino = week3.practice.domino:main',
            'sp_domino = week3.practice.sp_domino:main',
        ],
    },
)
