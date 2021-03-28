from setuptools import setup

package_name = 'edo_navigation'

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
    maintainer='allen',
    maintainer_email='allen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'edo_nav = edo_navigation.obj_detection_subscriber:main'
		#'edo_nav = edo_navigation.location_subscriber:main'
		#'edo_nav = edo_navigation.classification_subscriber:main'
        ],
    },
)
