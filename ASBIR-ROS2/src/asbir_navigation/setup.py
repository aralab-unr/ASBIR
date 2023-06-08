from setuptools import setup

package_name = 'asbir_navigation'

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
    maintainer='aralab',
    maintainer_email='trichnak72399@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'PotentialField = asbir_navigation.potentialField:main',
            'FrameTest = asbir_navigation.frameTest:main',
            'ModelTest = asbir_navigation.modelTest:main',
            'GraphTest = asbir_navigation.graphTest:main',
            'BuildBestPath = asbir_navigation.buildBestPath:main',
            'PointPub = asbir_navigation.pointPub:main',
            'ProcessMesh = asbir_navigation.processMesh:main',
            'PathController = asbir_navigation.pathController:main',
            'SetZero = asbir_navigation.servoSetZero:main',
            'AlignAprilTag = asbir_navigation.alignApriltag:main',
            'AlignStructure = asbir_navigation.alignStructure:main',
        ],
    },
)

