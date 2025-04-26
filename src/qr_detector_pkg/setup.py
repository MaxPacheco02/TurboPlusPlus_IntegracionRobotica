from setuptools import setup

package_name = 'qr_detector_pkg'

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
    maintainer='Neil Hernandez',
    maintainer_email='neil210170@gmail.com',
    description='Nodo ROS 2 para detectar códigos QR usando la cámara de laptop',
    license='MIT',
    entry_points={
        'console_scripts': [
            'qr_detector = qr_detector_pkg.qr_detector:main',
        ],
    },
)
