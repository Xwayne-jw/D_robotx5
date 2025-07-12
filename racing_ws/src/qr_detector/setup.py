from setuptools import setup

package_name = 'qr_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    # 添加pyzbar到安装依赖
    install_requires=['setuptools', 'pyzbar'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='QR code detection node using pyzbar',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qr_detector_node = qr_detector.qr_detector_node:main',
        ],
    },
)
    