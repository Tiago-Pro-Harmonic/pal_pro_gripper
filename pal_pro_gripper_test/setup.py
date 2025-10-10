from setuptools import find_packages, setup

package_name = 'pal_pro_gripper_test'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Susanna Mastromauro',
    maintainer_email='susanna.mastromauro@pal-robotics.com',
    description='A package to control the PAL pro gripper.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'check_gripper_close = pal_pro_gripper_test.check_gripper_close:main',
            'check_gripper_open = pal_pro_gripper_test.check_gripper_open:main',
        ],
    },
)