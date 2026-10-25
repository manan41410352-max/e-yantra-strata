from setuptools import setup

package_name = 'algorithms'

# Team 6131 task executables. Keep names unique across task directories.
# Rebuild after editing: ros2 run executes the installed copy of each script.
SCRIPTS = [
    'scripts/task1a/task1A.py',
    'scripts/task1b/task1B.py',
    'scripts/task1c/task1C.py',
    'scripts/task2a/task2A_perception.py',
    'scripts/task2a/task2A_manipulation.py',
    'scripts/task2b/task2B_planning.py',
    'scripts/task2b/task2B_navigation.py',
]

setup(
    name=package_name,
    version='2.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='e-Yantra',
    maintainer_email='support@e-yantra.org',
    author='Agrim Chauhan, Saksham Kaushal, Rayhan Khan, Manan Sood',
    description='Team 6131 solutions for StrataCobot Tasks 1A, 1B, 1C, 2A and 2B.',
    license='Apache-2.0',
    scripts=SCRIPTS,
)
