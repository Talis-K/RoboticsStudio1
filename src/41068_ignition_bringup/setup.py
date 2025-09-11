from setuptools import setup, find_packages
from pathlib import Path

package_name = '41068_ignition_bringup'     # ROS package name (can start with digits)
python_pkg   = 'bringup_41068'              # Python module dir (must be a valid identifier)

# Collect non-Python data (launch, config, worlds) for install
def files_in(dirpath, install_subdir):
    base = Path(dirpath)
    return [
        (f'share/{package_name}/{install_subdir}', [str(p)]) 
        for p in base.glob('*') if p.is_file()
    ]

data_files = [
    # ament index resource so ROS finds the package
    (f'share/ament_index/resource_index/packages', [f'resource/{package_name}']),
    # package.xml
    (f'share/{package_name}', ['package.xml']),
]
data_files += files_in('launch', 'launch')
data_files += files_in('config', 'config')
data_files += files_in('worlds', 'worlds')

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[python_pkg, f'{python_pkg}.*']),
    data_files=data_files,
    install_requires=['setuptools', 'numpy', 'Pillow', 'sensor_msgs_py'],
    zip_safe=True,
    maintainer='YOUR NAME',
    maintainer_email='you@example.com',
    description='Bringup for 41068 project with GUI (Tk/Pillow), Gazebo, bridges, and nav.',
    license='Apache-2.0',
    keywords=['ROS2', 'Gazebo', 'GUI', 'Tkinter', 'Pillow'],
    # Console entry points you can run via ros2 run / launch as executable='gui_panel'
    entry_points={
        'console_scripts': [
            # console-name  =  module.path:function
            'gui_panel = bringup_41068.gui_panel:main',
        ],
    },
)
