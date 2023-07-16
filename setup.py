from distutils.core import setup
import os

# User-friendly description from README.md
current_directory = os.path.dirname(os.path.abspath(__file__))
package_name = "pid_controller"

setup(
    name=package_name,
    packages=[package_name],
    version="0.0.1",
    license="MIT",
    description="A simple PID controller writtin in Python",
    author="Pedro Soares",
    author_email="pmbs.123@gmail.com",
    url="https://github.com/PedroS235/pid_controller_py",
)
