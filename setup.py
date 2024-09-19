from setuptools import setup, find_packages


install_requires = []
module = "vehicle-controls"
package = "vehicle-controls"
src = "src"

setup(
        name=package,
        package_dir={"": src},
        packages=[module],
        version="0.0.0",
        zip_safe=False,
        install_requires=install_requires,
)
