from setuptools import find_packages, setup
from pathlib import Path

package_name = "drn_navigation_route_planning"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            str(Path("share") / package_name / "launch"),
            [str(file) for file in Path("launch").glob("*") if file.is_file()],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kenny",
    maintainer_email="leonardo.treichel@senairs.org.br",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            f"service = {package_name}.route_planning_service:main",
            f"client = {package_name}.route_planning_client:main",
        ],
    },
)
