from setuptools import find_packages, setup

package_name = "ur5e_teaching_bench"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Giridhar J Shenoy",
    maintainer_email="giridharjshenoy@gmail.com",
    description="Mode manager and benchmark drivers for the UR5e teaching controllers.",
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "mode_manager = ur5e_teaching_bench.mode_manager:main",
            "bench_tracking = ur5e_teaching_bench.bench_tracking:main",
            "bench_handguide = ur5e_teaching_bench.bench_handguide:main",
            "bench_jog = ur5e_teaching_bench.bench_jog:main",
            "record_mode = ur5e_teaching_bench.record_mode:main",
        ],
    },
)
