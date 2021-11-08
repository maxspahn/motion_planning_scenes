import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="MotionPlanningScenes",
    version="0.0.1",
    author="maxspahn",
    author_email="m.spahn@tudelft.nl",
    description="Motion Planning Scenes.",
    long_description=long_description,
    url="https://github.com/maxspahn/motion_planning_scenes",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    install_requires=['numpy',
                      'casadi',
                      'matplotlib']
)

