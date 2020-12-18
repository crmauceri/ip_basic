import setuptools

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setuptools.setup(
    name="ip_basic", # Replace with your own username
    version="0.0.1",
    description="Depth completion algorithm from Jason Ku, Ali Harakeh, and Steven Waslander",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/crmauceri/ip_basic",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)