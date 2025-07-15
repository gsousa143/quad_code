from setuptools import setup, find_packages

setup(
    name="control",
    version="0.1.0",
    author="Gabriel dos Santos Sousa",
    author_email="gabrielsousa4242@gmail.com",
    description="Biblioteca para controle de quadricópteros",
    long_description=open("README.md", encoding="utf-8").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/gsousa143/quad_code/Control",
    license="MIT",
    packages=find_packages(),
    install_requires=[
        "smbus",
    ],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)
