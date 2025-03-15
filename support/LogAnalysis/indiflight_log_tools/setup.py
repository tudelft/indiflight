from setuptools import setup, Extension
import os

# Define the C extension
blackbox_decode = Extension(
    'indiflight_log_tools.blackbox_decode',  # Name of the extension
    sources=[
        "indiflight_log_tools/blackbox-tools/src/blackbox_decode.c",
        "indiflight_log_tools/blackbox-tools/src/blackbox_fielddefs.c",
        "indiflight_log_tools/blackbox-tools/src/decoders.c",
        "indiflight_log_tools/blackbox-tools/src/parser.c",
        "indiflight_log_tools/blackbox-tools/src/platform.c",
        "indiflight_log_tools/blackbox-tools/src/stats.c",
        "indiflight_log_tools/blackbox-tools/src/stream.c",
        "indiflight_log_tools/blackbox-tools/src/tools.c",
        "indiflight_log_tools/blackbox-tools/src/units.c"
    ],
    include_dirs=["indiflight_log_tools/blackbox-tools/src/"],  # Include directories for header files
    extra_compile_args=["-fPIC", "-O3"],     # Additional compiler flags
    extra_link_args=["-lm"],                  # Linker arguments
)

# Call the setup function
setup(
    name="indiflight_log_tools",               # Package name
    version="0.2.0",                           # Initial version
    description="Import Indiflight BFL logs fast and in correct units.",  # Package description
    author="Till Blaha",                       # Author name
    author_email="t.m.blaha@tudelft.nl",      # Author email
    license="LICENSE",                         # License file (specify separately)
    long_description=open("README.md").read(), # Read long description from README file
    long_description_content_type="text/markdown",  # Specify content type for long description
    keywords=["example", "keywords"],          # Keywords
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: Linux",
    ],
    packages=["indiflight_log_tools"],             # Specify your package (replace with actual package name)
    ext_modules=[blackbox_decode],              # Include the C extension in the build
    install_requires=[                          # Package dependencies
        "matplotlib >=3.9.2",
        "numpy >=1.26.4",
        "pandas >=2.2.2",
        "platformdirs >=4.2.2",
        "scipy >=1.14.0",
    ],
    scripts=[
        "bin/bfl2csv",
        "bin/bfl2rerun",
    ]
)