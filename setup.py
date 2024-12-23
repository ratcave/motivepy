from setuptools import setup, Extension, find_packages
from Cython.Build import cythonize
import numpy
import os
from os import path
from warnings import warn


MOTIVE_INC_DIR = path.join("C:\\", "Program Files", "OptiTrack", "Motive", "inc")
MOTIVE_LIB_DIR = path.join("C:\\", "Program Files", "OptiTrack", "Motive", "lib")
MOTIVE_LINK_ARG="/DEFAULTLIB:MotiveAPI"

#TODO: Link to correct path automatically
if not os.path.exists(MOTIVE_INC_DIR):
    raise IOError("Can't find path {0}. Continuing installation, but please ensure Motive 64bit files are there before running.".format(MOTIVE_INC_DIR))


native = Extension(
    'motive.native',
    sources=[path.join("motive", "native.pyx")],
    include_dirs=[MOTIVE_INC_DIR, "src"],
    library_dirs=[MOTIVE_LIB_DIR ],
    extra_compile_args=["/std:c++17"],
    extra_link_args=[MOTIVE_LINK_ARG],
    language="c++"
)

camera = Extension(
    'motive.camera',
    sources=[path.join("motive", "camera.pyx")],
    include_dirs=[MOTIVE_INC_DIR, "src", numpy.get_include()],
    library_dirs=[MOTIVE_LIB_DIR ],
    extra_compile_args=["/std:c++17"],
    extra_link_args=[MOTIVE_LINK_ARG],
    language="c++"
)

rigidbody = Extension(
    'motive.rigidbody',
    sources=[path.join("motive", "rigidbody.pyx")],
    include_dirs=[MOTIVE_INC_DIR, "src"],
    library_dirs=[MOTIVE_LIB_DIR ],
    extra_compile_args=["/std:c++17"],
    extra_link_args=[MOTIVE_LINK_ARG],
    language="c++"
)

# pointcloudgroup = Extension(
    # 'motive.pointcloudgroup',
    # sources=["src\\pointcloudgroup.pyx" ],
    # include_dirs=[MOTIVE_INC_DIR, "src"],
    # library_dirs=[MOTIVE_LIB_DIR ],
    # extra_link_args=[MOTIVE_LINK_ARG],
    # language="c++"
# )


transformations  = Extension('_transformations', sources=['third_party/transformations.c'], include_dirs=[numpy.get_include()])

setup(
    name="motive",
    ext_modules= cythonize([native, rigidbody, transformations], language_level = "3"), # pointcloudgroup
#    ext_modules= cythonize([native, rigidbody, camera, transformations], language_level = "3"), # pointcloudgroup
    packages= find_packages(),
    scripts=['scripts/vislight.py', 'scripts/viewer.py', 'scripts/video.py'],
    install_requires=['cython', 'appdirs', 'numpy', 'pyqtgraph'], #, 'btk' ],
    package_data= {'': ['data/*.ttp']}
)
