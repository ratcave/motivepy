from setuptools import setup, Extension, find_packages
from Cython.Build import cythonize
import numpy
import os


MOTIVE_INC_DIR="C:\\Program Files\\OptiTrack\\Motive\\inc"
MOTIVE_LIB_DIR="C:\\Program Files\\OptiTrack\\Motive\\lib"
MOTIVE_LINK_ARG="/DEFAULTLIB:NPTrackingToolsx64"

#TODO: Link to correct path automatically
assert os.path.exists(MOTIVE_INC_DIR), "\n \n Can't find path {0}. Please use Motive 64bit \n \n".format(MOTIVE_INC_DIR)


native = Extension(
    'motive.native',
    sources=["src\\native.pyx"],
    include_dirs=[MOTIVE_INC_DIR, "src"],
    library_dirs=[MOTIVE_LIB_DIR ],
    extra_link_args=[MOTIVE_LINK_ARG],
    language="c++"
)

camera = Extension(
    'motive.camera',
    sources=["src\\camera.pyx"],
    include_dirs=[MOTIVE_INC_DIR, "src", numpy.get_include()],
    library_dirs=[MOTIVE_LIB_DIR ],
    extra_link_args=[MOTIVE_LINK_ARG],
    language="c++"
)

rigidbody = Extension(
    'motive.rigidbody',
    sources=["src\\rigidbody.pyx" ],
    include_dirs=[MOTIVE_INC_DIR, "src"],
    library_dirs=[MOTIVE_LIB_DIR ],
    extra_link_args=[MOTIVE_LINK_ARG],
    language="c++"
)

pointcloudgroup = Extension(
    'motive.pointcloudgroup',
    sources=["src\\pointcloudgroup.pyx" ],
    include_dirs=[MOTIVE_INC_DIR, "src"],
    library_dirs=[MOTIVE_LIB_DIR ],
    extra_link_args=[MOTIVE_LINK_ARG],
    language="c++"
)



setup(
    name="motive",
    ext_modules= cythonize([native, rigidbody, camera, pointcloudgroup]),
    packages= find_packages(),
    scripts=['scripts/vislight.py', 'scripts/viewer.py', 'scripts/video.py'],
    install_requires=['cython', 'appdirs', 'numpy', 'pyqtgraph', 'btk' ],
    package_data= {'': ['data/*.ttp']}
)
