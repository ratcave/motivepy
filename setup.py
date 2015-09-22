from setuptools import setup, Extension
from Cython.Build import cythonize

native = Extension(
    'motive.native',
    sources=["src\\native.pyx" ],
    include_dirs=["C:\\Program Files (x86)\\OptiTrack\\Motive\\inc", "src"],
    library_dirs=["C:\\Program Files (x86)\\OptiTrack\\Motive\\lib" ],
    extra_link_args=["/DEFAULTLIB:NPTrackingTools"],
    language="c++"
)

setup(
    name="motive",
    ext_modules=cythonize(native),
    packages=["motive"],
    scripts=['scripts/vislight.py', 'scripts/showmarkers.py']
)
