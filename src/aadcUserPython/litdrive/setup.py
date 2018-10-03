import os
import numpy as np

from distutils.core import setup
from setuptools.extension import Extension
from Cython.Build import cythonize

ext_modules = []
if os.name == 'nt':
    ext_modules = [
        Extension("litdrive.util.darkflow.nms",
                  sources=["litdrive/util/darkflow/nms.pyx"],
                  include_dirs=[np.get_include()]
                  ),
        Extension("litdrive.util.darkflow.cy_yolo2_findboxes",
                  sources=["litdrive/util/darkflow/cy_yolo2_findboxes.pyx"],
                  include_dirs=[np.get_include()]
                  ),
        Extension("litdrive.util.darkflow.cy_yolo_findboxes",
                  sources=["litdrive/util/darkflow/cy_yolo_findboxes.pyx"],
                  include_dirs=[np.get_include()]
                  )
    ]

elif os.name == 'posix':
    ext_modules = [
        Extension("litdrive.util.darkflow.nms",
                  sources=["litdrive/util/darkflow/nms.pyx"],
                  include_dirs=[np.get_include()],
                  libraries=["m"]
                  ),
        Extension("litdrive.util.darkflow.cy_yolo2_findboxes",
                  sources=["litdrive/util/darkflow/cy_yolo2_findboxes.pyx"],
                  include_dirs=[np.get_include()],
                  libraries=["m"]
                  ),
        Extension("litdrive.util.darkflow.cy_yolo_findboxes",
                  sources=["litdrive/util/darkflow/cy_yolo_findboxes.pyx"],
                  include_dirs=[np.get_include()],
                  libraries=["m"]
                  )
    ]

setup(ext_modules=cythonize(ext_modules))
