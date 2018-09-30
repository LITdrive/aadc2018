import os
import numpy as np

from distutils.core import setup
from setuptools.extension import Extension
from Cython.Build import cythonize

if os.name == 'nt':
    ext_modules = [
        Extension("darkflow_utils.nms",
                  sources=["darkflow_utils/nms.pyx"],
                  include_dirs=[np.get_include()]
                  ),
        Extension("darkflow_utils.cy_yolo2_findboxes",
                  sources=["darkflow_utils/cy_yolo2_findboxes.pyx"],
                  include_dirs=[np.get_include()]
                  ),
        Extension("darkflow_utils.cy_yolo_findboxes",
                  sources=["darkflow_utils/cy_yolo_findboxes.pyx"],
                  include_dirs=[np.get_include()]
                  )
    ]

elif os.name == 'posix':
    ext_modules = [
        Extension("darkflow_utils.nms",
                  sources=["darkflow_utils/nms.pyx"],
                  include_dirs=[np.get_include()],
                  libraries=["m"]
                  ),
        Extension("darkflow_utils.cy_yolo2_findboxes",
                  sources=["darkflow_utils/cy_yolo2_findboxes.pyx"],
                  include_dirs=[np.get_include()],
                  libraries=["m"]
                  ),
        Extension("darkflow_utils.cy_yolo_findboxes",
                  sources=["darkflow_utils/cy_yolo_findboxes.pyx"],
                  include_dirs=[np.get_include()],
                  libraries=["m"]
                  )
    ]

setup(ext_modules=cythonize(ext_modules))
