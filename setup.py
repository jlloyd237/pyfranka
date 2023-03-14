import os
import sys
import re
import subprocess

from setuptools import setup
from setuptools import setup, Extension, find_packages
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError(
                'CMake must be installed to build the following extensions: ' +
                ', '.join(e.name for e in self.extensions)
            )

        cmake_version = LooseVersion(re.search(r'version\s*([\d.]+)', out.decode()).group(1))
        if cmake_version < LooseVersion('3.21.0'):
            raise RuntimeError('CMake >= 3.21.0 is required')

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

        # Required for auto-detection of auxiliary "native" libs
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        build_type = os.environ.get('BUILD_TYPE', 'Release')
        build_args = ['--config', build_type]

        cmake_args = [
            '-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
            '-DCMAKE_BUILD_TYPE=' + build_type,
            '-DCMAKE_INSTALL_RPATH={}'.format('$ORIGIN'),
        ]

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=self.build_temp)
        subprocess.check_call(['cmake', '--build', '.', '--target', ext.name] + build_args, cwd=self.build_temp)


# setup(name='pyfranka',
#       version='0.1',
#       description='Python 3 control interface for Franka Panda robot',
#       author='John Lloyd',
#       author_email='jlloyd237@gmail.com',
#       url='https://github.com/jlloyd237/pyfranka',
#       packages=['pyfranka'],
#       package_data={'pyfranka': ['*.so']},
#       include_package_data=True,
#       python_requires='>=3.7',
#       zip_safe=False,
#  )

setup(name='pyfranka',
    version='0.2',
    description='Python 3 control interface for Franka Panda robot',
    author='John Lloyd',
    author_email='jlloyd237@gmail.com',
    url='https://github.com/jlloyd237/pyfranka',
    license='GPLv3',
    packages=find_packages(),
    ext_modules=[CMakeExtension('_pyfranka')],
    cmdclass=dict(build_ext=CMakeBuild),
    # package_data={'pyfranka': ['*.so']},
    # include_package_data=True,
    python_requires='>=3.7',
    zip_safe=False,
 )
