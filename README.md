# Pyfranka
>A Python/C++ library for controlling the Franka Panda robot.

The library supports basic joint/Cartesian position control and velocity control.

Pyfranka depends on [libfranka](https://github.com/frankaemika/libfranka) to interface with the robot, [Eigen](https://eigen.tuxfamily.org) for linear algebra operations and transformations, and [pybind11](https://github.com/pybind/pybind11) for the Python bindings.

## Installation

To install the library on Linux, first build and install the [libfranka](https://github.com/frankaemika/libfranka), [Eigen](https://eigen.tuxfamily.org), and [pybind11](https://github.com/pybind/pybind11) libraries.  Then clone the Pyfranka repository, move to the installation root directory, activate the python environment (e.g., [Conda](https://docs.conda.io/en/latest/)) you wish to install the library in, and run the following command:

```sh
pip install -e .
```
While the library has not been tested on Windows, it may still be possible to install it, assuming that the various dependencies can be installed and the toolchain adapted for the Windows environment.

## Examples

See the Franka Panda examples in the [Common Robot Interface (CRI)](https://github.com/jlloyd237/cri) library.


## Meta

John Lloyd – jlloyd237@gmail.com

Distributed under the GPL v3 license. See ``LICENSE`` for more information.

[https://github.com/jloyd237/pyfranka](https://github.com/jlloyd237/)
