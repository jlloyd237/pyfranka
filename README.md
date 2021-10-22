# Pyfranka
>Provides a Python 3 control interface for the Franka Panda robot.

At present, only joint and Cartesian position control have been implemented in the API, but the aim is to extend this to include velocity control in the near future.

Pyfranka depends on [libfranka](https://github.com/frankaemika/libfranka) to interface with the robot, [Eigen](https://eigen.tuxfamily.org) for linear algebra operations and transformations, and [pybind11](https://github.com/pybind/pybind11) for the Python 3 bindings.

## Installation

To install the library on Linux, first build and install the [libfranka](https://github.com/frankaemika/libfranka), [Eigen](https://eigen.tuxfamily.org), and [pybind11](https://github.com/pybind/pybind11) libraries.  Then clone the Pyfranka repository, move to the package root directory, activate the python environment (e.g., Conda) you wish to install the library in, and run the following command:

```sh
pip install -e .
```
While the library has not been tested on Windows, it may still be possible to install it, assuming that the various dependencies can be installed and the toolchain adapted for the Windows environment.

## Usage examples
Coming soon ...


## Meta

John Lloyd – jlloyd237@gmail.com

Distributed under the GPL v3 license. See ``LICENSE`` for more information.

[https://github.com/jloyd237/pyfranka](https://github.com/jlloyd237/)
