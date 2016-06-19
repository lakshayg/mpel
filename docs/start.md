# Getting Started

## Obtaining the Library

MPEL depends on boost and OpenCV for some of its functionality
you need to install these libraries before you can compile
MPEL. They can be installed using the following commands

```
sudo apt-get install libboost-all-dev
sudo apt-get install libopencv-dev
```

The source code for the library can be downloaded from
[GitHub](http://github.com/lakshayg/mpel)

```
git clone https://github.com/lakshayg/mpel
mkdir mpel/build
cd mpel/build
cmake .. && make
```

## Your First Code (The Easy Way)

The easiest way to compile your first program is to use the
`user` directory in the repository. The `CMakeLists.txt` file
in the directory takes care of linking the code.

```
cd mpel/user
# write your first code
cd ../build
cmake .. && make
cd user  # your executable should be present here
```
