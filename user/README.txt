This directory is meant to quickly get started with MPEL.
keep your code in this directory and it gets built along
with the library. The generated executable is placed in
a folder named user in the build directory.

For example, if the user folder contains user_code.cpp
then it is built using the following commands

cd ${MPEL_DIR}/build
cmake ..
make user_code

