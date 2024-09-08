# Get the parameter -clean
if [ "$1" == "-clean" ]; then
    rm -r build
    rm -r lib
fi

# Build Shell
mkdir -p build
cd build
cmake ..
make -j
cd ..
