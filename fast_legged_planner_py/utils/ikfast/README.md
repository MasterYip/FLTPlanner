# IKFast warper for c++ & python

1. Use [IKFast Generator](https://www.hamzamerzic.info/ikfast_generator/) to generate your IK algorithm
  - Note: for 3DTranslation there should be an intermediate link and joint between root link BASE and the arm (unknown reason)
2. Replace ikfast_gen.cpp with your file
3. Rename pybind module in CMakeLists.txt

```CMakeLists
## Settings
set(PYBIND_NAME <replace with your favored name>) # Change this to the name of your python module
```

4. Complie it.

```bash
mkdir build
cd build
cmake ..
make
```

or specifying python exec path

```bash
mkdir build
cd build
cmake .. -DPYTHON_EXECUTABLE=$(which python3)
make
```

5. Get your cpython module in /build
