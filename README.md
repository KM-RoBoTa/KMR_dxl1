# KMR_dxl

Library for an easy use of Dynamixel motors.  
It abstracts the hardware (no need to concern yourself with memory addresses) and automates the creation of reading/writing handlers (automatically assigns indirect address reading/writing).

It works with Dynamixel's **protocol 2**.

Dependencies:
- dynamixel API
- yaml-cpp

How to use: go to the KMR_dxl folder, then: 
```bash
mkdir build
cd build
cmake ../
cmake --build .
```

If you have Doxygen and Graphviz installed, you can regenerate the documentation locally with
```bash
make docs
```
from the `build` folder after the cmake.  
The generated documentation can be found in "docs/generated_docs/html".

