#!/bin/bash
cmake -Bbuild -H. -GNinja -DLIMA_ENABLE_PYTHON=1 -DCAMERA_ENABLE_TESTS=1 -DCMAKE_INSTALL_PREFIX=$PREFIX -DCMAKE_FIND_ROOT_PATH=$PREFIX -DPython3_FIND_STRATEGY=LOCATION
cmake --build build --target install
