#!/bin/bash
current_directory=$(pwd)
project_root_dir=${current_directory}/..

cd ${project_root_dir}

# compile opencv
mkdir -p cd ${project_root_dir}/third_party
cd ${project_root_dir}/third_party
if [ ! -d "eigen" ]; then
  git clone https://gitlab.com/libeigen/eigen.git
  cd eigen
  git checkout 3dc3a0ea2d0773af4c0ffd7bbcb21c608e28fcef
fi

cd ${project_root_dir}/third_party/eigen
rm -rf build install
mkdir -p build && mkdir -p install && cd build
cmake -DCMAKE_INSTALL_PREFIX=${project_root_dir}/third_party/eigen/install -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j 4
cmake --install .

# compile gflags
mkdir -p cd ${project_root_dir}/third_party
cd ${project_root_dir}/third_party
if [ ! -d "gflags" ]; then
  git clone git@github.com:gflags/gflags.git
  cd gflags
  git checkout e171a
fi

cd ${project_root_dir}/third_party/gflags
rm -rf build install
mkdir -p build && mkdir -p install && cd build
cmake -DCMAKE_INSTALL_PREFIX=${project_root_dir}/third_party/gflags/install -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j 4
cmake --install .

# compile glogs
mkdir -p cd ${project_root_dir}/third_party
cd ${project_root_dir}/third_party
if [ ! -d "glog" ]; then
  git clone git@github.com:google/glog.git
  cd glog
  git checkout 8f9ccf
fi

cd ${project_root_dir}/third_party/glog
rm -rf build install
mkdir -p build && mkdir -p install && cd build
cmake -DCMAKE_INSTALL_PREFIX=${project_root_dir}/third_party/glog/install -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:${project_root_dir}/third_party/gflags/install/lib ..
cmake --build . -j 4
cmake --install .

# compile ceres
mkdir -p cd ${project_root_dir}/third_party
cd ${project_root_dir}/third_party
if [ ! -d "ceres-solver" ]; then
  git clone git@github.com:ceres-solver/ceres-solver.git
  cd ceres-solver
  git checkout 8533139
fi

cd ${project_root_dir}/third_party/ceres-solver
rm -rf build install
mkdir -p build && mkdir -p install && cd build
cmake -DCMAKE_INSTALL_PREFIX=${project_root_dir}/third_party/ceres-solver/install -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:${project_root_dir}/third_party/eigen/install/share:${project_root_dir}/third_party/gflags/install/lib:${project_root_dir}/third_party/glog/install/lib .. -G "Unix Makefiles"
cmake --build . -j8
cmake --install .