### Dependencies

- C++11
- CMake:  `sudo apt-get install cmake`
- Eigen3: `sudo apt-get install libeigen3-dev`
- glog: https://github.com/google/glog

---

### Install

该工程默认编译不安装即可使用。如果准备安装该库，可以通过` -DEXPORT_BUILD_DIR=OFF `来开启安装。

```cmake
cmake -H. -B_build # 构建
cmake --build _build # 编译
cmake --target install # 安装
或者
cmake --build --build --install # 编译安装
```

or

```shell
mkdir build && cd build && cmake .. && make
```

---

### Documentation

GLib 目前分成 2 部分，一部分是常用的工具存放在 `utils` 文件夹。另一部分存放在 `math `文件夹中:

- `utils`: 一些常用工具，包含如下类:
  - `Random.cc`: 随机数生成器。包括无重复序列随机数、高斯分布随机数。
- `math`: 一些可能需要的数学工具。
  - `ransac`: 该文件夹中包含了 ransac 实现的基本框架。内嵌多模型拟合 3d 平面、2d 圆。