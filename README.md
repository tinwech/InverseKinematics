# Inverse Kinematics

This repository implements inverse kinematics using C++ and OpenGL library.

## Demo

![ezgif-5-878900bbf4](https://github.com/tinwech/InverseKinematics/assets/80531783/d6f07b85-2d1f-4c43-bba3-270a8e95c645)


## Build on Microsoft Windows with Visual Studio 2017/2019

### Instruction

- Open InverseKinematics.sln
- Build
- Executable will be in ./bin

## Build on other platforms and/or compilers

### Prerequisite

- [Cmake](https://cmake.org) (version >= 3.14)
- Compiler (e.g. GCC)

### Instruction

- Run:

```bash=
cmake -S . -B build
cmake --build build --config Release --target install --parallel 8
```
- Executable will be in ./bin

If you are building on Linux, you need one of these dependencies, usually `xorg-dev`

- `xorg-dev` (For X11)
- `libwayland-dev wayland-protocols extra-cmake-modules libxkbcommon-dev` (For Wayland)
- `libosmesa6-dev` (For OSMesa)
