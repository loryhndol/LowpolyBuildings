# LowpolyBuildings

Implementation of `Xifeng Gao, Kui Wu, Zherong Pan. Low-poly Mesh Generation for Building Models. SIGGRAPH 2022`

## Dependencies

- CGAL(latest git): polygon mesh processing

- Eigen: linear algebra

- OpenMP: parallelism

- GLFW & GLAD: rendering

## Build

```bash
cmake -B build -DEigen3_DIR=<Eigen3_DIR> -DCGAL_DIR=<CGAL_DIR> -DGLFW_DIR=<GLFW_DIR> -DGLAD_DIR=<GLAD_DIR>
cmake --build build
```

## Demo

on Windows

```powershell
.\init.ps1
.\build\demo\Release\demo.exe .\resources\bunny5K.obj
```

## TODO

- The code is currently built and running on Windows 10 with Visual Studio 2022.
