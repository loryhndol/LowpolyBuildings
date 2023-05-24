## LowpolyBuildings

Implementation of `Xifeng Gao, Kui Wu, Zherong Pan. Low-poly Mesh Generation for Building Models. SIGGRAPH 2022`

## Dependencies

- CGAL: polygon mesh processing

- Eigen: linear algebra

- OpenMP: parallelism

- GLFW & GLAD: rendering

## Build

```bash
cmake -B build -DEigen3_DIR=$HOME/mypackages/share/eigen3/cmake/
cmake --build build
```

## Run Demo
### Windows
```powershell
# copy engine's dll to the directory where the executable lies at
cp .\build\algorithms\Debug\Engine.dll .\build\demo\Debug
.\build\demo\Debug\demo.exe
```

## TODO
- testing
