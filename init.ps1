# copy engine's dll to the directory where the executable lies at
cp .\build\algorithms\Debug\Engine.dll .\build\demo\Debug;
mkdir -p .\build\demo\Debug\resources\shaders;
mkdir -p .\build\demo\Debug\tmp;
# copy the shaders
cp .\resources\shaders\shaderVert.glsl .\build\demo\Debug\resources\shaders;
# copy the shaders
cp .\resources\shaders\shaderFrag.glsl .\build\demo\Debug\resources\shaders;