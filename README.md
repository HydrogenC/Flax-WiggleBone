# Flax-WiggleBone
A Wiggle bone plugin for [FlaxEngine](https://github.com/FlaxEngine/FlaxEngine) (WIP). 

# How to use
Enable C++ scripting ([Docs](https://docs.flaxengine.com/manual/scripting/cpp/index.html)). 
Copy `Source/Game/WiggleScript.h` and `Source/Game/WiggleScript.cpp` to your project and use the script. 
P.S. Bones must be appended to the `TargetBones` property by the order from parent to child. 

# Work to do
Expose the project as a flax plugin
Use node selector instead of index to specify nodes
Use per-node damping and stiffness
Add attenuation to nodes closer to the root of the chain so that they are harder to wiggle
