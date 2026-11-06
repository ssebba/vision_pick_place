# Downstream packages need only the PCL components used by perception_core. A plain
# find_package(PCL) also loads VTK (visualization), whose CMake config fails on some
# distributions (missing MPI targets).
find_package(PCL REQUIRED COMPONENTS common filters search kdtree sample_consensus segmentation)
