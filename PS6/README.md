Eigen3 package needs to be installed, or the #include statements should be
modified appropriately to work with the included Eigen folder.


The programs can be compiled and run as follows for each part:


Part1/Single_Spring_Pendulum:

make
./single_pendulum [xres] [yres] [x_start] [y_start]


Part1/Double_Spring_Pendulum:

make
./double_pendulum [xres] [yres] [x_start_1] [y_start_1] [x_start_2] [y_start_2]


Part1/Elasticity:

make
./simulate [obj_filename]


Part2/I_Bar:

make
./keyframe [script_filename] [xres] [yres]

Key binding for stepping forward one frame is "f".


Part2/Bunny_Frames:

make
./frames_generator

The program generates 16 .obj files in the output folder.
