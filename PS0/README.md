Eigen3 package needs to be installed, or the #include statements should be
modified appropriately to work with the included Eigen folder.

Each part can be compiled and run as follows. 

Part 1:
make
./obj_parser obj_file1.obj obj_file2.obj ... obj_fileN.obj

Part 2:
make
./transformation_parser transform_data1.txt

Part 3:
make
./data_loader test_file1.txt

Part 4:
make
./ppm_circle xres yres | display -
