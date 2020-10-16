Eigen3 package needs to be installed, or the #include statements should be
modified appropriately to work with the included Eigen folder.

The program can be compiled and run as follows:

make
./wireframe_renderer [scene_description_file.txt] [xres] [yres]


Bresenham's line algorithm can be generalized by swapping coordinates
appropriately. The algorithm that works for the first octant could be slightly
modified to support negative slope, to make it work for the eighth octant.
I defined a function low_slope_bresenham() that rasterizes lines in the first
and eighth octants, which correspond to a low slope (magnitude less than 1).
I defined an additional function high_slope_bresenham() that rasterizes lines
in the second and seventh octants, which correspond to a high slope
(magnitude greater than 1). Also note that the first and the fifth octant are
equivalent if we switch the two coordinates, and it works similarly for second
and sixth, third and seventh, and fourth and eight. Thus we can call
low_slope_bresenham(x1, y1, x2, y2) for first and eighth octants,
low_slope_bresemham(x2, y2, x1, y1) for fourth and fifth octants,
high_slope_bresenham(x1, y1, x2, y2) for the second and seventh octants,
and high_slope_brensemham(x2, y2, x1, y1) for the third and sixth octants.
