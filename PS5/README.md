Eigen3 package needs to be installed, or the #include statements should be
modified appropriately to work with the included Eigen folder.

The program can be compiled and run as follows:

make
./smooth [scene_description_file.txt] [xres] [yres] [h]


Key binding for smoothing using implicit fairing is "s".


Notes on how to build F as a matrix operator:
Denoting the matrix operator for the Laplacian as L, we can break up the
expression inside the summation into two (one for x_j and one for x_i) and move
A into inside the summation, such that we can write L = (1 / 2)(B - C) where
B and C are both n x n matrices.
B_ij is cot(alpha(i, j)) + cot(beta(i, j)) / A_i if v_i and v_j are adjacent,
and 0 otherwise. C is a diagonal matrix, where C_ii is the summation of all
the elements in the i-th row of B. Then we can write F = I - hL.
