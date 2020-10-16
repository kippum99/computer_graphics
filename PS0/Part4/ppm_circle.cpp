#include <cmath>
#include <iostream>

using namespace std;

int main(int argc, char *argv[]) {
    int xres = stoi(argv[1]);
    int yres = stoi(argv[2]);
    double radius = min(xres, yres) / 4;

    // Print header, resolution, and max pixel intensity
    cout << "P3" << endl;
    cout << xres << " " << yres << endl;
    cout << 255 << endl;

    // Print grid
    for (int i = 0; i < yres; i++) {
        for (int j = 0; j < xres; j++) {
            // x and y positions offset such that origin is center of grid
            double xpos = j - xres / 2;
            double ypos = i - yres / 2;

            if (pow(xpos, 2) + pow(ypos, 2) <= pow(radius, 2)) {
                cout << 255 << " " << 255 << " " << 255 << endl;
            }
            else {
                cout << 0 << " " << 0 << " " << 0 << endl;
            }
        }
    }
}
