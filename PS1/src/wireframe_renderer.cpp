#include "scene.hpp"

#include <string>


using namespace std;


int main(int argc, char *argv[]) {
    string scene_filename = argv[1];
    int xres = stoi(argv[2]);
    int yres = stoi(argv[3]);

    Scene scene(scene_filename);
    scene.render_scene(xres, yres);
}
