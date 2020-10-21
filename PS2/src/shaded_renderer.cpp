#include "image.hpp"
#include "scene.hpp"

#include <string>


using namespace std;


int main(int argc, char *argv[]) {
    if (argc != 5) {
        return 1;
    }

    string scene_filename = argv[1];
    int xres = stoi(argv[2]);
    int yres = stoi(argv[3]);
    int mode = stoi(argv[4]);

    Scene scene(scene_filename);
    Image image(xres, yres);
    scene.render_scene(image, mode);
    image.output_image();

    return 0;
}
