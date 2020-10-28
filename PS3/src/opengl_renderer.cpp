#include "helpers.hpp"
#include "quaternion.hpp"

#include <Eigen/Dense>
#include <GL/glew.h>
#include <GL/glut.h>
#include <math.h>
#define _USE_MATH_DEFINES

#include <fstream>
#include <iostream>
#include <map>
#include <vector>

using namespace std;

///////////////////////////////////////////////////////////////////////////////////////////////////

/* Function prototypes
 */

void init(void);
void display(void);

void init_lights();
void set_lights();
void draw_objects();

void mouse_pressed(int button, int state, int x, int y);
void mouse_moved(int x, int y);
void key_pressed(unsigned char key, int x, int y);

Eigen::Vector3f screen_to_ndc(int x, int y);
Eigen::Matrix4d get_current_rotation();

///////////////////////////////////////////////////////////////////////////////////////////////////

/* Structs storing information for rendering
 */

/* The following struct is used for representing a point light.
 *
 * We specify the positions in the 'set_lights' function.
 */
struct Point_Light
{
    /* Index 0 has the x-coordinate
     * Index 1 has the y-coordinate
     * Index 2 has the z-coordinate
     * Index 3 has the w-coordinate
     */
    float position[4];

    /* Index 0 has the r-component
     * Index 1 has the g-component
     * Index 2 has the b-component
     */
    float color[3];

    /* 'k' factor for attenuation.
     */
    float attenuation_k;
};

/* The following struct is used for representing points and normals in world
 * coordinates.
 */
struct Triple
{
    float x;
    float y;
    float z;
};

struct Transform
{
    // 0: translation, 1: rotation, 2, scaling
    int transform_type;

    float x;
    float y;
    float z;

    // Angle in degrees
    float rotation_angle;
};

/* The following struct is used to represent objects.
 */
struct Object
{
    /* See the note above and the comments in the 'draw_objects' and
     * 'create_cubes' functions for details about these buffer vectors.
     */
    vector<Triple> vertex_buffer;
    vector<Triple> normal_buffer;

    vector<Transform> transforms;

    /* Index 0 has the r-component
     * Index 1 has the g-component
     * Index 2 has the b-component
     */
    float ambient_reflect[3];
    float diffuse_reflect[3];
    float specular_reflect[3];

    float shininess;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

/* The following are the typical camera specifications and parameters.
 */

/* Index 0 has the x-coordinate
 * Index 1 has the y-coordinate
 * Index 2 has the z-coordinate
 */
float cam_position[3];
float cam_orientation_axis[3];

/* Angle in degrees.
 */
float cam_orientation_angle;

float near_param, far_param, left_param, right_param, top_param, bottom_param;

///////////////////////////////////////////////////////////////////////////////////////////////////

/* Lists of lights and objects.
 */

vector<Point_Light> lights;
vector<Object> objects;

///////////////////////////////////////////////////////////////////////////////////////////////////

/* The following are parameters for creating an interactive Arcball mouse UI.
 */

int xres;
int yres;

int mouse_start_x, mouse_start_y;
int mouse_current_x, mouse_current_y;

float x_view_angle = 0, y_view_angle = 0;

bool is_pressed = false;

Quaternion last_rotation;
Quaternion current_rotation;

bool wireframe_mode = false;

/* Function implementations.
 */

/* Initializes and sets up the
 * program. It should always be called before anything else.
 */
void init(void)
{
    // Use "smooth shading" (aka Gouraud shading) when rendering.
    glShadeModel(GL_SMOOTH);

    // Use backface culling.
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    // Use depth buffering.
    glEnable(GL_DEPTH_TEST);

    // Automatically normalize our normal vectors.
    glEnable(GL_NORMALIZE);

    // Enable "vertex array" and "normal array" functionality.
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);

    // Work with Projection Matrix.
    glMatrixMode(GL_PROJECTION);

    glLoadIdentity();
    glFrustum(left_param, right_param,
              bottom_param, top_param,
              near_param, far_param);

    glMatrixMode(GL_MODELVIEW);

    init_lights();

    // Arcball setup
    last_rotation = Quaternion::Identity();
    current_rotation = Quaternion::Identity();
}

/* The 'display' function is supposed to handle all the processing of points
 * in world and camera space.
 */
void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    // glRotatef(y_view_angle, 1, 0, 0);
    // glRotatef(x_view_angle, 0, 1, 0);

    /* Specify the inverse rotation of the camera by its
     * orientation angle about its orientation axis:
     */
    glRotatef(-cam_orientation_angle,
              cam_orientation_axis[0], cam_orientation_axis[1], cam_orientation_axis[2]);

    glTranslatef(-cam_position[0], -cam_position[1], -cam_position[2]);

    glMultMatrixd(get_current_rotation().data());

    set_lights();
    draw_objects();
    glutSwapBuffers();
}

/* 'init_lights' function:
 *
 * This function has OpenGL enable its built-in lights to represent our point
 * lights.
 *
 * OpenGL has 8 built-in lights in all, each one with its own unique, integer
 * ID value. When setting the properties of a light, we need to tell OpenGL
 * the ID value of the light we are modifying.
 *
 * The first light's ID value is stored in 'GL_LIGHT0'. The second light's ID
 * value is stored in 'GL_LIGHT1'. And so on. The eighth and last light's ID
 * value is stored in 'GL_LIGHT7'.
 *
 * The properties of the lights are set using the 'glLightfv' and 'glLightf'
 * functions as you will see below.
 */
void init_lights()
{
    /* The following line of code tells OpenGL to enable lighting calculations
     * during its rendering process. This tells it to automatically apply the
     * Phong reflection model or lighting model to every pixel it will render.
     */
    glEnable(GL_LIGHTING);

    int num_lights = lights.size();

    for(int i = 0; i < num_lights; ++i)
    {
        /* In this loop, we are going to associate each of our point lights
         * with one of OpenGL's built-in lights. The simplest way to do this
         * is to just let our first point light correspond to 'GL_LIGHT0', our
         * second point light correspond to 'GL_LIGHT1', and so on. i.e. let:
         *
         * 'lights[0]' have an ID value of 'GL_LIGHT0'
         * 'lights[1]' have an ID value of 'GL_LIGHT1'
         * etc...
         */
        int light_id = GL_LIGHT0 + i;

        glEnable(light_id);

        /* The following lines of code use 'glLightfv' to set the color of
         * the light. The parameters for 'glLightfv' are:
         *
         * - enum light_ID: an integer between 'GL_LIGHT0' and 'GL_LIGHT7'
         * - enum property: this varies depending on what you are setting
         *                  e.g. 'GL_AMBIENT' for the light's ambient component
         * - float* values: a set of values to set for the specified property
         *                  e.g. an array of RGB values for the light's color
         *
         * OpenGL actually lets us specify different colors for the ambient,
         * diffuse, and specular components of the light. However, since we
         * are used to only working with one overall light color, we will
         * just set every component to the light color.
         */
        glLightfv(light_id, GL_AMBIENT, lights[i].color);
        glLightfv(light_id, GL_DIFFUSE, lights[i].color);
        glLightfv(light_id, GL_SPECULAR, lights[i].color);

        /* The following line of code sets the attenuation k constant of the
         * light. The difference between 'glLightf' and 'glLightfv' is that
         * 'glLightf' is used for when the parameter is only one value like
         * the attenuation constant while 'glLightfv' is used for when the
         * parameter is a set of values like a color array. i.e. the third
         * parameter of 'glLightf' is just a float instead of a float*.
         */
        glLightf(light_id, GL_QUADRATIC_ATTENUATION, lights[i].attenuation_k);
    }
}

/* 'set_lights' function:
 *
 * While the 'init_lights' function enables and sets the colors of the lights,
 * the 'set_lights' function is supposed to position the lights.
 *
 * You might be wondering why we do not just set the positions of the lights in
 * the 'init_lights' function in addition to the other properties. The reason
 * for this is because OpenGL does lighting computations after it applies the
 * Modelview Matrix to points. This means that the lighting computations are
 * effectively done in camera space. Hence, to ensure that we get the correct
 * lighting computations, we need to make sure that we position the lights
 * correctly in camera space.
 *
 * Now, the 'glLightfv' function, when used to position a light, applies all
 * the current Modelview Matrix to the given light position. This means that
 * to correctly position lights in camera space, we should call the 'glLightfv'
 * function to position them AFTER the Modelview Matrix has been modified by
 * the necessary camera transformations. As you can see in the 'display'
 * function, this is exactly what we do.
 */
void set_lights()
{
    int num_lights = lights.size();

    for(int i = 0; i < num_lights; ++i)
    {
        int light_id = GL_LIGHT0 + i;

        glLightfv(light_id, GL_POSITION, lights[i].position);
    }
}

/* 'draw_objects' function:
 *
 * This function has OpenGL render our objects to the display screen. It
 */
void draw_objects()
{
    int num_objects = objects.size();

    for(int i = 0; i < num_objects; ++i)
    {
        glPushMatrix();

        {
            int num_transforms = objects[i].transforms.size();

            for(int j = num_transforms - 1; j >= 0; j--)
            {
                if (objects[i].transforms[j].transform_type == 0) {
                    glTranslatef(objects[i].transforms[j].x,
                                objects[i].transforms[j].y,
                                objects[i].transforms[j].z);
                }
                else if (objects[i].transforms[j].transform_type == 1) {
                    glRotatef(objects[i].transforms[j].rotation_angle,
                                objects[i].transforms[j].x,
                                objects[i].transforms[j].y,
                                objects[i].transforms[j].z);
                }
                else {
                    glScalef(objects[i].transforms[j].x,
                                objects[i].transforms[j].y,
                                objects[i].transforms[j].z);
                }
            }

            glMaterialfv(GL_FRONT, GL_AMBIENT, objects[i].ambient_reflect);
            glMaterialfv(GL_FRONT, GL_DIFFUSE, objects[i].diffuse_reflect);
            glMaterialfv(GL_FRONT, GL_SPECULAR, objects[i].specular_reflect);
            glMaterialf(GL_FRONT, GL_SHININESS, objects[i].shininess);

            glVertexPointer(3, GL_FLOAT, 0, &objects[i].vertex_buffer[0]);
            glNormalPointer(GL_FLOAT, 0, &objects[i].normal_buffer[0]);

            int buffer_size = objects[i].vertex_buffer.size();

            if(!wireframe_mode)
                glDrawArrays(GL_TRIANGLES, 0, buffer_size);
            else
                for(int j = 0; j < buffer_size; j += 3)
                    glDrawArrays(GL_LINE_LOOP, j, 3);
        }

        glPopMatrix();
    }
}

Quaternion compute_rotation_quaternion(float start_x, float start_y,
                                        float current_x, float current_y)
{
    Eigen::Vector3f start_ndc = screen_to_ndc(start_x, start_y);
    Eigen::Vector3f current_ndc = screen_to_ndc(current_x, current_y);

    float angle = acos(min(1.f,
                    start_ndc.dot(current_ndc)
                    / start_ndc.norm() / current_ndc.norm()));
    Eigen::Vector3f u = start_ndc.cross(current_ndc);

    u = u.normalized();

    float r = cos(angle / 2);
    float i = u(0) * sin(angle / 2);
    float j = u(1) * sin(angle / 2);
    float k = u(2) * sin(angle / 2);

    return Quaternion{r, i, j, k};
}

Eigen::Matrix4d get_current_rotation() {
    Quaternion q = current_rotation * last_rotation;

    return q.get_rotation_matrix();
}

void mouse_pressed(int button, int state, int x, int y)
{
    /* If the left-mouse button was clicked down, then...
     */
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
        /* Store the mouse position in our global variables.
         */
        mouse_start_x = x;
        mouse_start_y = y;

        is_pressed = true;
    }
    /* If the left-mouse button was released up, then...
     */
    else if(button == GLUT_LEFT_BUTTON && state == GLUT_UP)
    {
        /* Mouse is no longer being pressed, so set our indicator to false.
         */
        is_pressed = false;

        last_rotation = current_rotation * last_rotation;
        current_rotation = Quaternion::Identity();
    }
}

void mouse_moved(int x, int y)
{
    /* If the left-mouse button is being clicked down...
     */
    if(is_pressed)
    {
        mouse_current_x = x;
        mouse_current_y = y;
        current_rotation = compute_rotation_quaternion(
                            mouse_start_x, mouse_start_y,
                            mouse_current_x, mouse_current_y);
        glutPostRedisplay();
    }
}

/* 'deg2rad' function:
 *
 * Converts given angle in degrees to radians.
 */
float deg2rad(float angle)
{
    return angle * M_PI / 180.0;
}

/* 'rad2deg' function:
 *
 * Converts given angle in radians to degrees.
 */
float rad2deg(float angle)
{
    return angle / M_PI * 180.0;
}

// Returns the (x', y', z') NDC coordinate of the given (x, y) screen
// coordinate, with Arcball mapping (map points on the surface of a unit
// sphere.)
Eigen::Vector3f screen_to_ndc(int x, int y) {
    float x_ndc = (float) x / xres * 2 - 1;
    float y_ndc = (float) (yres - 1 - y) / yres * 2 - 1;
    float z_ndc = 0;

    if (pow(x_ndc, 2) + pow(y_ndc, 2) <= 1) {
        z_ndc = sqrt(1 - pow(x_ndc, 2) - pow(y_ndc, 2));
    }

    return Eigen::Vector3f{x_ndc, y_ndc, z_ndc};
}

/* 'key_pressed' function:
 *
 * This function is meant to respond to key pressed on the keyboard. The
 * parameters are:
 *
 * - unsigned char key: the character of the key itself or the ASCII value of
 *                      of the key
 * - int x: the x screen coordinate of where the mouse was when the key was pressed
 * - int y: the y screen coordinate of where the mouse was when the key was pressed
 *
 * Our function is pretty straightforward as you can see below. We also do not make
 * use of the 'x' and 'y' parameters.
 */
void key_pressed(unsigned char key, int x, int y)
{
    /* If 'q' is pressed, quit the program.
     */
    if(key == 'q')
    {
        exit(0);
    }
    /* If 't' is pressed, toggle our 'wireframe_mode' boolean to make OpenGL
     * render our cubes as surfaces of wireframes.
     */
    else if(key == 't')
    {
        wireframe_mode = !wireframe_mode;
        /* Tell OpenGL that it needs to re-render our scene with the cubes
         * now as wireframes (or surfaces if they were wireframes before).
         */
        glutPostRedisplay();
    }
}

void parse_object(const string &filename, Object &obj)
{
    vector<Triple> vertices;
    vector<Triple> normals;

    // To help with indexing from 1
    vertices.push_back(Triple{0, 0, 0});
    normals.push_back(Triple{0, 0, 0});

    ifstream infile(filename);
    string t;

    while (infile >> t) {
        if (t == "v") {
            float x, y, z;
            infile >> x >> y >> z;
            Triple v{x, y, z};
            vertices.push_back(v);
        }
        else if (t == "vn") {
            float x, y, z;
            infile >> x >> y >> z;
            Triple vn{x, y, z};
            normals.push_back(vn);
        }
        else if (t == "f") {
            string s1, s2, s3;
            infile >> s1 >> s2 >> s3;
            int v1, v2, v3, n1, n2, n3;

            tokenize_string(s1, "//") >> v1 >> n1;
            tokenize_string(s2, "//") >> v2 >> n2;
            tokenize_string(s3, "//") >> v3 >> n3;

            obj.vertex_buffer.push_back(vertices[v1]);
            obj.vertex_buffer.push_back(vertices[v2]);
            obj.vertex_buffer.push_back(vertices[v3]);
            obj.normal_buffer.push_back(normals[n1]);
            obj.normal_buffer.push_back(normals[n2]);
            obj.normal_buffer.push_back(normals[n3]);
        }
    }
}

void parse_scene(const string &filename)
{
    ifstream infile(filename);
    string line;

    // Skip "camera:" token
    getline(infile, line);
    getline(infile, line);

    // Read camera and perspective paramters
    while (!line.empty()) {
        istringstream iss(line);
        string label;
        iss >> label;

        if (label == "position") {
            float tx, ty, tz;
            iss >> tx >> ty >> tz;
            cam_position[0] = tx;
            cam_position[1] = ty;
            cam_position[2] = tz;
        }
        else if (label == "orientation") {
            float rx, ry, rz, angle;
            iss >> rx >> ry >> rz >> angle;
            cam_orientation_axis[0] = rx;
            cam_orientation_axis[1] = ry;
            cam_orientation_axis[2] = rz;
            cam_orientation_angle = rad2deg(angle);
        }
        else if (label == "near") {
            iss >> near_param;
        }
        else if (label == "far") {
            iss >> far_param;
        }
        else if (label == "left") {
            iss >> left_param;
        }
        else if (label == "right") {
            iss >> right_param;
        }
        else if (label == "top") {
            iss >> top_param;
        }
        else if (label == "bottom") {
            iss >> bottom_param;
        }

        getline(infile, line);
    }

    getline(infile, line);

    // Read point light sources
    while (!line.empty()) {
        istringstream iss(line);
        string _;
        float x, y, z, r, g, b, k;

        iss >> _ >> x >> y >> z >> _ >> r >> g >> b >> _ >> k;

        Point_Light light;

        light.position[0] = x;
        light.position[1] = y;
        light.position[2] = z;
        light.position[3] = 1;

        light.color[0] = r;
        light.color[1] = g;
        light.color[2] = b;
        light.attenuation_k = k;

        lights.push_back(light);

        getline(infile, line);
    }

    // Skip empty line and "objects:" token
    getline(infile, line);
    getline(infile, line);

    // Read objects and filenames

    // Mapping from object label to original objects
    map<string, Object> object_map;

    while (!line.empty()) {
        int break_idx = line.find(" ");
        string obj_label = line.substr(0, break_idx);
        string filename = line.substr(break_idx + 1, line.length());
        Object obj;
        parse_object("data/" + filename, obj);
        object_map[obj_label] = obj;
        getline(infile, line);
    }

    // Read and store material fields and transformations
    while (getline(infile, line)) {
        string obj_label = line;

        // Creates a copy from the object stored in object_map
        Object obj = object_map[obj_label];

        getline(infile, line);

        // Read material fields for the object
        while (!line.empty()) {
            istringstream iss(line);
            string t;
            iss >> t;

            if (t == "ambient") {
                float r, g, b;
                iss >> r >> g >> b;
                obj.ambient_reflect[0] = r;
                obj.ambient_reflect[1] = g;
                obj.ambient_reflect[2] = b;
            }
            else if (t == "diffuse") {
                float r, g, b;
                iss >> r >> g >> b;
                obj.diffuse_reflect[0] = r;
                obj.diffuse_reflect[1] = g;
                obj.diffuse_reflect[2] = b;
            }
            else if (t == "specular") {
                float r, g, b;
                iss >> r >> g >> b;
                obj.specular_reflect[0] = r;
                obj.specular_reflect[1] = g;
                obj.specular_reflect[2] = b;
            }
            else if (t == "shininess") {
                iss >> obj.shininess;
                getline(infile, line);
                break;
            }

            getline(infile, line);
        }

        // Read all transformations for the object and store
        while (!line.empty()) {
            istringstream iss(line);
            string t;
            iss >> t;

            if (t == "t") {
                float tx, ty, tz;
                iss >> tx >> ty >> tz;
                obj.transforms.push_back(Transform{0, tx, ty, tz, 0});
            }
            else if (t == "r") {
                float rx, ry, rz, angle;
                iss >> rx >> ry >> rz >> angle;
                obj.transforms.push_back(
                                    Transform{1, rx, ry, rz, rad2deg(angle)});
            }
            else if (t == "s") {
                float sx, sy, sz;
                iss >> sx >> sy >> sz;
                obj.transforms.push_back(Transform{2, sx, sy, sz, 0});
            }

            if (infile.eof()) {
                break;
            }

            getline(infile, line);
        }

        // Store transformed copy
        objects.push_back(obj);
    }
}

/* The 'main' function:
 *
 * This function is short, but is basically where everything comes together.
 */
int main(int argc, char* argv[])
{
    if (argc != 4) {
        cout << "Incorrect number of arguments" << endl;
        return 1;
    }

    string scene_filename = argv[1];
    xres = stoi(argv[2]);
    yres = stoi(argv[3]);

    // Parse the scene description file and put data into data structures.
    parse_scene(scene_filename);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(xres, yres);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("OpenGL Renderer");

    init();
    glutDisplayFunc(display);

    glutMouseFunc(mouse_pressed);
    glutMotionFunc(mouse_moved);
    glutKeyboardFunc(key_pressed);

    glutMainLoop();
}
