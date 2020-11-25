#include "helpers.hpp"
// #include "quaternion.hpp"

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
/* Structs storing information for rendering
 */

/* The following struct is used for representing points and normals in world
 * coordinates.
 */
struct Triple
{
    float x;
    float y;
    float z;
};

struct Rotation {
    float x, y, z;
    float angle;    // angle in degrees
};

struct Quaternion {
    float r;
    float i;
    float j;
    float k;
};

///////////////////////////////////////////////////////////////////////////////////////////////////

/* Function prototypes
 */

void init(void);
void display(void);

// void init_lights();
// void set_lights();
// void draw_objects();
void draw_frame();
void draw_IBar();
void parse_script();
void interpolate_transformations();

// void mouse_pressed(int button, int state, int x, int y);
// void mouse_moved(int x, int y);
void key_pressed(unsigned char key, int x, int y);

Rotation quat2rot(Quaternion quat);
Quaternion rot2quat(Rotation rot);

Eigen::Vector3f screen_to_ndc(int x, int y);
// Eigen::Matrix4d get_current_rotation();


// struct Transform
// {
//     // 0: translation, 1: rotation, 2, scaling
//     int transform_type;
//
//     float x;
//     float y;
//     float z;
//
//     // Angle in degrees
//     float rotation_angle;
// };

// /* The following struct is used to represent objects.
//  */
// struct Object
// {
//     /* See the note above and the comments in the 'draw_objects' and
//      * 'create_cubes' functions for details about these buffer vectors.
//      */
//     vector<Triple> vertex_buffer;
//     vector<Triple> normal_buffer;
//
//     vector<Transform> transforms;
//
//     /* Index 0 has the r-component
//      * Index 1 has the g-component
//      * Index 2 has the b-component
//      */
//     float ambient_reflect[3];
//     float diffuse_reflect[3];
//     float specular_reflect[3];
//
//     float shininess;
// };

///////////////////////////////////////////////////////////////////////////////////////////////////

/* The following are the typical camera specifications and parameters.
 */

 /* Index 0 has the x-coordinate
  * Index 1 has the y-coordinate
  * Index 2 has the z-coordinate
  */
float cam_position[] = {0, 0, 40};

float near_param = 1.f, far_param = 60.f,
    left_param = -1.f, right_param = 1.f,
    top_param = 1.f, bottom_param = -1.f;

/* Arbitrary light */
const float light_color[3] = {1, 1, 1};
const float light_position[3] = {0, 0, -2};

/* Arbitrary material properties for IBar*/
const float ambient_reflect[3] = {1, 1, 1};
const float diffuse_reflect[3] = {1, 1, 1};
const float specular_reflect[3] = {1, 1, 1};
const float shininess = 0.1;

////////////////////////////////////////////////////////////////////////////////
/* I-Bar global variables. */

/* Needed to draw the cylinders using glu */
GLUquadricObj *quadratic;

////////////////////////////////////////////////////////////////////////////////
/* Animation global variables. */


// Total number of frames
int num_frames;

// Current frame number (loops from 0 to num_frames - 1)
int curr_frame;

// Vector of key frame numbers (i.e. [0, 15, 30, 45]), length = num_keyframes
vector<int> keyframes;

// Vector of translation vectors at all the frames, length = num_frames
vector<Triple> translations;

// Vector of scaling vectors at all the frames, length = num_frames
vector<Triple> scales;

// Vector of rotation quaternions at all the frames, length = num_frames
vector<Quaternion> rotations;


////////////////////////////////////////////////////////////////////////////////



// /* Lists of lights and objects.
//  */
//
// vector<Point_Light> lights;
// vector<Object> objects;

///////////////////////////////////////////////////////////////////////////////////////////////////


int xres;
int yres;

// int mouse_start_x, mouse_start_y;
// int mouse_current_x, mouse_current_y;

// bool is_pressed = false;

// Quaternion last_rotation;
// Quaternion current_rotation;

bool wireframe_mode = false;

/* Function implementations.
 */

/* Initializes and sets up the
 * program. It should always be called before anything else.
 */
void init(void)
{
    // // Use "smooth shading" (aka Gouraud shading) when rendering.
    // glShadeModel(GL_SMOOTH);
    //
    // // Use backface culling.
    // glEnable(GL_CULL_FACE);
    // glCullFace(GL_BACK);
    //
    // // Use depth buffering.
    // glEnable(GL_DEPTH_TEST);
    //
    // // Automatically normalize our normal vectors.
    // glEnable(GL_NORMALIZE);

    glEnable(GL_COLOR_MATERIAL);

    // Enable "vertex array" and "normal array" functionality.
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    // Set lighting
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_color);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_color);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_color);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);

    // Set material properties of IBar
    // glMaterialfv(GL_FRONT, GL_AMBIENT, ambient_reflect);
    // glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse_reflect);
    // glMaterialfv(GL_FRONT, GL_SPECULAR, specular_reflect);
    // glMaterialf(GL_FRONT, GL_SHININESS, shininess);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glFrustum(left_param, right_param,
              bottom_param, top_param,
              near_param, far_param);

    glMatrixMode(GL_MODELVIEW);

    // Set up quadratic for I-bar
    quadratic = gluNewQuadric();

    // Set up animation
    curr_frame = 0;

    // init_lights();
}

void reshape(int width, int height)
{
    height = (height == 0) ? 1 : height;
    width = (width == 0) ? 1 : width;

    glViewport(0, 0, width, height);

    glutPostRedisplay();
}

/* The 'display' function is supposed to handle all the processing of points
 * in world and camera space.
 */
void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glTranslatef(-cam_position[0], -cam_position[1], -cam_position[2]);

    // glMultMatrixd(get_current_rotation().data());

    // set_lights();
    // draw_objects();

    // Draw frame
    glEnable(GL_LIGHTING);
    draw_frame();
    glDisable(GL_LIGHTING);

    glutSwapBuffers();
}

// void init_lights()
// {
//     glEnable(GL_LIGHTING);
//
//     int num_lights = lights.size();
//
//     for(int i = 0; i < num_lights; ++i)
//     {
//         int light_id = GL_LIGHT0 + i;
//
//         glEnable(light_id);
//
//         glLightfv(light_id, GL_AMBIENT, lights[i].color);
//         glLightfv(light_id, GL_DIFFUSE, lights[i].color);
//         glLightfv(light_id, GL_SPECULAR, lights[i].color);
//
//         glLightf(light_id, GL_QUADRATIC_ATTENUATION, lights[i].attenuation_k);
//     }
// }

// void set_lights()
// {
//     int num_lights = lights.size();
//
//     for(int i = 0; i < num_lights; ++i)
//     {
//         int light_id = GL_LIGHT0 + i;
//
//         glLightfv(light_id, GL_POSITION, lights[i].position);
//     }
// }

/* Draws the current frame for the animated IBar, applying transform
 * specifications for the frame. */
void draw_frame() {
    cout << "curr frame " << curr_frame << endl;
    // Get current transformations
    Triple translation = translations[curr_frame];
    Triple scale = scales[curr_frame];
    Rotation rotation = quat2rot(rotations[curr_frame]);

    glPushMatrix();
    glTranslatef(translation.x, translation.y, translation.z);
    glScalef(scale.x, scale.y, scale.z);
    // glRotatef(rotation.angle, rotation.x, rotation.y, rotation.z);
    draw_IBar();
    glPopMatrix();
}

// TODO: remove this and just call from other file
void draw_IBar()
{
    /* Parameters for drawing the cylinders */
    float cyRad = 0.2, cyHeight = 1.0;
    int quadStacks = 4, quadSlices = 4;

    glPushMatrix();
    glColor3f(0, 0, 1);
    glTranslatef(0, cyHeight, 0);
    glRotatef(90, 1, 0, 0);
    gluCylinder(quadratic, cyRad, cyRad, 2.0 * cyHeight, quadSlices, quadStacks);
    glPopMatrix();

    glPushMatrix();
    glColor3f(0, 1, 1);
    glTranslatef(0, cyHeight, 0);
    glRotatef(90, 0, 1, 0);
    gluCylinder(quadratic, cyRad, cyRad, cyHeight, quadSlices, quadStacks);
    glPopMatrix();

    glPushMatrix();
    glColor3f(1, 0, 1);
    glTranslatef(0, cyHeight, 0);
    glRotatef(-90, 0, 1, 0);
    gluCylinder(quadratic, cyRad, cyRad, cyHeight, quadSlices, quadStacks);
    glPopMatrix();

    glPushMatrix();
    glColor3f(1, 1, 0);
    glTranslatef(0, -cyHeight, 0);
    glRotatef(-90, 0, 1, 0);
    gluCylinder(quadratic, cyRad, cyRad, cyHeight, quadSlices, quadStacks);
    glPopMatrix();

    glPushMatrix();
    glColor3f(0, 1, 0);
    glTranslatef(0, -cyHeight, 0);
    glRotatef(90, 0, 1, 0);
    gluCylinder(quadratic, cyRad, cyRad, cyHeight, quadSlices, quadStacks);
    glPopMatrix();
}

/* Updates IBar's position. */
void update()
{
    // float norm = sqrt(pow(m1.x, 2) + pow(m1.y, 2));
    //
    // m1.px -= dt * m1.k * m1.x * (norm - m1.rl) / norm;
    // m1.x += (dt / m1.m) * m1.px;
    //
    // m1.py = m1.py - dt * m1.k * m1.y * (norm - m1.rl) / norm + dt * m1.m * g;
    // m1.y += (dt / m1.m) * m1.py;
    //
    // t += dt;
}


//
//
// /* This function has OpenGL render our objects to the display screen.
//  */
// void draw_objects()
// {
//     int num_objects = objects.size();
//
//     for(int i = 0; i < num_objects; ++i)
//     {
//         glPushMatrix();
//
//         {
//             int num_transforms = objects[i].transforms.size();
//
//             for(int j = num_transforms - 1; j >= 0; j--)
//             {
//                 if (objects[i].transforms[j].transform_type == 0) {
//                     glTranslatef(objects[i].transforms[j].x,
//                                 objects[i].transforms[j].y,
//                                 objects[i].transforms[j].z);
//                 }
//                 else if (objects[i].transforms[j].transform_type == 1) {
//                     glRotatef(objects[i].transforms[j].rotation_angle,
//                                 objects[i].transforms[j].x,
//                                 objects[i].transforms[j].y,
//                                 objects[i].transforms[j].z);
//                 }
//                 else {
//                     glScalef(objects[i].transforms[j].x,
//                                 objects[i].transforms[j].y,
//                                 objects[i].transforms[j].z);
//                 }
//             }
//
//             glMaterialfv(GL_FRONT, GL_AMBIENT, objects[i].ambient_reflect);
//             glMaterialfv(GL_FRONT, GL_DIFFUSE, objects[i].diffuse_reflect);
//             glMaterialfv(GL_FRONT, GL_SPECULAR, objects[i].specular_reflect);
//             glMaterialf(GL_FRONT, GL_SHININESS, objects[i].shininess);
//
//             glVertexPointer(3, GL_FLOAT, 0, &objects[i].vertex_buffer[0]);
//             glNormalPointer(GL_FLOAT, 0, &objects[i].normal_buffer[0]);
//
//             int buffer_size = objects[i].vertex_buffer.size();
//
//             if(!wireframe_mode)
//                 glDrawArrays(GL_TRIANGLES, 0, buffer_size);
//             else
//                 for(int j = 0; j < buffer_size; j += 3)
//                     glDrawArrays(GL_LINE_LOOP, j, 3);
//         }
//
//         glPopMatrix();
//     }
// }

// Quaternion compute_rotation_quaternion(float start_x, float start_y,
//                                         float current_x, float current_y)
// {
//     Eigen::Vector3f start_ndc = screen_to_ndc(start_x, start_y);
//     Eigen::Vector3f current_ndc = screen_to_ndc(current_x, current_y);
//
//     float angle = acos(min(1.f,
//                     start_ndc.dot(current_ndc)
//                     / start_ndc.norm() / current_ndc.norm()));
//     Eigen::Vector3f u = start_ndc.cross(current_ndc);
//     u = u.normalized();
//
//     float r = cos(angle / 2);
//     float i = u(0) * sin(angle / 2);
//     float j = u(1) * sin(angle / 2);
//     float k = u(2) * sin(angle / 2);
//
//     return Quaternion{r, i, j, k};
// }

// Eigen::Matrix4d get_current_rotation() {
//     Quaternion q = current_rotation * last_rotation;
//
//     return q.get_rotation_matrix();
// }

// void mouse_pressed(int button, int state, int x, int y)
// {
//     /* If the left-mouse button was clicked down, then...
//      */
//     if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
//     {
//         /* Store the mouse position in our global variables.
//          */
//         mouse_start_x = x;
//         mouse_start_y = y;
//
//         is_pressed = true;
//     }
//     /* If the left-mouse button was released up, then...
//      */
//     else if(button == GLUT_LEFT_BUTTON && state == GLUT_UP)
//     {
//         /* Mouse is no longer being pressed, so set our indicator to false.
//          */
//         is_pressed = false;
//
//         last_rotation = current_rotation * last_rotation;
//         current_rotation = Quaternion::Identity();
//     }
// }
//
// void mouse_moved(int x, int y)
// {
//     /* If the left-mouse button is being clicked down...
//      */
//     if(is_pressed)
//     {
//         mouse_current_x = x;
//         mouse_current_y = y;
//         // current_rotation = compute_rotation_quaternion(
//         //                     mouse_start_x, mouse_start_y,
//         //                     mouse_current_x, mouse_current_y);
//         glutPostRedisplay();
//     }
// }

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

// TODO: COMPLETE
Quaternion rot2quat(Rotation rot) {
    return Quaternion{};
}

Rotation quat2rot(Quaternion quat) {
    return Rotation{};
}

/* Returns the (x', y', z') NDC coordinate of the given (x, y) screen
 * coordinate, with Arcball mapping (map points on the surface of a unit
 * sphere.)
 */
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
 */
void key_pressed(unsigned char key, int x, int y)
{
    /* If 'q' is pressed, quit the program.
     */
    if (key == 'q')
    {
        exit(0);
    }
    /* If 't' is pressed, toggle our 'wireframe_mode' boolean to make OpenGL
     * render our cubes as surfaces of wireframes.
     */
    else if (key == 't')
    {
        wireframe_mode = !wireframe_mode;
        glutPostRedisplay();
    }
    /* If 'f' is pressed, step forward one frame.
     */
    else if (key == 'f')
    {
        curr_frame = (curr_frame + 1) % num_frames;
        glutPostRedisplay();
    }
}
//
// void parse_object(const string &filename, Object &obj)
// {
//     vector<Triple> vertices;
//     vector<Triple> normals;
//
//     // To help with indexing from 1
//     vertices.push_back(Triple{0, 0, 0});
//     normals.push_back(Triple{0, 0, 0});
//
//     ifstream infile(filename);
//     string t;
//
//     while (infile >> t) {
//         if (t == "v") {
//             float x, y, z;
//             infile >> x >> y >> z;
//             Triple v{x, y, z};
//             vertices.push_back(v);
//         }
//         else if (t == "vn") {
//             float x, y, z;
//             infile >> x >> y >> z;
//             Triple vn{x, y, z};
//             normals.push_back(vn);
//         }
//         else if (t == "f") {
//             string s1, s2, s3;
//             infile >> s1 >> s2 >> s3;
//             int v1, v2, v3, n1, n2, n3;
//
//             tokenize_string(s1, "//") >> v1 >> n1;
//             tokenize_string(s2, "//") >> v2 >> n2;
//             tokenize_string(s3, "//") >> v3 >> n3;
//
//             obj.vertex_buffer.push_back(vertices[v1]);
//             obj.vertex_buffer.push_back(vertices[v2]);
//             obj.vertex_buffer.push_back(vertices[v3]);
//             obj.normal_buffer.push_back(normals[n1]);
//             obj.normal_buffer.push_back(normals[n2]);
//             obj.normal_buffer.push_back(normals[n3]);
//         }
//     }
// }

/* Parses the script file and populates the lists of translation vectors,
 * scaling vectors, and rotation quaternions.
 */
void parse_script(const string &filename)
{
    // Vector of key frame numbers (i.e. [0, 15, 30, 45]), length = num_keyframes
    // Vector of translation vectors at all the frames, length = num_frames
    // Vector of scaling vectors at all the frames, length = num_frames
    // Vector of rotation quaternions at all the frames, length = num_frames


    ifstream infile(filename);
    string line;

    // Get total number of frames in the animation
    getline(infile, line);
    num_frames = stoi(line);

    // Initialize transformation vectors
    translations = vector<Triple>(num_frames);
    scales = vector<Triple>(num_frames);
    rotations = vector<Quaternion>(num_frames);

    // Read keyframes
    while (!line.empty()) {
        istringstream iss(line);
        string label;
        iss >> label;

        int curr_keyframe;

        if (label == "Frame") {
            iss >> curr_keyframe;
            keyframes.push_back(curr_keyframe);
        }
        else if (label == "translation") {
            float tx, ty, tz;
            iss >> tx >> ty >> tz;

            translations[curr_keyframe] = Triple{tx, ty, tz};
        }
        else if (label == "scale") {
            float sx, sy, sz;
            iss >> sx >> sy >> sz;

            scales[curr_keyframe] = Triple{sx, sy, sz};
        }
        else if (label == "rotation") {
            float rx, ry, rz, angle;
            iss >> rx >> ry >> rz >> angle;

            rotations[curr_keyframe] = rot2quat(Rotation{rx, ry, rz, angle});
        }

        getline(infile, line);
    }
}

void interpolate_transformations() {

}

int main(int argc, char* argv[])
{
    if (argc != 4) {
        cout << "Incorrect number of arguments" << endl;
        return 1;
    }

    string keyframe_filename = argv[1];
    xres = stoi(argv[2]);
    yres = stoi(argv[3]);

    // // Parse the scene description file and put data into data structures.
    // parse_scene(scene_filename);

    // Parse the keyframe script file and store keyframes
    parse_script(keyframe_filename);

    // Interpolate transformations for all frames to populate lists of
    // transformation vectors.
    interpolate_transformations();

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(xres, yres);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("I-Bar Animator");

    init();
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(key_pressed);
    glutMainLoop();
}
