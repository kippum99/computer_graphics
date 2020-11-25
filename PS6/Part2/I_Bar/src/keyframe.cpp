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


////////////////////////////////////////////////////////////////////////////////
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
    float s, x, y, z;
};

////////////////////////////////////////////////////////////////////////////////
/* Function prototypes
 */

void init(void);
void display(void);

void draw_frame();
void draw_IBar();
void parse_script();
void interpolate_transformations();

void key_pressed(unsigned char key, int x, int y);

Rotation quat2rot(Quaternion q);
Quaternion rot2quat(Rotation r);
Quaternion normalized(Quaternion q);

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

// Basis matrix for Catmull-Rom splines
Eigen::Matrix4f B;

////////////////////////////////////////////////////////////////////////////////
/* Screen resolution */

int xres;
int yres;

////////////////////////////////////////////////////////////////////////////////

/* Function implementations.
 */

/* Initializes and sets up the
 * program. It should always be called before anything else.
 */
void init(void)
{
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

    B << 0, 2, 0, 0,
        -1, 0, 1, 0,
        2, -5, 4, -1,
        -1, 3, -3, 1;
    B /= 2;

    // Interpolate transformations for all frames
    interpolate_transformations();
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

    // Draw frame
    glEnable(GL_LIGHTING);
    draw_frame();
    glDisable(GL_LIGHTING);

    glutSwapBuffers();
}

/* Draws the current frame for the animated IBar, applying transform
 * specifications for the frame. */
void draw_frame() {
    // Get current transformations
    Triple translation = translations[curr_frame];
    Triple scale = scales[curr_frame];
    Rotation rotation = quat2rot(rotations[curr_frame]);

    glPushMatrix();
    glTranslatef(translation.x, translation.y, translation.z);
    glScalef(scale.x, scale.y, scale.z);
    glRotatef(rotation.angle, rotation.x, rotation.y, rotation.z);
    draw_IBar();
    glPopMatrix();
}

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

/* Converts given angle in degrees to radians.
 */
float deg2rad(float angle)
{
    return angle * M_PI / 180.0;
}

/* Converts given angle in radians to degrees.
 */
float rad2deg(float angle)
{
    return angle / M_PI * 180.0;
}

Quaternion rot2quat(Rotation r) {
    float angle = deg2rad(r.angle);     // Angle in radians

    Quaternion q;
    q.s = cos(angle / 2);
    q.x = r.x * sin(angle / 2);
    q.y = r.y * sin(angle / 2);
    q.z = r.z * sin(angle / 2);

    return q;
}

Rotation quat2rot(Quaternion q) {
    // Choose an arbitrary rotation axis if sqrt is 0 (corresponds to
    // rotation angle being zero)
    float denom = sqrt(1 - pow(q.s, 2));

    if (denom == 0) {
        return Rotation{1, 0, 0, 0};
    }

    Rotation r;
    r.x = q.x / denom;
    r.y = q.y / denom;
    r.z = q.z / denom;
    r.angle = rad2deg(2 * acos(q.s));

    return r;
}

Quaternion normalized(Quaternion q) {
    float norm = sqrt(pow(q.s, 2) + pow(q.x, 2) + pow(q.y, 2) + pow(q.z, 2));
    assert(norm != 0);

    return Quaternion{q.s / norm, q.x / norm, q.y / norm, q.z / norm};
}

/* This function is meant to respond to key pressed on the keyboard. The
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
    /* If 'f' is pressed, step forward one frame.
     */
    else if (key == 'f')
    {
        curr_frame = (curr_frame + 1) % num_frames;
        glutPostRedisplay();
    }
}

/* Parses the script file and populates the lists of translation vectors,
 * scaling vectors, and rotation quaternions.
 */
void parse_script(const string &filename)
{
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

            Rotation r{rx, ry, rz, angle};
            rotations[curr_keyframe] = normalized(rot2quat(r));
        }

        getline(infile, line);
    }
}

/* Returns the value of f(u) where f(u) is the Catmull-Rom spline between
 * two control points p_i and p_i+1.
 *
 * vec_p = [p_i-1, p_i, p_i+1, p_i+2]
 */
float compute_f(float u, Eigen::Vector4f vec_p) {
    Eigen::Vector4f vec_u{1, u, pow(u, 2), pow(u, 3)};

    return vec_u.dot(B * vec_p);
}

/* Interpolate triple between points t2 and t3, given four control points
 * t1, t2, t3, and t4, and u in the unit domain.
 */
Triple interpolate_triple(const float u, const Triple &t1, const Triple &t2,
                            const Triple &t3, const Triple &t4) {
    // Interpolate each component of the triple
    Triple t;
    t.x = compute_f(u, Eigen::Vector4f{t1.x, t2.x, t3.x, t4.x});
    t.y = compute_f(u, Eigen::Vector4f{t1.y, t2.y, t3.y, t4.y});
    t.z = compute_f(u, Eigen::Vector4f{t1.z, t2.z, t3.z, t4.z});

    return t;
}

Quaternion interpolate_quaternion(const float u, const Quaternion &q1,
                                    const Quaternion &q2, const Quaternion &q3,
                                    const Quaternion &q4) {
    // Interpolate each component of the quaternion
    Quaternion q;
    q.s = compute_f(u, Eigen::Vector4f{q1.s, q2.s, q3.s, q4.s});
    q.x = compute_f(u, Eigen::Vector4f{q1.x, q2.x, q3.x, q4.x});
    q.y = compute_f(u, Eigen::Vector4f{q1.y, q2.y, q3.y, q4.y});
    q.z = compute_f(u, Eigen::Vector4f{q1.z, q2.z, q3.z, q4.z});

    return normalized(q);
}

/* Interpolates transformations for all frames and populate lists of
 * transformation vectors.
 */
void interpolate_transformations() {
    // For every pair of keyframes, interpolate transformation values for
    // each frame between the two keyframes.

    int num_keyframes = keyframes.size();

    for (int i = 0; i < num_keyframes; i++) {
        int keyframe1 = keyframes[i % num_keyframes];
        int keyframe2 = keyframes[(i + 1) % num_keyframes]; // May loop to 0

        // Need control point before keyframe1 and one after keyframe2 for
        // cardinal cubic splines
        int keyframe0 = keyframes[(num_keyframes + i - 1) % num_keyframes];
        int keyframe3 = keyframes[(i + 2) % num_keyframes];

        int max_frame = (num_frames + keyframe2 - 1) % num_frames;

        for (int frame = keyframe1 + 1; frame <= max_frame; frame++) {
            // Compute normalized value 0 < u < 1
            // Note that we use max_frame + 1 instead of keyframe2, because
            // keyframe2 may loop back to 0.
            float u = (float)(frame - keyframe1) / (max_frame + 1 - keyframe1);
            assert(u > 0 && u < 1);

            // Interpolate translation
            translations[frame] = interpolate_triple(u,
                                                        translations[keyframe0],
                                                        translations[keyframe1],
                                                        translations[keyframe2],
                                                        translations[keyframe3]
                                                        );

            // Interpolate scale
            scales[frame] = interpolate_triple(u,
                                                scales[keyframe0],
                                                scales[keyframe1],
                                                scales[keyframe2],
                                                scales[keyframe3]);

            // Interpolate rotation
            rotations[frame] = interpolate_quaternion(u,
                                                        rotations[keyframe0],
                                                        rotations[keyframe1],
                                                        rotations[keyframe2],
                                                        rotations[keyframe3]);
        }
    }
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

    // Parse the keyframe script file and store keyframes
    parse_script(keyframe_filename);

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
