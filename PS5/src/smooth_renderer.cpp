#include "halfedge.hpp"
#include "helpers.hpp"
#include "quaternion.hpp"
#include "structs.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>
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

////////////////////////////////////////////////////////////////////////////////

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

    /* 'k' factor for attenuation. */
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
    // Halfedge data structures
    vector<HEV*> *hevs;
    vector<HEF*> *hefs;

    // Buffers for OpenGL specifications
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

// Time step for smoothing
float h;

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

    // Specify the inverse rotation of the camera by its orientation angle about
    // its orientation axis:
    glRotatef(-cam_orientation_angle,
              cam_orientation_axis[0], cam_orientation_axis[1], cam_orientation_axis[2]);

    glTranslatef(-cam_position[0], -cam_position[1], -cam_position[2]);

    glMultMatrixd(get_current_rotation().data());

    set_lights();
    draw_objects();
    glutSwapBuffers();
}

void init_lights()
{
    glEnable(GL_LIGHTING);

    int num_lights = lights.size();

    for(int i = 0; i < num_lights; ++i)
    {
        int light_id = GL_LIGHT0 + i;

        glEnable(light_id);

        glLightfv(light_id, GL_AMBIENT, lights[i].color);
        glLightfv(light_id, GL_DIFFUSE, lights[i].color);
        glLightfv(light_id, GL_SPECULAR, lights[i].color);

        glLightf(light_id, GL_QUADRATIC_ATTENUATION, lights[i].attenuation_k);
    }
}

void set_lights()
{
    int num_lights = lights.size();

    for(int i = 0; i < num_lights; ++i)
    {
        int light_id = GL_LIGHT0 + i;

        glLightfv(light_id, GL_POSITION, lights[i].position);
    }
}

/* This function has OpenGL render our objects to the display screen.
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
        glutPostRedisplay();
    }
}

// Converts HEV * to Eigen::Vector3f.
Eigen::Vector3f get_eigen_vec3(HEV *hev) {
    return Eigen::Vector3f{hev->x, hev->y, hev->z};
}

Eigen::Vector3f calc_face_normal(HEF *face) {
    Eigen::Vector3f v1 = get_eigen_vec3(face->edge->vertex);
    Eigen::Vector3f v2 = get_eigen_vec3(face->edge->next->vertex);
    Eigen::Vector3f v3 = get_eigen_vec3(face->edge->next->next->vertex);

    return (v2 - v1).cross(v3 - v1);
}

// Calculates and returns the area-weighted normal of the given vertex.
Vec3f calc_vertex_normal(HEV *vertex) {
    Eigen::Vector3f normal{0, 0, 0};

    HE *he = vertex->out;

    do {
        // compute the normal of the plane of the face
        Eigen::Vector3f face_normal = calc_face_normal(he->face);
        float face_area = face_normal.norm() / 2.f;

        // accummulate onto our normal vector
        normal += face_normal * face_area;

        // get halfedge to the next adjacent vertex
        he = he->flip->next;
    } while (he != vertex->out);

    return Vec3f{normal(0), normal(1), normal(2)};
}

// Fills vector_buffer and normal_buffer of the given object using the info in
// its Halfedge data structures, and stores normal value for each vertex in
// obj.hevs.
void fill_object_buffers(Object &obj) {
    for (HEF *hef : *obj.hefs) {
        HE *he = hef->edge;

        // Traverse the three vertices of the triangular face
        for (int _ = 0; _ < 3; _++) {
            HEV *hev = he->vertex;

            Vec3f n = calc_vertex_normal(hev);
            hev->normal = n;

            obj.vertex_buffer.push_back(
                Triple{(float)hev->x, (float)hev->y, (float)hev->z}
            );
            obj.normal_buffer.push_back(Triple{n.x, n.y, n.z});

            he = he->next;
        }
    }
}

// Parses the object from the given .obj file and stores it in obj.
//
// TODO: clean up comment / clean up memory
// This function parses .obj file to fill Mesh_Data. Then builds
// HEF and HEV and stores in obj. Then use HEF to to iterate through
// each face, and for each face's 3 vertices (get by traversing with
// half edge), compute vertex normal and fill vertex_buffer and
// normal_buffer in obj.
void parse_object(const string &filename, Object &obj)
{
    Mesh_Data *mesh = new Mesh_Data;
    mesh->vertices = new vector<Vertex*>;
    mesh->faces = new vector<Face*>;

    // To help with indexing from 1
    mesh->vertices->push_back(NULL);

    // Read .obj file and store vertices and faces in mesh.
    ifstream infile(filename);
    string t;

    while (infile >> t) {
        if (t == "v") {
            float x, y, z;
            infile >> x >> y >> z;

            mesh->vertices->push_back(new Vertex{x, y, z});
        }
        else if (t == "f") {
            int v1, v2, v3;
            infile >> v1 >> v2 >> v3;

            mesh->faces->push_back(new Face{v1, v2, v3});
        }
    }

    // Build Halfedge data structures
    obj.hevs = new vector<HEV*>();
    obj.hefs = new vector<HEF*>();
    build_HE(mesh, obj.hevs, obj.hefs);

    // Fill vector_buffer and normal_buffer
    fill_object_buffers(obj);
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

        // Store copy with transformations stored
        objects.push_back(obj);
    }
}

// Assigns an index to each vertex in hevs
void index_vertices(vector<HEV*> *hevs) {
    for (int i = 1; i < hevs->size(); i++) {
        hevs->at(i)->index = i;
    }
}

// Computes cotangent of the angle between the two given vectors.
float calc_cot(Eigen::Vector3f v1, Eigen::Vector3f v2) {
    return v1.dot(v2) / v1.cross(v2).norm();
}

// Computes and returns the total area sum of the indicent faces to given hev.
float calc_neighbor_area(HEV *hev) {
    float A = 0;

    HE *he = hev->out;

    do {
        // compute the area of the face
        Eigen::Vector3f face_normal = calc_face_normal(he->face);
        float face_area = face_normal.norm() / 2.f;
        A += face_area;

        // get halfedge to the next adjacent vertex
        he = he->flip->next;
    } while (he != hev->out);

    return A;
}

// Constructs an n x n sparse identity matrix.
Eigen::SparseMatrix<float> get_identity_matrix(int n) {
    Eigen::SparseMatrix<float> I{n, n};

    for (int i = 0; i < n; i++) {
        I.insert(i, i) = 1.f;
    }

    return I;
}

// Constructs F operator in matrix form, where F = I - hL and L is discrete
// Laplacian
Eigen::SparseMatrix<float> build_F_operator(vector<HEV*> *hevs) {
    index_vertices(hevs);

    // Index 0 is not a real vertex
    int num_vertices = hevs->size() - 1;

    Eigen::SparseMatrix<float> F{num_vertices, num_vertices};
    F.reserve(Eigen::VectorXi::Constant(num_vertices, 7));

    for (int i = 1; i < hevs->size(); i++) {
        HEV *hev1 = hevs->at(i);    // v_i

        // Compute the neighbor area sum A
        float A = calc_neighbor_area(hev1);

        if (A == 0.f) {
            // Entire row should be zero
            continue;
        }

        Eigen::Vector3f v1 = get_eigen_vec3(hev1);

        HE *he = hev1->out;

        // Iterate over all vertices adjacent to v_i
        do {
            HEV *hev2 = he->next->vertex;   // v_j
            HEV *hev3 = he->next->next->vertex;
            HEV *hev4 = he->flip->next->next->vertex;

            Eigen::Vector3f v2 = get_eigen_vec3(hev2);
            Eigen::Vector3f v3 = get_eigen_vec3(hev3);
            Eigen::Vector3f v4 = get_eigen_vec3(hev4);

            // Calculate cot(alpha)
            float cot_a = calc_cot(v2 - v3, v1 - v3);

            // Calculate cot(beta)
            float cot_b = calc_cot(v2 - v4, v1 - v4);

            int j = hev2->index;
            F.insert(i - 1, j - 1) = (cot_a + cot_b) / A;

            he = he->flip->next;
        } while (he != hevs->at(i)->out);

        F.coeffRef(i - 1, i - 1) -= F.row(i - 1).sum();
    }

    F = get_identity_matrix(num_vertices) - (h / 2.f) * F;
    F.makeCompressed();

    return F;
}

// Smoothes mesh using implicit fairing, by recomputing vertex coordinates and
// vertex normals.
void apply_implicit_fairing() {
    for (Object &obj : objects) {
        // Solve for new x, y, z coordinates using matrix operator F
        Eigen::SparseMatrix<float> F = build_F_operator(obj.hevs);
        Eigen::SparseLU<Eigen::SparseMatrix<float>, Eigen::COLAMDOrdering<int>>
            solver;
        solver.analyzePattern(F);
        solver.factorize(F);

        int num_vertices = obj.hevs->size() - 1;
        Eigen::VectorXf x_0{num_vertices};
        Eigen::VectorXf y_0{num_vertices};
        Eigen::VectorXf z_0{num_vertices};

        for (int i = 1; i < obj.hevs->size(); i++) {
            HEV *hev = obj.hevs->at(i);
            x_0(i - 1) = hev->x;
            y_0(i - 1) = hev->y;
            z_0(i - 1) = hev->z;
        }

        Eigen::VectorXf x_h{num_vertices};
        Eigen::VectorXf y_h{num_vertices};
        Eigen::VectorXf z_h{num_vertices};

        x_h = solver.solve(x_0);
        y_h = solver.solve(y_0);
        z_h = solver.solve(z_0);

        // Update x, y, z coordinate of every vertex in obj.hevs
        for (int i = 1; i < obj.hevs->size(); i++) {
            HEV *hev = obj.hevs->at(i);
            hev->x = x_h(i - 1);
            hev->y = y_h(i - 1);
            hev->z = z_h(i - 1);
        }

        // Update vector and normal buffers
        fill_object_buffers(obj);
    }
}

int main(int argc, char* argv[])
{
    if (argc != 5) {
        cout << "Incorrect number of arguments" << endl;
        return 1;
    }

    string scene_filename = argv[1];
    xres = stoi(argv[2]);
    yres = stoi(argv[3]);
    h = stof(argv[4]);    // time step

    // Parse the scene description file and put data into data structures.
    parse_scene(scene_filename);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(xres, yres);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("Smooth Renderer");

    init();
    glutDisplayFunc(display);

    glutMouseFunc(mouse_pressed);
    glutMotionFunc(mouse_moved);
    glutKeyboardFunc(key_pressed);

    glutMainLoop();
}
