#include "helpers.hpp"
#include "quaternion.hpp"

#include <Eigen/Dense>

#define GL_GLEXT_PROTOTYPES 1
#define GL_SILENCE_DEPRECATION
#include <GL/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>

#include <math.h>
#define _USE_MATH_DEFINES

#include <fstream>
#include <iostream>
#include <map>
#include <vector>

using namespace std;

extern GLenum readpng(const char *filename);

///////////////////////////////////////////////////////////////////////////////////////////////////

/* Function prototypes
 */

void init();
void read_shaders();
void display();

void init_lights();
void set_lights();
void draw_objects();

void mouse_pressed(int button, int state, int x, int y);
void mouse_moved(int x, int y);
void key_pressed(unsigned char key, int x, int y);

Eigen::Vector3f screen_to_ndc(int x, int y);
Eigen::Matrix4d get_current_rotation();

void create_light();
void create_square();

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
    /* See the note above and the comments in the 'draw_objects' and
     * 'create_cubes' functions for details about these buffer vectors.
     */
    vector<Triple> vertex_buffer;
    vector<Triple> normal_buffer;
    vector<Triple> tangent_buffer;
    vector<Triple> tex_coord_buffer;

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
float cam_position[] = {0.9, 0, 5.4};
float cam_orientation_axis[] = {0, 0, 1};

/* Angle in degrees.
 */
float cam_orientation_angle = 0;

float near_param = 1, far_param = 20,
      left_param = -0.5, right_param = 0.5,
      top_param = 0.5, bottom_param = -0.5;

///////////////////////////////////////////////////////////////////////////////////////////////////

/* Lists of lights and objects.
 */

vector<Point_Light> lights;
vector<Object> objects;

///////////////////////////////////////////////////////////////////////////////////////////////////

/* The following are parameters for creating an interactive Arcball mouse UI.
 */

int xres = 500;
int yres = 500;

int mouse_start_x, mouse_start_y;
int mouse_current_x, mouse_current_y;

bool is_pressed = false;

Quaternion last_rotation;
Quaternion current_rotation;

bool wireframe_mode = false;

///////////////////////////////////////////////////////////////////////////////////////////////////

/* Global variables for shader program and texture
 */
GLenum shader_program;
string vert_prog_filename, frag_prog_filename;
GLenum color_texture, normal_map;

 ///////////////////////////////////////////////////////////////////////////////////////////////////


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

    create_light();
    create_square();

    init_lights();

    // Arcball setup
    last_rotation = Quaternion::Identity();
    current_rotation = Quaternion::Identity();
}


/* Reads shader programs. */
void read_shaders() {
   string vert_program_source, frag_program_source;

   ifstream vert_prog_file(vert_prog_filename.c_str());
   if (!vert_prog_file)
      cerr << "Error opening vertex shader program\n";
   ifstream frag_prog_file(frag_prog_filename.c_str());
   if (!frag_prog_file)
      cerr << "Error opening fragment shader program\n";

   getline(vert_prog_file, vert_program_source, '\0');
   const char* vert_shader_source = vert_program_source.c_str();

   getline(frag_prog_file, frag_program_source, '\0');
   const char* frag_shader_source = frag_program_source.c_str();

   char buf[1024];
   GLsizei blah;

   // Initialize shaders
   GLenum vert_shader, frag_shader;

   shader_program = glCreateProgram();

   vert_shader = glCreateShader(GL_VERTEX_SHADER);
   glShaderSource(vert_shader, 1, &vert_shader_source, NULL);
   glCompileShader(vert_shader);

   GLint is_compiled = 0;
   glGetShaderiv(vert_shader, GL_COMPILE_STATUS, &is_compiled);
   if(is_compiled == GL_FALSE)
   {
      GLint max_length = 0;
      glGetShaderiv(vert_shader, GL_INFO_LOG_LENGTH, &max_length);

      // The maxLength includes the NULL character
      std::vector<GLchar> error_log(max_length);
      glGetShaderInfoLog(vert_shader, max_length, &max_length, &error_log[0]);

      // Provide the infolog in whatever manor you deem best.
      // Exit with failure.
      for (int i = 0; i < error_log.size(); i++)
         cout << error_log[i];
      glDeleteShader(vert_shader); // Don't leak the shader.
      return;
   }

   frag_shader = glCreateShader(GL_FRAGMENT_SHADER);
   glShaderSource(frag_shader, 1, &frag_shader_source, NULL);
   glCompileShader(frag_shader);

   is_compiled = 0;
   glGetShaderiv(frag_shader, GL_COMPILE_STATUS, &is_compiled);
   if(is_compiled == GL_FALSE)
   {
      GLint max_length = 0;
      glGetShaderiv(frag_shader, GL_INFO_LOG_LENGTH, &max_length);

      // The maxLength includes the NULL character
      std::vector<GLchar> error_log(max_length);
      glGetShaderInfoLog(frag_shader, max_length, &max_length, &error_log[0]);

      // Provide the infolog in whatever manor you deem best.
      // Exit with failure.
      for (int i = 0; i < error_log.size(); i++)
         cout << error_log[i];
      glDeleteShader(frag_shader); // Don't leak the shader.
      return;
   }

   glAttachShader(shader_program, vert_shader);
   glAttachShader(shader_program, frag_shader);
   glLinkProgram(shader_program);
   cerr << "Enabling fragment program: " << gluErrorString(glGetError()) << endl;
   glGetProgramInfoLog(shader_program, 1024, &blah, buf);
   cerr << buf;

   cerr << "Enabling program object" << endl;
   glUseProgram(shader_program);

   GLint color_texture_uniform_pos = glGetUniformLocation(shader_program, "color_texture");
   GLint normal_map_uniform_pos = glGetUniformLocation(shader_program, "normal_map");

   glActiveTexture(GL_TEXTURE0);
   glBindTexture(GL_TEXTURE_2D, color_texture);
   glUniform1i(color_texture_uniform_pos, 0);

   glActiveTexture(GL_TEXTURE1);
   glBindTexture(GL_TEXTURE_2D, normal_map);
   glUniform1i(normal_map_uniform_pos, 1);
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

            // Pass tangent and texture coordinate to vertexProgram as an attribute
            GLint tangent_attribute_loc = glGetAttribLocation(
                                                    shader_program, "tangent");
            GLint texCoord_attribute_loc = glGetAttribLocation(
                                                shader_program, "tex_coord");

            glVertexAttribPointer(tangent_attribute_loc, 3, GL_FLOAT, GL_FALSE,
                                    0, &objects[i].tangent_buffer[0]);
            glVertexAttribPointer(texCoord_attribute_loc, 3, GL_FLOAT, GL_FALSE,
                                    0, &objects[i].tex_coord_buffer[0]);

            glEnableVertexAttribArray(tangent_attribute_loc);
            glEnableVertexAttribArray(texCoord_attribute_loc);

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

/* Hardcodes a light source. */
void create_light()
{
    Point_Light light;

    light.position[0] = -0.8;
    light.position[1] = 0;
    light.position[2] = 1;
    light.position[3] = 1;

    light.color[0] = 1;
    light.color[1] = 1;
    light.color[2] = 1;
    light.attenuation_k = 0.2;

    lights.push_back(light);
}

/* Hardcodes a square object. */
void create_square() {
    Object square;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Reflectances
    ///////////////////////////////////////////////////////////////////////////////////////////////

    square.ambient_reflect[0] = 0.2;
    square.ambient_reflect[1] = 0.2;
    square.ambient_reflect[2] = 0.2;

    square.diffuse_reflect[0] = 0.6;
    square.diffuse_reflect[1] = 0.6;
    square.diffuse_reflect[2] = 0.6;

    square.specular_reflect[0] = 1;
    square.specular_reflect[1] = 1;
    square.specular_reflect[2] = 1;

    square.shininess = 5.0;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Points
    ///////////////////////////////////////////////////////////////////////////////////////////////

    Triple point1;
    point1.x = -1;
    point1.y = -1;
    point1.z = 0;

    Triple point2;
    point2.x = 1;
    point2.y = -1;
    point2.z = 0;

    Triple point3;
    point3.x = 1;
    point3.y = 1;
    point3.z = 0;

    Triple point4;
    point4.x = -1;
    point4.y = 1;
    point4.z = 0;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Normal and Tangent
    ///////////////////////////////////////////////////////////////////////////////////////////////

    Triple normal;
    normal.x = 0;
    normal.y = 0;
    normal.z = 1;

    Triple tangent;
    tangent.x = 1;
    tangent.y = 0;
    tangent.z = 0;


    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Texture Coordinates
    ///////////////////////////////////////////////////////////////////////////////////////////////

    Triple tex_coord1;
    tex_coord1.x = 0;
    tex_coord1.y = 0;
    tex_coord1.z = 0;

    Triple tex_coord2;
    tex_coord2.x = 1;
    tex_coord2.y = 0;
    tex_coord2.z = 0;

    Triple tex_coord3;
    tex_coord3.x = 1;
    tex_coord3.y = 1;
    tex_coord3.z = 0;

    Triple tex_coord4;
    tex_coord4.x = 0;
    tex_coord4.y = 1;
    tex_coord4.z = 0;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Vertex, normal, tangent, and texture coordinate arrays
    ///////////////////////////////////////////////////////////////////////////////////////////////

    /* Face 1: */

    square.vertex_buffer.push_back(point1);
    square.normal_buffer.push_back(normal);
    square.tangent_buffer.push_back(tangent);
    square.tex_coord_buffer.push_back(tex_coord1);

    square.vertex_buffer.push_back(point2);
    square.normal_buffer.push_back(normal);
    square.tangent_buffer.push_back(tangent);
    square.tex_coord_buffer.push_back(tex_coord2);

    square.vertex_buffer.push_back(point3);
    square.normal_buffer.push_back(normal);
    square.tangent_buffer.push_back(tangent);
    square.tex_coord_buffer.push_back(tex_coord3);

    /* Face 2: */

    square.vertex_buffer.push_back(point1);
    square.normal_buffer.push_back(normal);
    square.tangent_buffer.push_back(tangent);
    square.tex_coord_buffer.push_back(tex_coord1);

    square.vertex_buffer.push_back(point3);
    square.normal_buffer.push_back(normal);
    square.tangent_buffer.push_back(tangent);
    square.tex_coord_buffer.push_back(tex_coord3);

    square.vertex_buffer.push_back(point4);
    square.normal_buffer.push_back(normal);
    square.tangent_buffer.push_back(tangent);
    square.tex_coord_buffer.push_back(tex_coord4);

    /* Push to objects. */
    objects.push_back(square);
}

int main(int argc, char* argv[])
{
    if (argc != 3) {
        cerr << "Incorrect number of arguments" << endl;
        return 1;
    }

    char *color_texture_file = argv[1];
    char *normal_map_file = argv[2];

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(xres, yres);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("OpenGL Renderer");

    init();

    cerr << "Loading textures" << endl;
    if(!(color_texture = readpng(color_texture_file))) {
        cerr << "Error loading color texture" << endl;
        exit(1);
    }
    if(!(normal_map = readpng(normal_map_file))) {
        cerr << "Error loading normal map" << endl;
        exit(1);
    }

    vert_prog_filename = "src/vertexProgram.glsl";
    frag_prog_filename = "src/fragmentProgram.glsl";
    read_shaders();

    glutDisplayFunc(display);

    glutMouseFunc(mouse_pressed);
    glutMotionFunc(mouse_moved);
    glutKeyboardFunc(key_pressed);

    glutMainLoop();
}
