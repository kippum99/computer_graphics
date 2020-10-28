#include "helpers.hpp"

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
void reshape(int width, int height);
void display(void);

void init_lights();
void set_lights();
void draw_objects();

void mouse_pressed(int button, int state, int x, int y);
void mouse_moved(int x, int y);
void key_pressed(unsigned char key, int x, int y);

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
 *
 * The main things to note here are the 'vertex_buffer' and 'normal_buffer'
 * vectors.
 *
 * You will see later in the 'draw_objects' function that OpenGL requires
 * us to supply it all the faces that make up an object in one giant
 * "vertex array" before it can render the object. The faces are each specified
 * by the set of vertices that make up the face, and the giant "vertex array"
 * stores all these sets of vertices consecutively. Our "vertex_buffer" vector
 * below will be our "vertex array" for the object.
 *
 * As an example, let's say that we have a cube object. A cube has 6 faces,
 * each with 4 vertices. Each face is going to be represented by the 4 vertices
 * that make it up. We are going to put each of these 4-vertex-sets one by one
 * into 1 large array. This gives us an array of 36 vertices. e.g.:
 *
 * [face1vertex1, face1vertex2, face1vertex3, face1vertex4,
 *  face2vertex1, face2vertex2, face2vertex3, face2vertex4,
 *  face3vertex1, face3vertex2, face3vertex3, face3vertex4,
 *  face4vertex1, face4vertex2, face4vertex3, face4vertex4,
 *  face5vertex1, face5vertex2, face5vertex3, face5vertex4,
 *  face6vertex1, face6vertex2, face6vertex3, face6vertex4]
 *
 * This array of 36 vertices becomes our 'vertex_array'.
 *
 * While it may be obvious to us that some of the vertices in the array are
 * repeats, OpenGL has no way of knowing this. The redundancy is necessary
 * since OpenGL needs the vertices of every face to be explicitly given.
 *
 * The 'normal_buffer' stores all the normals corresponding to the vertices
 * in the 'vertex_buffer'. With the cube example, since the "vertex array"
 * has "36" vertices, the "normal array" also has "36" normals.
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

/* The following are the typical camera specifications and parameters. In
 * general, it is a better idea to keep all this information in a camera
 * struct, like how we have been doing it in Assignemtns 1 and 2. However,
 * if you only have one camera for the scene, and all your code is in one
 * file (like this one), then it is sometimes more convenient to just have
 * all the camera specifications and parameters as global variables.
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

/* The following are parameters for creating an interactive first-person camera
 * view of the scene. The variables will make more sense when explained in
 * context, so you should just look at the 'mousePressed', 'mouseMoved', and
 * 'keyPressed' functions for the details.
 */

int mouse_x, mouse_y;
float mouse_scale_x, mouse_scale_y;

const float step_size = 0.2;
const float x_view_step = 90.0, y_view_step = 90.0;
float x_view_angle = 0, y_view_angle = 0;

bool is_pressed = false;
bool wireframe_mode = false;

///////////////////////////////////////////////////////////////////////////////////////////////////

/* The following function prototypes are for helper functions that we made to
 * initialize some point lights and cube objects.
 */

void create_lights();
void create_cubes();

///////////////////////////////////////////////////////////////////////////////////////////////////

/* Function implementations.
 */


/* 'init' function:
 *
 * As you would expect, the 'init' function initializes and sets up the
 * program. It should always be called before anything else.
 *
 * Writing an 'init' function is not required by OpenGL. If you wanted to, you
 * could just put all your initializations in the beginning of the 'main'
 * function instead. However, doing so is bad style; it is cleaner to have all
 * your initializations contained within one function.
 *
 * Before we go into the function itself, it is important to mention that
 * OpenGL works like a state machine. It will do different procedures depending
 * on what state it is in.
 *
 * For instance, OpenGL has different states for its shading procedure. By
 * default, OpenGL is in "flat shading state", meaning it will always use flat
 * shading when we tell it to render anything. With some syntax, we can change
 * the shading procedure from the "flat shading state" to the "Gouraud shading
 * state", and then OpenGL will render everything using Gouraud shading.
 *
 * The most important task of the 'init' function is to set OpenGL to the
 * states that we want it to be in.
 */
void init(void)
{
    /* The following line of code tells OpenGL to use "smooth shading" (aka
     * Gouraud shading) when rendering.
     *
     * Yes. This is actually all you need to do to use Gouraud shading in
     * OpenGL (besides providing OpenGL the vertices and normals to render).
     * Short and sweet, right?
     *
     * If you wanted to tell OpenGL to use flat shading at any point, then you
     * would use the following line:

       glShadeModel(GL_FLAT);

     * Phong shading unfortunately requires GLSL, so it will be covered in a
     * later demo.
     */
    glShadeModel(GL_SMOOTH);

    /* The next line of code tells OpenGL to use "culling" when rendering. The
     * line right after it tells OpenGL that the particular "culling" technique
     * we want it to use is backface culling.
     *
     * "Culling" is actually a generic term for various algorithms that
     * prevent the rendering process from trying to render unnecessary
     * polygons. Backface culling is the most commonly used method, but
     * there also exist other, niche methods like frontface culling.
     */
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    /* The following line tells OpenGL to use depth buffering when rendering.
     */
    glEnable(GL_DEPTH_TEST);

     /* The following line tells OpenGL to automatically normalize our normal
     * vectors before it passes them into the normal arrays discussed below.
     * This is required for correct lighting, but it also slows down our
     * program. An alternative to this is to manually scale the normal vectors
     * to correct for each scale operation we call. For instance, if we were
     * to scale an object by 3 (via glScalef() discussed below), then
     * OpenGL would scale the normals of the object by 1/3, as we would
     * expect from the inverse normal transform. But since we need unit
     * normals for lighting, we would either need to enable GL_NORMALIZE
     * or manually scale our normals by 3 before passing them into the
     * normal arrays; this is of course to counteract the 1/3 inverse
     * scaling when OpenGL applies the normal transforms. Enabling GL_NORMALIZE
     * is more convenient, but we sometimes don't use it if it slows down
     * our program too much.
     */
    glEnable(GL_NORMALIZE);

    /* The following two lines tell OpenGL to enable its "vertex array" and
     * "normal array" functionality. More details on these arrays are given
     * in the comments on the 'Object' struct and the 'draw_objects' and
     * 'create_objects' functions.
     */
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);

    /* The next 4 lines work with OpenGL's two main matrices: the "Projection
     * Matrix" and the "Modelview Matrix". Only one of these two main matrices
     * can be modified at any given time. We specify the main matrix that we
     * want to modify with the 'glMatrixMode' function.
     *
     * The Projection Matrix is the matrix that OpenGL applies to points in
     * camera space. For our purposes, we want the Projection Matrix to be
     * the perspective projection matrix, since we want to convert points into
     * NDC after they are in camera space.
     *
     * The line of code below:
     */
    glMatrixMode(GL_PROJECTION);
    /* ^tells OpenGL that we are going to modify the Projection Matrix. From
     * this point on, any matrix comamnds we give OpenGL will affect the
     * Projection Matrix. For instance, the line of code below:
     */
    glLoadIdentity();
    /* ^tells OpenGL to set the current main matrix (which is the Projection
     * Matrix right now) to the identity matrix. Then, the next line of code:
     */
    glFrustum(left_param, right_param,
              bottom_param, top_param,
              near_param, far_param);
    /* ^ tells OpenGL to create a perspective projection matrix using the
     * given frustum parameters. OpenGL then post-multiplies the current main
     * matrix (the Projection Matrix) with the created matrix. i.e. let 'P'
     * be our Projection Matrix and 'F' be the matrix created by 'glFrustum'.
     * Then, after 'F' is created, OpenGL performs the following operation:
     *
     * P = P * F
     *
     * Since we had set the Projection Matrix to the identity matrix before the
     * call to 'glFrustum', the above multiplication results in the Projection
     * Matrix being the perspective projection matrix, which is what we want.
     */

    /* The Modelview Matrix is the matrix that OpenGL applies to untransformed
     * points in world space. OpenGL applies the Modelview Matrix to points
     * BEFORE it applies the Projection Matrix.
     *
     * Thus, for our purposes, we want the Modelview Matrix to be the overall
     * transformation matrix that we apply to points in world space before
     * applying the perspective projection matrix. This means we would need to
     * factor in all the individual object transformations and the camera
     * transformations into the Modelview Matrix.
     *
     * The following line of code tells OpenGL that we are going to modify the
     * Modelview Matrix. From this point on, any matrix commands we give OpenGL
     * will affect the Modelview Matrix.
     *
     * We generally modify the Modelview Matrix in the 'display' function,
     * right before we tell OpenGL to render anything. See the 'display'
     * for details.
     */
    glMatrixMode(GL_MODELVIEW);

    /* The next two lines call our 2 helper functions that create some Point
     * Light and Object structs for us to render. Further details will be given
     * in the functions themselves.
     *
     * The reason we have these procedures as separate functions is to make
     * the code more organized.
     */
    // create_cubes();
    // create_lights();

    /* The next line calls our function that tells OpenGL to initialize some
     * lights to represent our Point Light structs. Further details will be
     * given in the function itself.
     *
     * The reason we have this procedure as a separate function is to make
     * the code more organized.
     */
    init_lights();
}

/* 'reshape' function:
 *
 * You will see down below in the 'main' function that whenever we create a
 * window in OpenGL, we have to specify a function for OpenGL to call whenever
 * the window resizes. We typically call this function 'reshape' or 'resize'.
 *
 * The 'reshape' function is supposed to tell your program how to react
 * whenever the program window is resized. It is also called in the beginning
 * when the window is first created. You can think of the first call to
 * 'reshape' as an initialization phase and all subsequent calls as update
 * phases.
 *
 * Anything that needs to know the dimensions of the program window should
 * be initialized and updated in the 'reshape' function. You will see below
 * that we use the 'reshape' function to initialize and update the conversion
 * scheme between NDC and screen coordinates as well as the mouse interaction
 * parameters.
 */
void reshape(int width, int height)
{
    /* The following two lines of code prevent the width and height of the
     * window from ever becoming 0 to prevent divide by 0 errors later.
     * Typically, we let 1x1 square pixel be the smallest size for the window.
     */
    height = (height == 0) ? 1 : height;
    width = (width == 0) ? 1 : width;

    /* The 'glViewport' function tells OpenGL to determine how to convert from
     * NDC to screen coordinates given the dimensions of the window. The
     * parameters for 'glViewport' are (in the following order):
     *
     * - int x: x-coordinate of the lower-left corner of the window in pixels
     * - int y: y-coordinate of the lower-left corner of the window in pixels
     * - int width: width of the window
     * - int height: height of the window
     *
     * We typically just let the lower-left corner be (0,0).
     *
     * After 'glViewport' is called, OpenGL will automatically know how to
     * convert all our points from NDC to screen coordinates when it tries
     * to render them.
     */
    glViewport(0, 0, width, height);

    /* The following two lines are specific to updating our mouse interface
     * parameters. Details will be given in the 'mouse_moved' function.
     */
    mouse_scale_x = (float) (right_param - left_param) / (float) width;
    mouse_scale_y = (float) (top_param - bottom_param) / (float) height;

    /* The following line tells OpenGL that our program window needs to
     * be re-displayed, meaning everything that was being displayed on
     * the window before it got resized needs to be re-rendered.
     */
    glutPostRedisplay();
}

/* 'display' function:
 *
 * You will see down below in the 'main' function that whenever we create a
 * window in OpenGL, we have to specify a function for OpenGL to call whenever
 * it wants to render anything. We typically name this function 'display' or
 * 'render'.
 *
 * The 'display' function is supposed to handle all the processing of points
 * in world and camera space.
 */
void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    /* With the program window cleared, OpenGL is ready to render a new scene.
     * Of course, before we can render anything correctly, we need to make all
     * the appropriate camera and object transformations to our coordinate
     * space.
     *
     * Recall that the 'init' function used the glMatrixMode function to put
     * OpenGL into a state where we can modify its Modelview Matrix. Also
     * recall that we want the Modelview Matrix to be the overall transform-
     * ation matrix that we apply to points in world space before applying the
     * perspective projection matrix. This means that we need to factor in all
     * the individual object transformations and the camera transformations
     * into the Modelview Matrix.
     *
     * To do so, our first step is to "reset" the Modelview Matrix by setting it
     * to the identity matrix:
     */
    glLoadIdentity();
    /* Now, if you recall, for a given object, we want to FIRST multiply the
     * coordinates of its points by the translations, rotations, and scalings
     * applied to the object and THEN multiply by the inverse camera rotations
     * and translations.
     *
     * HOWEVER, OpenGL modifies the Modelview Matrix using POST-MULTIPLICATION.
     * This means that if were to specify to OpenGL a matrix modification 'A',
     * then letting the Modelview Matrix be 'M', OpenGL would perform the
     * following operation:
     *
     * M = M * A
     *
     * So, for instance, if the Modelview Matrix were initialized to the
     * identity matrix 'I' and we were to specify a translation 'T' followed by
     * a rotation 'R' followed by a scaling 'S' followed by the inverse camera
     * transform 'C', then the Modelview Matrix is modified in the following
     * order:
     *
     * M = I * T * R * S * C
     *
     * Then, when OpenGL applies the Modelview Matrix to a point 'p', we would
     * get the following multiplication:
     *
     * M * p = I * T * R * S * C * p
     *
     * ^ So the camera transformation ends up being applied first even though
     * it was specified last. This is not what we want. What we want is
     * something like this:
     *
     * M * p = C * T * R * S * I * p
     *
     * Hence, to correctly transform a point, we actually need to FIRST specify
     * the inverse camera rotations and translations and THEN specify the
     * translations, rotations, and scalings applied to an object.
     *
     * We start by specifying any camera rotations caused by the mouse. We do
     * so by using the 'glRotatef' function, which takes the following parameters
     * in the following order:
     *
     * - float angle: rotation angle in DEGREES
     * - float x: x-component of rotation axis
     * - float y: y-component of rotation axis
     * - float z: z-component of rotation axis
     *
     * The 'glRotatef' function tells OpenGL to create a rotation matrix using
     * the given angle and rotation axis.
     */
    glRotatef(y_view_angle, 1, 0, 0);
    glRotatef(x_view_angle, 0, 1, 0);
    /* 'y_view_angle' and 'x_view_angle' are parameters for our mouse user
     * interface. They keep track of how much the user wants to rotate the
     * camera from its default, specified orientation. See the 'mouse_moved'
     * function for more details.
     *
     * Our next step is to specify the inverse rotation of the camera by its
     * orientation angle about its orientation axis:
     */
    glRotatef(-cam_orientation_angle,
              cam_orientation_axis[0], cam_orientation_axis[1], cam_orientation_axis[2]);
    /* We then specify the inverse translation of the camera by its position using
     * the 'glTranslatef' function, which takes the following parameters in the
     * following order:
     *
     * - float x: x-component of translation vector
     * - float y: x-component of translation vector
     * - float z: x-component of translation vector
     */
    glTranslatef(-cam_position[0], -cam_position[1], -cam_position[2]);
    /* ^ And that should be it for the camera transformations.
     */

    /* Our next step is to set up all the lights in their specified positions.
     * Our helper function, 'set_lights' does this for us. See the function
     * for more details.
     *
     * The reason we have this procedure as a separate function is to make
     * the code more organized.
     */
    set_lights();
    /* Once the lights are set, we can specify the points and faces that we
     * want drawn. We do all this in our 'draw_objects' helper function. See
     * the function for more details.
     *
     * The reason we have this procedure as a separate function is to make
     * the code more organized.
     */
    draw_objects();

    /* The following line of code has OpenGL do what is known as "double
     * buffering".
     *
     * Imagine this: You have a relatively slow computer that is telling OpenGL
     * to render something to display in the program window. Because your
     * computer is slow, OpenGL ends up rendering only part of the scene before
     * it displays it in the program window. The rest of the scene shows up a
     * second later. This effect is referred to as "flickering". You have most
     * likely experienced this sometime in your life when using a computer. It
     * is not the most visually appealing experience, right?
     *
     * To avoid the above situation, we need to tell OpenGL to display the
     * entire scene at once rather than rendering the scene one pixel at a
     * time. We do so by enabling "double buffering".
     *
     * Basically, double buffering is a technique where rendering is done using
     * two pixel grids of RGB values. One pixel grid is designated as the
     * "active buffer" while the other is designated as the "off-screen buffer".
     * Rendering is done on the off-screen buffer while the active buffer is
     * being displayed. Once the scene is fully rendered on the off-screen buffer,
     * the two buffers switch places so that the off-screen buffer becomes the
     * new active buffer and gets displayed while the old active buffer becomes
     * the new off-screen buffer. This process allows scenes to be fully rendered
     * onto the screen at once, avoiding the flickering effect.
     *
     * We actually enable double buffering in the 'main' function with the line:

       glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

     * ^ 'GLUT_DOUBLE' tells OpenGL to use double buffering. The other two
     * parameters, 'GLUT_RGB' and 'GLUT_DEPTH', tell OpenGL to initialize the
     * RGB pixel grids and the depth buffer respectively.
     *
     * The following function, 'glutSwapBuffers', tells OpenGL to swap the
     * active and off-screen buffers.
     */
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
        /* The current Modelview Matrix is actually stored at the top of a
         * stack in OpenGL. The following function, 'glPushMatrix', pushes
         * another copy of the current Modelview Matrix onto the top of the
         * stack. This results in the top two matrices on the stack both being
         * the current Modelview Matrix. Let us call the copy on top 'M1' and
         * the copy that is below it 'M2'.
         *
         * The reason we want to use 'glPushMatrix' is because we need to
         * modify the Modelview Matrix differently for each object we need to
         * render, since each object is affected by different transformations.
         * We use 'glPushMatrix' to essentially keep a copy of the Modelview
         * Matrix before it is modified by an object's transformations. This
         * copy is our 'M2'. We then modify 'M1' and use it to render the
         * object. After we finish rendering the object, we will pop 'M1' off
         * the stack with the 'glPopMatrix' function so that 'M2' returns to
         * the top of the stack. This way, we have the old unmodified Modelview
         * Matrix back to edit for the next object we want to render.
         */
        glPushMatrix();
        /* The following brace is not necessary, but it keeps things organized.
         */
        {
            int num_transforms = objects[i].transforms.size();

            /* The loop tells OpenGL to modify our modelview matrix with the
             * desired geometric transformations for this object. Remember
             * though that our 'transform_sets' struct assumes that transformations
             * are conveniently given in sets of translation -> rotate -> scaling;
             * and THIS IS NOT THE CASE FOR YOUR SCENE FILES. DO NOT BLINDLY
             * COPY THE FOLLOWING CODE.
             *
             * To explain how to correctly transform your objects, consider the
             * following example. Suppose our object has the following desired
             * transformations:
             *
             *    scale by 2, 2, 2
             *    translate by 2, 0, 5
             *    rotate about 0, 1, 0.5 and by angle = 0.6 radians
             *
             * Obviously, we cannot use the following loop for this, because the order
             * is not translate -> rotate -> scale. Instead, we need to make the following
             * calls in this exact order:
             *
             *    glRotatef(0.6 * 180.0 / M_PI, 0, 1, 0.5);
             *    glTranslatef(2, 0, 5);
             *    glScalef(2, 2, 2);
             *
             * We make the calls in the REVERSE order of how the transformations are specified
             * because OpenGL edits our modelview matrix using post-multiplication (see above
             * at the notes regarding the camera transforms in display()).
             *
             * Keep all this in mind to come up with an appropriate way to store and apply
             * geometric transformations for each object in your scenes.
             */
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

            /* The 'glMaterialfv' and 'glMaterialf' functions tell OpenGL
             * the material properties of the surface we want to render.
             * The parameters for 'glMaterialfv' are (in the following order):
             *
             * - enum face: Options are 'GL_FRONT' for front-face rendering,
             *              'GL_BACK' for back-face rendering, and
             *              'GL_FRONT_AND_BACK' for rendering both sides.
             * - enum property: this varies on what you are setting up
             *                  e.g. 'GL_AMBIENT' for ambient reflectance
             * - float* values: a set of values for the specified property
             *                  e.g. an array of RGB values for the reflectance
             *
             * The 'glMaterialf' function is the same, except the third
             * parameter is only a single float value instead of an array of
             * values. 'glMaterialf' is used to set the shininess property.
             */
            glMaterialfv(GL_FRONT, GL_AMBIENT, objects[i].ambient_reflect);
            glMaterialfv(GL_FRONT, GL_DIFFUSE, objects[i].diffuse_reflect);
            glMaterialfv(GL_FRONT, GL_SPECULAR, objects[i].specular_reflect);
            glMaterialf(GL_FRONT, GL_SHININESS, objects[i].shininess);

            /* The next few lines of code are how we tell OpenGL to render
             * geometry for us. First, let us look at the 'glVertexPointer'
             * function.
             *
             * 'glVertexPointer' tells OpenGL the specifications for our
             * "vertex array". As a recap of the comments from the 'Object'
             * struct, the "vertex array" stores all the faces of the surface
             * we want to render. The faces are stored in the array as
             * consecutive points. For instance, if our surface were a cube,
             * then our "vertex array" could be the following:
             *
             * [face1vertex1, face1vertex2, face1vertex3, face1vertex4,
             *  face2vertex1, face2vertex2, face2vertex3, face2vertex4,
             *  face3vertex1, face3vertex2, face3vertex3, face3vertex4,
             *  face4vertex1, face4vertex2, face4vertex3, face4vertex4,
             *  face5vertex1, face5vertex2, face5vertex3, face5vertex4,
             *  face6vertex1, face6vertex2, face6vertex3, face6vertex4]
             *
             * Obviously to us, some of the vertices in the array are repeats.
             * However, the repeats cannot be avoided since OpenGL requires
             * this explicit specification of the faces.
             *
             * The parameters to the 'glVertexPointer' function are as
             * follows:
             *
             * - int num_points_per_face: this is the parameter that tells
             *                            OpenGL where the breaks between
             *                            faces are in the vertex array.
             *                            Below, we set this parameter to 3,
             *                            which tells OpenGL to treat every
             *                            set of 3 consecutive vertices in
             *                            the vertex array as 1 face. So
             *                            here, our vertex array is an array
             *                            of triangle faces.
             *                            If we were using the example vertex
             *                            array above, we would have set this
             *                            parameter to 4 instead of 3.
             * - enum type_of_coordinates: this parameter tells OpenGL whether
             *                             our vertex coordinates are ints,
             *                             floats, doubles, etc. In our case,
             *                             we are using floats, hence 'GL_FLOAT'.
             * - sizei stride: this parameter specifies the number of bytes
             *                 between consecutive vertices in the array.
             *                 Most often, you will set this parameter to 0
             *                 (i.e. no offset between consecutive vertices).
             * - void* pointer_to_array: this parameter is the pointer to
             *                           our vertex array.
             */
            glVertexPointer(3, GL_FLOAT, 0, &objects[i].vertex_buffer[0]);
            /* The "normal array" is the equivalent array for normals.
             * Each normal in the normal array corresponds to the vertex
             * of the same index in the vertex array.
             *
             * The 'glNormalPointer' function has the following parameters:
             *
             * - enum type_of_normals: e.g. int, float, double, etc
             * - sizei stride: same as the stride parameter in 'glVertexPointer'
             * - void* pointer_to_array: the pointer to the normal array
             */
            glNormalPointer(GL_FLOAT, 0, &objects[i].normal_buffer[0]);

            int buffer_size = objects[i].vertex_buffer.size();

            if(!wireframe_mode)
                /* Finally, we tell OpenGL to render everything with the
                 * 'glDrawArrays' function. The parameters are:
                 *
                 * - enum mode: in our case, we want to render triangles,
                 *              so we specify 'GL_TRIANGLES'. If we wanted
                 *              to render squares, then we would use
                 *              'GL_QUADS' (for quadrilaterals).
                 * - int start_index: the index of the first vertex
                 *                    we want to render in our array
                 * - int num_vertices: number of vertices to render
                 *
                 * As OpenGL renders all the faces, it automatically takes
                 * into account all the specifications we have given it to
                 * do all the lighting calculations for us. It also applies
                 * the Modelview and Projection matrix transformations to
                 * the vertices and converts everything to screen coordinates
                 * using our Viewport specification. Everything is rendered
                 * onto the off-screen buffer.
                 */
                glDrawArrays(GL_TRIANGLES, 0, buffer_size);
            else
                /* If we are in "wireframe mode" (see the 'key_pressed'
                 * function for more information), then we want to render
                 * lines instead of triangle surfaces. To render lines,
                 * we use the 'GL_LINE_LOOP' enum for the mode parameter.
                 * However, we need to draw each face frame one at a time
                 * to render the wireframe correctly. We can do so with a
                 * for loop:
                 */
                for(int j = 0; j < buffer_size; j += 3)
                    glDrawArrays(GL_LINE_LOOP, j, 3);
        }
        /* As discussed before, we use 'glPopMatrix' to get back the
         * version of the Modelview Matrix that we had before we specified
         * the object transformations above. We then move on in our loop
         * to the next object we want to render.
         */
        glPopMatrix();
    }


    /* The following code segment uses OpenGL's built-in sphere rendering
     * function to render the blue-ground that you are walking on when
     * you run the program. The blue-ground is just the surface of a big
     * sphere of radius 100.
     */
    glPushMatrix();
    {
        glTranslatef(0, -103, 0);
        glutSolidSphere(100, 100, 100);
    }
    glPopMatrix();
}

/* 'mouse_pressed' function:
 *
 * This function is meant to respond to mouse clicks and releases. The
 * parameters are:
 *
 * - int button: the button on the mouse that got clicked or released,
 *               represented by an enum
 * - int state: either 'GLUT_DOWN' or 'GLUT_UP' for specifying whether the
 *              button was pressed down or released up respectively
 * - int x: the x screen coordinate of where the mouse was clicked or released
 * - int y: the y screen coordinate of where the mouse was clicked or released
 *
 * The function doesn't really do too much besides set some variables that
 * we need for the 'mouse_moved' function.
 */
void mouse_pressed(int button, int state, int x, int y)
{
    /* If the left-mouse button was clicked down, then...
     */
    if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
        /* Store the mouse position in our global variables.
         */
        mouse_x = x;
        mouse_y = y;

        /* Since the mouse is being pressed down, we set our 'is_pressed"
         * boolean indicator to true.
         */
        is_pressed = true;
    }
    /* If the left-mouse button was released up, then...
     */
    else if(button == GLUT_LEFT_BUTTON && state == GLUT_UP)
    {
        /* Mouse is no longer being pressed, so set our indicator to false.
         */
        is_pressed = false;
    }
}

/* 'mouse_moved' function:
 *
 * This function is meant to respond to when the mouse is being moved. There
 * are just two parameters to this function:
 *
 * - int x: the x screen coordinate of where the mouse was clicked or released
 * - int y: the y screen coordinate of where the mouse was clicked or released
 *
 * We compute our camera rotation angles based on the mouse movement in this
 * function.
 */
void mouse_moved(int x, int y)
{
    /* If the left-mouse button is being clicked down...
     */
    if(is_pressed)
    {
        /* You see in the 'mouse_pressed' function that when the left-mouse button
         * is first clicked down, we store the screen coordinates of where the
         * mouse was pressed down in 'mouse_x' and 'mouse_y'. When we move the
         * mouse, its screen coordinates change and are captured by the 'x' and
         * 'y' parameters to the 'mouse_moved' function. We want to compute a change
         * in our camera angle based on the distance that the mouse traveled.
         *
         * We have two distances traveled: a dx equal to 'x' - 'mouse_x' and a
         * dy equal to 'y' - 'mouse_y'. We need to compute the desired changes in
         * the horizontal (x) angle of the camera and the vertical (y) angle of
         * the camera.
         *
         * Let's start with the horizontal angle change. We first need to convert
         * the dx traveled in screen coordinates to a dx traveled in camera space.
         * The conversion is done using our 'mouse_scale_x' variable, which we
         * set in our 'reshape' function. We then multiply by our 'x_view_step'
         * variable, which is an arbitrary value that determines how "fast" we
         * want the camera angle to change. Higher values for 'x_view_step' cause
         * the camera to move more when we drag the mouse. We had set 'x_view_step'
         * to 90 at the top of this file (where we declared all our variables).
         *
         * We then add the horizontal change in camera angle to our 'x_view_angle'
         * variable, which keeps track of the cumulative horizontal change in our
         * camera angle. 'x_view_angle' is used in the camera rotations specified
         * in the 'display' function.
         */
        x_view_angle += ((float) x - (float) mouse_x) * mouse_scale_x * x_view_step;

        /* We do basically the same process as above to compute the vertical change
         * in camera angle. The only real difference is that we want to keep the
         * camera angle changes realistic, and it is unrealistic for someone in
         * real life to be able to change their vertical "camera angle" more than
         * ~90 degrees (they would have to detach their head and spin it vertically
         * or something...). So we decide to restrict the cumulative vertical angle
         * change between -90 and 90 degrees.
         */
        float temp_y_view_angle = y_view_angle +
                                  ((float) y - (float) mouse_y) * mouse_scale_y * y_view_step;
        y_view_angle = (temp_y_view_angle > 90 || temp_y_view_angle < -90) ?
                       y_view_angle : temp_y_view_angle;

        /* We update our 'mouse_x' and 'mouse_y' variables so that if the user moves
         * the mouse again without releasing it, then the distance we compute on the
         * next call to the 'mouse_moved' function will be from this current mouse
         * position.
         */
        mouse_x = x;
        mouse_y = y;

        /* Tell OpenGL that it needs to re-render our scene with the new camera
         * angles.
         */
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
    else
    {
        /* These might look a bit complicated, but all we are really doing is
         * using our current change in the horizontal camera angle (ie. the
         * value of 'x_view_angle') to compute the correct changes in our x and
         * z coordinates in camera space as we move forward, backward, to the left,
         * or to the right.
         *
         * 'step_size' is an arbitrary value to determine how "big" our steps
         * are.
         *
         * We make the x and z coordinate changes to the camera position, since
         * moving forward, backward, etc is basically just shifting our view
         * of the scene.
         */

        float x_view_rad = deg2rad(x_view_angle);

        /* 'w' for step forward
         */
        if(key == 'w')
        {
            cam_position[0] += step_size * sin(x_view_rad);
            cam_position[2] -= step_size * cos(x_view_rad);
            glutPostRedisplay();
        }
        /* 'a' for step left
         */
        else if(key == 'a')
        {
            cam_position[0] -= step_size * cos(x_view_rad);
            cam_position[2] -= step_size * sin(x_view_rad);
            glutPostRedisplay();
        }
        /* 's' for step backward
         */
        else if(key == 's')
        {
            cam_position[0] -= step_size * sin(x_view_rad);
            cam_position[2] += step_size * cos(x_view_rad);
            glutPostRedisplay();
        }
        /* 'd' for step right
         */
        else if(key == 'd')
        {
            cam_position[0] += step_size * cos(x_view_rad);
            cam_position[2] += step_size * sin(x_view_rad);
            glutPostRedisplay();
        }
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

/* 'create_lights' function:
 *
 * This function is relatively uninteresting. We are just hardcoding all the
 * properties of our point light objects. You do not need to do this because
 * the parser will already have all the point light objects initialized.
 */
void create_lights()
{
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Light 1 Below
    ///////////////////////////////////////////////////////////////////////////////////////////////

    Point_Light light1;

    light1.position[0] = -0.8;
    light1.position[1] = 0;
    light1.position[2] = 1;
    light1.position[3] = 1;

    light1.color[0] = 1;
    light1.color[1] = 1;
    light1.color[2] = 0;
    light1.attenuation_k = 0.2;

    lights.push_back(light1);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Light 2 Below
    ///////////////////////////////////////////////////////////////////////////////////////////////

    Point_Light light2;

    light2.position[0] = 0.15;
    light2.position[1] = 0.85;
    light2.position[2] = 0.7;
    light2.position[3] = 1;

    light2.color[0] = 1;
    light2.color[1] = 0;
    light2.color[2] = 1;

    light2.attenuation_k = 0.1;

    lights.push_back(light2);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Light 3 Below
    ///////////////////////////////////////////////////////////////////////////////////////////////

    Point_Light light3;

    light3.position[0] = 0.5;
    light3.position[1] = -0.5;
    light3.position[2] = 0.85;
    light3.position[3] = 1;

    light3.color[0] = 0;
    light3.color[1] = 1;
    light3.color[2] = 1;

    light3.attenuation_k = 0;

    lights.push_back(light3);
}

/* 'create_cubes' function:
 *
 * We hardcode all the properties of our cubes in this function. The only
 * relatively interesting part of this function is seeing how we form our
 * vertex and normal arrays. You will be able to form your vertex and normal
 * arrays more elegantly than this because the parser will have all the vertices,
 * normals, and facesets all stored in nice vectors. You will also not have to
 * hardcode any reflectances, transformations, etc, since the parser will also
 * have those all initialized for you.
 *
 * We are rendering our cubes with triangles, so each cube has 12 (triangle) faces
 * in all.
 */
void create_cubes()
{
    Object cube1;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Reflectances
    ///////////////////////////////////////////////////////////////////////////////////////////////

    cube1.ambient_reflect[0] = 0.2;
    cube1.ambient_reflect[1] = 0.2;
    cube1.ambient_reflect[2] = 0.2;

    cube1.diffuse_reflect[0] = 0.6;
    cube1.diffuse_reflect[1] = 0.6;
    cube1.diffuse_reflect[2] = 0.6;

    cube1.specular_reflect[0] = 1;
    cube1.specular_reflect[1] = 1;
    cube1.specular_reflect[2] = 1;

    cube1.shininess = 5.0;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Points
    ///////////////////////////////////////////////////////////////////////////////////////////////

    Triple point1;
    point1.x = -1;
    point1.y = -1;
    point1.z = 1;

    Triple point2;
    point2.x = 1;
    point2.y = -1;
    point2.z = 1;

    Triple point3;
    point3.x = 1;
    point3.y = 1;
    point3.z = 1;

    Triple point4;
    point4.x = -1;
    point4.y = 1;
    point4.z = 1;

    Triple point5;
    point5.x = -1;
    point5.y = -1;
    point5.z = -1;

    Triple point6;
    point6.x = 1;
    point6.y = -1;
    point6.z = -1;

    Triple point7;
    point7.x = 1;
    point7.y = 1;
    point7.z = -1;

    Triple point8;
    point8.x = -1;
    point8.y = 1;
    point8.z = -1;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Normals
    ///////////////////////////////////////////////////////////////////////////////////////////////

    Triple normal1;
    normal1.x = 0;
    normal1.y = 0;
    normal1.z = 1;

    Triple normal2;
    normal2.x = 0;
    normal2.y = 0;
    normal2.z = -1;

    Triple normal3;
    normal3.x = 0;
    normal3.y = 1;
    normal3.z = 0;

    Triple normal4;
    normal4.x = 0;
    normal4.y = -1;
    normal4.z = 0;

    Triple normal5;
    normal5.x = 1;
    normal5.y = 0;
    normal5.z = 0;

    Triple normal6;
    normal6.x = -1;
    normal6.y = 0;
    normal6.z = 0;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Vertex and Normal Arrays
    ///////////////////////////////////////////////////////////////////////////////////////////////

    /* We are rendering our cubes with triangles, so each cube has 12 (triangle) faces
     * in all.
     */

    /* Face 1: */

    cube1.vertex_buffer.push_back(point1);
    cube1.normal_buffer.push_back(normal1);

    cube1.vertex_buffer.push_back(point2);
    cube1.normal_buffer.push_back(normal1);

    cube1.vertex_buffer.push_back(point3);
    cube1.normal_buffer.push_back(normal1);

    /* Face 2: */

    cube1.vertex_buffer.push_back(point1);
    cube1.normal_buffer.push_back(normal1);

    cube1.vertex_buffer.push_back(point3);
    cube1.normal_buffer.push_back(normal1);

    cube1.vertex_buffer.push_back(point4);
    cube1.normal_buffer.push_back(normal1);

    /* Face 3: */

    cube1.vertex_buffer.push_back(point6);
    cube1.normal_buffer.push_back(normal2);

    cube1.vertex_buffer.push_back(point5);
    cube1.normal_buffer.push_back(normal2);

    cube1.vertex_buffer.push_back(point7);
    cube1.normal_buffer.push_back(normal2);

    /* Face 4: */

    cube1.vertex_buffer.push_back(point7);
    cube1.normal_buffer.push_back(normal2);

    cube1.vertex_buffer.push_back(point5);
    cube1.normal_buffer.push_back(normal2);

    cube1.vertex_buffer.push_back(point8);
    cube1.normal_buffer.push_back(normal2);

    /* Face 5: */

    cube1.vertex_buffer.push_back(point2);
    cube1.normal_buffer.push_back(normal5);

    cube1.vertex_buffer.push_back(point6);
    cube1.normal_buffer.push_back(normal5);

    cube1.vertex_buffer.push_back(point3);
    cube1.normal_buffer.push_back(normal5);

    /* Face 6: */

    cube1.vertex_buffer.push_back(point3);
    cube1.normal_buffer.push_back(normal5);

    cube1.vertex_buffer.push_back(point6);
    cube1.normal_buffer.push_back(normal5);

    cube1.vertex_buffer.push_back(point7);
    cube1.normal_buffer.push_back(normal5);

    /* Face 7: */

    cube1.vertex_buffer.push_back(point5);
    cube1.normal_buffer.push_back(normal6);

    cube1.vertex_buffer.push_back(point4);
    cube1.normal_buffer.push_back(normal6);

    cube1.vertex_buffer.push_back(point8);
    cube1.normal_buffer.push_back(normal6);

    /* Face 8: */

    cube1.vertex_buffer.push_back(point4);
    cube1.normal_buffer.push_back(normal6);

    cube1.vertex_buffer.push_back(point5);
    cube1.normal_buffer.push_back(normal6);

    cube1.vertex_buffer.push_back(point1);
    cube1.normal_buffer.push_back(normal6);

    /* Face 9: */

    cube1.vertex_buffer.push_back(point4);
    cube1.normal_buffer.push_back(normal3);

    cube1.vertex_buffer.push_back(point3);
    cube1.normal_buffer.push_back(normal3);

    cube1.vertex_buffer.push_back(point8);
    cube1.normal_buffer.push_back(normal3);

    /* Face 10: */

    cube1.vertex_buffer.push_back(point7);
    cube1.normal_buffer.push_back(normal3);

    cube1.vertex_buffer.push_back(point8);
    cube1.normal_buffer.push_back(normal3);

    cube1.vertex_buffer.push_back(point3);
    cube1.normal_buffer.push_back(normal3);

    /* Face 11: */

    cube1.vertex_buffer.push_back(point1);
    cube1.normal_buffer.push_back(normal4);

    cube1.vertex_buffer.push_back(point5);
    cube1.normal_buffer.push_back(normal4);

    cube1.vertex_buffer.push_back(point2);
    cube1.normal_buffer.push_back(normal4);

    /* Face 12: */

    cube1.vertex_buffer.push_back(point2);
    cube1.normal_buffer.push_back(normal4);

    cube1.vertex_buffer.push_back(point5);
    cube1.normal_buffer.push_back(normal4);

    cube1.vertex_buffer.push_back(point6);
    cube1.normal_buffer.push_back(normal4);


    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Cube 2
    ///////////////////////////////////////////////////////////////////////////////////////////////

    /* We are just going to make them identical out of laziness... */
    Object cube2 = cube1;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Transformations for Cube 1
    ///////////////////////////////////////////////////////////////////////////////////////////////

    cube1.transforms.push_back(Transform{2, 0.5, 0.5, 0.5, 0});
    cube1.transforms.push_back(Transform{1, 1, 1, 0, 60});
    cube1.transforms.push_back(Transform{0, -0.6, 0, 0, 0});

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Transformations for Cube 2
    ///////////////////////////////////////////////////////////////////////////////////////////////

    cube2.transforms.push_back(Transform{2, 0.5, 0.5, 0.5, 0});
    cube2.transforms.push_back(Transform{1, 1, 0, 0, -45});
    cube2.transforms.push_back(Transform{2, 1.5, 1.5, 1.5, 0});
    cube2.transforms.push_back(Transform{1, 0, 1, 0, 135});
    cube2.transforms.push_back(Transform{0, 2.0, 0, 0, 0});

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Push to Objects
    ///////////////////////////////////////////////////////////////////////////////////////////////

    objects.push_back(cube1);
    objects.push_back(cube2);
}

/* The 'main' function:
 *
 * This function is short, but is basically where everything comes together.
 */
int main(int argc, char* argv[])
{
    // int xres = 500;
    // int yres = 500;
    if (argc != 4) {
        cout << "Incorrect number of arguments" << endl;
        return 1;
    }

    string scene_filename = argv[1];
    int xres = stoi(argv[2]);
    int yres = stoi(argv[3]);

    // Parse the scene description file and put data into data structures.
    parse_scene(scene_filename);

    /* 'glutInit' intializes the GLUT (Graphics Library Utility Toolkit) library.
     * This is necessary, since a lot of the functions we used above and below
     * are from the GLUT library.
     *
     * 'glutInit' takes the 'main' function arguments as parameters. This is not
     * too important for us, but it is possible to give command line specifications
     * to 'glutInit' by putting them with the 'main' function arguments.
     */
    glutInit(&argc, argv);
    /* The following line of code tells OpenGL that we need a double buffer,
     * a RGB pixel buffer, and a depth buffer.
     */
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    /* The following line tells OpenGL to create a program window of size
     * 'xres' by 'yres'.
     */
    glutInitWindowSize(xres, yres);
    /* The following line tells OpenGL to set the program window in the top-left
     * corner of the computer screen (0, 0).
     */
    glutInitWindowPosition(0, 0);
    /* The following line tells OpenGL to name the program window "Test".
     */
    glutCreateWindow("OpenGL Renderer");

    /* Call our 'init' function...
     */
    init();
    /* Specify to OpenGL our display function.
     */
    glutDisplayFunc(display);
    /* Specify to OpenGL our reshape function.
     */
    glutReshapeFunc(reshape);
    /* Specify to OpenGL our function for handling mouse presses.
     */
    glutMouseFunc(mouse_pressed);
    /* Specify to OpenGL our function for handling mouse movement.
     */
    glutMotionFunc(mouse_moved);
    /* Specify to OpenGL our function for handling key presses.
     */
    glutKeyboardFunc(key_pressed);
    /* The following line tells OpenGL to start the "event processing loop". This
     * is an infinite loop where OpenGL will continuously use our display, reshape,
     * mouse, and keyboard functions to essentially run our program.
     */
    glutMainLoop();
}
