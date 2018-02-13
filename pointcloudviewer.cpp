/*!
** \file pointcloudviewer.cpp
** \brief 3D Point Cloud Viewer
**
** ### About
** Created for: CMPUT 511 Project\n
** Fall 2017\n
** Bernard Llanos\n
** Department of Computing Science, University of Alberta
**
** ## Building
** Run `make pointCloudViewer`
**
** ## Usage
** `./pointCloudViewer $FILENAME` will open the CSV file for viewing.
** The format of the CSV file is described in 'README.md'
**
** ## Keyboard Controls
** - Refer to 'README.md'.
**
** ## References
** - 'modelviewer.cpp' created for Assignment 1
** - "glut - How to get and use delta time", posted on Game Development Stack Exchange
**   (https://gamedev.stackexchange.com/questions/13008/how-to-get-and-use-delta-time)
**
** ### Textbook Code
** From 'ExperimenterSource.zip' downloaded from the course textbook website
** (http://www.sumantaguha.com/)
** - 'ExperimenterSource/Chapter4/BallAndTorus/ballAndTorus.cpp'
** - 'ExperimenterSource/Chapter4/RotatingHelixFPS/rotatingHelixFPS.cpp'
*/

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>
#include <exception>
#include <string>
#include <sstream>
#include <cmath>

#include "pointcloud.h"
#include "litmaterial.h"
#include "camera.h"
#include "object.h"
#include "utilities.h"
#include "error.h"

using std::cout;
using std::endl;
using std::string;

#define OUTPUT_FILENAME "out.csv"
#define EVALUATION_OUTPUT_FILENAME "results.csv"
#define VERTEX_BUFFER_BLOCK_SIZE 500
#define SPEED_DECREASE_FACTOR 0.75f
#define SPEED_INCREASE_FACTOR 1.25f
#define DEFAULT_SPEED 10.0f
#define POINT_SIZE_INCREMENT 1.0f
#define LINE_WIDTH_INCREMENT 1.0f
#define MIN_UPDATE_TIME (SECONDS_TO_MILLISECONDS / 60.0f)

static std::string* file = 0;

/*!
 *\brief The point cloud loaded from the input file
 */
static Object<PointCloud>* cloud = 0;
static PointCloud* cloudModel = 0;

/*!
 * \brief Lighting and material properties
 */
static LitMaterial* litMaterial = 0;

/*!
 * \brief Controls the view and perspective transformations
 */
static Camera* camera = 0;

// Fog parameters
static float fogColor[4] = {0.0f, 0.0f, 0.0f, 1.0f};
static int fogMode = GL_LINEAR;
static bool isFog = true;
static float fogStart = 10.0f;
static float fogEnd = 11.0f;

// Animation controls
static bool playAnimation = true; // Play or pause
static float pointsPerSecond = 0.0f; // To be initialized in resetPointCloud()
/*!
 * \brief GLUT time when the rate of point insertion was last changed, or when
 *   the animation was last restarted
 */
static int timeAtSpeedChange = 0;
static int pointsAtSpeedChange = 0;
static int frameCount = 0; // Number of frames
static int frameRate = 0;

/*!
 * \brief Reset the counters used in the animation, but not the speed
 *   of the animation
 */
void resetAnimation(void) {
    timeAtSpeedChange = glutGet(GLUT_ELAPSED_TIME);
    pointsAtSpeedChange = cloudModel->nPoints();
    if( pointsPerSecond < 0.0f ) {
        pointsPerSecond = 0.0f;
    }
}

/*!
 * \brief Keep track of the frame rate
 */
void frameCounter(int value) {
    frameRate = frameCount;
    frameCount = 0;
    glutTimerFunc(1000, frameCounter, value);
}

/*!
 *\brief GLUT rendering callback
 */
void drawScene(void)
{
    frameCount++;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Fog
    if (isFog) {
       glEnable(GL_FOG);
    } else {
       glDisable(GL_FOG);
    }

    camera->applyViewTransformation();
    litMaterial->enable();
    cloud->draw();
    litMaterial->disable();

    std::ostringstream oss;
    if(!playAnimation) {
        oss << "(paused) ";
    } else {
        oss << "(playing) ";
    }
    oss << "FPS: " << frameRate;
    string s = oss.str();
    writeStrokeString(GLUT_STROKE_ROMAN, 0.05f, 0.2f, 0.3f, 0.06f, s);

    oss.str("");
    oss << cloudModel->nPoints();
    oss << " (limit " << pointsPerSecond << "/s)";
    s = oss.str();
    writeStrokeString(GLUT_STROKE_ROMAN, 0.05f, 0.12f, 0.3f, 0.06f, s);

    oss.str("");
    switch(cloudModel->algorithm) {
    case Point::ALGORITHM::NORMAL_HYPOTHESIS:
        oss << "Normal hypotheses";
        break;
    case Point::ALGORITHM::DELAUNAY:
        oss << "Delaunay balls";
        break;
    case Point::ALGORITHM::CURVE_DELAUNAY:
        oss << "DB tangent";
        break;
    case Point::ALGORITHM::VCM:
        oss << "Voronoi Cov. Mat.";
        break;
    case Point::ALGORITHM::CURVE_VCM:
        oss << "VCM tangent";
        break;
    case Point::ALGORITHM::PCA:
        oss << "PCA";
        break;
    case Point::ALGORITHM::CURVE_PCA:
        oss << "PCA tangent";
        break;
    default:
        throw Error("Unrecognized Point::ALGORITHM constant.");
    }
    s = oss.str();
    writeStrokeString(GLUT_STROKE_ROMAN, 0.05f, 0.05f, 0.4f, 0.06f, s);

    glutSwapBuffers(); // Double buffering
}

void update(int value)
{
    if(playAnimation) {
        int elapsed = glutGet(GLUT_ELAPSED_TIME); // milliseconds
        elapsed -= timeAtSpeedChange;
        float elapsedSeconds = static_cast<float>(elapsed) / SECONDS_TO_MILLISECONDS;
        int nPoints = cloudModel->nPoints();
        int desiredPoints = static_cast<int>(elapsedSeconds * pointsPerSecond) - (nPoints - pointsAtSpeedChange);
        cloudModel->setPointIncrement(desiredPoints);
        cloud->update();
    }
    glutPostRedisplay();
    float nextTime = SECONDS_TO_MILLISECONDS / pointsPerSecond;
    if(nextTime < MIN_UPDATE_TIME) {
        nextTime = MIN_UPDATE_TIME;
    }
    if(value != 0) {
        glutTimerFunc(nextTime, update, value);
    }
}

void resetPointCloud(void) {
    if(cloud != 0) {
        delete cloud;
        cloud = 0;
    }
    pointsPerSecond = 1.0f;
    cloudModel = new PointCloud(*file, VERTEX_BUFFER_BLOCK_SIZE, static_cast<int>(pointsPerSecond));
    PointCloud* cloudModelTemp = cloudModel;
    cloud = new Object<PointCloud>(cloudModelTemp);
    cloud->tf.translateZ(-10.0f);
    pointsPerSecond = DEFAULT_SPEED;
}

/*!
 *\brief First-time setup
 */
void setup(void)
{
   glClearColor(0.0, 0.0, 0.0, 0.0);
   glEnable(GL_DEPTH_TEST);

   glFogfv(GL_FOG_COLOR, fogColor);
   glFogi(GL_FOG_MODE, fogMode);
   glFogf(GL_FOG_START, fogStart);
   glFogf(GL_FOG_END, fogEnd);
   glHint(GL_FOG_HINT, GL_NICEST); // or GL_DONT_CARE or GL_FASTEST

   resetPointCloud();
   camera = new Camera();
   litMaterial = new LitMaterial();

   resetAnimation();
}

/*!
 *\brief GLUT window reshape callback
 */
void resize(int w, int h)
{
   glViewport(0, 0, w, h);
   camera->applyProjection();
}

/*!
 *\brief GLUT ASCII keys key event callback
 */
void keyInput(unsigned char key, int x, int y)
{
    switch(key)
    {
        case 27: // ESC - Quit
        case 'q':
            exit(0);
            break;

        // Reset transformations
        case 'x':
        case 'X':
            playAnimation = false;
            resetPointCloud();
            resetAnimation();
            camera->tf.identity();
            break;

        // Camera projection
        case 'v':
            camera->applyProjection(true);
            break;
        case 'V':
            camera->applyProjection(false);
            break;

        // File output
        case 'w':
        case 'W':
            cout << "Writing world coordinates of geometry to '" << OUTPUT_FILENAME << "'." << endl;
            cloud->writeData(OUTPUT_FILENAME);
            cout << " Done." << endl;
            break;

        // Model z-translation
        case 'n':
            cloud->tf.translateZ(-0.1f);
            break;
        case 'N':
            cloud->tf.translateZ(0.1f);
            break;

        // Model rotation
        case 'u':
            cloud->tf.rotateX(deg2rad(-10.0f));
            break;
        case 'U':
            cloud->tf.rotateX(deg2rad(10.0f));
            break;
        case 'y':
            cloud->tf.rotateY(deg2rad(-10.0f));
            break;
        case 'Y':
            cloud->tf.rotateY(deg2rad(10.0f));
            break;
        case 'r':
            cloud->tf.rotateZ(deg2rad(-10.0f));
            break;
        case 'R':
            cloud->tf.rotateZ(deg2rad(10.0f));
            break;

        // Camera translation
        case 'd':
            camera->tf.translateX(-0.1f);
            break;
        case 'D':
            camera->tf.translateX(0.1f);
            break;
        case 'c':
            camera->tf.translateY(-0.1f);
            break;
        case 'C':
            camera->tf.translateY(0.1f);
            break;
        case 'z':
            camera->tf.translateZ(-0.1f);
            break;
        case 'Z':
            camera->tf.translateZ(0.1f);
            break;

    	// Camera rotation
        case 't':
            camera->tf.rotateX(deg2rad(-1.0f));
            break;
        case 'T':
            camera->tf.rotateX(deg2rad(1.0f));
            break;
        case 'a':
            camera->tf.rotateY(deg2rad(-1.0f));
            break;
        case 'A':
            camera->tf.rotateY(deg2rad(1.0f));
            break;
        case 'l':
            camera->tf.rotateZ(deg2rad(-10.0f));
            break;
        case 'L':
            camera->tf.rotateZ(deg2rad(10.0f));
            break;

        // Fog
        case 'f':
            isFog = false;
            break;
        case 'F':
            isFog = true;
            break;

        // Animation controls
        case 'p':
            playAnimation = true;
            resetAnimation();
            break;
        case 'P':
            playAnimation = false;
            break;
        case '-':
            if(pointsPerSecond < (1.0f / SPEED_DECREASE_FACTOR)) {
                pointsPerSecond *= SPEED_DECREASE_FACTOR;
            } else if(pointsPerSecond <= 10.0f) {
                pointsPerSecond = round(pointsPerSecond) - 1.0f;
            } else if(pointsPerSecond <= (100.0f / SPEED_DECREASE_FACTOR)) {
                pointsPerSecond = round(pointsPerSecond / 10.0f) * 10.0f - 10.0f;
            } else {
                pointsPerSecond *= SPEED_DECREASE_FACTOR;
            }
            resetAnimation();
            break;
        case '+':
            if(pointsPerSecond < 1.0f) {
                pointsPerSecond *= SPEED_INCREASE_FACTOR;
            } else if(pointsPerSecond < 10.0f) {
                pointsPerSecond = round(pointsPerSecond) + 1.0f;
            } else if(pointsPerSecond < 100.0f) {
                pointsPerSecond = round(pointsPerSecond / 10.0f) * 10.0f + 10.0f;
            } else {
                pointsPerSecond *= SPEED_INCREASE_FACTOR;
            }
            resetAnimation();
            break;

        // Point size
        case 's':
            cloudModel->incrementPointSize(-POINT_SIZE_INCREMENT);
            break;
        case 'S':
            cloudModel->incrementPointSize(POINT_SIZE_INCREMENT);
            break;

        // Line width
        case 'b':
            cloudModel->incrementLineWidth(-LINE_WIDTH_INCREMENT);
            break;
        case 'B':
            cloudModel->incrementLineWidth(LINE_WIDTH_INCREMENT);
            break;

        // Normal estimation algorithm
        case 'g':
            cloudModel->cycleNormalEstimationAlgorithm(false);
            break;
        case 'G':
            cloudModel->cycleNormalEstimationAlgorithm(true);
            break;

        // Save results for evaluation
        case 'e':
        case 'E':
            cout << "Writing normal estimation results to '" << EVALUATION_OUTPUT_FILENAME << "'." << endl;
            cloudModel->writeComparison(EVALUATION_OUTPUT_FILENAME);
            cout << " Done." << endl;
            break;

        default:
            cout << "Unhandled keypress '" << key << "'." << endl;
            break;
    }
    update(0);
}

void specialKeyInput(int key, int x, int y)
{
    switch(key)
    {
        // Model x and y-translation
        case GLUT_KEY_UP:
            cloud->tf.translateY(0.1f);
            break;
        case GLUT_KEY_DOWN:
            cloud->tf.translateY(-0.1f);
            break;
        case GLUT_KEY_LEFT:
            cloud->tf.translateX(-0.1f);
            break;
        case GLUT_KEY_RIGHT:
            cloud->tf.translateX(0.1f);
            break;


        case 112: // Left SHIFT
        case 113: // Right SHIFT
            break;
        default:
            cout << "Unhandled special keypress '" << key << "'." << endl;
            break;
    }
    update(0);
}

/*!
 *\brief Program main function (entry point)
 */
int main(int argc, char **argv)
{
    if(argc < 2) {
        cout << "Usage: `./pointCloudViewer $FILENAME`" << endl
        << "    Load and display a point cloud from the file FILENAME." << endl;
        return 0;
    }

    try {
        glutInit(&argc, argv);
        // glutInitContextVersion(4, 3);
        glutInitContextVersion(3, 0);
        glutInitContextProfile(GLUT_COMPATIBILITY_PROFILE);

        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH); // Double buffering
        glutInitWindowSize(500, 500);
        glutInitWindowPosition(100, 100);
        glutCreateWindow(*argv);
        glutDisplayFunc(drawScene);
        glutTimerFunc(0, update, 1);
        glutReshapeFunc(resize);
        glutKeyboardFunc(keyInput);
        glutSpecialFunc(specialKeyInput);
        glutTimerFunc(0, frameCounter, 0);

        glewExperimental = GL_TRUE;
        glewInit();

        file = new string(argv[1]);
        setup();

        glutMainLoop();
    } catch( std::exception& e ) {
        cout << "Exception thrown, with message \"" << e.what() << "\"" << endl;
    } catch( ... ) {
        cout << "Non-exception thrown." << endl;
    }

    // Cleanup
    if(cloud != 0) {
        delete cloud;
        cloud = 0;
    }
    if(litMaterial != 0) {
        delete litMaterial;
        litMaterial = 0;
    }
    if(camera != 0) {
        delete camera;
        camera = 0;
    }
    if(file != 0) {
        delete file;
        file = 0;
    }
}
