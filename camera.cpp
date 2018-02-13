/*!
** \file camera.cpp
** \brief Implementation of the Camera class.
**
** ### About
** Created for: CMPUT 511 Assignment 1: 3D Model Viewer\n
** Fall 2017\n
** Bernard Llanos\n
** Department of Computing Science, University of Alberta
*/

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "camera.h"
#include "utilities.h"

using Eigen::Vector3f;

Camera::Camera(void) : useOrtho(false), tf() {
    orthoParams[0] = -1.0f;
    orthoParams[1] = 1.0f;
    orthoParams[2] = -1.0f;
    orthoParams[3] = 1.0f;
    orthoParams[4] = 0.1f; //8.0f
    orthoParams[5] = 100.0f;
    perspectiveParams[0] = -(0.1f / 8.0f);
    perspectiveParams[1] = (0.1f / 8.0f);
    perspectiveParams[2] = -(0.1f / 8.0f);
    perspectiveParams[3] = (0.1f / 8.0f);
    perspectiveParams[4] = 0.1f; //8.0f
    perspectiveParams[5] = 100.0f;
}

void Camera::applyViewTransformation(void) const {
   glLoadIdentity();
   tf.applyInverse();
}

void Camera::applyProjection(void) {
    applyProjection(useOrtho);
}

void Camera::applyProjection(const bool& ortho) {
    useOrtho = ortho;
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if(ortho) {
        glOrtho(
            orthoParams[0],
            orthoParams[1],
            orthoParams[2],
            orthoParams[3],
            orthoParams[4],
            orthoParams[5]
        );
    } else {
        glFrustum(
            perspectiveParams[0],
            perspectiveParams[1],
            perspectiveParams[2],
            perspectiveParams[3],
            perspectiveParams[4],
            perspectiveParams[5]
        );
    }
    glMatrixMode(GL_MODELVIEW);
}

void Camera::setFromBoundingBox(const Vector3f& min, const Vector3f& max, const bool expandDepth) {
    Vector3f verticesCenter = (max + min) / 2.0f;

    // Set the desired field of view angle
    Vector3f verticesScale =(max - min) / 2.0f;
    // Preserve scaling ratio in X and Y
    if(verticesScale(0) > verticesScale(1)) {
        verticesScale(1) = verticesScale(0);
    }
    float cameraZ = verticesScale(0) / tan(deg2rad(20.0f));

    // Position the camera
    tf.identity();
    tf.translateX(verticesCenter(0));
    tf.translateY(verticesCenter(1));
    tf.translateZ(max(2) + cameraZ);

    // Set the view volume
    float farZ = cameraZ + verticesScale(2) * 2.0f;
    float ratio = 1.0f;
    if(expandDepth) {
        farZ += verticesScale(2) * 8.0f;
        float minCameraZ = 0.1f;
        if(cameraZ > minCameraZ) {
            ratio = minCameraZ / cameraZ;
            cameraZ = minCameraZ;
        }
    }

    orthoParams[0] = -verticesScale(0);
    orthoParams[1] = verticesScale(0);
    orthoParams[2] = -verticesScale(1);
    orthoParams[3] = verticesScale(1);
    orthoParams[4] = cameraZ;
    orthoParams[5] = farZ;

    verticesScale *= ratio;

    perspectiveParams[0] = -verticesScale(0);
    perspectiveParams[1] = verticesScale(0);
    perspectiveParams[2] = -verticesScale(1);
    perspectiveParams[3] = verticesScale(1);
    perspectiveParams[4] = cameraZ;
    perspectiveParams[5] = farZ;
}
