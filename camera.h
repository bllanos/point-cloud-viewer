#ifndef CAMERA_H
#define CAMERA_H

/*!
** \file camera.h
** \brief Definition of the Camera class.
**
** ### About
** Created for: CMPUT 511 Assignment 1: 3D Model Viewer\n
** Fall 2017\n
** Bernard Llanos\n
** Department of Computing Science, University of Alberta
**
** ## References
*/

#include <Eigen/Core>

#include "affinetransform.h"

/*!
 * \brief View and projection transformation
 *
 * A class encapsulating the view and projection transformations
 * of an OpenGL scene
 */
class Camera
{
public:

    /*!
     * \brief Create a camera with default parameters
     */
    Camera(void);

    /*!
     * \brief Push the view transformation onto the modelview stack
     */
    void applyViewTransformation(void) const;

    /*!
     * \brief Push the projection transformation onto the projection stack
     *
     * Use orthographic projection (true) or perspective projection (false),
     * according to whichever was previously active.
     */
    void applyProjection(void);

    /*!
     * \brief Push the projection transformation onto the projection stack
     *
     * \param [in] ortho Use orthographic projection (true) or perspective
     * projection (false).
     */
    void applyProjection(const bool& ortho);

    /*!
     * \brief Set a frustum to encompass the given bounding box, and translate
     *   the camera so that it is looking at the centre of the bounding box.
     *
     * Both the orthographic and perspective frustum parameters are updated.
     *
     * \param [in] min The corner of the bounding box at lower coordinates (left, bottom, back)
     * \param [in] max The corner of the bounding box at higher coordinates (right, top, front)
     * \param [in] expandDepth If true, set the near and far distances of the
     *   frustums to extend beyond the bounding box.
     */
    void setFromBoundingBox(const Eigen::Vector3f& min, const Eigen::Vector3f& max, const bool expandDepth);

private:
    /*!
     * \brief The arguments of glOrtho()
     */
    float orthoParams[6];
    /*!
     * \brief The arguments of glFrustum()
     */
    float perspectiveParams[6];
    bool useOrtho;

public:
    /*!
     * \brief The 3D pose of the object
     *
     * At construction, this is the identity pose
     */
    AffineTransform tf;

    // Currently not implemented - will cause linker errors if called
private:
    Camera(const Camera& other);
    Camera& operator=(const Camera& other);
};

#endif
