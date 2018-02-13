#ifndef AFFINE_TRANSFORM_H
#define AFFINE_TRANSFORM_H

/*!
** \file affinetransform.h
** \brief Definition of the AffineTransform class.
**
** ### About
** Created for: CMPUT 511 Assignment 1: 3D Model Viewer\n
** Fall 2017\n
** Bernard Llanos\n
** Department of Computing Science, University of Alberta
**
** ## References
** - Eigen3 documentation: http://eigen.tuxfamily.org/dox/index.html
*/

#include <Eigen/Geometry>

/*!
 * \brief 3D affine transformation state
 *
 * Encapsulates three-dimensional scaling, orientation, and position
 */
class AffineTransform
{
public:

    /*!
     * \brief Create an identity transformation
     */
    AffineTransform(void);

    virtual ~AffineTransform(void) {}

    /*!
     * \brief Apply the affine transformation
     *
     * Assumes that the OpenGL modelview stack is active
     */
    void apply(void) const;

    /*!
     * \brief Apply the inverse affine transformation
     *
     * Assumes that the OpenGL modelview stack is active
     */
    void applyInverse(void) const;

    /*!
     * \brief Reset to the identity transformation
     */
    virtual void identity(void);

    /*!
     * \brief Rotate around local x-axis
     *
     * \param [in] a The angle of rotation in radians
     */
    void rotateX(const float a);

    /*!
     * \brief Rotate around local y-axis
     *
     * \param [in] a The angle of rotation in radians
     */
    void rotateY(const float a);

    /*!
     * \brief Rotate around local z-axis
     *
     * \param [in] a The angle of rotation in radians
     */
    void rotateZ(const float a);

    /*!
     * \brief Translate along local x-axis
     *
     * \param [in] d The number of units by which to translate
     */
    void translateX(const float d);

    /*!
     * \brief Translate along local y-axis
     *
     * \param [in] d The number of units by which to translate
     */
    void translateY(const float d);

    /*!
     * \brief Translate along local z-axis
     *
     * \param [in] d The number of units by which to translate
     */
    void translateZ(const float d);

    /*!
     * \brief Transforms the location of a point
     *
     * \param [out] p2 The transformed position
     * \param [in] p1 The position to transform
     */
    void transformPosition(Eigen::Vector3f& p2, const Eigen::Vector3f& p1) const;

public:
    /*!
     * \brief Interpolate between transformations
     *
     * Translation and scaling are linearly interpolated, whereas
     * rotation is interpolated by SLERP.
     *
     * \param [out] out The transformation resulting from the interpolation
     * \param [in] a The initial transformation
     * \param [in] b The final transformation
     * \param [in] t The interpolation parameter, where 0 will result in `a`,
     *   and 1 will result in `b`
     */
    static void interpolate(AffineTransform& out, const AffineTransform& a, const AffineTransform& b, const float t);

protected:
    void asTransform(Eigen::Transform<float, 3, Eigen::Affine>& tf) const;
    void updateRotation(const Eigen::AngleAxis<float>& a);
    void localRotate(Eigen::Vector3f& v) const;
    void axisX(Eigen::Vector3f& v) const;
    void axisY(Eigen::Vector3f& v) const;
    void axisZ(Eigen::Vector3f& v) const;
    void loadQuaternion(Eigen::Quaternion<float>& q) const;
    void storeQuaternion(const Eigen::Quaternion<float>& q);

public:
    /*!
     * \brief Scaling along the local coordinate directions (x, y, z)
     *
     * Relative to the local coordinate directions, before any rotation.
     */
    float s[3];

    /*!
     * \brief Angle of rotation in radians
     */
    float rt;

    /*!
     * \brief Axis of rotation (x, y, z)
     *
     * Only set it to normalized values.
     */
    float ra[3];

    /*!
     * \brief Translation (x, y, z)
     */
    float t[3];

    // Currently not implemented - will cause linker errors if called
private:
    AffineTransform(const AffineTransform& other);
    AffineTransform& operator=(const AffineTransform& other);
};

#endif
