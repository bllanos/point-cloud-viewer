/*!
** \file affinetransform.cpp
** \brief Implementation of the AffineTransform class.
**
** ### About
** Created for: CMPUT 511 Assignment 1: 3D Model Viewer\n
** Fall 2017\n
** Bernard Llanos\n
** Department of Computing Science, University of Alberta
*/

#include <GL/glew.h>
//#include <iostream>
//using std::cout;
//using std::endl;

#include "affinetransform.h"

using Eigen::Transform;
using Eigen::Vector3f;
using Eigen::Quaternion;
using Eigen::AngleAxis;

AffineTransform::AffineTransform(void) {
    identity();
}

void AffineTransform::identity(void) {
    s[0] = 1;
    s[1] = 1;
    s[2] = 1;
    rt = 0;
    ra[0] = 0;
    ra[1] = 0;
    ra[2] = 0;
    t[0] = 0;
    t[1] = 0;
    t[2] = 0;
}

void AffineTransform::apply(void) const {
    Transform<float, 3, Eigen::Affine> tf;
    asTransform(tf);
    glMultMatrixf(tf.data());
}

void AffineTransform::applyInverse(void) const {
    Transform<float, 3, Eigen::Affine> tf;
    asTransform(tf);
    tf = tf.inverse();
    glMultMatrixf(tf.data());
}

void AffineTransform::rotateX(const float a) {
    Vector3f v;
    axisX(v);
    //cout << "axisX: (" << v(0) << ", " << v(1) << ", " << v(2) << ")" << endl;
    //cout << "angle: " << a << endl;
    AngleAxis<float> angleAxis(a, v);
    updateRotation(angleAxis);
}

void AffineTransform::rotateY(const float a) {
    Vector3f v;
    axisY(v);
    AngleAxis<float> angleAxis(a, v);
    updateRotation(angleAxis);
}

void AffineTransform::rotateZ(const float a) {
    Vector3f v;
    axisZ(v);
    AngleAxis<float> angleAxis(a, v);
    updateRotation(angleAxis);
}

void AffineTransform::translateX(const float d) {
    Vector3f v;
    axisX(v);
    t[0] += v(0) * d;
    t[1] += v(1) * d;
    t[2] += v(2) * d;
}

void AffineTransform::translateY(const float d) {
    Vector3f v;
    axisY(v);
    t[0] += v(0) * d;
    t[1] += v(1) * d;
    t[2] += v(2) * d;
}

void AffineTransform::translateZ(const float d) {
    Vector3f v;
    axisZ(v);
    t[0] += v(0) * d;
    t[1] += v(1) * d;
    t[2] += v(2) * d;
}

void AffineTransform::transformPosition(Eigen::Vector3f& p2, const Eigen::Vector3f& p1) const {
    Transform<float, 3, Eigen::Affine> tf;
    asTransform(tf);
    p2 = tf * p1;
}

void AffineTransform::interpolate(AffineTransform& out, const AffineTransform& a, const AffineTransform& b, const float t) {

    // Linear interpolation
    const float oneMinusT = 1.0f - t;
    for(int i = 0; i < 3; ++i) {
        out.s[i] = (a.s[i] * oneMinusT) + (b.s[i] * t);
        out.t[i] = (a.t[i] * oneMinusT) + (b.t[i] * t);
    }

    // SLERP
    Quaternion<float> qA;
    a.loadQuaternion(qA);
    Quaternion<float> qB;
    b.loadQuaternion(qB);
    if( qA.dot(qB) < 0.0f ) {
        qB.w() *= -1;
        qB.x() *= -1;
        qB.y() *= -1;
        qB.z() *= -1;
    }
    Quaternion<float> qOut = qA.slerp(t,qB);
    out.storeQuaternion(qOut);
}

void AffineTransform::asTransform(Transform<float, 3, Eigen::Affine>& tf) const {
    tf.setIdentity();
    tf.prescale(Vector3f(s[0], s[1], s[2]));
    tf.prerotate(Eigen::AngleAxis<float>(rt, Vector3f(ra[0], ra[1], ra[2])));
    tf.pretranslate(Vector3f(t[0], t[1], t[2]));
}

void AffineTransform::updateRotation(const Eigen::AngleAxis<float>& a) {
    Quaternion<float> q;
    loadQuaternion(q);
    Quaternion<float> q2(a);
    q = q2 * q;
    storeQuaternion(q);
    //cout << "axisNew: (" << ra[0] << ", " << ra[1] << ", " << ra[2] << ")" << endl;
    //cout << "angleNew: " << rt << endl;
}

void AffineTransform::localRotate(Eigen::Vector3f& v) const {
    Transform<float, 3, Eigen::Affine> tf;
    tf = Eigen::AngleAxis<float>(rt, Vector3f(ra[0], ra[1], ra[2]));
    v = tf * v;
}

void AffineTransform::axisX(Vector3f& v) const {
    v(0) = 1.0f;
    v(1) = 0.0f;
    v(2) = 0.0f;
    localRotate(v);
}

void AffineTransform::axisY(Vector3f& v) const {
    v(0) = 0.0f;
    v(1) = 1.0f;
    v(2) = 0.0f;
    localRotate(v);
}

void AffineTransform::axisZ(Vector3f& v) const {
    v(0) = 0.0f;
    v(1) = 0.0f;
    v(2) = 1.0f;
    localRotate(v);
}

void AffineTransform::loadQuaternion(Quaternion<float>& q) const {
    q = AngleAxis<float>(rt, Vector3f(ra[0], ra[1], ra[2]));
}

void AffineTransform::storeQuaternion(const Quaternion<float>& q) {
    Quaternion<float> qN = q.normalized();
    AngleAxis<float> angleAxis(qN);
    rt = angleAxis.angle();
    Vector3f v = angleAxis.axis();
    ra[0] = v(0);
    ra[1] = v(1);
    ra[2] = v(2);
}
