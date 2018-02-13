#ifndef OBJECT_H
#define OBJECT_H

/*!
** \file object.h
** \brief Definition of the Object class.
**
** ### About
** Created for: CMPUT 511 Assignment 1: 3D Model Viewer\n
** Fall 2017\n
** Bernard Llanos\n
** Department of Computing Science, University of Alberta
*/

#include "affinetransform.h"

/*!
 * \brief A renderable 3D model with a 3D pose
 *
 * \tparam Model Instances of Model can be drawn, and written to a file
 */
template <typename Model> class Object
{
public:

    /*!
     * \brief Create an identity transformation
     * \param [in] model_ The renderable geometry. This object takes ownership
     * of model_
     */
    Object(Model*& model_);

    ~Object(void);

    /*!
     * \brief Render the object
     *
     * Assumes that the OpenGL modelview stack is active.
     */
    void draw(void) const;

    /*!
     * \brief Update the object for one timestep
     */
    void update(void);

    /*!
     * \brief Output geometry to a file
     *
     * \param [in] file The filepath of the geometry file to write.
     */
    void writeData(const std::string & file) const;

    /*!
     * \brief Access the model geoemtry
     *
     * \return The geometry encapsulated by this object.
     */
    Model* getModel(void) const;

public:
    /*!
     * \brief The 3D pose of the object
     *
     * At construction, this is the identity pose
     */
    AffineTransform tf;

private:
    /*!
     * \brief The model geometry
     */
    Model* model;

    // Currently not implemented - will cause linker errors if called
private:
    Object(const Object& other);
    Object& operator=(const Object& other);
};

template <typename Model> Object<Model>::Object(Model*& model_) :
    tf(), model(model_)
{
    model_ = 0;
}

template <typename Model> Object<Model>::~Object(void) {
    if(model != 0) {
        delete model;
        model = 0;
    }
}

template <typename Model> void Object<Model>::draw(void) const {
    glPushMatrix();
    tf.apply();
    model->draw();
    glPopMatrix();
}

template <typename Model> void Object<Model>::update(void) {
    model->update();
}

template <typename Model> void Object<Model>::writeData(const std::string & file) const {
    glPushMatrix();
    glLoadIdentity();
    tf.apply();
    model->writeData(file);
    glPopMatrix();
}

template <typename Model> Model* Object<Model>::getModel(void) const {
    return model;
}

#endif
