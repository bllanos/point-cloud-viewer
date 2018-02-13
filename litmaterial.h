#ifndef LIT_MATERIAL_H
#define LIT_MATERIAL_H

/*!
** \file litmaterial.h
** \brief Definition of the LitMaterial class.
**
** ### About
** Created for: CMPUT 511 Project\n
** Fall 2017\n
** Bernard Llanos\n
** Department of Computing Science, University of Alberta
**
** ## References
**
** ### Textbook Code
** From 'ExperimenterSource.zip' downloaded from the course textbook website
** (http://www.sumantaguha.com/)
** - 'ExperimenterSource/Chapter11/LitCylinder/litcylinder.cpp'
*/

#include <GL/glew.h>
#include <string>

/*!
 * \brief LitMaterial class.
 *
 * A single point light source, and a single set of material properties,
 * providing a quick lighting configuration.
 */
class LitMaterial
{
public:

    /*!
     * \brief Create a set of light and material properties
     *
     * Material properties are set to default values.
     */
    LitMaterial(void);

    /*!
     * \brief Load material properties from a file
     * A '.mtl' (Material Template Library) file (described at
     * https://en.wikipedia.org/wiki/Wavefront_.obj_file#Material_template_library)
     * is expected.
     *
     * Only lines starting with the following tokens are recognized:
     * - newmtl
     * - Ka
     * - Kd
     * - Ks
     * - Ns
     *
     * Material properties loaded from the file will be applied to front faces only.
     *
     * \param [in] file The filepath of the material file to read.
     * \param [in] file The name of the material definition to use.
     */
    LitMaterial(const std::string & file, const std::string & mtlName);

    ~LitMaterial(void) {}

    /*!
     * \brief Configure OpenGL with this object's light and material properties
     */
    void enable(void);

    /*!
     * \brief Undo the effects of enable()
     */
    void disable(void);

    /*!
     * \brief Output the filename and material name used to load material properties
     *
     * \param [out] filename The filepath of the material file.
     * \param [out] name The name of the selected material defined in the material
     *   file
     */
    void getSource(std::string& filename, std::string& name);

protected:
    void readFile(const std::string & file);
    /*!
     * \brief Read three consecutive floating point values
     *
     * Assumes the input stream has been configured to skip whitespace during
     * formatted input (using `ifs >> std::skipws`).
     *
     * \param [out] out Array into which the values are stored
     * \param [in] ifs Data input stream
     */
    static void readTriple(float* out, std::ifstream& ifs);

protected:
    /*!
     * \brief Material filename
     *
     * The source for the material properties in this object.
     */
    std::string filename;
    /*!
     * \brief Material name
     *
     * Used to find the material in a '.mtl' file.
     */
    std::string name;

    /*!
     * \brief Global ambient light intensity
     */
    GLfloat globAmb[4];
    /*!
     * \brief Light ambient intensity
     */
    GLfloat lightAmb[4];
    /*!
     * \brief Light diffuse intensity
     */
    GLfloat lightDiff[4];
    /*!
     * \brief Light specular intensity
     */
    GLfloat lightSpec[4];
    /*!
     * \brief Light position
     */
    GLfloat lightPos[4];

    /*!
     * \brief Material front face ambient reflectances
     */
    GLfloat matAmbFront[4];
    /*!
     * \brief Material front face diffuse reflectances
     */
    GLfloat matDiffFront[4];
    /*!
     * \brief Material back face ambient reflectances
     */
    GLfloat matAmbBack[4];
    /*!
     * \brief Material back face diffuse reflectances
     */
    GLfloat matDiffBack[4];
    /*!
     * \brief Material front face specular reflectance
     */
    GLfloat matSpecFront[4];
    /*!
     * \brief Material back face specular reflectances
     */
    GLfloat matSpecBack[4];
    /*!
     * \brief Material front face specular exponent
     */
    GLfloat matShineFront[1];
    /*!
     * \brief Material back face specular exponent
     */
    GLfloat matShineBack[1];

    // Currently not implemented - will cause linker errors if called
private:
    LitMaterial(const LitMaterial& other);
    LitMaterial& operator=(const LitMaterial& other);
};

#endif
