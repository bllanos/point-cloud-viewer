/*!
** \file litmaterial.cpp
** \brief Implementation of the LitMaterial class.
**
** ### About
** Created for: CMPUT 511 Project\n
** Fall 2017\n
** Bernard Llanos\n
** Department of Computing Science, University of Alberta
*/

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>

#include "litmaterial.h"
#include "utilities.h"
#include "error.h"

using std::string;
using std::endl;
using std::cout;

LitMaterial::LitMaterial(void) {
    globAmb[0] = 0.2f;
    globAmb[1] = 0.2f;
    globAmb[2] = 0.2f;
    globAmb[3] = 1.0f;

    lightAmb[0] = 0.0f;
    lightAmb[1] = 0.0f;
    lightAmb[2] = 0.0f;
    lightAmb[3] = 1.0f;
    lightDiff[0] = 0.6f;
    lightDiff[1] = 0.6f;
    lightDiff[2] = 0.6f;
    lightDiff[3] = 1.0f;
    lightSpec[0] = 0.5f;
    lightSpec[1] = 0.5f;
    lightSpec[2] = 0.5f;
    lightSpec[3] = 1.0f;
    lightPos[0] = 10.0f;
    lightPos[1] = 10.0f;
    lightPos[2] = 0.0f;
    lightPos[3] = 1.0f;

    matAmbFront[0] = 1.0f;
    matAmbFront[1] = 1.0f;
    matAmbFront[2] = 1.0f;
    matAmbFront[3] = 1.0f;
    matDiffFront[0] = 0.0f;
    matDiffFront[1] = 0.0f;
    matDiffFront[2] = 1.0f;
    matDiffFront[3] = 1.0f;

    matAmbBack[0] = 1.0f;
    matAmbBack[1] = 1.0f;
    matAmbBack[2] = 1.0f;
    matAmbBack[3] = 1.0f;
    matDiffBack[0] = 1.0f;
    matDiffBack[1] = 0.0f;
    matDiffBack[2] = 0.0f;
    matDiffBack[3] = 1.0f;

    matSpecFront[0] = 1.0f;
    matSpecFront[1] = 1.0f;
    matSpecFront[2] = 1.0f;
    matSpecFront[3] = 1.0f;
    matSpecBack[0] = 1.0f;
    matSpecBack[1] = 1.0f;
    matSpecBack[2] = 1.0f;
    matSpecBack[3] = 1.0f;
    matShineFront[0] = 50.0f;
    matShineBack[0] = 50.0f;
}

LitMaterial::LitMaterial(const string & file, const string & mtlName) :
  LitMaterial() {
      filename = file;
      name = mtlName;
      readFile(file);
}

void LitMaterial::enable(void) {
    glEnable(GL_LIGHTING);

    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiff);
    glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpec);

    // Keep the light fixed in world space
    glPushMatrix();
    glLoadIdentity();
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glPopMatrix();

    glEnable(GL_LIGHT0);

    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, globAmb);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
    glLightModeli(GL_LIGHT_MODEL_COLOR_CONTROL, GL_SEPARATE_SPECULAR_COLOR);

    glMaterialfv(GL_FRONT, GL_AMBIENT, matAmbFront);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, matDiffFront);
    glMaterialfv(GL_BACK, GL_AMBIENT, matAmbBack);
    glMaterialfv(GL_BACK, GL_DIFFUSE, matDiffBack);
    glMaterialfv(GL_FRONT, GL_SPECULAR, matSpecFront);
    glMaterialfv(GL_BACK, GL_SPECULAR, matSpecBack);
    glMaterialfv(GL_FRONT, GL_SHININESS, matShineFront);
    glMaterialfv(GL_BACK, GL_SHININESS, matShineBack);

    /* A really annoying feature of the fixed-function lighting
     * is that it does not correct for scaling in the modelview
     * matrix when transforming normal vectors.
     * See https://www.opengl.org/archives/resources/faq/technical/lights.htm
     *
     * Use `GL_NORMALIZE` instead if there is non-uniform scaling.
     */
     glEnable(GL_RESCALE_NORMAL);
}

void LitMaterial::disable(void) {
    glDisable(GL_LIGHT0);
    glDisable(GL_LIGHTING);
    glDisable(GL_RESCALE_NORMAL);
}

void LitMaterial::getSource(std::string& f, std::string& n) {
    f = filename;
    n = name;
}

void LitMaterial::readFile(const std::string & file) {
    std::ifstream ifs(file, std::ofstream::in);
    if( !ifs.is_open() ) {
        throw Error("Unable to open the material file: " + file);
    }

    if( !seekToLineStartingWith(ifs, "newmtl " + name) ) {
        throw Error("The file " + file + "' does not contain the material " + name + ".");
    }

    ifs >> std::skipws; // Remove leading whitespace during formatted input

    string word;
    char nextChar = 0;
    while(ifs) {
        if(!(ifs >> word)) {
            break;
        }
        if(word == "Ka") {
            readTriple(matAmbFront, ifs);
        } else if(word == "Kd") {
            readTriple(matDiffFront, ifs);
        } else if(word == "Ks") {
            readTriple(matSpecFront, ifs);
        } else if(word == "Ns") {
            ifs >> *matShineFront;
        }
        // Go to the next line
        while(ifs && ifs.get(nextChar) && (nextChar != '\n')) {}
    }

    if(!ifs.eof() && !ifs.good()) {
        throw Error("Error reading from the open file: " + file);
    } else if(ifs.eof()) {
        cout << "Finished reading material file for material properties: " << file << endl;
    }
}

void LitMaterial::readTriple(float* out, std::ifstream& ifs) {
    for(int i = 0; i < 3; ++i) {
        ifs >> out[i];
    }
}
