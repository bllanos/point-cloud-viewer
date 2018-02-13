/*!
** \file utilities.cpp
** \brief Global utility function Implementations
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
** - 'ExperimenterSource/Chapter3/Fonts/fonts.cpp'
*/

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>
#include <cctype> // for std::isspace

#include "utilities.h"
#include "error.h"

using std::string;

float deg2rad(const float deg) {
    return (deg * PI) / 180.0f;
}

void writeStrokeString(void* font, const float x, const float y, const float szx, const float szy, const std::string& str)
{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0f, 1.0f, 0.0f, 1.0f, -1.0f, 1.0f);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glTranslatef(x, y, 0.0f);
    int nChar = str.length();
    unsigned char* cstr = new unsigned char[nChar+1];
    cstr[nChar] = 0;
    for(int i = 0; i < nChar; ++i) {
        cstr[i] = static_cast<unsigned char>(str[i]);
    }
    float length = static_cast<float>(glutStrokeLength(font, cstr));
    float height = glutStrokeHeight(font);
    glScalef(szx / length, szy / height, 1.0f);

    glColor3f(0.0f, 1.0f, 0.0f);

    glutStrokeString(font, cstr);

    delete [] cstr;
    cstr = 0;

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

bool seekToLineStartingWith(std::ifstream& ifs, const std::string& token) {
    std::streamsize n = token.size() + 1;
    char* words = new char[n];
    bool result = false;
    char nextChar = 0;
    while(ifs) {
        // Check the current line
        ifs >> std::ws;
        if(!ifs.getline(words, n)) {
            break;
        }
        if(words == token) {
            result = true;
            break;
        }
        // Proceed to the next line
        while(ifs && ifs.get(nextChar) && (nextChar != '\n')) {}
    }
    delete [] words;
    words = 0;

    if(!ifs.eof() && !ifs.good()) {
        throw Error("Error reading from the file in seekToLineStartingWith().");
    }
    return result;
}

void trim(std::string& str) {
    std::size_t i = str.size() - 1;
    while(std::isspace(str[i]) && i <= 0) {
        str.pop_back();
        --i;
    }
    i = 0;
    while(std::isspace(str[i])) {
        ++i;
    }
    str = str.substr(i);
}

char upFilepath(std::string& out, const std::string& in) {
    std::size_t found = in.find_last_of("/\\");
    if(found == string::npos) {
        out = in;
        return 0;
    } else {
        out = in.substr(0,found);
        return in[found];
    }
}
