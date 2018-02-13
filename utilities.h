#ifndef UTILITIES_H
#define UTILITIES_H

/*!
** \file utilities.h
** \brief Global utility functions
**
** ### About
** Created for: CMPUT 511 Project\n
** Fall 2017\n
** Bernard Llanos\n
** Department of Computing Science, University of Alberta
**
** ## References
** - "opengl - How to cast int to const GLvoid*?", posted on Stack Overflow
**   (https://stackoverflow.com/questions/23177229/how-to-cast-int-to-const-glvoid#23177754)
*/

#include <string>
#include <fstream>

#define PI 3.1415926535897f

#define SECONDS_TO_MILLISECONDS 1000.0f

float deg2rad(const float deg);

/*!
 * \brief Routine to draw a GLUT stroke character string.
 *
 * \param [in] font The font to use
 * \param [in] x The x-coordinate of the start of the string,
 *   where 0 is the left of the viewport, and 1 is the right of the viewport
 * \param [in] y The y-coordinate of the start of the string
 *   where 0 is the bottom of the viewport, and 1 is the top of the viewport
 * \param [in] szx The horizontal scale of the string, expressed as a fraction
 *   of the viewport width.
 * \param [in] szy The vertical scale of the string, expressed as a fraction
 *   of the viewport height.
 * \param [in] str The string to render, assumed not to contain newline characters
 */
void writeStrokeString(void* font, const float x, const float y, const float szx, const float szy, const std::string& str);

/*!
 * \brief Find a line in a file starting with the given token
 *
 * The file pointer will be advanced to the position after the given
 * token, or to the end of the file, if the token is not found
 * at the start of any lines. This function does not read
 * the part of the file before the current file pointer position.
 * The current file pointer position is assumed to be the start
 * of a line.
 *
 * \param [in] ifs The file stream to read from
 * \param [in] token The token to search for at the start of lines
 *   in the file. Whitespace at the start of each line is ignored.
 *   The token can contain whitespace other than newlines.
 * \return `true`, if a line starting with the token was found.
 */
bool seekToLineStartingWith(std::ifstream& ifs, const std::string& token);

/*!
 * \brief Remove whitespace from the start and end of a string.
 *
 * \param [in] str The string to modify
 */
void trim(std::string& str);

/*!
 * \brief Trim the last element from a filepath
 *
 * \param [out] out The filepath with the last directory/filename removed
 * \param [in] in The input filepath
 * \return The filepath separating character which was used to make
 * the change. If no changes were made, '\0' is returned.
 */
char upFilepath(std::string& out, const std::string& in);

/*!
 * \brief A macro for casting to GLvoid*
 *
 * From https://stackoverflow.com/questions/23177229/how-to-cast-int-to-const-glvoid#23177754
 */
#define BUFFER_OFFSET(idx) (static_cast<char*>(0) + (idx))

#endif
