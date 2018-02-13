#ifndef ERROR_H
#define ERROR_H

/*!
** \file error.h
** \brief Definition and implementation of the Error class.
**
** ### About
** Created for: CMPUT 511 Assignment 1: 3D Model Viewer\n
** Fall 2017\n
** Bernard Llanos\n
** Department of Computing Science, University of Alberta
**
** ## References
** - Standard Template Library exception class:
**   http://www.cplusplus.com/reference/exception/exception/
*/

#include <exception>
#include <string>

/*!
 * \brief A basic error message class
 *
 * Extends the STL exception class to hold a custom what() message
 */
class Error : public std::exception
{
private:
    std::string msg;

public:
    /*!
     * \brief Create an error message
     *
     * \param [in] m The error message to display using what().
     */
    Error(const std::string & m) :
        msg(m)
    {}

    virtual const char* what() const noexcept override {
        return msg.c_str();
    }
};

#endif
