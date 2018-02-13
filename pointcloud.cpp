/*!
** \file pointcloud.cpp
** \brief Implementation of the PointCloud class.
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
#include <fstream>
#include <cctype> // for std::isdigit
#include <cfloat>
#include <cmath>
#include <algorithm>
#include <utility> // for std::pair
#include <sstream>
#include <Eigen/Core>

#include <CGAL/vcm_estimate_normals.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/property_map.h>
//#include <CGAL/Eigen_vcm_traits.h>

#include "pointcloud.h"
#include "error.h"
#include "point.h"
#include "utilities.h"

// #ifdef CGAL_EIGEN3_ENABLED
// #include <CGAL/Eigen_diagonalize_traits.h>
// #else
// #include <CGAL/Diagonalize_traits.h>
// #endif
// typedef double FT;
// // If Eigen is enabled, use it, otherwise fallback to the internal model
// #ifdef CGAL_EIGEN3_ENABLED
// typedef CGAL::Eigen_diagonalize_traits<FT, 3> Diagonalize_traits;
// #else
// typedef CGAL::Diagonalize_traits<FT, 3> Diagonalize_traits;
// #endif

using std::cout;
using std::endl;
using std::vector;
using std::string;
using std::istringstream;
using std::set;
using Eigen::Vector3f;

typedef std::pair<TriangulationPoint, TriangulationVector_3> PointVectorPair;

/*!
  \brief The noise in point positions, as a factor of the average spacing
    between adjacent points on curves
 */
#define NOISE_FACTOR 2.0f

/*!
\brief The minimum surface feature size, as a factor of the average spacing
  between adjacent points on curves
 */
#define FEATURE_FACTOR 10.0f

/*!
\brief Number of neighbours to use for PCA-based normal estimation
 */
#define PCA_NEIGHBORHOOD_SIZE 20

/*!
  \brief The point cloud is always scaled so that it spans this value
  in any dimension.
 */
#define MAX_COORDINATE_DIAMETER 1.25f

/*!
  \brief The initial size at which to rasterize points.
 */
#define DEFAULT_POINT_SIZE 3.0f

/*!
  \brief The initial width at which to rasterize lines.
 */
#define DEFAULT_LINE_WIDTH 1.0f

/*!
 * \brief Number of floating point values per vertex buffer
 */
#define V_BUFFER_SIZE (countCoordinates(blockSize) * 3)

/*!
 * \brief Number of integer values in an index buffer
 *
 * At most n - 1 edges between n vertices, and 2 indices per edge
 */
#define I_BUFFER_SIZE ((blockSize - 1) * 2)

PointCloud::PointCloud(const string & file, const int bs, const int pinc) :
    blockSize(bs), pointIncrement(pinc), pointSize(DEFAULT_POINT_SIZE), lineWidth(DEFAULT_LINE_WIDTH),
    inputStream(file, std::ofstream::in),
    colorOffset(0), sumSegments(0.0f), nSegments(0), lastCurveIndex(-1),
    algorithm(Point::ALGORITHM::DELAUNAY)
{
    for(int i = 0; i < 3; ++i) {
        centroidSum[i] = 0.0f;
        minPoints[i] = FLT_MAX;
        maxPoints[i] = -FLT_MAX;
    }
    colorOffset = countCoordinates(blockSize);
    normalOffset = 2 * colorOffset;
    if( !inputStream.is_open() ) {
        throw Error("Unable to open the input file: " + file);
    }
    if(blockSize < 2) {
        throw Error("Vertex buffer block size must be at least two.");
    }
}

PointCloud::~PointCloud(void) {
    vector<GLuint>::iterator end = vertexBuffers.end();
    for(vector<GLuint>::iterator it = vertexBuffers.begin(); it != end; ++it) {
        if(*it != 0) {
            glDeleteBuffers(1, &(*it));
            *it = 0;
        }
    }

    end = indexBuffers.end();
    for(vector<GLuint>::iterator it = indexBuffers.begin(); it != end; ++it) {
        if(*it != 0) {
            glDeleteBuffers(1, &(*it));
            *it = 0;
        }
    }

    end = vertexArrayObjects.end();
    for(vector<GLuint>::iterator it = vertexArrayObjects.begin(); it != end; ++it) {
        if(*it != 0) {
            glDeleteVertexArrays(1, &(*it));
            *it = 0;
        }
    }

    vector<Point*>::iterator pEnd = points.end();
    for(vector<Point*>::iterator it = points.begin(); it != pEnd; ++it) {
        if(*it != 0) {
            delete *it;
            *it = 0;
        }
    }
}

void PointCloud::draw(void) const {
    glPushMatrix();
    canonicalTransform.apply();

    bool lightingEnabled = glIsEnabled(GL_LIGHTING);
    if(lightingEnabled) {
        glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
        glEnable(GL_COLOR_MATERIAL);
    }

    glPointSize(pointSize);
    glLineWidth(lineWidth);
    vector<GLuint>::const_iterator end = vertexArrayObjects.cend();
    vector<GLuint>::const_iterator vaoIt = vertexArrayObjects.cbegin();
    vector<GLuint>::const_iterator vboIt = vertexBuffers.cbegin();
    vector<GLuint>::const_iterator ibIt = indexBuffers.cbegin();
    vector<GLsizei>::const_iterator vcIt = verticesCounts.cbegin();
    vector<GLsizei>::const_iterator icIt = indicesCounts.cbegin();
    for(; vaoIt != end; ++vaoIt, ++vboIt, ++ibIt, ++vcIt, ++icIt) {
        glBindVertexArray(*vaoIt);
        glBindBuffer(GL_ARRAY_BUFFER, *vboIt);
        glDrawArrays(GL_POINTS, 0, *vcIt);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, *ibIt);
        glDrawElements(GL_LINES, *icIt, GL_UNSIGNED_INT, 0);
    }

    if(lightingEnabled) {
        glDisable(GL_COLOR_MATERIAL);
    }

    glPopMatrix();
}

void PointCloud::update(void) {
    if(pointIncrement <= 0) {
        return;
    }
    vector<float> positions;
    vector<float> normalHypotheses;
    vector<GLuint> indices;
    readFile(positions, normalHypotheses, indices);
    preprocessPoints(positions);
    addPoints(positions, normalHypotheses, indices);
    updateBlocks();
}

void PointCloud::writeData(const std::string & file) const {
    std::ofstream ofs(file, std::ofstream::out);
    if( !ofs.is_open() ) {
        throw Error("Unable to open the output file: " + file);
    }

    vector<vector<Point*>>::const_iterator cEnd = curves.cend();
    for(vector<vector<Point*>>::const_iterator cIt = curves.cbegin(); cIt != cEnd; ++cIt) {
        vector<Point*>::const_iterator pEnd = cIt->cend();
        for(vector<Point*>::const_iterator pIt = cIt->cbegin(); pIt != pEnd; ++pIt) {
            (*pIt)->writeData(ofs);
        }
        if(!ofs.good()) {
            throw Error("Failure to write to open file: " + file);
        }
    }
}

void PointCloud::writeComparison(const std::string & file) {
    Point::ALGORITHM oldAlgorithm = algorithm;
    std::ofstream ofs(file, std::ofstream::out);
    if( !ofs.is_open() ) {
        throw Error("Unable to open the output file: " + file);
    }

    // Write curve indices
    vector<Point*>::iterator pEnd = points.end();
    for(vector<Point*>::iterator it = points.begin(); ; ) {
        Point& p = **it;
        ofs << p.curve() << ',' << p.curve() << ',' << p.curve();
        if(++it != pEnd) {
            ofs << ',';
        } else {
            ofs << endl;
            if(!ofs.good()) {
                throw Error("Failure to write to open file: " + file);
            }
            break;
        }
    }

    // Write positions
    for(vector<Point*>::iterator it = points.begin(); ; ) {
        Point& p = **it;
        ofs << p.x() << ',' << p.y() << ',' << p.z();
        if(++it != pEnd) {
            ofs << ',';
        } else {
            ofs << endl;
            if(!ofs.good()) {
                throw Error("Failure to write to open file: " + file);
            }
            break;
        }
    }

    // Write tangent vectors
    TriangulationVector_3 tangent(0,0,0);
    for(vector<Point*>::iterator it = points.begin(); ; ) {
        Point& p = **it;
        if(p.calculateTangent(tangent)) {
            ofs << tangent.x() << ',' << tangent.y() << ',' << tangent.z();
        } else {
            ofs << 0.0f << ',' << 0.0f << ',' << 1.0f;
        }
        if(++it != pEnd) {
            ofs << ',';
        } else {
            ofs << endl;
            if(!ofs.good()) {
                throw Error("Failure to write to open file: " + file);
            }
            break;
        }
    }

    // Write normal vector estimates
    for(int i = 0; i < Point::nAlgorithms; ++i) {
        setNormalEstimationAlgorithm(Point::listOfAlgorithms[i]);
        for(vector<Point*>::iterator it = points.begin(); ; ) {
            Point& p = **it;
            ofs << p.nx() << ',' << p.ny() << ',' << p.nz();
            if(++it != pEnd) {
                ofs << ',';
            } else {
                ofs << endl;
                if(!ofs.good()) {
                    throw Error("Failure to write to open file: " + file);
                }
                break;
            }
        }
    }

    setNormalEstimationAlgorithm(oldAlgorithm);
}

int PointCloud::nPoints(void) const {
    return points.size();
}

int PointCloud::nCurves(void) const {
    return curves.size();
}

int PointCloud::nPointsOnCurve(const int c) const {
    if(c < 0) {
        throw Error("PointCloud::nPointsOnCurve() was passed a negative curve index.");
    } else if(c >= nCurves()) {
        throw Error("PointCloud::nPointsOnCurve() was an out of range curve index (too large).");
    }
    return curves[c].size();
}

void PointCloud::setPointIncrement(const int n) {
    pointIncrement = n;
}

void PointCloud::incrementPointSize(const float s) {
    if((pointSize + s) <= 1.0f) {
        pointSize = 1.0f;
    } else {
        pointSize += s;
    }
}

void PointCloud::incrementLineWidth(const float w) {
    if((lineWidth + w) <= 1.0f) {
        lineWidth = 1.0f;
    } else {
        lineWidth += w;
    }
}

void PointCloud::queuePointUpdate(const Point& p) {
    int nBufferIndices = 0;
    int blocks[] = {0,0};
    int internals[] = {0,0};
    bufferIndex(nBufferIndices, blocks, internals, p.index());
    staleBlocks.emplace(blocks[0]);
    if(nBufferIndices > 1) {
        staleBlocks.emplace(blocks[1]);
    }
}

void PointCloud::queueCurveUpdate(const int c) {
    if(c < 0) {
        throw Error("PointCloud::queueCurveUpdate() was passed a negative curve index.");
    } else if(c >= nCurves()) {
        throw Error("PointCloud::queueCurveUpdate() was an out of range curve index (too large).");
    }
    vector<Point*>::const_iterator pEnd = curves[c].cend();
    for(vector<Point*>::const_iterator pIt = curves[c].cbegin(); pIt != pEnd; ++pIt) {
        queuePointUpdate(**pIt);
    }
}

Point* PointCloud::getPoint(const int c, const int ci) const {
    if(c < 0) {
        throw Error("PointCloud::getPoint() was passed a negative curve index.");
    } else if(c >= nCurves()) {
        throw Error("PointCloud::getPoint() was an out of range curve index (too large).");
    }
    const vector<Point*>& curve = curves[c];
    if(ci < 0) {
        throw Error("PointCloud::getPoint() was passed a negative within-curve index.");
    } else if(ci >= static_cast<int>(curve.size())) {
        throw Error("PointCloud::getPoint() was an out of range within-curve index (too large).");
    }
    return curve[ci];
}

void PointCloud::setNormalEstimationAlgorithm(const Point::ALGORITHM& alg) {
    algorithm = alg;
    if((algorithm == Point::ALGORITHM::VCM) || (algorithm == Point::ALGORITHM::CURVE_VCM)) {
        calculateVCMNormals(algorithm == Point::ALGORITHM::CURVE_VCM);
        markAllBlocksForUpdate();
    } else if((algorithm == Point::ALGORITHM::PCA) || (algorithm == Point::ALGORITHM::CURVE_PCA)) {
        calculatePCANormals(algorithm == Point::ALGORITHM::CURVE_PCA);
        markAllBlocksForUpdate();
    } else {
        updateExistingPoints();
    }
    updateBlocks();
}

void PointCloud::cycleNormalEstimationAlgorithm(const bool direction) {
    switch(algorithm) {
    case Point::ALGORITHM::NORMAL_HYPOTHESIS:
        if(direction) {
            algorithm = Point::ALGORITHM::DELAUNAY;
        } else {
            algorithm = Point::ALGORITHM::CURVE_PCA;
        }
        break;
    case Point::ALGORITHM::DELAUNAY:
        if(direction) {
            algorithm = Point::ALGORITHM::CURVE_DELAUNAY;
        } else {
            algorithm = Point::ALGORITHM::NORMAL_HYPOTHESIS;
        }
        break;
    case Point::ALGORITHM::CURVE_DELAUNAY:
        if(direction) {
            algorithm = Point::ALGORITHM::VCM;
        } else {
            algorithm = Point::ALGORITHM::DELAUNAY;
        }
        break;
    case Point::ALGORITHM::VCM:
        if(direction) {
            algorithm = Point::ALGORITHM::CURVE_VCM;
        } else {
            algorithm = Point::ALGORITHM::CURVE_DELAUNAY;
        }
        break;
    case Point::ALGORITHM::CURVE_VCM:
        if(direction) {
            algorithm = Point::ALGORITHM::PCA;
        } else {
            algorithm = Point::ALGORITHM::VCM;
        }
        break;
    case Point::ALGORITHM::PCA:
        if(direction) {
            algorithm = Point::ALGORITHM::CURVE_PCA;
        } else {
            algorithm = Point::ALGORITHM::CURVE_VCM;
        }
        break;
    case Point::ALGORITHM::CURVE_PCA:
        if(direction) {
            algorithm = Point::ALGORITHM::NORMAL_HYPOTHESIS;
        } else {
            algorithm = Point::ALGORITHM::PCA;
        }
        break;
    default:
        throw Error("Unrecognized Point::ALGORITHM constant in cycleNormalEstimationAlgorithm().");
    }
    if((algorithm == Point::ALGORITHM::VCM) || (algorithm == Point::ALGORITHM::CURVE_VCM)) {
        calculateVCMNormals(algorithm == Point::ALGORITHM::CURVE_VCM);
        markAllBlocksForUpdate();
    } else if((algorithm == Point::ALGORITHM::PCA) || (algorithm == Point::ALGORITHM::CURVE_PCA)) {
        calculatePCANormals(algorithm == Point::ALGORITHM::CURVE_PCA);
        markAllBlocksForUpdate();
    } else {
        updateExistingPoints();
    }
    updateBlocks();
}

float PointCloud::averageSegmentLength(void) const {
    return sumSegments / static_cast<float>(nSegments);
}

void PointCloud::readFile(vector<float>& v, vector<float>& n, vector<unsigned int>& ind) {
    string line;
    istringstream isstream;
    isstream >> std::skipws; // Remove leading whitespace during formatted input
    int index = 0;
    int curveIndex = 0;
    if(!curves.empty()) {
        index = curves[curves.size() - 1].size();
    }
    float nextValuef = 0.0f;

    bool lineFinished = true;
    while(inputStream && (countVertices(v) < pointIncrement)) {
        if(!std::getline(inputStream, line)) {
            goto CLEANUP_READ_FILE;
        }
        lineFinished = false;
        isstream.clear();
        isstream.str(line);

        // Curve index
        if(!(isstream >> curveIndex)) {
            goto CLEANUP_READ_FILE;
        }
        if(index != 0 && curveIndex != lastCurveIndex) {
            // Curve start
            index = 0;
        }
        if(index == 0) {
            lastCurveIndex = curveIndex;
        }

        // Ignore the frame index
        // `ignore()` skips the comma
        if(!(isstream.ignore() && isstream >> curveIndex)) {
            goto CLEANUP_READ_FILE;
        }

        // Vertex coordinates
        for(int i = 0; i < 3; ++i) {
            if(!(isstream.ignore() && isstream >> nextValuef)) {
                goto CLEANUP_READ_FILE;
            }
            v.emplace_back(nextValuef);
        }

        // Normal hypothesis components
        for(int i = 0; i < 2; ++i) {
            if(!(isstream.ignore() && isstream >> nextValuef)) {
                goto CLEANUP_READ_FILE;
            }
            n.emplace_back(nextValuef);
        }
        if(!(isstream.ignore() && isstream >> nextValuef) && !isstream.eof()) {
            goto CLEANUP_READ_FILE;
        }
        n.emplace_back(nextValuef);

        ind.emplace_back(index);
        ++index;
        lineFinished = true;
    }

CLEANUP_READ_FILE:
    if(!lineFinished && !isstream) {
        throw Error("Malformed line in input file: \"" + line + "\".");
    }
    if(!inputStream.eof() && !inputStream.good()) {
        throw Error("Error reading from the input stream.");
    }
    inputStream.clear();
}

void PointCloud::preprocessPoints(const std::vector<float>& v) {
    canonicalTransform.identity();

    // Find the point cloud centroid and maximum and minimum coordinates
    int nNewVertices = countVertices(v);
    int nVertices = nPoints() + nNewVertices;

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    for(int i = 0; i < nNewVertices; ++i) {
        x = v[i * 3];
        y = v[i * 3 + 1];
        z = v[i * 3 + 2];
        centroidSum[0] += x;
        centroidSum[1] += y;
        centroidSum[2] += z;
        maxPoints[0] = (maxPoints[0] < x) ? x : maxPoints[0];
        maxPoints[1] = (maxPoints[1] < y) ? y : maxPoints[1];
        maxPoints[2] = (maxPoints[2] < z) ? z : maxPoints[2];
        minPoints[0] = (minPoints[0] > x) ? x : minPoints[0];
        minPoints[1] = (minPoints[1] > y) ? y : minPoints[1];
        minPoints[2] = (minPoints[2] > z) ? z : minPoints[2];
    }
    x = -centroidSum[0] / nVertices;
    y = -centroidSum[1] / nVertices;
    z = -centroidSum[2] / nVertices;

    // Find the scale
    Vector3f verticesScale(
        maxPoints[0] - minPoints[0],
        maxPoints[1] - minPoints[1],
        maxPoints[2] - minPoints[2]
    );
    float scale = verticesScale.maxCoeff();
    if(fabs(scale) > FLT_EPSILON) {
        scale = MAX_COORDINATE_DIAMETER / scale;
        canonicalTransform.s[0] = scale;
        canonicalTransform.s[1] = scale;
        canonicalTransform.s[2] = scale;
    } else {
        scale = 1.0f;
    }

    // Scaling needs to come after translation
    canonicalTransform.translateX(scale * x);
    canonicalTransform.translateY(scale * y);
    canonicalTransform.translateZ(scale * z);
}

void PointCloud::addPoints(const vector<float>& v, const vector<float>& n, const vector<GLuint>& ind) {
    int i = nPoints();
    int c = curves.size() - 1;
    Point* point = 0;
    vector<float>::const_iterator vEnd = v.cend();
    vector<float>::const_iterator vIt = v.cbegin();
    vector<float>::const_iterator nIt = n.cbegin();
    vector<GLuint>::const_iterator iIt = ind.cbegin();
    for(; vIt != vEnd; vIt += 3, nIt += 3, ++iIt, ++i) {
        if(*iIt == 0) {
            ++c;
            curves.emplace_back();
        } else {
            Point& prevPoint = *(curves[c][(*iIt) - 1]);
            Vector3f segment(
                *vIt - prevPoint.x(),
                *(vIt + 1) - prevPoint.y(),
                *(vIt + 2) - prevPoint.z()
            );
            sumSegments += segment.norm();
            ++nSegments;
        }
        point = new Point(
            *vIt, *(vIt + 1), *(vIt + 2),
            *nIt, *(nIt + 1), *(nIt + 2),
            i, c, *iIt, this
        );
        points.emplace_back(point);
        curves[c].emplace_back(point);

        if(triangulation.dimension() < 3) {
            TriangulationPoint p(*vIt, *(vIt + 1), *(vIt + 2));
            Vertex_handle vh = triangulation.insert(p);
            point->setTriangulationVertex(vh);
            updateExistingPoints();
        } else {
            point->update();
        }
    }
}

void PointCloud::updateExistingPoints(void) {
    // Only estimate normal vectors once the triangulation is 3D
    if(triangulation.dimension() == 3) {
        vector<Point*>::iterator pEnd = points.end();
        for(vector<Point*>::iterator it = points.begin(); it != pEnd; ++it) {
            (*it)->update();
        }
    }
}

void PointCloud::calculateVCMNormals(const bool& useTangent) {
    if(triangulation.dimension() < 3) {
        return;
    }

    vector<PointVectorPair> pointsAndNormals;
    pointsAndNormals.reserve(nPoints());
    vector<Point*>::const_iterator pEnd = points.cend();
    for(vector<Point*>::const_iterator it = points.cbegin(); it != pEnd; ++it) {
        Point& p = **it;
        pointsAndNormals.emplace_back(
            TriangulationPoint(p.x(), p.y(), p.z()),
            TriangulationVector_3(0,0,0)
        );
    }

    float averageSpacing = averageSegmentLength();
    CGAL::vcm_estimate_normals<
    vector<PointVectorPair>::iterator,
    CGAL::First_of_pair_property_map<PointVectorPair>,
    CGAL::Second_of_pair_property_map<PointVectorPair>,
    CGAL::Eigen_diagonalize_traits<double,3>
    >(
        pointsAndNormals.begin(), pointsAndNormals.end(),
        CGAL::First_of_pair_property_map<PointVectorPair>(),
        CGAL::Second_of_pair_property_map<PointVectorPair>(),
        averageSpacing * FEATURE_FACTOR, // Feature size parameter
        averageSpacing * NOISE_FACTOR, // Noise radius parameter
        CGAL::Eigen_diagonalize_traits<double,3>()
    );

    vector<PointVectorPair>::const_iterator pAndNEnd = pointsAndNormals.cend();
    vector<Point*>::iterator pIt = points.begin();
    for(vector<PointVectorPair>::const_iterator it = pointsAndNormals.cbegin(); it != pAndNEnd; ++it, ++pIt) {
        (*pIt)->setNormalEstimate(it->second, useTangent);
    }
}

void PointCloud::calculatePCANormals(const bool& useTangent) {
    if(triangulation.dimension() < 3) {
        return;
    }

    vector<PointVectorPair> pointsAndNormals;
    pointsAndNormals.reserve(nPoints());
    vector<Point*>::const_iterator pEnd = points.cend();
    for(vector<Point*>::const_iterator it = points.cbegin(); it != pEnd; ++it) {
        Point& p = **it;
        pointsAndNormals.emplace_back(
            TriangulationPoint(p.x(), p.y(), p.z()),
            TriangulationVector_3(0,0,0)
        );
    }

    CGAL::pca_estimate_normals<
    CGAL::Sequential_tag
    >(
        pointsAndNormals.begin(), pointsAndNormals.end(),
        CGAL::First_of_pair_property_map<PointVectorPair>(),
        CGAL::Second_of_pair_property_map<PointVectorPair>(),
        PCA_NEIGHBORHOOD_SIZE
    );

    vector<PointVectorPair>::const_iterator pAndNEnd = pointsAndNormals.cend();
    vector<Point*>::iterator pIt = points.begin();
    for(vector<PointVectorPair>::const_iterator it = pointsAndNormals.cbegin(); it != pAndNEnd; ++it, ++pIt) {
        (*pIt)->setNormalEstimate(it->second, useTangent);
    }
}

void PointCloud::updateBlocks(void) {
    if(staleBlocks.empty()) {
        return;
    }

    int i = 0;
    int ic = 0;
    int block = 0;
    vector<float> v(V_BUFFER_SIZE);
    vector<GLuint> ind(I_BUFFER_SIZE);
    const int nBlocks = static_cast<int>(vertexBuffers.size());
    bool isNewBlock = false;
    int nNewInd = 0;
    int currentInd = 0;
    int nCurrentV = 0;

    set<int>::const_iterator sEnd = staleBlocks.cend();
    // The following relies on the set being sorted.
    for(set<int>::const_iterator sIt = staleBlocks.cbegin(); sIt != sEnd; ++sIt) {
        block = *sIt;
        isNewBlock = (block >= nBlocks);

        // Zero the data
        std::fill(v.begin(), v.end(), 0.0f);
        std::fill(ind.begin(), ind.end(), 0);

        // Fill the data
        i = 0;
        ic = -1;
        nNewInd = 0;
        currentInd = -1;
        nCurrentV = 0;
        if(!isNewBlock) {
            nCurrentV = verticesCounts[block];
        }
        vector<Point*>::const_iterator bEnd = beforeEndBlock(block);
        vector<Point*>::const_iterator bIt = beginBlock(block);
        do {
            v[++ic] = (*bIt)->x();
            v[ic + colorOffset] = (*bIt)->r();
            v[ic + normalOffset] = (*bIt)->nx();
            v[++ic] = (*bIt)->y();
            v[ic + colorOffset] = (*bIt)->g();
            v[ic + normalOffset] = (*bIt)->ny();
            v[++ic] = (*bIt)->z();
            v[ic + colorOffset] = (*bIt)->b();
            v[ic + normalOffset] = (*bIt)->nz();
            if(i > 0 && (*bIt)->curveIndex() != 0) {
                ind[++currentInd] = i-1;
                ind[++currentInd] = i;
                if(isNewBlock || (i >= nCurrentV)) {
                    nNewInd += 2;
                }
            }
            ++i;
        } while(bIt++ != bEnd);

        // Add to graphics server memory
        if(isNewBlock) {
            addBlock(v, ind, i, nNewInd);
        } else {
            overwriteBlock(block, v, ind, i - nCurrentV, nNewInd);
        }
    }

    staleBlocks.clear();
}

void PointCloud::markAllBlocksForUpdate(void) {
    staleBlocks.clear();
    int nBlocks = static_cast<int>(vertexBuffers.size());
    for(int i = 0; i < nBlocks; ++i) {
        staleBlocks.insert(i);
    }
}

void PointCloud::addBlock(const vector<float>& v, const vector<GLuint>& ind, const int nVertices, const int nIndices) {

    int blockIndex = vertexArrayObjects.size();
    vertexArrayObjects.emplace_back(0);
    glGenVertexArrays(1, &vertexArrayObjects[blockIndex]);
    glBindVertexArray(vertexArrayObjects[blockIndex]);

    // Vertices
    vertexBuffers.emplace_back(0);
    glGenBuffers(1, &vertexBuffers[blockIndex]);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers[blockIndex]);
    glBufferData(GL_ARRAY_BUFFER, V_BUFFER_SIZE * sizeof(float), &v[0], GL_DYNAMIC_DRAW);

    // Positions
    glEnableClientState(GL_VERTEX_ARRAY); // Deprecated
    glVertexPointer(3, GL_FLOAT, 0, 0);

    // Colour
    glEnableClientState(GL_COLOR_ARRAY);
    glColorPointer(3, GL_FLOAT, 0, BUFFER_OFFSET(colorOffset * sizeof(float)));

    // Normal vectors
    glEnableClientState(GL_NORMAL_ARRAY);
    glNormalPointer(GL_FLOAT, 0, BUFFER_OFFSET(normalOffset * sizeof(float)));

    // Indices
    indexBuffers.emplace_back(0);
    glGenBuffers(1, &indexBuffers[blockIndex]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffers[blockIndex]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, I_BUFFER_SIZE * sizeof(GLuint), &ind[0], GL_DYNAMIC_DRAW);

    verticesCounts.emplace_back(nVertices);
    indicesCounts.emplace_back(nIndices);
}

void PointCloud::overwriteBlock(const int block, const vector<float>& v, const vector<GLuint>& ind, const int nNewVertices, const int nNewIndices) {

    // Vertex buffer
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers[block]);
    float* vBufferData = (float*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    if(vBufferData == 0) {
        throw Error("Failure to map vertex buffer in PointCloud::addToBlock().");
    }
    std::copy( v.cbegin(), v.cend(), vBufferData );
    glUnmapBuffer(GL_ARRAY_BUFFER);

    if(nNewVertices > 0) {
        verticesCounts[block] += nNewVertices;
    }

    // Index buffer
    if(nNewIndices > 0) {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffers[block]);
        GLuint* iBufferData = (GLuint*)glMapBuffer(GL_ELEMENT_ARRAY_BUFFER, GL_WRITE_ONLY);
        if(iBufferData == 0) {
            throw Error("Failure to map index buffer in PointCloud::addToBlock().");
        }
        std::copy( ind.cbegin(), ind.cend(), iBufferData );
        glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);

        indicesCounts[block] += nNewIndices;
    }
}

void PointCloud::bufferIndex(int& n, int blocks[2], int internals[2], const int ind) const {
    n = 1;
    /* There are really `blockSize - 1` vertices per block, since
     * the last vertex in each block is duplicated as the first
     * vertex of the next block.
     */
    if(ind < blockSize) {
        blocks[0] = 0;
        internals[0] = ind;
        if(ind == (blockSize - 1)) {
            n = 2;
            blocks[1] = 1;
            internals[1] = 0;
        }
    } else {
        if((ind % (blockSize - 1)) == 0) {
            n = 2;
            blocks[0] = (ind / (blockSize - 1)) - 1;
            internals[0] = blockSize - 1;
            blocks[1] = blocks[0] + 1;
            internals[1] = 0;
        } else {
            blocks[0] = (ind / (blockSize - 1));
            internals[0] = ind % (blockSize - 1);
        }
    }
}

vector<Point*>::const_iterator PointCloud::beginBlock(const int block) const {
    int ind = block * (blockSize - 1);
    if(ind >= static_cast<int>(points.size())) {
        throw Error("Request for start of block which corresponds to a point index beyond the current range of points.");
    }
    return points.cbegin() + ind;
}

vector<Point*>::const_iterator PointCloud::beforeEndBlock(const int block) const {
    int ind = (block + 1) * (blockSize - 1);
    const int nPoints = static_cast<int>(points.size());
    if(ind >= nPoints) {
        if((ind - (nPoints - 1)) >= blockSize) {
            throw Error("Request for end of a block which should not exist, based on the current range of points.");
        }
        ind = nPoints - 1;
    }
    return points.cbegin() + ind;
}

int PointCloud::countVertices(const std::vector<float>& v) {
    int sz = v.size();
    if( sz % 3 != 0 ) {
        throw Error("Vertex vector has a size which is not a multiple of 3: " + sz);
    }
    return sz / 3;
}

int PointCloud::countCoordinates(const int n) {
    return n * 3;
}
