/*!
** \file point.cpp
** \brief Implementation of the Point class.
**
** ### About
** Created for: CMPUT 511 Project\n
** Fall 2017\n
** Bernard Llanos\n
** Department of Computing Science, University of Alberta
*/

#include <iostream>
#include <fstream>
#include <iterator>
#include <cmath>
#include <Eigen/Geometry>
#include <CGAL/linear_least_squares_fitting_3.h>

#include "pointcloud.h"
#include "point.h"
#include "error.h"
#include "utilities.h"

using std::endl;
using std::vector;
using Eigen::Vector3f;

// Colours for points on or off the current curve
#define CURRENT_CURVE_R 0.0f
#define CURRENT_CURVE_G 1.0f
#define CURRENT_CURVE_B 1.0f
#define NOT_CURRENT_CURVE_R1 0.0f
#define NOT_CURRENT_CURVE_G1 1.0f
#define NOT_CURRENT_CURVE_B1 0.0f
#define NOT_CURRENT_CURVE_R2 1.0f
#define NOT_CURRENT_CURVE_G2 1.0f
#define NOT_CURRENT_CURVE_B2 0.0f
#define NOT_CURRENT_CURVE_R3 1.0f
#define NOT_CURRENT_CURVE_G3 0.0f
#define NOT_CURRENT_CURVE_B3 0.0f
#define ANGLE_THRESHOLD_1 deg2rad(10.0f)
#define ANGLE_THRESHOLD_2 deg2rad(45.0f)

#define N_VERTICES_IN_TETRAHEDRON 4

/*!
 * \brief The neighbourhood radius for curve tangent estimation, as a factor
 *   of the average spacing between adjacent points on curves
 *
 * Neighbourhoods will always be at least one point in size, regardless of the
 * radius setting.
 *
 * Note: The radius is a geodesic distance (an arc length)
 */
#define TANGENT_RADIUS_FACTOR 5.0f

const Point::ALGORITHM Point::listOfAlgorithms[] = {
    Point::ALGORITHM::NORMAL_HYPOTHESIS,
    Point::ALGORITHM::DELAUNAY,
    Point::ALGORITHM::CURVE_DELAUNAY,
    Point::ALGORITHM::VCM,
    Point::ALGORITHM::CURVE_VCM,
    Point::ALGORITHM::PCA,
    Point::ALGORITHM::CURVE_PCA
};

const int Point::nAlgorithms = static_cast<int>(sizeof(Point::listOfAlgorithms) / sizeof(Point::ALGORITHM));

Point::Point(
    const float x, const float y, const float z,
    const float nx, const float ny, const float nz,
    const int ind_, const int curve, const int curveIndex,
    PointCloud* pointCloud_
) :
    px(x), py(y), pz(z),
    nex(nx), ney(ny), nez(nz), nhx(nx), nhy(ny), nhz(nz),
    ind(ind_), c(curve), cI(curveIndex),
    pointCloud(pointCloud_)
{}

void Point::update(void) {
    if(vh == Vertex_handle()) {
        // New point

        // Add the new point
        TriangulationPoint p(px, py, pz);
        vh = pointCloud->triangulation.insert(p);
        vh->info() = this;

        // Find the neighbourhood that should be updated
        vector<Edge> edges;
        pointCloud->triangulation.incident_edges(vh, std::back_inserter(edges));

        // Update neighbourhood
        Vertex_handle v;
        vector<Edge>::const_iterator edgesEnd = edges.cend();
        for(vector<Edge>::const_iterator edgesIt = edges.begin(); edgesIt != edgesEnd; ++edgesIt) {
            for(int i = 0; i < 2; ++i) {
                if(i == 0) {
                    v = edgesIt->first->vertex(edgesIt->second);
                } else {
                    v = edgesIt->first->vertex(edgesIt->third);
                }
                if(!pointCloud->triangulation.is_infinite(v) && (v != vh)) {
                    v->info()->update();
                }
            }
        }
    }

    updateNormal();

    // Update the colours of points on the previous curve
    if((cI == 0) && (c > 0)) {
        pointCloud->queueCurveUpdate(c - 1);
    }
    pointCloud->queuePointUpdate(*this);
}

void Point::writeData(std::ofstream& ofs) const {
    ofs << c << ", "
        << cI << ", "
        << px << ", "
        << py << ", "
        << pz << ", "
        << nex << ", "
        << ney << ", "
        << nez << ", ";
    TriangulationVector_3 t(0.0f, 0.0f, 0.0f);
    calculateTangent(t);
    ofs << t.x() << ", "
        << t.y() << ", "
        << t.z()
        << endl;
}


void Point::position(Eigen::Vector3f& v) const {
    v(0) = px;
    v(1) = py;
    v(2) = pz;
}


void Point::position(std::vector<float>& v) const {
    v.emplace_back(px);
    v.emplace_back(py);
    v.emplace_back(pz);
}


float Point::x() const {
    return px;
}

float Point::y() const {
    return py;
}

float Point::z() const {
    return pz;
}

float Point::nx() const {
    return nex;
}

float Point::ny() const {
    return ney;
}

float Point::nz() const {
    return nez;
}

float Point::r() const {
    float out = CURRENT_CURVE_R;
    if(pointCloud->nCurves() != (c + 1)) {
        out = colorComponentFromError(
            NOT_CURRENT_CURVE_R1, NOT_CURRENT_CURVE_R2, NOT_CURRENT_CURVE_R3
        );
    }
    return out;
}

float Point::g() const {
    float out = CURRENT_CURVE_G;
    if(pointCloud->nCurves() != (c + 1)) {
        out = colorComponentFromError(
            NOT_CURRENT_CURVE_G1, NOT_CURRENT_CURVE_G2, NOT_CURRENT_CURVE_G3
        );
    }
    return out;
}

float Point::b() const {
    float out = CURRENT_CURVE_B;
    if(pointCloud->nCurves() != (c + 1)) {
        out = colorComponentFromError(
            NOT_CURRENT_CURVE_B1, NOT_CURRENT_CURVE_B2, NOT_CURRENT_CURVE_B3
        );
    }
    return out;
}

int Point::index() const {
    return ind;
}

int Point::curve() const {
    return c;
}

int Point::curveIndex() const {
    return cI;
}

void Point::setTriangulationVertex(const Vertex_handle& vh_) {
    vh = vh_;
    vh->info() = this;
}

void Point::orient(Eigen::Vector3f& dest, const Eigen::Vector3f& ref) {
    if(ref.dot(dest) < 0) {
        dest = -dest;
    }
}

void Point::orient(TriangulationVector_3& dest, const TriangulationVector_3& ref) {
    if((ref * dest) < 0) {
        dest = -dest;
    }
}

void Point::setNormalEstimate(const TriangulationVector_3& ne, const bool& useTangent) {
    TriangulationVector_3 neOriented = ne;
    orient(neOriented, TriangulationVector_3(nhx, nhy, nhz));
    nex = neOriented.x();
    ney = neOriented.y();
    nez = neOriented.z();
    if(useTangent) {
        updateNormalFromTangent();
    }
}

void Point::updateNormal(void) {
    nex = 0.0f;
    ney = 0.0f;
    nez = 1.0f;
    switch(pointCloud->algorithm) {
    case ALGORITHM::NORMAL_HYPOTHESIS:
        nex = nhx;
        ney = nhy;
        nez = nhz;
        break;
    case ALGORITHM::DELAUNAY:
        calculateDelaunayNormal();
        break;
    case ALGORITHM::CURVE_DELAUNAY:
        calculateDelaunayNormal();
        updateNormalFromTangent();
        break;
    /* This normal estimation algorithm is run globally on the point cloud,
     * rather than incrementally.
     */
    case ALGORITHM::VCM:
    case ALGORITHM::CURVE_VCM:
    case ALGORITHM::PCA:
    case ALGORITHM::CURVE_PCA:
        break;
    default:
        throw Error("Unrecognized ALGORITHM constant.");
    }
}

void Point::calculateDelaunayNormal(void) {
    // Get all incident tetrahedra (cells)
    vector<Cell_handle> cells;
    pointCloud->triangulation.incident_cells(vh, std::back_inserter(cells));

    // Find the largest finite cell, or average the normal vector estimates from all infinite cells
    const Vector3f nh(nhx, nhy, nhz);
    Vector3f normal(0.0f, 0.0f, 0.0f);
    Vector3f normalSum(0.0f, 0.0f, 0.0f);
    float maxRadius = 0.0f;
    bool useInfiniteCells = false;
    Cell_handle c;
    Vertex_handle v;
    TriangulationPoint vertices[3];
    TriangulationPoint circumcenter(0,0,0);
    int nVertices = 0;
    vector<Cell_handle>::const_iterator cellsEnd = cells.cend();
    for(vector<Cell_handle>::const_iterator cellsIt = cells.cbegin(); cellsIt != cellsEnd; ++cellsIt) {
        c = *cellsIt;
        if(pointCloud->triangulation.is_infinite(c)) {
            if(!useInfiniteCells) {
                useInfiniteCells = true;
                normalSum = Vector3f(0.0f, 0.0f, 0.0f);
            }

            // Find the finite face of this cell
            nVertices = 0;
            for(int i = 0; i < N_VERTICES_IN_TETRAHEDRON; ++i) {
                v = c->vertex(i);
                if(!pointCloud->triangulation.is_infinite(v) && v != vh) {
                    vertices[nVertices] = v->point();
                    ++nVertices;
                }
            }
            planeOrthogonalVector(vertices[0], vertices[1], vertices[2], normal);
            orient(normal, nh);
            normalSum += normal;
        } else if(!useInfiniteCells) {
            circumcenter = pointCloud->triangulation.dual(c);
            normal = Vector3f(
                circumcenter.x() - px,
                circumcenter.y() - py,
                circumcenter.z() - pz
            );
            if(maxRadius < normal.norm()) {
                maxRadius = normal.norm();
                orient(normal, nh);
                normalSum += normal;
            }
        }
    }

    // Normalize
    normalSum.normalize();

    nex = normalSum(0);
    ney = normalSum(1);
    nez = normalSum(2);
}

void Point::updateNormalFromTangent(void) {
    TriangulationVector_3 t;
    if(calculateTangent(t)) {
        TriangulationVector_3 ne(nex, ney, nez);
        ne = ne - ((t * ne) * t); // Vector rejection
        normalize(ne);
        nex = ne.x();
        ney = ne.y();
        nez = ne.z();
    }
}

bool Point::calculateTangent(TriangulationVector_3& t) const {
    const float radius = pointCloud->averageSegmentLength() * TANGENT_RADIUS_FACTOR;
    // Collect neighbourhoods
    TriangulationVector_3 prevNextTangents[2];
    int prevNextNPoints[2];
    TriangulationVector_3 firstSegment(0, 0, 0);
    bool usePrevNextTangents[] = {false, false};
    vector<TriangulationPoint> points;
    points.emplace_back(px, py, pz);
    float length = 0.0f;
    int curveIndex = 0;
    const int curveSize = pointCloud->nPointsOnCurve(c);
    TriangulationLine_3 line;
    for(int i = 0; i < static_cast<int>(sizeof(prevNextTangents) / sizeof(prevNextTangents[0])); ++i) {
        points.resize(1);
        length = 0.0f;
        curveIndex = cI;
        while((i == 0 && curveIndex > 0) || (i != 0 && curveIndex < (curveSize - 1))) {
            if(i == 0) {
                --curveIndex;
            } else {
                ++curveIndex;
            }
            Point& neighbor = *(pointCloud->getPoint(c, curveIndex));
            TriangulationVector_3 segment(
                neighbor.px - px,
                neighbor.py - py,
                neighbor.pz - pz
            );
            length += norm(segment);
            if(points.size() == 1) {
                /* Assumes that noise is low enough so that the curve never
                 * doubles back on itself.
                 */
                firstSegment = (i == 0) ? (-segment) : segment;
            }
            if((points.size() == 1) || (length <= radius)) {
                points.emplace_back(neighbor.px, neighbor.py, neighbor.pz);
            } else {
                break;
            }
        }
        prevNextNPoints[i] = points.size();
        if(prevNextNPoints[i] > 1) {
            usePrevNextTangents[i] = true;
            linear_least_squares_fitting_3(points.cbegin(),points.cend(),line, CGAL::Dimension_tag<0>());
            prevNextTangents[i] = line.to_vector();
            orient(prevNextTangents[i], firstSegment);
            // Normalize
            normalize(prevNextTangents[i]);
        }
    }

    // Combine the estimates from each side of the point
    bool result = (usePrevNextTangents[0] || usePrevNextTangents[1]);
    if(usePrevNextTangents[0]) {
        if(usePrevNextTangents[1]) {
            // Dot product
            float weight = (prevNextTangents[0] * prevNextTangents[1]) / 2.0f;
            if(weight < 0.0f) {
                weight = 0.0f;
            }
            if(prevNextNPoints[0] < prevNextNPoints[1]) {
                t = (prevNextTangents[0] * weight)
                    + (prevNextTangents[1] * (1.0f - weight));
            } else {
                t = (prevNextTangents[0] * (1.0f - weight))
                    + (prevNextTangents[1] * weight);
            }
            normalize(t);
        } else {
            t = prevNextTangents[0];
        }
    } else {
        if(usePrevNextTangents[1]) {
            t = prevNextTangents[1];
        }
    }
    // Normalize
    if(!result) {
        t = TriangulationVector_3(0,0,0);
    }
    return result;
}

float Point::angleFromNormalHypothesis(void) const {
    Vector3f nh(nhx, nhy, nhz);
    Vector3f ne(nex, ney, nez);
    float dotProduct = nh.dot(ne);
    if(dotProduct >= 1.0f) {
        return 0;
    } else if(dotProduct <- 1.0f) {
        return PI;
    } else {
        return acos(dotProduct);
    }
}

float Point::colorComponentFromError(const float& good, const float& ok, const float& bad) const {
    float err = angleFromNormalHypothesis();
    float weight = 1.0f;
    float a = ok;
    float b = bad;
    if(err <= ANGLE_THRESHOLD_1) {
        weight = err / ANGLE_THRESHOLD_1;
        a = good;
        b = ok;
    } else if(err <= ANGLE_THRESHOLD_2) {
        weight = (err - ANGLE_THRESHOLD_1) / (ANGLE_THRESHOLD_2 - ANGLE_THRESHOLD_1);
    }
    return (a * (1.0f - weight)) + (b * weight);
}

void Point::planeOrthogonalVector(
    const TriangulationPoint& p1, const TriangulationPoint& p2, const TriangulationPoint& p3,
    Vector3f& n
) {
    Vector3f v1(p2.x() - p1.x(), p2.y() - p1.y(), p2.z() - p1.z());
    Vector3f v2(p3.x() - p1.x(), p3.y() - p1.y(), p3.z() - p1.z());
    n = v1.cross(v2);
}

void Point::normalize(TriangulationVector_3& t) {
    float length = sqrt(t.squared_length());
    t = TriangulationVector_3(t.x() / length, t.y() / length, t.z() / length);
}

float Point::norm(const TriangulationVector_3& t) {
    return sqrt(t.squared_length());
}
