#ifndef POINTCLOUD_H
#define POINTCLOUD_H

/*!
** \file pointcloud.h
** \brief Definition of the PointCloud class.
**
** ### About
** Created for: CMPUT 511 Project\n
** Fall 2017\n
** Bernard Llanos\n
** Department of Computing Science, University of Alberta
**
** ## References
** - 'mesh.h' created for Assignment 1
** - "logging - How to read a growing text file in C++", posted on Stack Overflow
**   (https://stackoverflow.com/questions/11757304/how-to-read-a-growing-text-file-in-c)
** - Clément Jamin and Sylvain Pion and Monique Teillaud. 3D Triangulations.
**   In CGAL User and Reference Manual. CGAL Editorial Board, 4.11 edition, 2017.
**   (http://doc.cgal.org/4.11/Manual/packages.html#PkgTriangulation3Summary)
** - Pierre Alliez and Simon Giraudot and Clément Jamin and Florent Lafarge and
**   Quentin Mérigot and Jocelyn Meyron and Laurent Saboret and Nader Salman and
**   Shihao Wu. Point Set Processing. In CGAL User and Reference Manual. CGAL
**   Editorial Board, 4.11 edition, 2017.
**   - For Voronoi Covariance Measure-based normal estimation
**
** ### Textbook Code
** From 'ExperimenterSource.zip' downloaded from the course textbook website
** (http://www.sumantaguha.com/)
** - 'ExperimenterSource/Chapter3/SquareAnnulus4/squareAnnulus4.cpp'
** - 'ExperimenterSource/Chapter3/SquareAnnulusVBO/squareAnnulusVBO.cpp'
*/

#include <GL/glew.h>
#include <string>
#include <vector>
#include <set>
#include <fstream>

#include "affinetransform.h"
#include "point.h"

/*!
 * \brief Point cloud geometry class.
 *
 * A point cloud composed of curves, with operations for rendering
 * the cloud, loading geometry from a file, and writing geometry to
 * a file. Data files are formatted as described in the README ('README.md')
 */
class PointCloud
{
public:

    /*!
     * \brief Create a point cloud from the geometry in a CSV file
     * Input files are formatted as described in the README ('README.md')
     *
     * Throws an exception if the file cannot be opened.
     *
     * \param [in] file The filepath of the geometry file to read.
     * \param [in] blockSize The size of vertex buffers to allocate on the graphics
     *   server
     * \param [in] pointIncrement The initial number of points to add per timestep
     */
    PointCloud(const std::string & file, const int blockSize, const int pointIncrement);

    virtual ~PointCloud(void);

    /*!
     * \brief Render the point cloud
     *
     * Draws the point cloud under the current OpenGL modelview transformation,
     * but with local scaling and translation from PointCloud::canonicalTransform.
     */
    void draw(void) const;

    /*!
     * \brief Update the point cloud
     *
     * Adds new points to the point cloud, if available from the data stream
     */
    void update(void);

    /*!
     * \brief Output point cloud to a file
     *
     * The output file is formatted as described in the README ('README.md')
     *
     * \param [in] file The filepath of the geometry file to write.
     */
    void writeData(const std::string & file) const;

    /*!
     * \brief Output point cloud normal estimation results to a CSV file, for all normal estimation methods
     *
     * The rows of the file correspond to the following:
     * - Curve indices
     * - Positions
     * - Tangent vectors
     * - One row for the estimated normal according to each normal estimation
     *   method in Point::listOfAlgorithms
     *
     * \param [in] file The filepath of the CSV file to write.
     */
    void writeComparison(const std::string & file);

    /*!
     * \brief Number of points in the point cloud
     *
     * \return The number of points in the point cloud
     */
    int nPoints(void) const;

    /*!
     * \brief Number of curves in the point cloud
     *
     * \return The number of curves in the point cloud
     */
    int nCurves(void) const;

    /*!
     * \brief Number of points along a given curve
     *
     * \param [in] c The index of the curve
     * \return The number of points along the curve with the given index
     */
    int nPointsOnCurve(const int c) const;

    /*!
     * \brief Set the number of points to add each update
     *
     * \param [in] n The number of points to add when update() is called
     */
    void setPointIncrement(const int n);

    /*!
     * \brief Increment the rendered size of points
     *
     * The point size will not be allowed to drop below one.
     *
     * \param [in] s The amount by which to increase PointCloud::pointSize
     */
    void incrementPointSize(const float s);

    /*!
     * \brief Increment the rendered width of lines
     *
     * The line width will not be allowed to drop below one.
     *
     * \param [in] w The amount by which to increase PointCloud::lineWidth
     */
    void incrementLineWidth(const float w);

    /*!
     * \brief Register a point as needing to be updated in graphics
     *  server memory
     *
     * \param [in] p The point to update, which must belong to this point cloud
     */
    void queuePointUpdate(const Point& p);

    /*!
     * \brief Register a curve as needing to be updated in graphics
     *  server memory
     *
     * \param [in] c The index of the curve to update
     */
    void queueCurveUpdate(const int c);

    /*!
     * \brief Retrieve a point
     *
     * \param [in] c The index of the curve containing the point
     * \param [in] ci The index of the point along the curve
     * \return The Point at the given index along the curve with the given index
     */
    Point* getPoint(const int c, const int ci) const;

    /*!
     * \brief Recalculate normal vectors according to the new algorithm
     *
     * All points are updated, unless there are too few points in the point cloud.
     *
     * \param [in] alg The algorithm to use for normal estimation, for existing
     *   points, and for points added in the future
     */
    void setNormalEstimationAlgorithm(const Point::ALGORITHM& alg);

    /*!
     * \brief Switch to the next normal estimation algorithm in the cycle of
     *   possible algorithms defined in the Point::ALGORITHM enumeration
     *
     * All points are updated, unless there are too few points in the point cloud.
     *
     * \param [in] direction The direction in which to cycle
     */
    void cycleNormalEstimationAlgorithm(const bool direction);

    /*!
     * \brief The average spacing between adjacent points on a curve
     *
     * \return The average spacing between adjacent points on a curve
     */
    float averageSegmentLength(void) const;

protected:
    /*! \brief Reads points from a file
     *
     * Reads from the start of the file, or from the current cursor position
     * reached by a previous call to this function, to the end of the file,
     * or until PointCloud::pointIncrement points have been read (whichever
     * comes first).
     *
     * As such, this function can be used to tail a growing file.
     *
     * Input vectors are appended to, not overwritten. They should be passed
     * in empty, or at least `v` and `n` must have equal sizes.
     *
     * \param [out] v The new vertex positions read from the file.
     * \param [out] n The new vertex normal hypotheses read from the file.
     * \param [out] ind The new within-curve point indices read from the file.
     */
    void readFile(std::vector<float>& v, std::vector<float>& n, std::vector<GLuint>& ind);
    /*!
     * \brief Update the centroid and scale of the point cloud
     *
     * \param [in] v New points
     */
    void preprocessPoints(const std::vector<float>& v);
    /*!
     * \brief Add points to the point cloud
     *
     * \param [in] v New point locations
     * \param [in] n New point normal hypotheses
     * \param [in] ind New indices of point locations within curves
     */
    void addPoints(const std::vector<float>& v, const std::vector<float>& n, const std::vector<GLuint>& ind);
    /*!
     * \brief Update all existing points in the point cloud, if the point cloud
     *   is of dimension 3
     */
    void updateExistingPoints(void);
    /*!
     * \brief Update all points with normals computed by the Voronoi Covariance
     *   Measure method
     *
     * \param [in] useTangent If `true`, adjust the normal vector estimate based
     *   on the tangent to the curve.
     */
    void calculateVCMNormals(const bool& useTangent);
    /*!
     * \brief Update all points with normals computed by principal components
     *   analysis
     *
     * \param [in] useTangent If `true`, adjust the normal vector estimate based
     *   on the tangent to the curve.
     */
    void calculatePCANormals(const bool& useTangent);
    /*!
     * \brief Update graphics server memory with new or changed points
     *
     * Updates member variables keeping track of buffer data,
     * and delegates work to addBlock() and overwriteBlock().
     */
    void updateBlocks(void);
    /*!
     * \brief Mark all buffers as needing to be updated
     */
    void markAllBlocksForUpdate(void);
    /*!
     * \brief Make new buffers, and a new vertex array object
     *
     * The input data is expected to be padded with zeros to fill the buffers.
     *
     * Updates member variables keeping track of buffer data.
     *
     * \param [in] v Initial vertex position data for the vertex buffer
     * \param [in] ind Initial data for the index buffer. Note that these are not
     *   indices of points within curves, but indices of vertices in vertex buffers.
     * \param [in] nVertices Number of initial vertices
     * \param [in] nIndices Number of initial indices
     */
    void addBlock(const std::vector<float>& v, const std::vector<GLuint>& ind, const int nVertices, const int nIndices);
    /*!
     * \brief Overwrite an existing pair of vertex and index buffers
     *
     * The input data is expected to fill the buffers, or `nNewIndices` can be zero, in which case the index buffer will not be updated.
     *
     * Updates member variables keeping track of buffer data.
     *
     * \param [in] block Index of the buffer (in PointCloud::vertexBuffers, for example)
     * \param [in] v Additional vertex position data for the vertex buffer
     * \param [in] ind Additional data for the index buffer
     * \param [in] nNewVertices Number of new vertices (if any)
     * \param [in] nNewIndices Number of new indices (if any)
     */
    void overwriteBlock(const int block, const std::vector<float>& v, const std::vector<GLuint>& ind, const int nNewVertices, const int nNewIndices);

    /*!
     * \brief Calculate indices in OpenGL vertex buffers corresponding to a
     *   vertex ID
     *
     * \param [out] n Number of OpenGL vertex buffer indices
     * \param [out] blocks Indices into PointCloud::vertexBuffers, for example,
     *   of the buffers containing the point
     * \param [out] internals Indices of the point within the buffers
     * \param [in] ind Global index of the point
     */
    void bufferIndex(int& n, int blocks[2], int internals[2], const int ind) const;

    /*!
     * \brief Obtain an iterator to the first point in a block
     *
     * \param [in] block Index of the block
     * \return An iterator to the point in PointCloud::points at the start of the block
     */
    std::vector<Point*>::const_iterator beginBlock(const int block) const;

    /*!
     * \brief Obtain an iterator to the last point in a block
     *
     * Note: Contrast with STL container class end() functions, which return
     * iterators to positions past the last elements of containers.
     *
     * \param [in] block Index of the block
     * \return An iterator to the point in PointCloud::points at the end of the block
     */
    std::vector<Point*>::const_iterator beforeEndBlock(const int block) const;

protected:
    /*!
     * \brief OpenGL vertex buffer size
     *
     * Units are vertices, not bytes, nor words.
     */
    const GLint blockSize;

    /*!
     * \brief Maximum number of points to add each timestep
     */
    int pointIncrement;

    /*!
     * \brief The size argument passed to glPointSize()
     */
    float pointSize;

    /*!
     * \brief The width argument passed to glLineWidth()
     */
    float lineWidth;

    /*!
     * \brief Vertex buffer IDs
     */
    std::vector<GLuint> vertexBuffers;

    /*!
     * \brief Index buffer IDs
     */
    std::vector<GLuint> indexBuffers;

    /*!
     * \brief Vertex array object IDs
     */
    std::vector<GLuint> vertexArrayObjects;

    /*!
     * \brief Point cloud input data stream
     */
    std::ifstream inputStream;

    /*!
     * \brief The points in the point cloud
     */
    std::vector<Point*> points;
    /*!
     * \brief The curves in the point cloud
     */
    std::vector<std::vector<Point*>> curves;
    /*!
     * \brief Buffer indices of the points that need to be updated in graphics server memory
     */
    std::set<int> staleBlocks;

    /*!
     * \brief Counts of vertices in the vertex buffers,
     *   for point-wise rendering
     */
    std::vector<GLsizei> verticesCounts;

    /*!
     * \brief Counts of indices in the index buffers,
     *   for curve-wise rendering
     */
    std::vector<GLsizei> indicesCounts;

    /*!
     * \brief The offset into a vertex buffer corresponding to the start
     *   of colour data
     */
    int colorOffset;

    /*!
     * \brief The offset into a vertex buffer corresponding to the start
     *   of normal vector data
     */
    int normalOffset;

    /*!
     * \brief Point cloud normalization transformation
     *
     * The point cloud is always scaled so that it spans #MAX_COORDINATE_DIAMETER
     * in any dimension, and positioned such that its centroid is the origin
     * of its local coordinate system.
     */
    AffineTransform canonicalTransform;

    /*!
     * \brief Sum of point coordinates, for computing the point cloud centroid
     *
     * Relative to the local coordinate directions, before any rotation.
     */
    float centroidSum[3];

    /*!
     * \brief Minimum of point coordinates (component-wise)
     *
     * Used to compute point cloud scaling
     */
    float minPoints[3];

    /*!
     * \brief Maximum of point coordinates (component-wise)
     *
     * Used to compute point cloud scaling
     */
    float maxPoints[3];

    /*!
     * \brief Sum of distances between adjacent points on curves
     *
     * Used for Voronoi Covariance Measure-based normal estimation.
     */
    float sumSegments;
    /*!
     * \brief Number of curve segments, corresponding to the number of lengths
     *   added to produce PointCloud::sumSegments
     *
     * Used for Voronoi Covariance Measure-based normal estimation.
     */
    int nSegments;

    /*!
     * \brief The last curve index encountered in the input file
     *
     * Used to determine when to start a new curve
     */
    int lastCurveIndex;

public:
    /*!
     * \brief A 3D Delaunay triangulation of the point cloud
     */
    Triangulation triangulation;

    /*!
     * \brief The algorithm used to estimate normal vectors at points
     *
     * The best way to set this data member is through
     * setNormalEstimationAlgorithm(), which will ensure that the change is
     * applied to existing points.
     */
    Point::ALGORITHM algorithm;

protected:
    /*!
     * \brief Count the number of vertices in a vector storing
     * floating point coordinate values.
     *
     * \param [in] v Vertex data
     */
    static int countVertices(const std::vector<float>& v);

    /*!
     * \brief The number of floating point values corresponding to a number of vertices
     *
     * \param [in] n Number of vertices
     */
    static int countCoordinates(const int n);

    // Currently not implemented - will cause linker errors if called
private:
    PointCloud(const PointCloud& other);
    PointCloud& operator=(const PointCloud& other);
};

#endif
