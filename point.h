#ifndef POINT_H
#define POINT_H

/*!
** \file point.h
** \brief Definition of the Point class.
**
** ### About
** Created for: CMPUT 511 Project\n
** Fall 2017\n
** Bernard Llanos\n
** Department of Computing Science, University of Alberta
**
** ## References
** - Clément Jamin and Sylvain Pion and Monique Teillaud. 3D Triangulations.
**   In CGAL User and Reference Manual. CGAL Editorial Board, 4.11 edition, 2017.
**   (http://doc.cgal.org/4.11/Manual/packages.html#PkgTriangulation3Summary)
** - Dey, T. K., & Goswami, S. (2006). Provable surface reconstruction from noisy
**   samples. Computational Geometry: Theory and Applications, 35(1-2 SPEC. ISS.),
**   124-141. doi:10.1016/j.comgeo.2005.10.006
** - Pierre Alliez and Simon Giraudot and Clément Jamin and Florent Lafarge and
**   Quentin Mérigot and Jocelyn Meyron and Laurent Saboret and Nader Salman and
**   Shihao Wu. Point Set Processing. In CGAL User and Reference Manual. CGAL
**   Editorial Board, 4.11 edition, 2017.
**   - For Voronoi Covariance Measure-based normal estimation
** - Pierre Alliez and Sylvain Pion and Ankit Gupta. Principal Component Analysis.
**   In CGAL User and Reference Manual. CGAL Editorial Board, 4.11 edition, 2017.
*/

#include <Eigen/Core>
#include <vector>
#include <fstream>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>

class Point;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_with_info_3<Point*, K> Vb;
typedef CGAL::Triangulation_data_structure_3<Vb> Tds;
typedef CGAL::Delaunay_triangulation_3<K, Tds, CGAL::Fast_location> Triangulation;
typedef Triangulation::Cell_handle Cell_handle;
typedef Triangulation::Vertex_handle Vertex_handle;
typedef Triangulation::Triangulation_data_structure::Edge Edge;
typedef Triangulation::Cell_iterator Cell_iterator;
typedef Triangulation::Point TriangulationPoint;
typedef Triangulation::Geom_traits Geom_traits;
typedef Geom_traits::Vector_3 TriangulationVector_3;
typedef Geom_traits::Line_3 TriangulationLine_3;

class PointCloud;

/*!
 * \brief Point class.
 *
 * A point contained in a point cloud composed of curves.
 */
class Point
{
public:
    /*!
     * \brief Normal estimation method
     *
     * Values:
     * - NORMAL_HYPOTHESIS: Use the normal hypothesis as the estimated normal
     * - DELAUNAY: Use the direction to the centre of the largest Delaunay ball
     *   touching the point. The method by Dey & Goswami (2006).
     * - VCM: Voronoi Covariance Measure-based normal estimation, implemented in CGAL
     * - PCA: Principal components analysis-based normal estimation, implemented in CGAL
     * - CURVE_*: Runs the ALGORITHM::* method, then constrains the normal
     *   vector to the plane perpendicular to the estimated curve tangent.
     */
    enum class ALGORITHM : unsigned int {
        NORMAL_HYPOTHESIS, DELAUNAY, CURVE_DELAUNAY, VCM, CURVE_VCM, PCA, CURVE_PCA
    };

    /*!
     * \brief The number of constants in Point::ALGORITHM
     */
    static const int nAlgorithms;
    /*!
     * \brief The constants in Point::ALGORITHM
     */
    static const ALGORITHM listOfAlgorithms[];

public:

    /*!
     * \brief Create a point
     *
     * Call update() afterwards to initialize the point.
     *
     * \param [in] x The x-coordinate of the point
     * \param [in] y The y-coordinate of the point
     * \param [in] z The z-coordinate of the point
     * \param [in] nx The x-coordinate of the point's normal hypothesis
     * \param [in] ny The y-coordinate of the point's normal hypothesis
     * \param [in] nz The z-coordinate of the point's normal hypothesis
     * \param [in] ind The global index of the point in the point cloud
     * \param [in] curve The index of the curve containing the point
     * \param [in] curveIndex The index of the point along the curve
     * \param [in] pointCloud The point cloud to which this point belongs, and
     *   to which it will send updates for rendering
     */
    Point(
        const float x, const float y, const float z,
        const float nx, const float ny, const float nz,
        const int ind,
        const int curve, const int curveIndex,
        PointCloud* pointCloud
    );

    virtual ~Point(void) {}

    /*!
     * \brief Update the point
     *
     * Refreshes all data members of the point with the latest contextual
     * information from the environment. Also used as an initialization routine.
     */
    void update(void);

    /*!
     * \brief Output a description of the point to a file
     *
     * \param [in] file The file to write to.
     */
    void writeData(std::ofstream& file) const;

    /*!
     * \brief Output the position of the point
     *
     * \param [out] v The position of the point in model coordinates
     */
    void position(Eigen::Vector3f& v) const;

    /*!
     * \brief Output the position of the point
     *
     * \param [out] v The list of point coordinate to which the coordinates
     *   of this point will be appended
     */
    void position(std::vector<float>& v) const;

    /*!
     * \brief Output the x-coordinate of the point
     *
     * \return The x-coordinate of the point
     */
    float x() const;

    /*!
     * \brief Output the y-coordinate of the point
     *
     * \return The y-coordinate of the point
     */
    float y() const;

    /*!
     * \brief Output the z-coordinate of the point
     *
     * \return The z-coordinate of the point
     */
    float z() const;

    /*!
     * \brief Output the x-coordinate of the point's estimated normal vector
     *
     * \return The x-coordinate of the point's normal vector
     */
    float nx() const;

    /*!
     * \brief Output the y-coordinate of the point's estimated normal vector
     *
     * \return The y-coordinate of the point's normal vector
     */
    float ny() const;

    /*!
     * \brief Output the z-coordinate of the point's estimated normal vector
     *
     * \return The z-coordinate of the point's normal vector
     */
    float nz() const;

    /*!
     * \brief Output the Red component of the point's colour
     *
     * \return The Red component of the point's colour
     */
    float r() const;

    /*!
     * \brief Output the Green component of the point's colour
     *
     * \return The Green component of the point's colour
     */
    float g() const;

    /*!
     * \brief Output the Blue component of the point's colour
     *
     * \return The Blue component of the point's colour
     */
    float b() const;

    /*!
     * \brief Output the global index of the point in the point cloud
     *
     * \return The global index of the point in the point cloud
     */
    int index() const;

    /*!
     * \brief Output the index of the curve containing the point
     *
     * \return The index of the curve containing the point
     */
    int curve() const;

    /*!
     * \brief Output the index of the point along the curve
     *
     * \return The index of the point along the curve
     */
    int curveIndex() const;

    /*!
     * \brief Set the handle to the point's corresponding vertex in the Delaunay
     *   triangulation owned by Point::pointcloud
     *
     * \param [in] vh The new vertex handle
     */
    void setTriangulationVertex(const Vertex_handle& vh);

    /*!
     * \brief Set the estimated normal vector of the point
     *
     * The estimated normal vector will be reversed if it opposes the point's
     * normal hypothesis.
     *
     * \param [in] ne The new normal vector estimate
     * \param [in] useTangent If `true`, adjust the normal vector estimate based
     *   on the tangent to the curve.
     */
    void setNormalEstimate(const TriangulationVector_3& ne, const bool& useTangent);

    /*!
     * \brief Estimate the tangent to the curve at this point
     *
     * \param [out] t Unit tangent vector, or a zero vector, if the tangent
     *   cannot be estimated.
     * \return `true`, if a tangent can be estimated at this point
     */
    bool calculateTangent(TriangulationVector_3& t) const;

    /*!
     * \brief Calculate the normal vector of a plane through three points
     *
     * \param [in] p1 The first point
     * \param [in] p2 The second point
     * \param [in] p3 The third point
     * \param [out] n A vector orthogonal to the plane (not normalized)
     */
    static void planeOrthogonalVector(
        const TriangulationPoint& p1, const TriangulationPoint& p2, const TriangulationPoint& p3,
        Eigen::Vector3f& n
    );

    /*!
     * \brief Normalize a vector
     *
     * CGAL vectors do not have a normalization function.
     *
     * \param [in] v The vector to normalize
     */
    static void normalize(TriangulationVector_3& v);

    /*!
     * \brief Norm of a vector
     *
     * CGAL vectors only have a squared length function
     *
     * \param [in] v The vector whose length is to be calculated
     * \return The length of the vector
     */
    static float norm(const TriangulationVector_3& v);

    /*!
     * \brief Reverse a vector if it opposes a reference vector
     *
     * \param [in] ref The reference oriented vector
     * \param [in,out] dest The vector to reverse if its dot product with `ref` is negative
     */
    static void orient(TriangulationVector_3& dest, const TriangulationVector_3& ref);
    /*!
     * \brief Reverse a vector if it opposes a reference vector
     *
     * \param [in] ref The reference oriented vector
     * \param [in,out] dest The vector to reverse if its dot product with `ref` is negative
     */
    static void orient(Eigen::Vector3f& dest, const Eigen::Vector3f& ref);

protected:
    /*!
     * \brief Update the normal vectors of the point and its neighbours
     */
    void updateNormal(void);

    /*!
     * \brief The ALGORITHM::DELAUNAY algorithm for normal estimation
     */
    void calculateDelaunayNormal(void);

    /*!
     * \brief Set the estimated normal vector to its projection on the estimated
     * plane perpendicular to the curve tangent.
     */
    void updateNormalFromTangent(void);

    /*!
     * \brief Calculate the angle between the normal estimate and the normal hypothesis
     *
     * \return The angle in radians
     */
    float angleFromNormalHypothesis(void) const;

    /*!
     * \brief Interpolate colour values based on normal estimation error
     *
     * \param [in] good The colour of points with correct normal vectors
     * \param [in] ok The colour of points with normal estimation error at the first threshold
     * \param [in] bad The colour of points with normal estimation error at or above the second threshold
     * \return The appropriate interpolation of `good`, `ok`, and `bad`
     */
    float colorComponentFromError(const float& good, const float& ok, const float& bad) const;

protected:
    /*!
     * \brief x-coordinate of the point
     */
    const float px;
    /*!
     * \brief y-coordinate of the point
     */
    const float py;
    /*!
     * \brief z-coordinate of the point
     */
    const float pz;

    /*!
     * \brief x-coordinate of the point's normal vector estimate
     */
    float nex;
    /*!
     * \brief y-coordinate of the point's normal vector estimate
     */
    float ney;
    /*!
     * \brief z-coordinate of the point's normal vector estimate
     */
    float nez;

    /*!
     * \brief x-coordinate of the point's normal vector hypothesis
     */
    const float nhx;
    /*!
     * \brief y-coordinate of the point's normal vector hypothesis
     */
    const float nhy;
    /*!
     * \brief z-coordinate of the point's normal vector hypothesis
     */
    const float nhz;

    /*!
     * \brief Output the global index of the point in the point cloud
     */
    const int ind;
    /*!
     * \brief The index of the curve containing the point
     */
    const int c;
    /*!
     * \brief The index of the point along the curve
     */
    const int cI;

    /*!
     * \brief The point cloud containing this point
     */
    PointCloud* const pointCloud;

    /*!
     * \brief The handle to the point's corresponding vertex in the Delaunay
     *   triangulation owned by Point::pointcloud
     */
    Vertex_handle vh;

    // Currently not implemented - will cause linker errors if called
private:
    Point(const Point& other);
    Point& operator=(const Point& other);
};

#endif
