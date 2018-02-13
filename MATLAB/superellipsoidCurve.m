function [ x, n, t ] = superellipsoidCurve( r, n1, n2, phi, beta, theta, len, count )
% SUPERELLIPSOIDCURVE  Sample a superellipsoid along a line in parameter space
%
% ## Syntax
% x = superellipsoidCurve( r, n1, n2, phi, beta, theta, len, count )
% [ x, n ] = superellipsoidCurve( r, n1, n2, phi, beta, theta, len, count )
% [ x, n, t ] = superellipsoidCurve( r, n1, n2, phi, beta, theta, len, count )
%
% ## Description
% x = superellipsoidCurve( r, n1, n2, phi, beta, theta, len, count )
%   Returns positions on a curve on the surface of the superellipsoid
% [ x, n ] = superellipsoidCurve( r, n1, n2, phi, beta, theta, len, count )
%   Additionally returns normal vectors at the surface positions
% [ x, n, t ] = superellipsoidCurve( r, n1, n2, phi, beta, theta, len, count )
%   Additionally returns tangent vectors at the surface positions
%
% ## Input Arguments
%
% r -- Superellipsoid radii
%   A three-element vector giving the radii of the superellipsoid in the x,
%   y, and z dimensions, respectively.
%
% n1 -- First exponent
%   The exponent applied to trigonometric functions of 'phi' in the
%   parametric form of the superellipsoid. A scalar.
%
% n2 -- Second exponent
%   The exponent applied to trigonometric functions of 'beta' in the
%   parametric form of the superellipsoid. A scalar.
%
% phi -- Origin of the line, first coordinate
%   The 'phi' coordinate in parameter space of the origin of the curve
%   along which to sample positions. 'phi' must be in the range [-pi/2,pi/2].
%
% beta -- Origin of the line, second coordinate
%   The 'beta' coordinate in parameter space of the origin of the curve
%   along which to sample positions. 'beta' must be in the range [-pi,pi].
%
% theta -- Angle of the line
%   The angle of the line in parameter space, measured counterclockwise
%   from the positive 'phi' axis.
%
% len -- Length of the line
%   The length of the line in parameter space, expressed in the range (0,
%   1], where 1 corresponds to a length of `2 * pi`.
%
% count -- Number of points
%   The number of points to sample along the curve.
%
% ## Output Arguments
%
% x -- Positions
%   An count x 3 array, where `x(i, :)` is the 3D position of the i-th sample
%   point on the surface of the superellipsoid.
%
% n -- Normals
%   An count x 3 array, where `n(i, :)` is the 3D normal vector at the position
%   `x(i, :)`.
%
% t -- Tangents
%   An count x 3 array, where `t(i, :)` is the 3D unit tangent vector at
%   the position `x(i, :)`.
%
% ## Notes
% - Points are sampled uniformly according to arc length.
% - The endpoints of the curve are always included.
%
% ## References
% - Paul Bourke's webpage, "Superellipse and Superellipsoid: A Geometric
%   Primitive for Computer Aided Design" (January 1990), at
%   http://paulbourke.net/geometry/superellipse/

% Created for: CMPUT 511 Project
% Fall 2017
% Bernard Llanos
% Department of Computing Science, University of Alberta

nargoutchk(1, 3);
narginchk(8, 8);

% To sample points uniformly by arc length, I will assign probability
% values to points based on the speed of the curve at each point.
% Consequently, I need to find the range of speeds along the curve.
% I will oversample, to find the range of speeds, then subsample the
% points.
sampling_factor = 10;
super_count = count * sampling_factor;

% Find the endpoints of the line in parameter space
half_len = len / 2;
p = [ phiToU(phi), betaToV(beta) ];
d = [ cos(theta), sin(theta) ];
d_len = sqrt(dot(d, d));
d = (d ./ d_len) * half_len;
% The factor of 2 accounts for the difference in scaling between `phi` and
% `u` compared to between `beta` and `v`.
d(1) = 2 * d(1);
p0 = p - d;
p1 = p + d;

% Sample points on the surface
super_u = linspace(p0(1), p1(1), super_count).';
super_v = linspace(p0(2), p1(2), super_count).';
[super_phi, super_beta] = uvToPhiBeta(super_u, super_v);

[ super_x, super_n, super_T_phi, super_T_beta ] = superellipsoid( super_phi, super_beta, r, n1, n2 );
super_tangents = cos(theta) * super_T_phi + sin(theta) * super_T_beta;
super_speeds = sqrt(dot(super_tangents, super_tangents, 2));

% Sample points at random, accounting for arc length differentials.
% Always include the endpoints
probabilities = super_speeds(2:(end-1));
filter = weightedSampling(probabilities, count - 2);
x = [
    super_x(1, :)
    super_x(filter, :)
    super_x(end, :)
    ];

if nargout > 1
        n = [
        super_n(1, :)
        super_n(filter, :)
        super_n(end, :)
        ];
    if nargout > 2
        t = [
        super_tangents(1, :)
        super_tangents(filter, :)
        super_tangents(end, :)
        ];
    end
end

end
