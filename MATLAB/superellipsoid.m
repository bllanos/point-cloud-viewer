function [ x, n, T_phi, T_beta, A ] = superellipsoid( phi, beta, r, n1, n2 )
% SUPERELLIPSOID  Evaluate parametric equations for a superellipsoid
%
% ## Syntax
% x = superellipsoid( phi, beta, r, n1, n2 )
% [ x, n ] = superellipsoid( phi, beta, r, n1, n2 )
% [ x, n, T_phi ] = superellipsoid( phi, beta, r, n1, n2 )
% [ x, n, T_phi, T_beta ] = superellipsoid( phi, beta, r, n1, n2 )
% [ x, n, T_phi, T_beta, A ] = superellipsoid( phi, beta, r, n1, n2 )
%
% ## Description
% x = superellipsoid( phi, beta, r, n1, n2 )
%   Returns positions on the surface of the superellipsoid
% [ x, n ] = superellipsoid( phi, beta, r, n1, n2 )
%   Additionally returns normal vectors at the surface positions
% [ x, n, T_phi ] = superellipsoid( phi, beta, r, n1, n2 )
%   Additionally returns tangent vectors along the 'phi' direction
% [ x, n, T_phi, T_beta ] = superellipsoid( phi, beta, r, n1, n2 )
%   Additionally returns tangent vectors along the 'beta' direction
% [ x, n, T_phi, T_beta, A ] = superellipsoid( phi, beta, r, n1, n2 )
%   Additionally returns the magnitude of `cross(T_phi, T_beta)`
%
% ## Input Arguments
%
% phi -- First surface parameter
%   A column vector of length n, where 'n' is the number of points to
%   sample on the superellipsoid. 'phi' must be in the range [-pi/2,pi/2].
%   As 'phi' varies, with 'beta' constant, the point moves along a meridian
%   of the superellipsoid.
%
% beta -- Second surface parameter
%   A column vector of length n, where 'n' is the number of points to
%   sample on the superellipsoid. 'beta' must be in the range [-pi,pi]. As
%   'beta' varies, with 'phi' constant, the point moves along a level
%   curve in the xy-plane.
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
% ## Output Arguments
%
% x -- Positions
%   An n x 3 array, where `x(i, :)` is the 3D position of the i-th sample
%   point on the surface of the superellipsoid.
%
% n -- Normals
%   An n x 3 array, where `n(i, :)` is the 3D normal vector at the position
%   `x(i, :)`. Normal vectors are normalized.
%
% T_phi -- First tangent vectors
%   An n x 3 array, where `T_phi(i, :)` is the 3D tangent vector at the
%   position `x(i, :)` corresponding to a differential change in the 'phi'
%   parameter.
%
% T_beta -- Second tangent vectors
%   An n x 3 array, where `T_beta(i, :)` is the 3D tangent vector at the
%   position `x(i, :)` corresponding to a differential change in the 'beta'
%   parameter.
%
% A -- Area differential
%   An n x 1 vector, where `A(i)` is the surface area differential
%   `cross(T_phi(i, :), T_beta(i, :))` at the position `x(i, :)`.
%
% ## References
% - Paul Bourke's webpage, "Superellipse and Superellipsoid: A Geometric
%   Primitive for Computer Aided Design" (January 1990), at
%   http://paulbourke.net/geometry/superellipse/

% Created for: CMPUT 511 Project
% Fall 2017
% Bernard Llanos
% Department of Computing Science, University of Alberta

nargoutchk(1, 5);
narginchk(5, 5);

    function b = pow(a, p)
        s = sign(a);
        b = s .* (abs(a) .^ p);
    end

x = [
    r(1) .* pow(cos(phi), n1) .* pow(cos(beta), n2),...
    r(2) .* pow(cos(phi), n1) .* pow(sin(beta), n2),...
    r(3) .* pow(sin(phi), n1)...
];

if nargout > 1
    n = [
        (1/r(1)) .* pow(cos(phi), (2-n1)) .* pow(cos(beta), (2-n2)),...
        (1/r(2)) .* pow(cos(phi), (2-n1)) .* pow(sin(beta), (2-n2)),...
        (1/r(3)) .* pow(sin(phi), (2-n1))...
    ];
    n_length = sqrt(dot(n, n, 2));
    n = n ./ repmat(n_length, 1, size(n, 2));
end

if nargout > 2
    T_phi = [
        -n1*r(1)*pow(cos(beta),n2) .* pow(cos(phi),(n1 - 1)) .* sin(phi),...
        -n1*r(2)*pow(cos(phi),(n1 - 1)) .* pow(sin(beta),n2) .* sin(phi),...
        n1*r(3)*cos(phi) .* pow(sin(abs(phi)),(n1 - 1))...
    ];
end

if nargout > 3
    T_beta = [
        -n2*r(1)*pow(abs(cos(beta)),(n2 - 1)) .* pow(cos(phi),n1) .* sin(beta),...
        n2*r(2)*cos(beta).*pow(cos(phi),n1) .* pow(abs(sin(beta)),(n2 - 1)),...
        zeros(length(beta), 1) ...
    ];
end

if nargout > 4
    A = cross(T_phi, T_beta, 2);
end

end
