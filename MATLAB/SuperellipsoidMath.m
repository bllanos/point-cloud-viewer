syms phi beta n1 n2
r = sym('r', [1 3]);
x = [
    r(1) .* (cos(phi) .^ n1) .* (cos(beta) .^ n2),...
    r(2) .* (cos(phi) .^ n1) .* (sin(beta) .^ n2),...
    r(3) .* (sin(phi) .^ n1)...
];

T_phi = diff(x, phi);
T_beta = diff(x, beta);
N = cross(T_phi, T_beta);
N_length = norm(N);
n = N / N_length;