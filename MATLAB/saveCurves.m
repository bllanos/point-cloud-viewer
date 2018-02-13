function saveCurves( name, x, n, t )
% SAVECURVES  Output points along curves to a CSV file.
%
% ## Syntax
% saveCurves( name, x, n, t )
%
% ## Description
% saveCurves( name, x, n, t )
%   Output the points, and corresponding normal and tangent vectors, to a file.
%
% ## Input Arguments
%
% name -- Filename
%   The full name and path of the output file to generate.
%
% x -- Positions
%   A cell column vector, where the elements are 3-column matrices.
%   `x{k}(i, :)` is the 3D position of the i-th sample point in the k-th
%   curve.
%
% n -- Normals
%   A cell column vector of the same length as `x`, where the elements are
%   3-column matrices. `n{k}(i, :)` is the 3D normal vector at the
%   position `x{k}(i, :)`. Therefore, `n{k}` and `x{k}` must have the same
%   number of rows.
%
% t -- Tangents
%   Analogous to `n`, but contains 3D unit tangent vectors at the curve
%   points.
%
% ## File Output
%
% Overwrites the file referred to by `name` to produce a CSV file. The format
% of the CSV file is that of output files of './pointCloudViewer', described
% in the README (../README.md).
%
% The positions in the file are from `x`, the normal vectors are from `n` (i.e.
% they are ground truth normal vectors), and the tangent vectors are from `t`.
% The first column of the CSV file contains the indices (zero-based) of the
% cells (starting from zero) in `x`, `n`, and `t`, whereas the second column
% contains the indices (zero-based) of values within the cells.

% Created for: CMPUT 511 Project
% Fall 2017
% Bernard Llanos
% Department of Computing Science, University of Alberta

nargoutchk(0, 0);
narginchk(4, 4);

% Avoid using the built-in 'csvwrite()' function, which seems to output
% platform-dependent line endings.
fileID = fopen(name,'w');
for c = 1:length(x)
    x_c = x{c};
    n_c = n{c};
    t_c = t{c};
    for p = 1:size(x_c, 1)
        fprintf(...
            fileID, '%d, %d, %g, %g, %g, %g, %g, %g, %g, %g, %g\n',...
            c-1, p-1,...
            x_c(p, 1), x_c(p, 2), x_c(p, 3),...
            n_c(p, 1), n_c(p, 2), n_c(p, 3),...
            t_c(p, 1), t_c(p, 2), t_c(p, 3)...
        );
    end
end
fclose(fileID);

end
