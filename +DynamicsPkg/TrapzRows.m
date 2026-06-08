function [Integral] = TrapzRows(X, Y)
%
% [Integral] = TrapzRows(X, Y)
%
% Row-wise trapezoid integration for panel-specific station grids.
%

Integral = sum(0.5 * (X(:, 2:end) - X(:, 1:end - 1)) .* ...
    (Y(:, 1:end - 1) + Y(:, 2:end)), 2);

end
