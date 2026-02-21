function lambda = wrapTo2Pi(theta)
%WRAPTO2PI Wrap angles in radians to the interval [0, 2*pi].
% Local fallback implementation to avoid Mapping Toolbox dependency.

lambda = mod(theta, 2*pi);
lambda((lambda == 0) & (theta > 0)) = 2*pi;

end
