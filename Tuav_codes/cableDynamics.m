% cableDynamics.m
function dxdt = cableDynamics(~, x, n, k, d, m)
dxdt = zeros(2*n, 1);
for i = 1:n
    if i == 1
        % First mass (fixed at one end)
        dxdt(2*i-1) = x(2*i);
        dxdt(2*i) = 0;
    elseif i == n
        % Last mass (connected to the quadcopter)
        dxdt(2*i-1) = x(2*i);
        dxdt(2*i) = 0;
    else
        % Intermediate masses
        dxdt(2*i-1) = x(2*i);
        dxdt(2*i) = (-2*k*x(2*i-1) - d*x(2*i) + k*(x(2*i+1) + x(2*i-3))) / m;
    end
end
end
