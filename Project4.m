clear all;
clc;

% IDEAL BRAKING
% General Constants
g = 9.81; %[m/s^2]
alpha = 0;
a = 1.15; %[m]
l = 2.3; %[m]
b = l-a; %[m]
hG = .5; %[m]
ux(1:6, 1) = -0.2:-0.2:-1.2;
acceleration = -.2*g:-.2*g:-1.2*g; %[m/s^2]

% Standard B Constants
m_passenger = 75; %[kg]
m_baggage = 10; %[kg]
m_car = 970; %[kg]
M = m_car + m_baggage + m_passenger; %[kg]

% Force Calculations
Fz1 = M/l*(g*b*cos(alpha) - g*hG*sin(alpha) - hG*acceleration);
Fz2 = M/l*(g*a*cos(alpha) + g*hG*sin(alpha) + hG*acceleration);

Fx1 = ux*Fz1;
Fx2(1:6, 1:6,  1) = ux*Fz2;
for i = 1:6
    pNFx1 (i) = -Fx1(i, i);
   for j = 1:6
      Fx2(j, i, 2) = M*g*b*cos(alpha)/hG - Fx1(j, i)*(l/ux(j)/hG +1);
      Fx2(j, i, 3) = ux(j)*M*g*a*cos(alpha)/(l-ux(j)*hG) + Fx1(j, i)*(ux(j)*hG/(l-ux(j)*hG));
      Fx2(j, i, 4) = M*g*sin(alpha) + M*acceleration(j) - Fx1(j, i);
   end
   pNFx2(i) = -Fx2(i, i);
end

NFx1 = -Fx1;
NFx2 = -Fx2;

figure; hold on;
plot(pNFx1, pNFx2, 'o-');
for i = 1:6
    plot(NFx1(i, 1:6), NFx2(i, 1:6, 2), '*-');
    plot(NFx1(1:6, i), NFx2(i, 1:6, 3), '+-');
    plot(pNFx1, NFx2(1:6, i, 2), 'x-');
end
grid on;