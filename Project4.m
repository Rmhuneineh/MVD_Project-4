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

% BRAKING IN ACTUAL CONDITIONS
% Calculating Kb
Kb = (b - ux(2)*hG)/(l + ux(2)*hG - b);

% Valve Limit Condition
Fx2A = ux(2)*M*g*(a + hG*ux(2))/l;
Fx1A = Kb*Fx2A;
Fx2Ap = 0.9*Fx2A;
Fx1Ap = Kb*Fx2Ap;
Fx1B = ux(5)*M*g*(b - hG*ux(5))/l;
Fx2B = ux(5)*M*g*(a + hG*ux(5))/l;

% Calculating Kb'
Kbp = (Fx1B - Fx1Ap)/(Fx2B - Fx2Ap);

% Calculating Braking Efficiency
yIntercept = Fx2Ap - Fx1Ap/Kbp;

firstIncrement = Fx1Ap/30;
newFx1(1:31) = 0:firstIncrement:Fx1Ap;
newFx2(1:31) = newFx1(1:31)/Kb;

secondIncrement = (Fx1B - Fx1Ap)/70;
newFx1(32:101) = (Fx1Ap+secondIncrement):secondIncrement:Fx1B;
newFx2(32:101) = newFx1(32:101)/Kbp + yIntercept;

figure; hold on;
plot(pNFx1, pNFx2, 'o-');
plot(abs(newFx1(1:31)), abs(newFx2(1:31)), '--');
plot(abs(newFx1(32:101)), abs(newFx2(32:101)), '--');


for i = 1:101
    newUx(i) = l/((M*g*b/newFx1(i)) - hG*(1 + newFx2(i)/newFx1(i)));
end

actualAcceleration = (newFx1 + newFx2)/M;
idealAcceleration = newUx*g;

brakingEfficiency = actualAcceleration./idealAcceleration;

figure; hold on;
plot(abs(newUx), brakingEfficiency);
title('Braking Efficiency vs Longitudinal Slip');
xlabel('Longitudinal Slip [-]');
ylabel('Braking Efficiency [-]');
grid on;
xlim([0 1.2]);
ylim([0.4 1.2]);