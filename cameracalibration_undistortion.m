%camera calibration undistortion
clear
close all

k1=-0.4377;
k2=0.2563;
x = 0:320;
cc = 160;
fc = ( 323.7 + 324.3 ) / 2;
y = zeros(size(x));
xn = zeros(size(x));
for m=1 : length(x)
    %normalized image coordinates
    xn(m) = (x(m)-cc) / fc;
    %calculate y = f(x): this adds distortion to normalized image
    %coordinates
    y(m) = xn(m) + k1*xn(m)^3 + k2*xn(m)^5;
end

%functionplot
plot(xn,y);
hold on 
%plot of inversed function, this is the function we are searching for
plot(y,xn, 'color', 'r');
axis equal
grid on

%approximate xn by polynomial curve fitting
%this function maps a dis
k = polyfit(y,xn,3);
ynew = zeros(size(x));
%interpolate x by polynomapproximation
for m=1 : length(x)
    ynew(m) = k(1)*xn(m)^3 + k(2)*xn(m)^2 + k(3)*xn(m) + k(4);
end

plot(xn, ynew, 'color', 'g');