%camera calibration undistortion approximation
%approximation of radial undistortion function

clear
close all

%distortion coefficiants, the other coefficients can be ignored
k1 = -0.4377;
k2 = 0.2563;
%for simplicity we only calculate on the x-axis where y equals zero
%the values and datarange have to be authentic. the approximation only
%works on the chosen 
x = 1:318; %image width
cc = 158.5;
%we take mean value of normalized focal length
fc = ( 323.70584 + 324.26201 ) / 2;
r_dist = zeros(size(x)); % r distorted
r_n = zeros(size(x));    % r normalized coordinates

for m=1 : length(x)
    %normalized image coordinates (undistorted)
    r_n(m) = (x(m)-cc) / fc;
    %calculate y = f(x): this adds distortion to normalized image
    %coordinates
    %nomalized image coordinates distorted
    r_dist(m) = r_n(m) + k1*r_n(m)^3 + k2*r_n(m)^5;
end

%functionplot
plot(r_n,r_dist);
hold on 
%plot angle bisector
plot(r_n,r_n, ':k');
%plot of inversed function, this is the function we are searching for
plot(r_dist,r_n, 'color', 'r');
axis equal
grid on

%approximate xn by polynomial curve fitting
%this function maps a distorted value to an undistorted
p = polyfit(r_dist,r_n,3);
r_undist = zeros(size(x)); %undistorted radius
%interpolate x by polynomia approximation
%attention: this formular only works on the range, that it has been
%approximated on!
for m=1 : length(x)
    r_undist(m) = p(1)*r_dist(m)^3 + p(2)*r_dist(m)^2 + p(3)*r_dist(m) + p(4);
end
%we are getting a straight line which fits the angle bisector, this is what
%we wanted
plot(r_n, r_undist, 'color', 'g');

%distort a point in normalized coordinates
x=0.2 
y = 0.4
phi = atan2(y,x);
%radius in normalized image
r_n = sqrt(x^2+y^2);
%distorted radius
r_dist = r_n + k1*r_n^3 + k2*r_n^5;
%undistort image with approximated formula
r_undist = p(1)*r_dist^3 + p(2)*r_dist^2 + p(3)*r_dist + p(4);
%-> following calculations are simply wrong
%x_undist_t = p(1)*x^3 + p(2)*x^2 + p(3)*x + p(4);
%y_undist_t = p(1)*y^3 + p(2)*y^2 + p(3)*y + p(4);
x_undist = r_undist * cos(phi)
y_undist = r_undist * sin(phi)














