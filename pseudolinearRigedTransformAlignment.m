clear all 
close all 
  
IA = imread('book_A.jpg'); 
IB = imread('book_B.jpg'); 
figure, imshow(IA,[]); 
figure, imshow(IB,[]); 
  
%pseudo linear rigid transform alignment
% Using imtool, we find corresponding 
% points (x;y), which are the four  
% corners of the book 
pA = [ 
    213 398  401  223; 
    29   20  293  297]; 
pB = [ 
    207 391  339  164; 
    7    34  302  270]; 
N = size(pA,2); 
 
A = zeros(2*N,4); 
for i=1:N 
    A( 2*(i-1)+1, :) = [ pA(1,i) -pA(2,i)  1  0]; 
    A( 2*(i-1)+2, :) = [ pA(2,i)  pA(1,i)  0  1]; 
end 
b = reshape(pB, [], 1); 
  
x = A\b; 
  
theta = acos(x(1)); 
tx = x(3); 
ty = x(4); 

disp('Parameters (theta; tx; ty):'), disp(theta), disp(tx), disp(ty); 