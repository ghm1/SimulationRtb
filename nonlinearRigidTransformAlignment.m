clear all 
close all 

% code from william hoff
IA = imread('book_A.jpg'); 
IB = imread('book_B.jpg'); 
figure, imshow(IA,[]); 
figure, imshow(IB,[]); 
  
% Using imtool, we find corresponding 
% points (x;y), which are the four  
% corners of the book 
pA = [ 
    213 398  401  223; 
    29   20  293  297; 
    1   1   1   1]; 
pB = [ 
    207 391  339  164; 
    7    34  302  270]; 
N = size(pA,2); 
 
theta = 0.2083; %0; 
tx = 4.9087; %5.0; 
ty = -64.3735; %-64; 

x = [theta; tx; ty];    % initial guess

while true 
    disp('Parameters (theta; tx; ty):'), disp(x); 
    
    %x ist die aktuell beste Schätzung für die Transformationsparameter.
    %Diese transformation wird auf die punkte aus bild A (pA) angewandt,
    %woraus sich ein neues y ergibt.
    y = fRigid(x, pA); % Call function to compute expected measurements 
    %Das neue y ist das Ergebnis unserer aktuell besten schätzung der
    %Transformationsparameter.
    %Die Genauigkeit der Schätzung wird überprüft, indem die Differenz zu
    %unseren Zielpunkten (pB) ermittelt wird. Diese Diffenzen nennt man
    %Residuen. Die Residuen werden später als Schrittweite verwendet, mit
    %welcher der Gradientenabstieg bzw. Gradientenaufstiege vollzogen wird.
    dy = reshape(pB,[],1) - y;      % new residual 
  
    %Initalisierung der Jakobimatrix mit lauter nuller.
    J = zeros(2*N,3); 
   
    % 1. manual estimation of J
    %theta = x(1);
    %for i=1:N
    %    J( 2*(i-1)+1, :) = [ -sin(theta)*pA(1,i)-cos(theta)*pA(2,i) 1 0 ];
    %    J( 2*(i-1)+2, :) = [ cos(theta)*pA(1,i)-sin(theta)*pA(2,i) 0 1];
    %end
    
    % 2. Estimate J numerically 
    e = 1e-6;   % A tiny number 
    %Ableitungen der Transformationsfunktion an den Stellen der aktuell besten Schätung der
    %Transformationsparameter durchführen
    J(:,1) = (fRigid(x+[e;0;0],pA) - y)/e; %erste Spalte: Ableitungen nach theta
    J(:,2) = (fRigid(x+[0;e;0],pA) - y)/e; %zweite Spalte: Ableitungen nach x
    J(:,3) = (fRigid(x+[0;0;e],pA) - y)/e; %dritte Spalte: Ableitungen nach y
    
    %apply pseudo inverse of J to residuals
    %Berechnung von 
    dx = pinv(J)*dy;
    %warum nicht inverse?
     
    % Stop if parameters are no longer changing 
    if abs( norm(dx)/norm(x) ) < 1e-6 
        break; 
    end 
  
    x = x + dx;    % add correction 
end 

%visualize result
% Apply transform to image 
theta = x(1); 
tx = x(2); 
ty = x(3); 
  
A = [cos(theta)  -sin(theta) tx;  
     sin(theta)  cos(theta) ty; 
     0 0 1]; 
T = maketform('affine', A'); 
  
I2 = imtransform(IA,T, ... 
    'XData', [1 size(IA,2)], ... 
    'YData', [1 size(IA,1)]  ); 
figure, imshow(I2,[]); 
