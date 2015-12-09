function y = fRigid(x,pIn) 
% Do 2D rigid transform, on a set of input points 
  
% Get params 
theta = x(1); 
tx = x(2); 
ty = x(3); 
  
H = [ cos(theta)  -sin(theta) tx; 
      sin(theta)   cos(theta) ty; 
      0  0  1]; 
  
pOut = H*pIn; 
pOut = pOut(1:2, :);  % 1st two rows 

y = reshape(pOut,[],1); 
return 