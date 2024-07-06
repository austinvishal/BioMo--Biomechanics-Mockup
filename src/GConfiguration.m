function [ shld, elbow ] = GConfiguration( rconf )
%RCONF Summary of this function goes here
%   Detailed explanation goes here

shld= 1;
elbow = 1;
% wrist = 1; % note we dont' have wrist here

if(bitand(rconf,1))
   shld = -1; 
end

if(bitand(rconf,2))
   elbow = -1; 
end

% if(bitand(rconf,4))
%    wrist = -1; 
% end


end

