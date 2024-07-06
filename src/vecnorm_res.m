function vecnorm=vecnorm_res(V)
if((V(1)>0 || V(1)<0) && (V(2)>0 || V(2)<0) && (V(3)>0 || V(3)<0))
    vecnorm= V/norm(V);
else
    vecnorm= V;
end