function thetas = rotToEuler(rot_mat,conv)

r = rot_mat;


%% Refactoring
% constants
PI = pi;
%[r00 r01 r02
% r10 r11 r12
% r20  r21 r22]
% row1
r00 = r(1,1);
r01 = r(1,2);
r02 = r(1,3);
%row2
r10 = r(2,1);
r11 = r(2,2);
r12 = r(2,3);
% row3
r20 = r(3,1);
r21 = r(3,2);
r22 = r(3,3);

switch conv
    %% XYZ
    case 'XYZ'
        if (r02 < +1)
            if (r02 > -1)
                thetaY = asin(r02);
                thetaX = atan2(-r12,r22);
                thetaZ = atan2(-r01,r00);
            else  % r02 =-1 %Not a unique solution:thetaZ - thetaX= atan2(r10,r11)
                thetaY = -PI/2;
                thetaX = -atan2(r10,r11);
                thetaZ = 0;
            end
        else  % r02 =+1 %Not a unique solution:thetaZ + thetaX= atan2(r10,r11)
            thetaY = +PI/2;
            thetaX = atan2(r10,r11);
            thetaZ = 0;
        end
    thetas = [thetaX, thetaY, thetaZ];
    
    %% XZY    
    case 'XZY'       
        if (r01 < +1)
            if (r01 > -1)
                thetaZ = asin(-r01);
                thetaX = atan2(r21,r11);
                thetaY = atan2(r02,r00);
            else % r01 =-1 %Not a unique solution:thetaY - thetaX= atan2(-r20,r22)
                thetaZ = +PI/2;
                thetaX = -atan2(-r20,r22);
                thetaY = 0;
            end
        else % r01 =+1 %Not a unique solution:thetaY - thetaX= atan2(r20,r22)
            thetaZ = -PI/2;
            thetaX = atan2(-r20,r22);
            thetaY = 0;
        end
    thetas = [thetaX, thetaY, thetaZ];
    
    %% YXZ
    case 'YXZ'
        if (r12 < +1)
            if (r12 > -1)
                thetaX = asin(-r12);
                thetaY = atan2(r02,r22);
                thetaZ = atan2(r10,r11);
            else %r12=-1  %Not a unique solution:thetaZ - thetaY= atan2(-r01,r00)
                thetaX = +PI/2;
                thetaY = -atan2(-r01,r00);
                thetaZ = 0;
            end
        else %r12=+1  %Not a unique solution:thetaZ + thetaY= atan2(-r01,r00)
            thetaX = -PI/2;
            thetaY = atan2(-r01,r00);
            thetaZ = 0;
        end
    thetas = [thetaX, thetaY, thetaZ];
       
    %% YZX
    case 'YZX'
        if (r10 < +1)
            if (r10 > -1)
                thetaZ = asin(r10);
                thetaY = atan2(-r20,r00);
                thetaX = atan2(-r12,r11);
            else % r10=-1 %Not a unique solution:thetaX - thetaY= atan2(r21,r22)
                thetaZ = -PI/2;
                thetaY = -atan2(r21,r22);
                thetaX = 0;
            end
        else % r10=+1 %Not a unique solution:thetaX + thetaY= atan2(r21,r22)
            thetaZ = +PI/2;
            thetaY = atan2(r21,r22);
            thetaX = 0;
        end
    thetas = [thetaX, thetaY, thetaZ];
       
    %% ZXY
    case 'ZXY'
        if (r21 < +1)
            if (r21 > -1)
                thetaX = asin(r21);
                thetaZ = atan2(-r01,r11);
                thetaY = atan2(-r20,r22);
            else %r21=-1 % %Not a unique solution:thetaY - thetaZ= atan2(r02,r00)
                thetaX = -PI/2;
                thetaZ = -atan2(r02,r00);
                thetaY = 0;
            end
        else  %r21=+1 % %Not a unique solution:thetaY + thetaZ= atan2(r02,r00)
            thetaX = +PI/2;
            thetaZ = atan2(r02,r00);
            thetaY = 0;
        end
    thetas = [thetaX, thetaY, thetaZ];
               
    %% ZYX
    case 'ZYX'
        if (r20 < +1)
            if (r20 > -1)
                thetaY = asin(-r20);
                thetaZ = atan2(r10,r00);
                thetaX = atan2(r21,r22);
            else %r20 =-1 Not a unique solution:thetaX - thetaZ= atan2(-r12,r11)
                thetaY = +PI/2;
                thetaZ = -atan2(-r12,r11);
                thetaX = 0;
            end
        else %r20 =+1 Not a unique solution:thetaX + thetaZ= atan2(-r12,r11)
            thetaY = -PI/2;
            thetaZ = atan2(-r12,r11);
            thetaX = 0;
        end
    thetas = [thetaX, thetaY, thetaZ];
               
    %% X0YX1
    case 'XYX'
        if (r00 < +1)
            if (r00 > -1)
                thetaY = acos(r00);
                thetaX0 = atan2(r10,-r20);
                thetaX1 = atan2(r01,r02);
            else %r00 =-1 Not a unique solution:thetaX1 - thetaX0= atan2(-r12,r11)
                thetaY = +PI;
                thetaX0 = -atan2(-r12,r11);
                thetaX1 = 0;
            end
        else %r00 =+1 Not a unique solution:thetaX1 + thetaX0= atan2(-r12,r11)
            thetaY = 0;
            thetaX0 = atan2(-r12,r11);
            thetaX1 = 0;
        end
    thetas = [thetaX0, thetaY, thetaX1];
               
    %% X0ZX1
    case 'XZX'
        if (r00 < +1)
            if (r00 > -1)
                thetaZ = acos(r00);
                thetaX0 = atan2(r20,r10);
                thetaX1 = atan2(r02,-r01);
            else % r00=-1 ; Not a unique soln thetaX1 - thetaX0= atan2(r21,r22)
                thetaZ = +PI;
                thetaX0 = -atan2(r21,r22);
                thetaX1 = 0;
            end
        else % r00=+1 ; Not a unique soln thetaX1 + thetaX0= atan2(r21,r22)
            thetaZ = 0;
            thetaX0 = atan2(r21,r22);
            thetaX1 = 0;
        end
    thetas = [thetaX0, thetaZ, thetaX1];
               
    %% Y0XY1
    case 'YXY'
        if (r11 < +1)
            if (r11 > -1)
                thetaX = acos(r11);
                thetaY0 = atan2(r01,r21);
                thetaY1 = atan2(r10,-r12);
            else % r11=-1 ; Not a unique soln thetaY1 - thetaY0= atan2(r02,r00)
                thetaX = +PI;
                thetaY0 = -atan2(r02,r00);
                thetaY1 = 0;
            end
        else % r11=+1 ;Not a unique soln thetaY1 + thetaY0= atan2(r02,r00)
            thetaX = 0;
            thetaY0 = atan2(r02,r00);
            thetaY1 = 0;
        end
        
    thetas = [thetaY0, thetaX, thetaY1];
               
    %% Y0ZY1
    case 'YZY'
        if (r11 < +1)
            if (r11 > -1)
                thetaZ = acos(r11);
                thetaY0 = atan2(r21,-r01);
                thetaY1 = atan2(r12,r10);
            else % r11=-1 ; Not a unique soln thetaY1 - thetaY0= atan2(-r20,r22)
                thetaZ = +PI;
                thetaY0 = -atan2(-r20,r22);
                thetaY1 = 0;
            end
        else % r11=+1 ; Not a unique soln thetaY1 + thetaY0= atan2(-r20,r22)
            thetaZ = 0;
            thetaY0 = atan2(-r20,r22);
            thetaY1 = 0;
        end
        
    thetas = [thetaY0, thetaZ, thetaY1];
               
    %% Z0XZ1
    case 'ZXZ'
        if (r22 < +1)
            if (r22 > -1)
                thetaX = acos(r22);
                thetaZ0 = atan2(r02,-r12);
                thetaZ1 = atan2(r20,r21);
            else % r22=-1 ; Not a unique soln thetaZ1 - thetaZ0= atan2(-r01,r00)
                thetaX = +PI;
                thetaZ0 = -atan2(-r01,r00);
                thetaZ1 = 0;
            end
        else % r22=+1 ; Not a unique soln thetaZ1 + thetaZ0= atan2(-r01,r00)
            thetaX = 0;
            thetaZ0 = atan2(-r01,r00);
            thetaZ1 = 0;
        end
    thetas = [thetaZ0, thetaX, thetaZ1];
               
    %% Z0YZ1
    case 'ZYZ'
        if (r22 < +1)
            if (r22 > -1)
                thetaY = acos(r22);
                thetaZ0 = atan2(r12,r02);
                thetaZ1 = atan2(r21,-r20);
            else  % r22=-1 ; Not a unique soln thetaZ1 - thetaZ0= atan2(r10,r11)
                thetaY = +PI;
                thetaZ0 = -atan2(r10,r11);
                thetaZ1 = 0;
            end
        else % r22=+1 ; Not a unique soln thetaZ1 + thetaZ0= atan2(r10,r11)
            thetaY = 0;
            thetaZ0 = atan2(r10,r11);
            thetaZ1 = 0;
        end
    thetas = [thetaZ0, thetaY, thetaZ1];
               
    otherwise
        warning('Kindly select one of the 12 possible conventions of rotations.')
end 

end


        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
