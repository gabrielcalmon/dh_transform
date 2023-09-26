clear all;

# Receives a line of DH parameters and return the equivalent homogeneus transform
# Angles should be passed in degrees
function T = tansformationmatrix_deg(alfa, ai, di, theta)
    T = [[cosd(theta) -sind(theta) 0 ai]; 
        [sind(theta)*cosd(alfa) cosd(theta)*cosd(alfa) -sind(alfa) -sind(alfa)*di];
        [sind(theta)*sind(alfa) cosd(theta)*sind(alfa) cosd(alfa) cosd(alfa)*di];
        [0 0 0 1]];
endfunction

T01 = tansformationmatrix_deg(0, 0, 0, 0);
T12 = tansformationmatrix_deg(0.1, 0, 0, 0);
T23 = tansformationmatrix_deg(0.2, 0, 0, 0);
T34 = tansformationmatrix_deg(0.3, 0, 0, 0);

# Return the end effector resulting homogeneus transform
T04 = T01*T12*T23*T34