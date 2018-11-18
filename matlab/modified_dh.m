function T = modified_dh(thet, a , d, alpha)
    T = [cos(thet), -sin(thet), 0, a; 
         sin(thet)*cos(alpha), cos(thet)*cos(alpha), -sin(alpha), -sin(alpha)*d;
         sin(thet)*sin(alpha), cos(thet)*sin(alpha), cos(alpha), cos(alpha)*d;
         0, 0, 0, 1];
end