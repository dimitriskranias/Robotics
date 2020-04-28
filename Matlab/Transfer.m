function [ T ] = Transfer(a,b,c,d)% D-H Homogeneous Transformation Matrix (a alpha d theta) 
    %a = Rot(x)
    %b = Tra(x)
    %c = d = Tra(z)
    %d = theta =  Rot(z)
T = [cos(d) -sin(d) 0 b;
    sin(d)*round(cos(a)) cos(d)*round(cos(a)) -sin(a) -sin(a)*c;
    sin(d)*sin(a) cos(d)*sin(a) round(cos(a)) round(cos(a))*c;
    0 0 0 1];
end
