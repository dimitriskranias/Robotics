syms q1 q2 q3 q4 q5 q6 q7
syms l1 l2 l3 l4 l5

syms x y z w

A1 = Transfer(0,0,l1,t1);
A2 = Transfer(-pi/2,0,0,t2);
A3 = Transfer(pi/2,0,l2,t3);
A4 = Transfer(pi/2,l3,0,t4); 
A5 = Transfer(pi/2,x,y,t5);
A6 = Transfer(pi/2,0,0,t6);
A7 = Transfer(-pi/2,z,w,t7);

T2= simplify(A1*A2); 
T3= simplify(T2*A3); 
T4= simplify(T3*A4); 
T5= simplify(T4*A5); 
T6= simplify(T5*A6);
T7= simplify(T6*A7);

pex= T7(1,4);
pey= T7(2,4);
pez= T7(3,4);

J = simplify(jacobian([pex,pey,pez],[t1,t2,t3,t4,t5,t6,t7]));
