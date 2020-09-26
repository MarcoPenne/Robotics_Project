clear all
clc

syms q1 q2 q3 L1 L2 L3 d2 dq1 dq2 dq3 a b c d m1 m2 m3 I1xx I2xx I3xx I1yy I2yy I3yy I1zz I2zz I3zz g0 ddq1 ddq2 ddq3 real

q = [q1;q2;q3];
dq = [dq1;dq2;dq3];

rc1 = [0;-a;0];
rc2 = [-b;0;-c];
rc3 = [-d;0;0];

Ic1 = [I1xx 0 0; 0 I1yy 0; 0 0 I1zz];
Ic2 = [I2xx 0 0; 0 I2yy 0; 0 0 I2zz];
Ic3 = [I3xx 0 0; 0 I3yy 0; 0 0 I3zz];

% Direct kinematics

alpha = sym(pi/2);
d = L1;
a = 0;
theta = q1;

A01 = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
0 sin(alpha) cos(alpha) d;
0 0 0 1];

R01 = [A01(1:3,1:3)];
p01 = [A01(1:3,4)];

alpha = 0;
d = -d2;
a = L2;
theta = q2;

A12 = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
0 sin(alpha) cos(alpha) d;
0 0 0 1];

R12 = [A12(1:3,1:3)];
p12 = [A12(1:3,4)];

alpha = 0;
d = 0;
a = L3;
theta = q3;

A23 = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
0 sin(alpha) cos(alpha) d;
0 0 0 1];

R23 = [A23(1:3,1:3)];
p23 = [A23(1:3,4)];

A = simplify(A01*A12*A23);

p = A*[0;0;0;1];

p = [p(1:3,1)]

% Jacobian

J = simplify(jacobian(p,q))

% Moving frames algorithm

w00 = [0;0;0];
v00 = [0;0;0];

z = [0;0;1];

w11 = simplify(R01'*(w00+dq1*z));
v11 = simplify(R01'*v00+cross(w11,R01'*p01));
vc1 = simplify(v11+cross(w11,rc1));

w22 = simplify(R12'*[w11+dq2*z]);
v22 = simplify(R12'*v11+cross(w22,R12'*p12));
vc2 = simplify(v22+cross(w22,rc2));

w33 = simplify(R23'*[w22+dq3*z]);
v33 = simplify(R23'*v22+cross(w33,R23'*p23));
vc3 = simplify(v33+cross(w33,rc3));

% Kinetic energies

T1 = simplify((1/2)*m1*(norm(vc1))^2+(1/2)*w11'*Ic1*w11);
T2 = simplify((1/2)*m2*(norm(vc2))^2+(1/2)*w22'*Ic2*w22);
T3 = simplify((1/2)*m3*(norm(vc3))^2+(1/2)*w33'*Ic3*w33);

T = simplify(T1+T2+T3)

% Potential energies

A2 = A01*A12;

p02 = [A2(1:3,4)];

r0c1 = R01*[rc1+R01'*p01];
r0c2 = R01*R12*[rc2+R12'*R01'*p02];
r0c3 = R01*R12*R23*[rc3+R23'*R12'*R01'*p];

U1 = simplify(-m1*[0;0;-g0]'*r0c1);

U2 = simplify(-m2*[0;0;-g0]'*r0c2);

U3 = simplify(-m3*[0;0;-g0]'*r0c3);

U = simplify(U1+U2+U3)

% Gravity term

g = simplify(jacobian(U,q)')

% Automatic Inertia Matrix
syms B [3 3] real

for i=1:3
    for j=i:3
        if i==j
            B(i,i)=diff(T,dq(i),2);
        else
            Temp = diff(T,dq(i));
            B(i,j)=diff(Temp,dq(j));
            B(j,i)=B(i,j);
        end
    end
end

B = simplify(B);
B

B1 = [B(1:3,1)];
B2 = [B(1:3,2)];
B3 = [B(1:3,3)];

% Coriolis and centrifugal terms
 
C1 = simplify(1/2 * [jacobian(B1,q)+(jacobian(B1,q))'-diff(B,q1)]);

C2 = simplify(1/2 * [jacobian(B2,q)+(jacobian(B2,q))'-diff(B,q2)]);

C3 = simplify(1/2 * [jacobian(B3,q)+(jacobian(B3,q))'-diff(B,q3)]);

c1 = simplify(dq'*C1*dq);
c2 = simplify(dq'*C2*dq);
c3 = simplify(dq'*C3*dq);

cc = [c1;c2;c3]

% Dynamic model

ddq = [ddq1;ddq2;ddq3];

u = simplify(B*ddq+cc+g)