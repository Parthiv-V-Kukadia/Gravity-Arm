load('EOMs.mat');
M = symEOM.M;
C = symEOM.C;
N = symEOM.N;

tau = symEOM.tau;

syms q1 q2 v1 v2 tau1 real;

q=[q1;q2];
qdot=[v1;v2];
variables = [q;qdot;tau1];
qddot=M^(-1)*(-C*qdot-N+tau);

%state matrix
m = [q;qdot];
mdot = [qdot;qddot];

%Input
n = tau1;

%Output
o = [q2];


%{
finding equilibrium values
q_e = [1.8;0.63];
q1_e = q_e(1);
q2_e = q_e(2);
v1_e = 0;
v2_e = 0;
m_e = [q1_e; q2_e; v1_e; v2_e];
m_estr = mat2str(m_e);
%}


Qddot= subs(qddot,[q2,v1,v2],[.63,0,0]);


EoM_function = matlabFunction(Qddot,'vars',{variables});
var_guess = [1.8;0.63;0;0;0];
var_sol = fsolve(EoM_function,var_guess);
var_equilibrium = var_sol;


Asym = jacobian(mdot,m);
Bsym = jacobian(mdot,n);
Csym = jacobian(o,m);
Dsym = jacobian(o,n);

%finding Torque
n_e = double(subs((M*[0;0]+C*qdot+N),[m],[m_e]));
o_estr = mat2str(q2_e);


A = double(subs(Asym,[m;n],[var_equilibrium]));
B = double(subs(Bsym,[m;n],[var_equilibrium]));
C = double(subs(Csym,[m;n],[var_equilibrium]));
D = double(subs(Dsym,[m;n],[var_equilibrium]));


%Observer Design
Qo = [1.80];
Ro = [4.10 -0.35 0.00 -0.05; -0.35 3.90 -0.10 -0.05; 0.00 -0.10 3.80 -0.15; -0.05 -0.05 -0.15 4.00];

L = lqr(A',C',Ro^(-1),Qo^(-1))';

%Controller Design
Qc = [4.20 -0.60 0.05 -0.65; -0.60 4.90 0.35 0.35; 0.05 0.35 3.50 -0.90; -0.65 0.35 -0.90 3.80];
Rc = [0.20];

K = lqr(A,B,Qc,Rc);
Kref= -1/(C*inv(A-B*K)*B);

%Analysis of Time delay
syms s tau
Td = (2-s*tau)/(2+s*tau);
G = K*inv(s*eye(size(A))-(A-B*K-L*C))*L;
H = C*inv(s*eye(size(A))-A)*B;
F = (-K*inv(s*eye(size(A))-(A-B*K-L*C))*B*Kref+Kref)/G;

T = Td*H*G*F/(1+Td*H*G);
X = simplify(T);

[n,d] = numden(X);
tmax = 0;

while (roots(sym2poly(subs(d,tau,tmax)))<0)
    tmax = tmax +0.0001;
end
fprintf('\nTau_max = %f\n', tmax)

