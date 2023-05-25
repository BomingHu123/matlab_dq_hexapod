include_namespace_dq
r =-0.866+-0.4997*i_-0.00783*j_-0.0006739*k_;
r= normalize(r);
p = -0.275*i_ + 0.325*j_ + 0.15*k_;
a = r + E_*0.5*p*r;

r1 =cos(0) + k_*sin(0);
p1 = -0.275*i_ + 0.325*j_ + 0.15*k_;
b = r1 + E_*0.5*p1*r1;

c = a*b;
norm(a)
is_unit(a)