function m_o = Variable_Mass(t)
% variable mass
global m_o

m_o = 0.5*(0.5<=t & t<1.5) + 2*(1.5<=t & t<3) + 4*(3<=t & t<4.5)...
    + 3*(4.5<=t & t<6) + 1.5*(6<=t & t<7.5) + 0.5*(7.5<=t & t<9.0);

end