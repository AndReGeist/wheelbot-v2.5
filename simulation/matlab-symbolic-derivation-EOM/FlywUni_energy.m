function [TE, PE] = dbpend_energy(t,z, TE, PE, ...
        Mw, Mc, Mp, Iwx, Iwy, Iwz, Icx, Icy, Icz, Ipx, Ipy, Ipz, ...
        Rw, Lc, Lcp, Lp, g)

q2 = z(1);                          
q2_d = z(2);                          
q4 = z(3);                         
q4_d = z(4);  

PE
