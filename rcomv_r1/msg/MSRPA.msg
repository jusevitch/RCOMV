# type : = circular, square
#
# trajectory : =
#		(circular) [t0, xc, yc, Rad, wd, phi0]
#		(square)   [t0, xc, yc, Leng, psi, v, start_L, fillet]
#
# formation : =
# 		(circular) [Rf, n]

string type
float64[] trajectory
float64[] formation
