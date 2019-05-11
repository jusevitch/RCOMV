# type : = circular, square
#
# trajectory : =
#		(circular) [t0, xc, yc, Rad, wd, phi0]
#		(square)   [t0, xc, yc, Leng, psi, v, start_L]
#
# formation : =
# 		(circular) [Rf, n]
# smooth_transition: = boolean (determines whether the trajectory starts at a given phi0, or if phi0 is determined by the current position of agents)

string type
float64[] trajectory
float64[] formation
bool smooth_transition