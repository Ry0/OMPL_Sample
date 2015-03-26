#set terminal postscript eps color enhanced 20
#set output "out.eps"
set xrange [-1:1]
set yrange [-1:1]

set xlabel "x"
set ylabel "y"

set key outside
set key top right

set size square

plot "obstacle.dat" using 1:2 with filledcurves lt rgb "#ff0033" fill solid 0.5 notitle,\
'path.dat' using 1:2 with lines lt 1 lc rgb "#191970" lw 2 title 'Path'