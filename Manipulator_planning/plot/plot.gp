set mouse
set view equal xyz
set ticslevel 0
set xrange[-5:5]
set yrange[-5:5]
set zrange[0:10]
splot "test.dat" using 1:2:3 with lines,\
"frame_all.dat" w lp lt 3 pt 6 lw 1.5
pause 1
reread