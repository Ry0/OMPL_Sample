  #set terminal png size 800, 640 transparent                     
  #set terminal svg size 1200 780 fname 'Trebuchet MS' fsize 24   
  set xlabel 'x'         
  set ylabel 'y'         
  set zlabel 'z'         
  set hidden3d           
  set ticslevel 0        
  set size 0.7,1         
  set parametric         
  set urange [0:6.28]    
  set vrange [0:6.28]    
  set isosample 8,8      
  set samples 10         
  r= 0.5
splot \
  r*cos(u)*cos(v)+1.11365,r*sin(u)*cos(v)+3.71618,r*sin(v)+0.534873 w l lt 1 lw 0.2 t '',\
  r*cos(u)*cos(v)+3.83248,r*sin(u)*cos(v)+0.139475,r*sin(v)+3.62885 w l lt 1 lw 0.2 t '',\
  r*cos(u)*cos(v)+3.26724,r*sin(u)*cos(v)+0.142988,r*sin(v)+0.353984 w l lt 1 lw 0.2 t '',\
  r*cos(u)*cos(v)+5.09824,r*sin(u)*cos(v)+3.40307,r*sin(v)+2.9519 w l lt 1 lw 0.2 t '',\
  r*cos(u)*cos(v)+3.9338,r*sin(u)*cos(v)+4.86998,r*sin(v)+1.04574 w l lt 1 lw 0.2 t '',\
'frame_all.dat' w lp lt 3 pt 6 lw 1.5
