set terminal png large size 1400,1000

set grid 
set title "Velocity Trajectory"
set xlabel "time (x10ms)"
set ylabel "meter"
set output "trapezium.png"
set autoscale
#set xrange [1:2]
plot "trapezium.csv" usi 5 ti "yaw reff" w l
