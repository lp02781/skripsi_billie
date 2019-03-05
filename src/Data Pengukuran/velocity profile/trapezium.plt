set terminal png large size 1400,1000

set grid 
set title "Velocity Profile"
set xlabel "time (x10ms)"
set ylabel "position(m)"
set output "position.png"
set autoscale
#set xrange [1:2]
plot "trapezium.csv" usi 1 ti "position reff" w l, "" usi 2 ti "current position" w l

set grid 
set title "Velocity Profile"
set xlabel "time (x10ms)"
set ylabel "velocity(m/s)"
set output "velprof.png"
set autoscale
#set xrange [1:2]
plot "trapezium.csv" usi 3 ti "current velocity" w l, "" usi 5 ti "velocity profile" w l

