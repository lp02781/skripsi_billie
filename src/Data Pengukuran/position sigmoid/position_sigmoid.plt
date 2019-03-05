set terminal png large size 1400,1000

set grid 
set title "Trajectory garis lurus"
set xlabel "time(x10ms)"
set ylabel "position(m)"
set output "position_x.png"
set autoscale
#set xrange [1:2]
plot "position.csv" usi 1 ti "position reff" w l, "" usi 2 ti "current position" w l

set grid 
set title "Sudut Roll Pitch"
set xlabel "time (x10ms)"
set ylabel "Degree"
set output "RollPitch.png"
set autoscale
#set xrange [1:2]
plot "position.csv" usi 6 ti "roll" w l, "" usi 7 ti "pitch" w l

set grid 
set title "Accell"
set xlabel "time (x10ms)"
set ylabel "Accel"
set output "Accel.png"
set autoscale
#set xrange [1:2]
plot "position.csv" usi 5 ti "MSE" w l
