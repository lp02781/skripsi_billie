set terminal png large size 1400,1000

set grid 
set title "Z Trajectory"
set xlabel "time"
set ylabel "position(m)"
set output "power_consumption.png"
set autoscale
#set xrange [1:2]
plot "power_consumption.csv" usi 3 ti "position reff" w l, "" usi 4 ti "current position" w l

set grid 
set title "X Trajectory"
set xlabel "time"
set ylabel "position(m)"
set output "positionX.png"
set autoscale
#set xrange [1:2]
plot "power_consumption.csv" usi 1 ti "position reff" w l, "" usi 2 ti "current position" w l

set grid 
set title "Current Error"
set xlabel "time"
set ylabel "error (m)"
set output "error.png"
set autoscale
#set xrange [1:2]
plot "power_consumption.csv" usi 5 ti "x error" w l, "" usi 6 ti "z error" w l

set grid 
set title "MSE"
set xlabel "time"
set ylabel "error (m)"
set output "MSE.png"
set autoscale
#set xrange [1:2]
plot "power_consumption.csv" usi 7 ti "MSE x" w l, "" usi 8 ti "MSE z" w l

set grid 
set title "Roll Pitch"
set xlabel "time"
set ylabel "sudut"
set output "RollPitch.png"
set autoscale
#set xrange [1:2]
plot "power_consumption.csv" usi 9 ti "roll" w l, "" usi 10 ti "pitch" w l

