set terminal png large size 1400,1000

set grid 
set title "Circle Trajectory"
set xlabel "time"
set ylabel "position(m)"
set output "circle.png"
set autoscale
#set xrange [1:2]
plot "circle.csv" usi 1:3 ti "position reff" w l, "" usi 2:3 ti "current position" w l

set grid 
set title "Position Error"
set xlabel "time(x10ms)"
set ylabel "Error (m)"
set output "error.png"
set autoscale
#set xrange [1:2]
plot "circle.csv" usi 5 ti "x error" w l, "" usi 6 ti "y error" w l

set grid 
set title "Position X"
set xlabel "time (x10ms)"
set ylabel "Position (m)"
set output "PositionX.png"
set autoscale
#set xrange [1:2]
plot "circle.csv" usi 1 ti "x reff" w l, "" usi 2 ti "current x" w l

set grid 
set title "Position Y"
set xlabel "time (x10ms)"
set ylabel "Position (m)"
set output "PositionY.png"
set autoscale
#set xrange [1:2]
plot "circle.csv" usi 3 ti "y reff" w l, "" usi 4 ti "current y" w l

set grid 
set title "MSE"
set xlabel "time (x10ms)"
set ylabel "error"
set output "MSE.png"
set autoscale
#set xrange [1:2]
plot "circle.csv" usi 9 ti "MSE X" w l, "" usi 10 ti "MSE Y" w l

set grid 
set title "Roll Pitch"
set xlabel "time (x10ms)"
set ylabel "Degree"
set output "RollPitch.png"
set autoscale
#set xrange [1:2]
plot "circle.csv" usi 11 ti "Roll" w l, "" usi 12 ti "Pitch" w l
