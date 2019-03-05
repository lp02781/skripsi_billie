set terminal png large size 900,480 enhanced font "20"

set grid 
set title "Velocity reference"
set xlabel "time"
set ylabel "velocity(m/s)"
set output "velocity.png"

set xrange [0:6000]
set yrange [0:2.2]
plot "velocity_data.csv" usi 1 ti "velocity reference" w l


