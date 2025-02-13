import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys, time, math
import serial
import matplotlib.cm as cm

xsize=60
ysize = 60
maxtemp = float('-inf')
mintemp = float('inf')

cmap = cm.plasma # Choose colour map "Plasma"
norm = plt.Normalize(vmin=20, vmax = 60) # Range for colour map is from 20 to 60 degrees C

ser = serial.Serial(
 port='COM3',
 baudrate=115200,
 parity=serial.PARITY_NONE,
 stopbits=serial.STOPBITS_TWO,
 bytesize=serial.EIGHTBITS
)

start_time = time.perf_counter() # Stores start time

def data_gen():
    while True: # Infinite loop
        if ser.isOpen():
            serialnum = ser.readline().decode().strip() # Takes values from serial port
            temp = float(serialnum) # Converts values to floating point num.
            t = (time.perf_counter() - start_time) # Counts elapsed time in seconds, stored in t

            yield t, temp


def run(data):
    # update the data
    global maxtemp, mintemp
    t, temp = data
    if t>-1:
        xdata.append(t)
        ydata.append(temp)

        maxtemp = max(maxtemp, temp) # Records max and min temperatures
        mintemp = min(mintemp, temp)

        max_text.set_text(f"Max Temp: \n {maxtemp:.2f} °C")
        min_text.set_text(f"Min Temp: \n {mintemp:.2f} °C")
        current_text.set_text(f"Temperature: \n{temp:.2f} °C")

        colour = cmap(norm(temp)) #Picks colour based on current temperature and colour map picked
        line.set_color(colour) #Use colour map for line colour

        if t+5>xsize: # Scroll to the left.
            ax.set_xlim(t+5-xsize, t+5)
        
        line.set_data(xdata, ydata)

    return line,



def on_close_figure(event):
    sys.exit(0)

data_gen.t = -1
fig = plt.figure()
fig.canvas.mpl_connect('close_event', on_close_figure)
ax = fig.add_subplot(111)
line, = ax.plot([], [], lw=3)
ax.set_ylim(10, ysize)
ax.set_xlim(0, xsize)
ax.grid()
xdata, ydata = [], []
ax.set_xlabel("Time (s)", fontsize=14)
ax.set_ylabel("Temperature (°C)", fontsize=14)
ax.set_title("Temperature Strip-Chart", fontsize=22)
max_text = ax.text(1.0025,0.6,"Error: Reconnect", transform=ax.transAxes, fontsize=11, color = 'red') #Add text for max/min/current temp
min_text = ax.text(1.0025,0.4,"Error: Reconnect", transform=ax.transAxes, fontsize=11, color = 'blue')
current_text = ax.text(1.0025,0.5,"Error: Reconnect", transform=ax.transAxes, fontsize=14)

# Important: Although blit=True makes graphing faster, we need blit=False to prevent
# spurious lines to appear when resizing the stripchart.
ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=30, repeat=False, save_count=1000)
plt.show()
