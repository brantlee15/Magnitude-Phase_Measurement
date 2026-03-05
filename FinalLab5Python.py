import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial


ser = serial.Serial(
    port='COM15', 
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE, 
    bytesize=serial.EIGHTBITS,
    timeout=0.1
)


amp1 = 1.75
amp2=1.75
period = 1000.0
phase_rad = np.deg2rad(90)


fig = plt.figure()
xsize = 2000 # 2000ms viewing window
xdata, y1data, y2data = [], [], []
ax = fig.add_subplot(111)

line1, = ax.plot([], [], lw=2, color='red', label='Wave 1 (0°)')
line2, = ax.plot([], [], lw=2, color='blue', label='Wave 2 (Shifted)')

ax.set_xlim(0, xsize)
ax.grid()
ax.set_title("EFM8 Serial Signal Generator")
ax.set_ylabel("Voltage (V)")
ax.set_xlabel("Time (ms)")
ax.legend(loc='upper right')


def data_gen():
    global amp1,amp2, period, phase_rad
    t_ms = 0 
    dt =10#10ms step
    
    # clear old serial data before starting
    ser.reset_input_buffer()
    
    while True:
        
        if ser.in_waiting > 0:
            try:
                raw_data = ser.readline().decode('utf-8').strip()
                if raw_data:
                  
                    vals = raw_data.split(',')
                    if len(vals) == 4:
                        amp1 = float(vals[0])
                        amp2=float(vals[1])
                        period = float(vals[2])
                        phase_rad = np.deg2rad(float(vals[3]))
            except ValueError:
                pass # for errors

        if period > 0:
            amp1=amp2
            y1 = amp1 * np.sin((2 * np.pi / period) * t_ms)
            y2 = amp2 * np.sin((2 * np.pi / period) * t_ms + phase_rad)
        else:
            y1, y2 = 0, 0
            
        yield t_ms, y1, y2
        t_ms += dt


def run(data):
    t, y1, y2 = data
    
    xdata.append(t)
    y1data.append(y1)
    y2data.append(y2)
    
    #update graph to move
    if t > xsize:
        ax.set_xlim(t - xsize, t)
        
   
    ax.set_ylim(-amp1 - 1, amp1 + 1)
        
    line1.set_data(xdata, y1data)
    line2.set_data(xdata, y2data)
    
   #update degrees in legend
    line2.set_label(f'Wave 2 ({np.rad2deg(phase_rad):.0f}°)')
    ax.legend(loc='upper right')
    
    return line1, line2

ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=10, repeat=False)
plt.show()