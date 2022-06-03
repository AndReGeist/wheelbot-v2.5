#!/usr/bin/env python
# Code based on: https://thepoorengineer.com/en/python-gui/

import sys
from threading import Thread
import serial
import time
from os import system, name
import collections
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.backends.backend_tkagg as backend_tkagg
import copy
import struct
import pandas as pd
import tkinter as Tk
from tkinter.ttk import Frame
matplotlib.use("TkAgg")
plt.style.use('bmh')
#plt.style.use('dark_background')

class Window(Frame):
    def __init__(self, figure, master, SerialReference):
        Frame.__init__(self, master)
        self.entry = None
        self.setPoint = None
        self.master = master  # a reference to the master window
        self.serialReference = SerialReference  # keep a reference to our serial connection so that we can use it for bi-directional communicate from this class
        self.initWindow(figure)  # initialize the window with our settings
        self.packet_length_2WB = 32
        self.data_out = ['0'] * self.packet_length_2WB  # Last four bytes are reserved for CRC
        self.data_out[9] = 'J'

    def initWindow(self, figure):
        self.master.title("Real Time Plot")
        canvas = backend_tkagg.FigureCanvasTkAgg(figure, master=self.master)
        toolbar = backend_tkagg.NavigationToolbar2Tk(canvas, self.master)
        canvas.get_tk_widget().pack(side=Tk.TOP, fill=Tk.BOTH, expand=1)

        # create out widgets in the master frame
        lbl1 = Tk.Label(self.master, text="Enter command:")
        lbl1.pack(padx=5, pady=5)
        self.entry = Tk.Entry(self.master)
        self.entry.insert(0, 'e')  # (index, string)
        self.entry.pack(padx=5)

        SendButton = Tk.Button(self.master, text='Send', command=self.sendCommand)
        SendButton.pack(padx=5)
        Tk.Button(self.master, text="Store data", command=self.master.quit).pack()

        StartButton = Tk.Button(self.master, text='Activate wheelbot', command=self.sendStartCommand)
        StartButton.pack(side='right')

        StopButton = Tk.Button(self.master, text='STOP', command=self.sendStopCommand)
        StopButton.pack(side='right')

        motor1clockwise = Tk.Button(self.master, text='motor1+', command=self.sendMotor1Command)
        motor1clockwise.pack(side='left')

        #self.canvas.bind("<Up>", self.up)

    #def up(self, event):
     #   self.serialReference.sendSerialData("<Up>")

    def sendCommand(self):
        self.data_out[0] = self.entry.get()
        self.serialReference.sendSerialData(self.data_out)  # '%' is our ending marker

    def sendStartCommand(self):
        self.data_out[0] = 'i'
        self.serialReference.sendSerialData(self.data_out)  # '%' is our ending marker

    def sendStopCommand(self):
        self.data_out[0] = 'e'
        self.serialReference.sendSerialData(self.data_out)  # '%' is our ending marker

    def sendMotor1Command(self):
        self.data_out[0] = 'm'
        self.serialReference.sendSerialData(self.data_out)  # '%' is our ending marker


class serialPlot:
    def __init__(self, unscale_list, plotLength=100, numPlots=8, select=range(16)):
        self.serialPort0 = '/dev/ttyACM0'
        self.serialBaud = 9600

        self.unscale_list = unscale_list
        self.plotMaxLength = plotLength
        self.packetlength = 16  # A received package has length of 16 bytes
        self.numPlots = numPlots
        self.select = select
        self.rawData = bytearray(self.packetlength * 2)
        self.dataType = None
        self.check = 0

        self.dataType = ['!h'] * 16  # signed short (2 byte), the exclamation mark changes the byte order to big-endian
        #self.dataType[12] = 'H'  # The udriver flags are booleans stored in a 16 bit char (unsigned)
        self.dataType[11] = 'H'  # The safety flags are booleans stored in a 16 bit char, I convert it into unsigned short and then format it into a bit-string
        self.dataType[12] = 'H'  # Battery voltage (2 byte) is unsigned
        self.dataType[13] = 'H'  # Time measurements are unsigned
        #self.value = [0] * self.packetlength
        self.data = [0] * 16
        self.live_data = []
        for i in range(self.packetlength):   # give an array for each type of data and store them in a list
            self.live_data.append(collections.deque([0] * self.plotMaxLength, maxlen=self.plotMaxLength))
        self.isRun = True
        self.thread = None
        self.plotTimer = 0
        self.previousTimer = 0
        self.csvData = []

        print('Trying to connect to: ' + str(self.serialPort0) + ' at ' + str(self.serialBaud) + ' BAUD.')
        try:
            self.serialConnection = serial.Serial(port=self.serialPort0, baudrate=self.serialBaud, bytesize=8, timeout=1, write_timeout=0)
            print('Connected to ' + str(self.serialPort0) + ' at ' + str(self.serialBaud) + ' BAUD.')
        except:
            print("Failed to connect with " + str(self.serialPort0) + ' at ' + str(self.serialBaud) + ' BAUD.')
            sys.exit()

    def readSerialStart(self):
        if self.thread == None:
            self.thread = Thread(target=self.backgroundThread)
            self.thread.start()
            #self.serialConnection.reset_input_buffer()
            self.serialConnection.flush()

            # Block till we start receiving values
            while ( self.serialConnection.inWaiting() < (32+1) ):
                time.sleep(0.1)


    def getSerialData(self, frame, lines, lineValueText, lineLabel, timeText):
        currentTimer = time.perf_counter()
        self.plotTimer = int((currentTimer - self.previousTimer) * 1000)     # the first reading will be erroneous
        self.previousTimer = currentTimer
        timeText.set_text('Plot Interval = ' + str(self.plotTimer) + 'ms')

        for j in range(self.numPlots):
            self.live_data[j].append(self.data[j])
            lines[j].set_data(range(self.plotMaxLength), self.live_data[self.select[j]])
            lineValueText[j].set_text('[' + lineLabel[j] + '] = ' + "{:.2f}".format(self.data[self.select[j]]))

        # *******************************
        # Print M2 STATUS
        # *******************************
        U0, U1, U2, U3, U4, U5, U6, U7, M0, M1, M2, M3, M4, M5, M6, M7 = self.decode_booleans(self.data[11], 16)

        self.clear()  # Function clears console print outs
        print(f"""
            Connected to: {str(self.serialPort0)}
            M2 STATUS
            # {'-' * 40}
            # System : {"CHECK" if M0 else "X"}
            # Angle  : {"CHECK" if M1 else "X"}
            # Rate   : {"CHECK" if M2 else "X"}
            # Control: {"CHECK" if M3 else "X"}
            # Battery: {"CHECK" if M4 else "X"}
            # Wifi   : {"CHECK" if M5 else "X"}
            # Standup: {"ON" if M6 else "OFF"}
            # Balance: {"ON" if M7 else "OFF"}
            # {'-' * 40}

            uDriver STATUS
            # {'-' * 40}
            # System enabled : {"CHECK" if U0 else "X"}
            # Motor 1 ready  : {"CHECK" if U1 else "X"}
            # Motor 2 ready  : {"CHECK" if U2 else "X"}
            # Motor 1 enabled: {"CHECK" if U3 else "X"}
            # Motor 2 enabled: {"CHECK" if U4 else "X"}
            # Enc. detected  : {"CHECK" if U5 else "X"}
            # Error message  : {int(U7), int(U6)}
            #   --> (0,1): Enc. error, (1,0): Comm. timeout
            # {'-' * 40}
           """, end='\r')

        # crc_2PC1 = self.float_to_bin(self.data[8])
        # crc_2PC2  = self.float_to_bin(self.data[9])
        # crc1  = self.float_to_bin(self.data[14])
        # crc2 = self.float_to_bin(self.data[15])
        # print('crc1    :', crc1)
        # print('crc_2PC1:', crc_2PC1)
        # print('crc2    :', crc2)
        # print('crc_2PC2:',  crc_2PC2)
        # print('\n')


    def float_to_bin(self, num):
        return format(struct.unpack('!I', struct.pack('!f', num))[0], '032b')

    def sendSerialData(self, data):
        data2 = ['X'] + data # The M2 checks if message received via USB starts with X
        data2_string = ''.join([str(elem) for elem in data2])
        self.serialConnection.write(data2_string.encode())

    def backgroundThread(self):    # retrieve data
        while (self.isRun):
            #print(self.serialConnection.out_waiting)

            if self.serialConnection.inWaiting() > (63+2):
                self.check = int.from_bytes(self.serialConnection.read(1), 'little')

                if self.check == 88 : # 88 is ASCII for 'X'
                    #print(check)
                    #print(self.serialConnection.inWaiting())

                    self.rawData = self.serialConnection.read(32)
                    #self.rawData = self.rawData[1:]

                    for i in range(self.packetlength):
                        self.data[i] = self.unscale_list[i] * \
                                        struct.unpack(self.dataType[i], self.rawData[(i * 2):(i * 2 + 2)])[0]  # struct.unpack: The result is a tuple even if it contains exactly one item.

                    self.csvData.append([self.data[0], self.data[1], self.data[2], self.data[3],
                                         self.data[4], self.data[5], self.data[6], self.data[7],
                                         self.data[8], self.data[9], self.data[10], self.data[11],
                                         self.data[12], self.data[13], self.data[14], self.data[15]])

    def close(self, lineLabel):
        self.isRun = False
        self.thread.join()
        self.serialConnection.close()
        print('Disconnected...')
        df = pd.DataFrame(self.csvData, columns=lineLabel)
        df.to_csv('out.csv', encoding='utf-8', index=False, header='True')

    def encode_booleans(self, bool_lst):
    # Code from Jan Vlcinsky, https://stackoverflow.com/questions/24038353/python-decoding-binary-to-boolean
        res = 0
        for i, bval in enumerate(bool_lst):
            res += int(bval) << i
        return res

    def decode_booleans(self, intval, bits):
    # Code from Jan Vlcinsky, https://stackoverflow.com/questions/24038353/python-decoding-binary-to-boolean
        res = []
        for bit in range(bits):
            mask = 1 << bit
            res.append((intval & mask) == mask)
        return res

    # define our clear function
    def clear(self):
        # Credit: https://www.geeksforgeeks.org/clear-screen-python/
        # for windows
        if name == 'nt':
            _ = system('cls')

        # for mac and linux(here, os.name is 'posix')
        else:
            _ = system('clear')

def main():
    # portName = 'COM5'
    maxPlotLength = 50     # number of points in x-axis of real time plot

    # Live plot for roll control
    select = [0, 1, 2, 3]  # Standard, Last byte has index 15
    #select = [8, 9, 10]  # Last byte has index 15
    #select = [6, 9]  # Last byte has index 15

    # Live plot for pitch control
    #select = [1, 3, 9, 10, 14]  # Last byte has index 15
    #select = [4, 5, 6, 15]  # Last byte has index 15
    #select = range(16) #[0, 1, 15]      # Choose which data to
    numPlots = len(select)  # number of plots in 1 graph

    PI = 3.14159265358979
    unscaling_attitude = (180/PI)*(PI/32767)
    unscaling_current = 40/32767
    unscaling_rate = 3000 / 32767 # bit value to rpm
    gyro_scaling = 0.0002663161 # For gyro (500[deg/s] / 32768) * (PI / 180) so transforms bit value into rad/s
    accel_scaling = 0.000598755 # 2*9,81 / 32768 # For accelerometer transforms bit in m/s^2

    DATA_MODE = 1

    unscale_list = [1] * 16
    #unscale_list[0] = unscale_angle

    if (DATA_MODE == 1): # Control plots

        flag_debug = 9 # 9

        lineLabel = ['Roll', 'Pitch', 'Roll rate', 'Pitch rate',
                     'I_sent1', 'I_sent2', 'rate_m1', 'rate_m2',
                     'debug1', 'debug2', 'debug3', 'system_flags',
                     'battery_voltage', 'time step', 'crc1', 'crc2']
        unscale_list[0:8] = [unscaling_attitude,  # 0, roll
                             unscaling_attitude,  # 1, pitch
                             gyro_scaling,  # 2, roll rate
                             gyro_scaling ,  # 3, pitch rate
                             unscaling_current,  # 5, current1 sent
                             unscaling_current,  # 8, current2 sent
                             unscaling_rate,  # 6, rate1
                             unscaling_rate]  # 9, rate2

        if ( flag_debug == 1 ) : # MISC
            unscale_list[8:11] = [unscaling_current, 200.0 / 32767, unscaling_rate]
        elif ( flag_debug == 2 ) : # SYS FLAG + MOTOR POS
            unscale_list[8:11] = [1.0, (10000*PI)/32767.0, (10000*PI)/32767.0]
        elif ( flag_debug == 3 ) : # Flag, loop_time, tick_time
            unscale_list[8:11] = [1.0, 2.0, 2.0]
        elif ( flag_debug == 6 ) : # PIVOT ACC
            unscale_list[8:11] = [200.0/32767, unscaling_rate, unscaling_current]
        elif ( flag_debug == 7 ) : # ESTIMATOR BIAS
            unscale_list[8:11] = [1.0, 1.8/32767, 1.8/32767]
        elif (flag_debug == 8):  # DATA IN
            unscale_list[8:11] = [1.0, 1.0, 1.0]
        elif (flag_debug == 9):  # DATA IN
            unscale_list[8:11] = [1.0, unscaling_current, unscaling_current]
        elif (flag_debug == 10):  # DATA IN
            unscale_list[8:11] = [gyro_scaling, unscaling_attitude, unscaling_attitude]
        elif (flag_debug == 11):  # DATA IN
            unscale_list[8:11] = [accel_scaling, accel_scaling, accel_scaling]
        elif (flag_debug == 12):  # DATA IN
            unscale_list[8:11] = [unscaling_attitude, unscaling_attitude, unscaling_attitude]
        elif (flag_debug == 13):  # DATA IN
            unscale_list[8:11] = [accel_scaling, accel_scaling, unscaling_rate]

    elif (DATA_MODE == 2): # Pivot acc plots
        lineLabel = ['q1_acc', 'q2_acc', 'q1_acc_nopivot', 'q2_acc_nopivot',
                     'rate_m2_filtered', 'rate_m2',
                     'acc_imuC_x', 'acc_imuD_x', 'acc_imuC_y',
                     'pivot_acc_x', 'pivot_acc_y',
                     'system_flags', 'battery_voltage', 'time step', 'crc1', 'crc2']
        unscale_list[0:11] = [unscaling_attitude,
                             unscaling_attitude,
                             unscaling_attitude,
                             unscaling_attitude,
                             unscaling_rate,
                             unscaling_rate,
                             accel_scaling,
                             accel_scaling,
                             accel_scaling,
                             accel_scaling,
                             accel_scaling]

    elif (DATA_MODE == 3): # IMU accelerations
        lineLabel = ['acc_imuA_x', 'acc_imuA_y', 'acc_imuA_z',
                     'acc_imuB_x', 'acc_imuB_y', 'acc_imuB_z',
                     'acc_imuC_x', 'acc_imuC_y', 'acc_imuC_z',
                     'acc_imuD_x', 'acc_imuD_y',
                     'system_flags', 'battery_voltage', 'time step', 'crc1', 'crc2']
        unscale_list[0:11] = [accel_scaling,
                              accel_scaling,
                              accel_scaling,
                              accel_scaling,
                              accel_scaling,
                              accel_scaling,
                              accel_scaling,
                              accel_scaling,
                              accel_scaling,
                              accel_scaling,
                              accel_scaling]

    elif (DATA_MODE == 4): # IMU rates
        lineLabel = ['rate_imuA_x', 'rate_imuA_y', 'rate_imuA_z',
                     'rate_imuB_x', 'rate_imuB_y', 'rate_imuB_z',
                     'rate_imuC_x', 'rate_imuC_y', 'rate_imuC_z',
                     'rate_imuD_x', 'rate_imuD_y',
                     'system_flags', 'battery_voltage', 'time step', 'crc1', 'crc2']
        unscale_list[0:11] = [gyro_scaling,
                              gyro_scaling,
                              gyro_scaling,
                              gyro_scaling,
                              gyro_scaling,
                              gyro_scaling,
                              gyro_scaling,
                              gyro_scaling,
                              gyro_scaling,
                              gyro_scaling,
                              gyro_scaling]



    ## 11, are two bytes reserved for system checks
    #unscale_list[12] = 1/30.09  # Battery voltage measurement
    unscale_list[12] = 1 / 28.0  # Battery voltage measurement

    s = serialPlot(unscale_list, maxPlotLength, numPlots, select)   # initializes all required variables
    s.readSerialStart()  # starts background thread

    pltInterval = 100  # Period at which the plot animation updates [ms]
    xmin = 0
    xmax = maxPlotLength
    ymin = -100
    ymax = 100
    fig = plt.figure(figsize=(10, 8))

    # put our plot onto Tkinter's GUI
    root = Tk.Tk()
    app = Window(fig, root, s)

    ax = plt.axes(xlim=(xmin, xmax), ylim=(float(ymin - (ymax - ymin) / 10), float(ymax + (ymax - ymin) / 10)))
    #ax.set_title('Wheelbot interface')
    ax.set_xlabel("time steps")
    ax.set_ylabel("data")

    lineLabel_plot = [lineLabel[i] for i in select]

    style = ['y'] * 16
    style[0:12] = ['r-', 'c-', 'b-', 'g-', 'y--', 'y:', 'y-', 'c:', 'r-', 'c-', 'b-', 'g-']  # linestyles for the different plots
    style = [style[i] for i in select]

    timeText = ax.text(0.70, 0.95, '', transform=ax.transAxes)
    lines = []
    lineValueText = []

    for i in range(numPlots):
        lines.append(ax.plot([], [], style[i], label=lineLabel_plot[i])[0])
        lineValueText.append(ax.text(0.70, 0.90-i*0.05, '', transform=ax.transAxes))

    anim = animation.FuncAnimation(fig, s.getSerialData, fargs=(lines, lineValueText, lineLabel_plot, timeText), interval=pltInterval)    # fargs has to be a tuple

    plt.legend(loc="upper left")
    root.mainloop()   # use this instead of plt.show() since we are encapsulating everything in Tkinter
    s.close(lineLabel)


if __name__ == '__main__':
    main()