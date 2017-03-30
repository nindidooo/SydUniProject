import threading
from serial import Serial
from struct import unpack
import sys
import numpy as np
from PyQt4 import QtGui, QtCore
import ui_main
import pyqtgraph
 
 
# FIRST update baud_rate and port you are using
baud_rate = 115200
port = '/dev/cu.usbmodem1421'
 
 
# Threading setup
 
global lock
lock = threading.Lock()
 
 
""" serial data structure:
 
   For synchronization purposes, the following scheme was chosen:
   A0 data:   A09 (MSB) A08 A07 A06 A05 A04 A03 A02 A01 A00 (LSB)
   sent as byte 1:   1 1 1 A09 A08 A07 A06 A05
       and byte 2:   0 1 1 A04 A03 A02 A01 A00
 
           byte 1  A0 5 most significant bits + 224 (128+64+32), legitimate values are between 224 and 255
           byte 2  A0 5 least significant bits + 96 (64+32)    , legitimate values are between 96 and 127
"""
 
 
class DataReader(threading.Thread):
 
    # Thread event, stops the thread if it is set.
    stopthread = threading.Event()
 
    def __init__(self):
        threading.Thread.__init__(self)  # Call constructor of parent
        self.ser = Serial(port, baud_rate)  # Initialize serial port
        self.data_buff_size = 1024  # Buffer size
        self.data = np.zeros(self.data_buff_size)  # Data buffer
        self.start()
 
    def run(self):  # Run method, this is the code that runs while thread is alive.
 
        num_bytes = 400  # Number of bytes to read at once
        val = 0  # Read value
 
        while not self.stopthread.isSet():
            rslt = self.ser.read(num_bytes)  # Read serial data
            # Convert serial data to array of numbers
            byte_array = unpack('%dB' % num_bytes, rslt)
 
            first = False  # Flag to indicate weather we have the first byte of the number
            for byte in byte_array:
                if 224 <= byte <= 255:  # If first byte of number
                    val = (byte & 0b11111) << 5
                    first = True
                elif 96 <= byte <= 127:  # If second byte of number
                    val |= (byte & 0b11111)
                    if first:
                        lock.acquire()
                        self.data = np.roll(self.data, -1)
                        self.data[-1] = val
                        lock.release()
 
        self.ser.close()
 
    def stop(self):
        self.stopthread.set()
 
 
class ExampleApp(QtGui.QMainWindow, ui_main.Ui_MainWindow):
 
    def __init__(self, parent=None):
        pyqtgraph.setConfigOption('background', 'w')  # before loading widget
        super(ExampleApp, self).__init__(parent)
        self.setupUi(self)
        self.grFFT.plotItem.showGrid(True, True, 0.7)
        self.grPCM.plotItem.showGrid(True, True, 0.7)
        self.maxFFT = 0
        self.maxPCM = 0
        self.data_reader = DataReader()
 
    def getFFT(self, data, N):
        # Fill in FFT info here ########################################################################################
 
        # first window input
 
        # get FFT of windowed signal
 
        # Ensure NaNs are dealt with (set to 0)
 
        # normalise FFT data
 
        # Store into class input_FFT
        input_FFT = np.zeros(N/2)
        
        ################################################################################################################
        self.input_FFT = input_FFT
 
    def update(self):
 
        # get data
        data_max = np.max(np.abs(self.data_reader.data))
 
        N = self.data_reader.data_buff_size     # number of input pts
        input = self.data_reader.data           # input signal
        time_axis = np.arange(N)
 
        # GET FFT
        self.getFFT(input, N)
 
        # Setup FFT axis
        fs = 6900.0  # samples per second roughly
        T = 1.0 / fs
        x_fft = np.linspace(0.0, 1.0/(2.0*T), N/2)
 
        self.maxFFT = max(self.input_FFT)
 
        # Set up autoscaling of input
        if data_max > self.maxPCM:
            self.maxPCM = data_max
            self.grPCM.plotItem.setRange(
                yRange=[max(0, 0.2*data_max), 1.2*data_max])
 
        # Set up autoscaling of FFT data
        if np.max(self.input_FFT) > self.maxFFT:
            self.maxFFT = np.max(np.abs(self.input_FFT))
            self.grFFT.plotItem.setRange(yRange=[0, self.maxFFT])
 
            self.pbLevel.setValue(1000*data_max/self.maxPCM)
        pen = pyqtgraph.mkPen(color='b')
        self.grPCM.plot(time_axis, input, pen=pen, clear=True)
        pen = pyqtgraph.mkPen(color='r')
        self.grFFT.plot(x_fft, self.input_FFT, pen=pen, clear=True)
        QtCore.QTimer.singleShot(1, self.update)  # QUICKLY repeat
 
if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    form = ExampleApp()
    form.show()
    form.update()  # start with something
    app.exec_()
    print("DONE")
