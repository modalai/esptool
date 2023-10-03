import ctypes as ct
import numpy as np
import time

libvoxl_io = ct.CDLL("libvoxl_io.so")

voxl_uart_init  = libvoxl_io.voxl_uart_init_legacy_slpi
voxl_uart_init.argtypes = [ct.c_int,ct.c_int]
voxl_uart_init.restype  = ct.c_int

# close an initialized serial port
voxl_uart_close = libvoxl_io.voxl_uart_close
voxl_uart_close.argtypes = [ct.c_int]
voxl_uart_close.restype  = ct.c_int

#voxl_uart_drain = libvoxl_io.voxl_uart_drain

# write bytes to an initialized serial port
voxl_uart_write          = libvoxl_io.voxl_uart_write
voxl_uart_write.argtypes = [ct.c_int, ct.POINTER(ct.c_uint8), ct.c_size_t]
voxl_uart_write.restype  = ct.c_int

# read bytes from an initialized serial port
voxl_uart_read           = libvoxl_io.voxl_uart_read_bytes
voxl_uart_read.argtypes  = [ct.c_int, ct.POINTER(ct.c_uint8), ct.c_size_t]
voxl_uart_read.restype   = ct.c_int

# flush the serial port
voxl_uart_flush = libvoxl_io.voxl_uart_flush
voxl_uart_flush.argtypes = [ct.c_int]
voxl_uart_flush.restype  = ct.c_int




class VoxlSerialPort():
    initialized   = False
    _baudrate      = 0
    port          = ''
    port_num      = 7
    parity        = 0
    bytesize      = 8
    stopbits      = 1
    timeout       = 0
    writeTimeout  = 0
    read_buf_ptr  = np.zeros(1024, dtype='uint8')
    read_buf_size = 256

    def __init__(self):
        self.initialized = False

    def isOpen(self):
        return self.initialized

    def inWaiting(self):
        return self.read_buf_size

    def open(self):
        # self.port_num = int(self.port.split('-')[-1])
        self._baudrate = int(self._baudrate)
        voxl_uart_init(self.port_num, self._baudrate)
        self.initialized  = True

    @property
    def baudrate(self):
        return self._baudrate

    @baudrate.setter
    def baudrate(self,value):
        self._baudrate = value
        if self.initialized:
            voxl_uart_close(self.port_num)
            voxl_uart_init(self.port_num, self._baudrate)
            print('Updated baud rate to %d' % self._baudrate)

    def close(self):
        if not self.initialized:
            raise Exception('port is not initialized')

        voxl_uart_close(self.port_num)

    def read(self, num_bytes_to_read):
        if num_bytes_to_read > self.read_buf_size:
            num_bytes_to_read = self.read_buf_size
        # print("READING " + str(num_bytes_to_read))
        bytes_read = voxl_uart_read(self.port_num, self.read_buf_ptr.ctypes.data_as(ct.POINTER(ct.c_uint8)), num_bytes_to_read)
        # print("GOT " + str(bytes_read))
        data_array = bytearray(self.read_buf_ptr[0:bytes_read])
        return data_array

    def write(self, data):
        if not self.initialized:
            raise Exception('port is not initialized')
        # If data is larger than rx buffer on SLPI side, break into smaller chunks before writing
        if len(data) > self.write_buf_size:
            blocks = (len(data) + self.write_buf_size - 1) // self.write_buf_size
            for block in range(blocks):
                from_offs = block * self.write_buf_size
                to_offs = from_offs + self.write_buf_size

                # Prevent index out of range
                if to_offs > len(data):
                    to_offs = len(data) + 1

                data_slice = np.array(data[from_offs:to_offs])
                data_ptr = data_slice.ctypes.data_as(ct.POINTER(ct.c_uint8))
                voxl_uart_write(self.port_num,data_ptr,data_slice.nbytes)

        # Write normally
        else:
            data = np.array(data)
            data_ptr = data.ctypes.data_as(ct.POINTER(ct.c_uint8))
            voxl_uart_write(self.port_num,data_ptr,data.nbytes)

        # Give some time for SLPI to read and respond after writing
        time.sleep(0.013)

    def flush(self):
        voxl_uart_flush(self.port_num)

    # The functions below are not implemented in libqrb5165 so we need these fake functions to avoid error messages when flashing ESP based receivers
    def flushInput(self):
        self.flush()
        pass

    def flushOutput(self):
        self.flush()
        pass

    def setRTS(self, state):
        pass

    def setDTR(self, state):
        pass

    def dtr(self, state):
        pass