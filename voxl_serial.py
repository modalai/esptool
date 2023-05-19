import ctypes as ct
import numpy as np
import time

libvoxl_io = ct.CDLL("libslpi_uart_bridge_cpu.so")

voxl_uart_init  = libvoxl_io.voxl_uart_init
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
voxl_uart_read           = libvoxl_io.voxl_uart_read
voxl_uart_read.argtypes  = [ct.c_int, ct.POINTER(ct.c_uint8), ct.c_size_t]
voxl_uart_read.restype   = ct.c_int

# flush the serial port
voxl_uart_flush = libvoxl_io.voxl_uart_flush
voxl_uart_flush.argtypes = [ct.c_int]
voxl_uart_flush.restype  = ct.c_int

# allocate RPC buffer for reading data from DSP
voxl_rpc_shared_mem_alloc          = libvoxl_io.voxl_rpc_shared_mem_alloc
voxl_rpc_shared_mem_alloc.argtypes = [ct.c_size_t]
voxl_rpc_shared_mem_alloc.restype  = ct.POINTER(ct.c_uint8)

# free shared RPC buffer
voxl_rpc_shared_mem_free          = libvoxl_io.voxl_rpc_shared_mem_free
voxl_rpc_shared_mem_free.argtypes = [ct.POINTER(ct.c_uint8)]
voxl_rpc_shared_mem_free.restype  = None

# de-init RPC shared memory structures
voxl_rpc_shared_mem_deinit          = libvoxl_io.voxl_rpc_shared_mem_deinit
voxl_rpc_shared_mem_deinit.argtypes = []
voxl_rpc_shared_mem_deinit.restype  = None


class VoxlSerialPort():
    initialized   = False
    _baudrate      = 0
    port          = ''
    port_num      = -1
    parity        = 0
    bytesize      = 8
    stopbits      = 1
    timeout       = 0
    writeTimeout  = 0
    read_buf_ptr  = None
    read_buf_size = 256
    write_buf_size = 64

    def __init__(self):
        print("Initializing voxl_serial port!")
        self.initialized = False

    def isOpen(self):
        return self.initialized

    def inWaiting(self):
        return self.read_buf_size

    def open(self):
        self.port_num = int(self.port.split('-')[-1])
        print(f"Port num: {self.port_num}")
        self._baudrate = int(self._baudrate)
        print(f"Baud Rate: {self._baudrate}")
        voxl_uart_init(self.port_num, self._baudrate)
        self.read_buf_ptr = voxl_rpc_shared_mem_alloc(self.read_buf_size)
        self.initialized  = True

    @property
    def baudrate(self):
        return self._baudrate

    @baudrate.setter
    def baudrate(self,value):
        # print(f"voxl_serial.py: Setting new baud rate: {self.baudrate}->{value}\tInit: {self.initialized}")
        self._baudrate = value
        if self.initialized:
            ret = voxl_uart_init(self.port_num, self._baudrate)
            # print(f'Updated baud rate to {self._baudrate}')
        else:
            print("Port not initialized voxl_serial.py")

    def close(self):
        if not self.initialized:
            raise Exception('port is not initialized')

        voxl_uart_close(self.port_num)

    def read(self, num_bytes_to_read):
        if num_bytes_to_read > self.read_buf_size:
            num_bytes_to_read = self.read_buf_size
        bytes_read = voxl_uart_read(self.port_num,self.read_buf_ptr,num_bytes_to_read)
        data_array = bytearray(self.read_buf_ptr[0:bytes_read])
        return data_array

    def write(self, data):
        if not self.initialized:
            raise Exception('port is not initialized')
        
        # print(f"\tSize of data to be written: {np.array(data).nbytes} bytes")
        # print(f"\tReceived data: {data}")
        # If data is larger than rx buffer on SLPI side, break into smaller chunks before writing
        if len(data) > self.write_buf_size:
            # print(f"\tSize of data to be written to large! Breaking into {self.write_buf_size} byte chunks.")
            blocks = (len(data) + self.write_buf_size - 1) // self.write_buf_size
            for block in range(blocks):
                from_offs = block * self.write_buf_size
                to_offs = from_offs + self.write_buf_size

                # Prevent index out of range
                if to_offs > len(data):
                    # print(f"\tEnd offset too large: {to_offs}")
                    # print(f"\tSetting to {len(bytearray(data))}")
                    # print(f"\tLast byte: {hex(data[np.array(data).nbytes-1])} (should be c0)")
                    to_offs = len(data) + 1
                
                # print(f"data:\n{data}")
                data_slice = np.array(data[from_offs:to_offs])
                # print(f"\tWriting {len(bytearray(data_slice))} bytes...[{from_offs}:{to_offs-1}]")
                # print(f"data:\n{data_slice}")
                data_ptr = data_slice.ctypes.data_as(ct.POINTER(ct.c_uint8))
                voxl_uart_write(self.port_num,data_ptr,data_slice.nbytes)
        
        # Write normally
        else:
            data = np.array(data)
            data_ptr = data.ctypes.data_as(ct.POINTER(ct.c_uint8))
            voxl_uart_write(self.port_num,data_ptr,data.nbytes)

        # print(f"\tFinished writing {np.array(data).nbytes} bytes")

        # Give some time for SLPI to read and respond after writing
        time.sleep(0.013)

    def flush(self):
        voxl_uart_flush(self.port_num)

    def flushInput(self):
        pass
