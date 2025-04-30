import numpy as np
import time
from quanser.common import Timeout
from pal.utilities.stream import BasicStream
import os


#top 80 colors from xkcd color survey https://xkcd.com/color/rgb/
MASK_COLORS_HEX=[
    "7e1e9c",
    "15b01a",
    "0343df",
    "ff81c0",
    "653700",
    "e50000",
    "95d0fc",
    "029386",
    "f97306",
    "96f97b",
    "c20078",
    "ffff14",
    "75bbfd",
    "929591",
    "89fe05",
    "bf77f6",
    "9a0eea",
    "033500",
    "06c2ac",
    "c79fef",
    "00035b",
    "d1b26f",
    "00ffff",
    "13eac9",
    "06470c",
    "ae7181",
    "35063e",
    "01ff07",
    "650021",
    "6e750e",
    "ff796c",
    "e6daa6",
    "0504aa",
    "001146",
    "cea2fd",
    "000000",
    "ff028d",
    "ad8150",
    "c7fdb5",
    "ffb07c",
    "677a04",
    "cb416b",
    "8e82fe",
    "53fca1",
    "aaff32",
    "380282",
    "ceb301",
    "ffd1df",
    "cf6275",
    "0165fc",
    "0cff0c",
    "c04e01",
    "04d8b2",
    "01153e",
    "3f9b0b",
    "d0fefe",
    "840000",
    "be03fd",
    "c0fb2d",
    "a2cffe",
    "dbb40c",
    "8fff9f",
    "580f41",
    "4b006e",
    "8f1402",
    "014d4e",
    "610023",
    "aaa662",
    "137e6d",
    "7af9ab",
    "02ab2e",
    "9aae07",
    "8eab12",
    "b9a281",
    "341c02",
    "36013f",
    "c1f80a",
    "fe01b1",
    "fdaa48",
    "9ffeb0",
    ]
MASK_COLORS_RGB=[list(int(h[i:i+2], 16) for i in (0, 2, 4)) for h in MASK_COLORS_HEX]

class Obstacle():
    def __init__(self,
                 name = 'QCar',
                 distance = 0,
                 conf = 0,
                 x=0,
                 y=0      
                 ):
      
        self.name = name
        self.distance = distance 
        self.conf = conf
        self.x=x
        self.y=y

class TrafficLight(Obstacle):
    def __init__(self,
                 color = 'idle',
                 distance = 0
                 ):
        super().__init__(name='traffic light', distance=distance)
        self.lightColor = color

class QCar2DepthAligned():
    def __init__(self,ip='localhost',nonBlocking=True,manualStart=False,port='18777'):
        self.depth  = np.empty((480,640,1), dtype = np.float32)
        self.rgb  = np.empty((480,640,3), dtype = np.uint8) 
        if not manualStart:
            self.__initDepthAlign()
        self.uri='tcpip://'+ip+':'+port
        self._timeout = Timeout(seconds=0, nanoseconds=1000000)
        self._handle = BasicStream(uri=self.uri,
                                    agent='C',
                                    receiveBuffer=np.zeros((480,640,4),
                                                           dtype=np.float32),
                                    sendBufferSize=480*640*3,
                                    recvBufferSize=480*640*4*4,
                                    nonBlocking=nonBlocking,
                                    reshapeOrder='F')
        self._sendPacket = np.zeros((480,640,3),dtype=np.uint8)
        self.status_check('', iterations=20)
    
    def __initDepthAlign(self):
        self.__stopDepthAlign()
        depthAlignPath = os.path.normpath(os.path.join(
            os.path.dirname(__file__),
            '../../../resources/applications/QCarDepthAlign/'
                + 'QCar2DepthAlign.rt-linux_qcar2 '
        ))
        os.system(
            'quarc_run -r -t tcpip://localhost:17000 '
            + depthAlignPath + '-uri tcpip://localhost:17003'
        )

        time.sleep(4)
        print('Aligned Depth image is streaming')

    def __stopDepthAlign(self):
        # Quietly stop qcarLidarToGPS if it is already running:
        # the -q flag kills the executable
        # the -Q flag kills quietly (no errors thrown if its not running)
        os.system(
            'quarc_run -t tcpip://localhost:17000 -q -Q '
            + 'QCar2DepthAlign.rt-linux_qcar2'
        )

    def status_check(self, message, iterations=10):
        # blocking method to establish connection to the server stream.
        self._timeout = Timeout(seconds=0, nanoseconds=1000) #1000000
        counter = 0
        while not self._handle.connected:
            self._handle.checkConnection(timeout=self._timeout)
            counter += 1
            if self._handle.connected:
                print(message)
                break
            elif counter >= iterations:
                print('Server error: status check failed.')
                break

    def read(self):
        new = False
        self._timeout = Timeout(seconds=0, nanoseconds=100)
        if self._handle.connected:
            new, bytesReceived = self._handle.receive(timeout=self._timeout, iterations=5)
            # print('read:',new, bytesReceived)
            # if new is True, full packet was received
            if new:
                self.depth[:,:,:] = self._handle.receiveBuffer[:,:,:1]
                self.rgb[:,:,:] = self._handle.receiveBuffer[:,:,[3,2,1]].astype(np.uint8)

        else:
            self.status_check('Reconnected to Server')
        return new
    
    def read_reply(self,annotated_frame):

        # data received flag
        new = False

        # 1 us timeout parameter
        self._timeout = Timeout(seconds=0, nanoseconds=10000000)

        # set remaining packet to send
        self._sendPacket = annotated_frame

        # if connected to driver, send/receive
        if self._handle.connected:
            self._handle.send(self._sendPacket)
            new, bytesReceived = self._handle.receive(timeout=self._timeout, iterations=5)
            # print(new, bytesReceived)
            # if new is True, full packet was received
            if new:
                self.depth = self._handle.receiveBuffer[:,:,:1]
                self.rgb = self._handle.receiveBuffer[:,:,[3,2,1]].astype(np.uint8)

        else:
            self.status_check('Reconnected to QBot Platform Driver.')

        # if new is False, data is stale, else all is good
        return new

    def terminate(self):
        self.__stopDepthAlign()
        self._handle.terminate()