import json
import socket
import struct
import threading
import unittest
import numpy as np

from newton_ros2 import LatestSender
from farmily_newton_bridge.protocol import encode,receive,MAX_BYTES


class ROS2TransportTests(unittest.TestCase):
    def test_fragmented_numeric_images_preserve_nan_and_endian(self):
        arrays={'depth':np.array([[1.,np.nan],[2.,3.]],dtype='>f4'),
                'ids':np.array([[0,1000],[3,2]],dtype=np.uint32)}
        packet=encode(dict(kind='camera',time=1.25),arrays)
        a,b=socket.socketpair()
        def send():
            for i in range(0,len(packet),7):a.sendall(packet[i:i+7])
            a.close()
        thread=threading.Thread(target=send);thread.start()
        message,result=receive(b);b.close();thread.join()
        self.assertEqual(message,dict(kind='camera',time=1.25))
        np.testing.assert_equal(result['depth'],arrays['depth'])
        np.testing.assert_array_equal(result['ids'],arrays['ids'])
        self.assertEqual(result['depth'].dtype.str,'<f4')

    def test_invalid_packet_does_not_unpickle_or_allocate_unbounded_input(self):
        with self.assertRaises(ValueError):encode({},dict(x=np.array([object()])))
        a,b=socket.socketpair();a.sendall(struct.pack('!II',MAX_BYTES,MAX_BYTES))
        with self.assertRaises(ValueError):receive(b)
        a.close();b.close()

    def test_slow_consumer_coalesces_states_without_blocking_producer(self):
        class Slow:
            def __init__(self):self.entered=threading.Event();self.release=threading.Event();self.packets=[]
            def sendall(self,value):
                self.entered.set();self.release.wait(5.);self.packets.append(value)
            def shutdown(self,*_):pass
            def close(self):pass
        sock=Slow();sender=LatestSender(sock)
        sender.put(dict(kind='labels'))
        self.assertTrue(sock.entered.wait(1.))
        for i in range(1000):sender.put(dict(kind='state',time=float(i)))
        sender.put(dict(kind='camera',time=999.))
        sender.put(dict(kind='state',time=1000.))
        self.assertEqual(len(sender.pending),2)
        self.assertEqual(sender.dropped['state'],1000)
        sock.release.set();sender.close()
        headers=[]
        for packet in sock.packets:
            length,_=struct.unpack('!II',packet[:8]);headers.append(json.loads(packet[8:8+length]))
        self.assertEqual([h['kind'] for h in headers],['labels','camera','state'])
        self.assertEqual([h['time'] for h in headers[1:]],[999.,1000.])
        self.assertIsNone(sender.error)


if __name__=='__main__':unittest.main()
