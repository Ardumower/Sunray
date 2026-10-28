import asyncio
import json
import struct
import time
import unittest
from types import SimpleNamespace
from server import Link, Peer

class Channel:
    readyState = 'open'
    def __init__(self):
        self.bufferedAmount = 0
        self.sent = []
    def send(self, data):
        self.sent.append(data)
        self.bufferedAmount += len(data)

class Tests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.link = Link()
        self.peer = Peer(self.link, 'a'*32)
        self.peer.control = Channel()
        self.peer.images = Channel()
        self.peer.challenges = {'fresh': time.monotonic(), 'old': time.monotonic()-1}
    async def asyncTearDown(self):
        await self.peer.pc.close()
    def drive(self, seq=1, token='fresh', lin=.2, ang=0):
        self.peer.message(json.dumps(dict(type='drive', seq=seq, token=token, lin=lin, ang=ang)))
    async def test_old_nonce_nonfinite_out_of_order_and_limits(self):
        self.drive(token='old'); self.assertIsNone(self.link.drive)
        self.drive(lin=100); self.assertIsNone(self.link.drive)
        self.drive(seq=2, lin=float('nan')); self.assertIsNone(self.link.drive)
        self.drive(seq=3); self.assertAlmostEqual(self.link.drive[1], .2)
        self.drive(seq=2,lin=.1); self.assertAlmostEqual(self.link.drive[1], .2)
    async def test_one_direct_driver_and_disconnect_stop(self):
        other = object(); self.link.owner = other
        self.drive(); self.assertIsNone(self.link.drive)
        self.link.owner = None
        self.drive(seq=2); self.assertIs(self.link.owner, self.peer)
        self.link.stop_drive(self.peer); self.assertEqual(self.link.drive[1:],(0,0))
    async def test_expired_drive_is_zeroed_by_watchdog(self):
        self.drive(); self.link.last_drive = time.monotonic()-1
        lines=[]
        self.link.writer=SimpleNamespace(transport=SimpleNamespace(get_write_buffer_size=lambda:0),write=lines.append)
        task=asyncio.create_task(self.link.tick())
        await asyncio.sleep(.04);task.cancel();await asyncio.gather(task,return_exceptions=True)
        self.assertEqual([float(v) for v in lines[0].decode().split()[-2:]],[0,0])
    async def test_camera_selection_and_bounded_queue(self):
        self.peer.camera_until=time.monotonic()+10
        self.peer.camera_index=100
        header=bytearray(16);header[:4]=b'SRBX';header[5]=1;struct.pack_into('!H',header,6,0x8000)
        self.peer.frame(bytes(header)+b'wrong');self.assertIsNone(self.peer.pending_frame)
        struct.pack_into('!H',header,6,0x8000|100)
        self.peer.frame(bytes(header)+b'x'*100000)
        self.peer.send_frame_parts(time.monotonic())
        self.assertEqual(len(self.peer.images.sent),2)
        self.assertLess(self.peer.images.bufferedAmount,25000)
        self.peer.frame(bytes(header)+b'newest')
        self.assertEqual(self.peer.pending_frame[1],b'newest')
    async def test_browser_http_cannot_open_a_session(self):
        from aiohttp import web
        request=SimpleNamespace(headers={'Origin':'https://untrusted.example','X-Sunray-IPC':'1'})
        with self.assertRaises(web.HTTPForbidden): await self.link.rpc(request)

if __name__=='__main__': unittest.main()
