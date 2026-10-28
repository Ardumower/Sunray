#!/usr/bin/env python3
"""Authenticated LAN WebRTC data channels; signaling enters only via Sunray RPC."""
import asyncio
import json
import logging
import math
import re
import secrets
import struct
import time
from aiohttp import web
from aiortc import RTCPeerConnection, RTCConfiguration, RTCSessionDescription

LOG = logging.getLogger('sunray.direct')


class Link:
    def __init__(self):
        self.peers = {}
        self.writer = None
        self.owner = None
        self.last_drive = 0
        self.drive = None
        self.tasks = []

    def stop_drive(self, peer):
        if self.owner is peer:
            self.drive = (time.monotonic(), 0, 0)
            self.owner = None

    async def remove(self, peer):
        self.stop_drive(peer)
        self.peers.pop(peer.sid, None)
        await peer.pc.close()

    async def rpc(self, request):
        if request.headers.get('X-Sunray-IPC') != '1' or request.headers.get('Origin'):
            raise web.HTTPForbidden()
        try:
            data = await request.json()
            action, sid = data.get('action'), data.get('sid', '')
            if not re.fullmatch('[a-f0-9]{32}', sid):
                raise ValueError('Invalid session')
            if action == 'close':
                if sid in self.peers:
                    await self.remove(self.peers[sid])
                return web.json_response({'ok': True})
            if action == 'renew':
                if sid not in self.peers:
                    raise ValueError('Session expired')
                self.peers[sid].expires = time.monotonic()+120
                return web.json_response({'ok': True})
            if action != 'offer' or self.writer is None:
                raise ValueError('Sunray direct bridge unavailable')
            sdp = data.get('sdp', '')
            if not isinstance(sdp, str) or len(sdp)>20000 or 'm=application ' not in sdp:
                raise ValueError('Invalid data-channel offer')
            if sid in self.peers or len(self.peers)>=4:
                raise ValueError('Direct connections busy')
            peer = Peer(self, sid)
            self.peers[sid] = peer
            try:
                # No STUN/TURN: direct interface candidates only, never a relay.
                await asyncio.wait_for(peer.pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type='offer')), 3)
                await asyncio.wait_for(peer.pc.setLocalDescription(await peer.pc.createAnswer()), 3)
            except Exception:
                await self.remove(peer)
                raise
            return web.json_response({'ok': True, 'sid': sid, 'sdp': peer.pc.localDescription.sdp})
        except (ValueError, TypeError, asyncio.TimeoutError) as exc:
            return web.json_response({'ok': False, 'error': str(exc) or 'Direct negotiation timed out'})

    async def bridge(self):
        while True:
            writer = None
            try:
                reader, writer = await asyncio.open_connection('127.0.0.1', 18891)
                self.writer = writer
                while True:
                    size = struct.unpack('!I', await reader.readexactly(4))[0]
                    if not 16 <= size <= 1024*1024:
                        raise ValueError('Invalid frame length')
                    frame = await reader.readexactly(size)
                    for peer in list(self.peers.values()):
                        peer.frame(frame)
            except (OSError, ValueError, asyncio.IncompleteReadError):
                pass
            finally:
                self.writer = None
                self.drive = None
                for peer in list(self.peers.values()):
                    await self.remove(peer)
                if writer:
                    writer.close()
                    await writer.wait_closed()
            await asyncio.sleep(1)

    async def tick(self):
        while True:
            now = time.monotonic()
            for peer in list(self.peers.values()):
                if now > peer.expires or (peer.opened and now-peer.last_seen>3):
                    await self.remove(peer)
                    continue
                peer.heartbeat(now)
                peer.send_frame_parts(now)
            if self.owner and now-self.last_drive>.3:
                self.stop_drive(self.owner)
            if self.drive and self.writer:
                stamp, lin, ang = self.drive
                self.drive = None
                if now-stamp<.25 and self.writer.transport.get_write_buffer_size()<1024:
                    self.writer.write(f'D {stamp:.6f} {lin:.5f} {ang:.5f}\n'.encode())
            await asyncio.sleep(.025)


class Peer:
    def __init__(self, link, sid):
        self.link, self.sid = link, sid
        self.pc = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        self.control = self.images = None
        self.expires = time.monotonic()+120
        self.last_seen = time.monotonic()
        self.opened = False
        self.last_heartbeat = 0
        self.challenges = {}
        self.sequence = -1
        self.frame_sequence = 0
        self.camera_until = 0
        self.camera_index = 0
        self.pending_frame = None
        self.sending_frame = None

        @self.pc.on('datachannel')
        def channel(dc):
            if dc.label == 'control' and self.control is None:
                self.control = dc
                @dc.on('message')
                def message(value):
                    self.message(value)
                @dc.on('close')
                def closed():
                    self.link.stop_drive(self)
            elif dc.label == 'camera' and self.images is None:
                self.images = dc
            else:
                dc.close()

    def heartbeat(self, now):
        if not self.control or self.control.readyState != 'open' or now-self.last_heartbeat<.1:
            return
        self.opened = True
        self.last_heartbeat = now
        self.challenges = {k:v for k,v in self.challenges.items() if now-v<.3}
        token = secrets.token_hex(8)
        self.challenges[token] = now
        if self.control.bufferedAmount < 4096:
            self.control.send(json.dumps({'type': 'heartbeat', 'token': token}))

    def message(self, value):
        if not isinstance(value, str) or len(value)>2048:
            return
        try:
            msg = json.loads(value)
            now = time.monotonic()
            challenge = self.challenges.get(msg.get('token'), -100)
            if now-challenge>.3:
                return
            self.last_seen = now
            if msg.get('type') == 'pong':
                return
            if msg.get('type') == 'camera_subscribe':
                d = msg.get('data', {})
                index = d.get('index', self.camera_index)
                if isinstance(index, int) and 0 <= index <= 255:
                    self.camera_index = index
                    self.camera_until = now+15
                return
            if msg.get('type') != 'drive':
                return
            seq = msg.get('seq')
            if not isinstance(seq, int) or seq<=self.sequence:
                return
            self.sequence = seq
            lin, ang = float(msg['lin']), float(msg['ang'])
            if not math.isfinite(lin) or not math.isfinite(ang) or abs(lin)>.6 or abs(ang)>1:
                return
            if self.link.owner not in (None, self):
                return
            self.link.owner = self
            self.link.last_drive = now
            self.link.drive = (now, lin, ang)
            if lin == 0 and ang == 0:
                self.link.owner = None
        except (ValueError, TypeError, KeyError):
            return

    def frame(self, frame):
        dc = self.images
        if not dc or dc.readyState != 'open' or time.monotonic()>self.camera_until or dc.bufferedAmount>32768:
            return
        if frame[:4] != b'SRBX' or frame[5] != 1:
            return
        flags = struct.unpack_from('!H', frame, 6)[0]
        if not flags & 0x8000 or flags & 0xff != self.camera_index:
            return
        self.pending_frame = (time.monotonic(), frame[16:])

    def send_frame_parts(self, now):
        dc = self.images
        if not dc or dc.readyState != 'open' or now > self.camera_until:
            self.pending_frame = self.sending_frame = None
            return
        if self.sending_frame and now-self.sending_frame['started']>1:
            self.sending_frame = None
        if self.sending_frame is None and self.pending_frame:
            stamp, jpeg = self.pending_frame
            self.pending_frame = None
            if now-stamp>.5:
                return
            self.frame_sequence = (self.frame_sequence+1) & 0xffffffff
            self.sending_frame = dict(started=now, jpeg=jpeg, part=0,
                                      count=(len(jpeg)+11999)//12000, seq=self.frame_sequence)
        # Bound the video queue so heartbeats and drive packets can interleave.
        # The producer retains only its latest pending image, never a FIFO.
        while self.sending_frame and dc.bufferedAmount<16384:
            f = self.sending_frame
            i = f['part']
            dc.send(struct.pack('!4sIHH', b'SRC1', f['seq'], i, f['count'])+f['jpeg'][i*12000:(i+1)*12000])
            f['part'] += 1
            if f['part'] == f['count']:
                self.sending_frame = None


async def start(app):
    link = app['link']
    link.tasks = [asyncio.create_task(link.bridge()), asyncio.create_task(link.tick())]


async def stop(app):
    link = app['link']
    for task in link.tasks:
        task.cancel()
    await asyncio.gather(*link.tasks, return_exceptions=True)
    for peer in list(link.peers.values()):
        await link.remove(peer)


def make_app():
    app = web.Application(client_max_size=24000)
    app['link'] = Link()
    app.router.add_post('/rpc', app['link'].rpc)
    app.on_startup.append(start)
    app.on_cleanup.append(stop)
    return app


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    web.run_app(make_app(), host='127.0.0.1', port=18892, access_log=None)
