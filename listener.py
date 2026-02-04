import asyncio
import json
import zmq
import zmq.asyncio
from typing import Callable, Awaitable, Any, List

"""
ZMQ Listener Module.
Handles asynchronous communication with other agents (drones) via ZeroMQ.
Used to receive position data from the swarm network.
"""

# Global Context for ZMQ
_CTX = zmq.asyncio.Context()

class Listener:
    def __init__(self, listen_ports: list[int]):
        """
        Initializes the Listener with a list of ports to subscribe to.
        Example ports: [102, 103, 104] -> Maps to tcp ports 2102, 2103, 2104.
        """
        self.listen_ports = listen_ports
        self.poller = zmq.asyncio.Poller()
        self.sockets: list[zmq.Socket] = []
        self.callables: list[Callable[[dict], Any]] = []
        self.awaitables: list[Callable[[dict], Awaitable]] = []
        self.is_connected = False
    
    def set_listen_ports(self, ports: list[int]):
        self.listen_ports = ports
        print(f"[Listener] Listen ports updated to {self.listen_ports}")
        
    async def connect(self):
        if self.is_connected:
            print("[Listener] Already connected.")
            return
        
        self.is_connected = True
        for p in self.listen_ports:
            s = _CTX.socket(zmq.SUB)
            s.setsockopt(zmq.RECONNECT_IVL, 500)
            s.setsockopt(zmq.RECONNECT_IVL_MAX, 1000)
            
            # Port mapping logic (e.g., ID 2 -> Port 2002)
            # Adjust IP logic as per your network config
            portified = p + 2000
            target_address = f"tcp://192.168.0.{p}:{portified}"
            
            s.connect(target_address)
            s.setsockopt(zmq.SUBSCRIBE, b"")  # Accept all topics
            self.poller.register(s, zmq.POLLIN)
            self.sockets.append(s)
            
        # Start the receive loop as a background task
        asyncio.create_task(self._recv_loop())
        print(f"[Listener] Listening on mapped ports for IDs: {self.listen_ports}")

    async def disconnect(self):
        if not self.is_connected:
            print("[Listener] Not connected.")
            return
        
        self.is_connected = False
        for s in self.sockets:
            self.poller.unregister(s)
            s.close()
        self.sockets.clear()
        print("[Listener] Disconnected.")

    async def _recv_loop(self):
        """Main loop to poll sockets and trigger callbacks."""
        while True and self.is_connected:
            try:
                events = await self.poller.poll(timeout=20)
                for sock, _ in events:
                    result = await sock.recv_multipart()
                    
                    if len(result) > 1:
                        topic, payload = result
                    else:
                        payload = result[0]
                        topic = b"POSITION" # Default topic
                    
                    try:
                        msg = json.loads(payload)
                        # print(f"[Log] Received: Topic={topic.decode()}, Msg={msg}")
                    except json.JSONDecodeError:
                        msg = {}
                        print(f"[Listener] JSON Error on payload: {payload}")

                    # Trigger synchronous callbacks
                    for cb in self.callables:
                        cb(topic, msg)
                    
                    # Trigger asynchronous callbacks
                    for cb in self.awaitables:
                        asyncio.create_task(cb(topic, msg))
            except Exception as e:
                print(f"[Listener] Error in receive loop: {e}")
                await asyncio.sleep(1)

    def append_callable(self, fn):
        """Add a synchronous callback function."""
        self.callables.append(fn)

    def append_awaitable(self, fn):
        """Add an asynchronous callback function."""
        self.awaitables.append(fn)