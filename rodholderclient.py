import requests
import websockets
import asyncio
import logging

class HolderClient:
    def __init__(self, host="localhost", port=8000):
        self.base_url = f"http://{host}:{port}"
        self.ws_url = f"ws://{host}:{port}/ws"
        self.logger = logging.getLogger(__name__)

    def drop(self):
        """Send drop command to holder"""
        try:
            response = requests.get(f"{self.base_url}/drop")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            self.logger.error(f"Failed to send drop command: {e}")
            raise

    def toggle_drop_pull(self):
        """Send pullup command to holder"""
        try:
            response = requests.get(f"{self.base_url}/toggle_drop_pull")
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            self.logger.error(f"Failed to send toggle_drop_pull command: {e}")
            raise

    async def listen_for_updates(self, callback):
        """Listen for WebSocket updates from the holder"""
        async with websockets.connect(self.ws_url) as websocket:
            self.logger.info("Connected to holder WebSocket")
            while True:
                try:
                    message = await websocket.recv()
                    callback(message)
                except websockets.exceptions.ConnectionClosed:
                    self.logger.error("WebSocket connection closed")
                    break
                except Exception as e:
                    self.logger.error(f"WebSocket error: {e}")
                    break