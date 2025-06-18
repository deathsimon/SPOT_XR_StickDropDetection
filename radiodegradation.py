import requests
import logging

class VStingClient:
    def __init__(self, host='kn-adrz-vsting.local', port=8000):
        self.base_url = f"http://{host}:{port}"
        self.logger = logging.getLogger(__name__)


    def shape(self, latency_ms: int, jitter_ms: int = 0):
        """Set Latency and jitter in ms"""
        try:
            response = requests.post(f"{self.base_url}/shape", json = {
                "ul": {"delay": latency_ms, "jitter": jitter_ms},
                "dl": {"delay": latency_ms, "jitter": jitter_ms},
            })
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            self.logger.error(f"Failed to send shape command: {e}")