import logging
import sys
from aiohttp import web
from webrtc_server import create_app

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
    ]
)
logger = logging.getLogger("main")

if __name__ == "__main__":
    logger.info("Starting WebRTC server")
    app = create_app()
    web.run_app(app, host="0.0.0.0", port=8080)
    logger.info("WebRTC server stopped") 