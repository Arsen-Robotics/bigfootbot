import asyncio
import logging
import os
import aiohttp
import aiohttp_cors
from aiohttp import web

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
)
logger = logging.getLogger("http_server")

# Get environment variables
PORT = int(os.environ.get("PORT", 8080))
HOST = os.environ.get("HOST", "0.0.0.0")
LOG_LEVEL = os.environ.get("LOG_LEVEL", "INFO")
logging.getLogger().setLevel(getattr(logging, LOG_LEVEL))

async def index(request):
    """Serve the index.html page."""
    logger.info("Serving index.html")
    content = open(os.path.join(os.path.dirname(__file__), "../static/index.html"), "r").read()
    return web.Response(content_type="text/html", text=content)

async def on_startup(app):
    """Log when the server starts."""
    # Get the actual address people should connect to
    if HOST == "0.0.0.0":
        import socket
        try:
            # Try to get the local IP address
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            logger.info(f"HTTP server started at http://{ip}:{PORT}")
        except:
            logger.info(f"HTTP server started at http://{HOST}:{PORT}")
    else:
        logger.info(f"HTTP server started at http://{HOST}:{PORT}")

async def on_shutdown(app):
    """Log when the server shuts down."""
    logger.info("HTTP server shutting down")

def create_app():
    """Create the HTTP application."""
    app = web.Application()
    
    # Setup CORS
    cors = aiohttp_cors.setup(app, defaults={
        "*": aiohttp_cors.ResourceOptions(
            allow_credentials=True,
            expose_headers="*",
            allow_headers="*",
            allow_methods=["GET", "POST", "OPTIONS"]
        )
    })
    
    # Routes
    app.router.add_get("/", index)
    
    # Configure static routes
    static_path = os.path.join(os.path.dirname(__file__), "../static")
    app.router.add_static("/static", static_path)
    
    # Register startup and shutdown handlers
    app.on_startup.append(on_startup)
    app.on_shutdown.append(on_shutdown)
    
    # Apply CORS to all routes
    for route in list(app.router.routes()):
        cors.add(route)
    
    return app

if __name__ == "__main__":
    # Run the HTTP server
    logger.info(f"Starting HTTP server on {HOST}:{PORT}")
    app = create_app()
    web.run_app(app, host=HOST, port=PORT, access_log=None)
    logger.info("HTTP server stopped") 