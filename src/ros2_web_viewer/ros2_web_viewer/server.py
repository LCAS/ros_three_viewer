"""FastAPI web server for ros2_web_viewer.

Exposes:
  GET  /            → index.html (Three.js viewer)
  GET  /api/urdf    → raw URDF XML (204 if not yet received)
  GET  /mesh/{pkg}/{path:path} → proxy mesh files from ROS packages
  WS   /ws          → bidirectional WebSocket (server → client data stream)
"""

import asyncio
import logging
import os
from pathlib import Path
from typing import Callable, Set

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, Response
from fastapi.staticfiles import StaticFiles

log = logging.getLogger('ros2_web_viewer.server')

try:
    from ament_index_python.packages import get_package_share_directory
    _HAS_AMENT = True
except ImportError:
    _HAS_AMENT = False


class ViewerServer:
    """Self-contained FastAPI server, run in a dedicated thread/event loop."""

    def __init__(self, web_dir: str, urdf_getter: Callable[[], str | None]):
        self.web_dir = web_dir
        self._get_urdf = urdf_getter
        self._clients: Set[WebSocket] = set()
        self._loop: asyncio.AbstractEventLoop | None = None
        self.app = self._build_app()

    # ------------------------------------------------------------------
    # App construction
    # ------------------------------------------------------------------

    def _build_app(self) -> FastAPI:
        app = FastAPI(title='ROS2 Web Viewer', docs_url=None, redoc_url=None)

        @app.get('/api/urdf')
        async def urdf():
            content = self._get_urdf()
            if content is None:
                return Response(status_code=204)
            return Response(content=content, media_type='text/xml')

        @app.get('/mesh/{package}/{path:path}')
        async def mesh(package: str, path: str):
            if not _HAS_AMENT:
                return Response(status_code=404)
            try:
                pkg_dir = get_package_share_directory(package)
                fpath = Path(pkg_dir) / path
                if fpath.exists() and fpath.is_file():
                    return FileResponse(str(fpath))
            except Exception:
                pass
            # Also try meshes sub-folder conventions
            for sub in ('meshes', 'mesh', ''):
                try:
                    pkg_dir = get_package_share_directory(package)
                    fpath = Path(pkg_dir) / sub / path
                    if fpath.exists():
                        return FileResponse(str(fpath))
                except Exception:
                    pass
            return Response(status_code=404)

        @app.websocket('/ws')
        async def ws_endpoint(websocket: WebSocket):
            await websocket.accept()
            self._clients.add(websocket)
            log.info('Client connected (%d total)', len(self._clients))
            try:
                while True:
                    # Keep the connection alive; ignore incoming pings
                    await websocket.receive_text()
            except WebSocketDisconnect:
                pass
            except Exception:
                pass
            finally:
                self._clients.discard(websocket)
                log.info('Client disconnected (%d remaining)', len(self._clients))

        # Static files last (catches everything else)
        app.mount('/', StaticFiles(directory=self.web_dir, html=True), name='static')

        return app

    # ------------------------------------------------------------------
    # Broadcasting
    # ------------------------------------------------------------------

    async def _broadcast(self, message: str):
        dead: Set[WebSocket] = set()
        for ws in list(self._clients):
            try:
                await ws.send_text(message)
            except Exception:
                dead.add(ws)
        self._clients -= dead

    def broadcast_threadsafe(self, message: str):
        """Thread-safe: schedule a broadcast from any thread (e.g. ROS2 callback)."""
        if self._loop and not self._loop.is_closed():
            asyncio.run_coroutine_threadsafe(self._broadcast(message), self._loop)

    # ------------------------------------------------------------------
    # Entry point
    # ------------------------------------------------------------------

    def run(self, host: str = '0.0.0.0', port: int = 8080):
        """Blocking — runs the event loop.  Call in a daemon thread."""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        config = uvicorn.Config(
            self.app,
            host=host,
            port=port,
            loop='none',
            log_level='warning',
        )
        server = uvicorn.Server(config)
        self._loop.run_until_complete(server.serve())
