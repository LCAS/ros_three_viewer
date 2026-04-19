"""FastAPI web server for ros2_web_viewer.

Exposes:
  GET  /            → index.html (Three.js viewer)
  GET  /api/urdf    → raw URDF XML (204 if not yet received)
  POST /api/trigger → call a std_srvs/Trigger service
  GET  /mesh/{pkg}/{path:path} → proxy mesh files from ROS packages
  GET  <configured html routes> → custom HTML files from web/
  WS   /ws          → bidirectional WebSocket (server → client data stream)
"""

import asyncio
import logging
import os
from pathlib import Path
from typing import Callable, Set

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response
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
        self._client_init_messages_getter: Callable[[], list[str]] | None = None
        self._trigger_service_caller: Callable[[str, float], dict] | None = None
        self._html_routes: dict[str, str] = {}
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

        @app.post('/api/trigger')
        async def trigger_service(payload: dict):
            if self._trigger_service_caller is None:
                return JSONResponse(
                    status_code=503,
                    content={'ok': False, 'error': 'Trigger service bridge is not configured'},
                )

            service = str(payload.get('service', '')).strip()
            timeout_sec_raw = payload.get('timeout_sec', 2.0)
            try:
                timeout_sec = float(timeout_sec_raw)
            except (TypeError, ValueError):
                timeout_sec = 2.0
            timeout_sec = max(0.1, min(timeout_sec, 30.0))

            loop = asyncio.get_running_loop()
            result = await loop.run_in_executor(
                None,
                lambda: self._trigger_service_caller(service, timeout_sec),
            )
            status_code = 200 if result.get('ok', False) else 400
            return JSONResponse(status_code=status_code, content=result)

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
            if self._client_init_messages_getter:
                try:
                    for message in self._client_init_messages_getter():
                        await websocket.send_text(message)
                except Exception:
                    log.exception('Failed to send cached init messages to client')
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

        for route_path, file_path in self._resolve_static_html_routes().items():
            async def static_html_page(_request_path: str = file_path):
                return FileResponse(_request_path, media_type='text/html')
            app.add_api_route(route_path, static_html_page, methods=['GET'])

        # Static files last (catches everything else)
        app.mount('/', StaticFiles(directory=self.web_dir, html=True), name='static')

        return app

    def _resolve_static_html_routes(self) -> dict[str, str]:
        web_root = Path(self.web_dir).resolve()
        resolved_routes: dict[str, str] = {}
        for route, rel_path in self._html_routes.items():
            route_str = str(route or '').strip()
            path_str = str(rel_path or '').strip()
            if not route_str.startswith('/'):
                route_str = f'/{route_str}'
            if route_str in ('/', '/ws', '/api', '/api/urdf', '/api/trigger'):
                log.warning('Skipping html route "%s": reserved route', route_str)
                continue
            if route_str.startswith('/api/'):
                log.warning('Skipping html route "%s": reserved API namespace', route_str)
                continue

            candidate = Path(path_str)
            if candidate.is_absolute():
                resolved_file = candidate.resolve()
            else:
                resolved_file = (web_root / candidate).resolve()

            if web_root not in resolved_file.parents and resolved_file != web_root:
                log.warning(
                    'Skipping html route "%s": path "%s" is outside web root "%s"',
                    route_str, path_str, web_root)
                continue
            if not resolved_file.is_file():
                log.warning(
                    'Skipping html route "%s": file "%s" not found',
                    route_str, resolved_file)
                continue
            if resolved_file.suffix.lower() not in ('.html', '.htm'):
                log.warning(
                    'Skipping html route "%s": file "%s" is not HTML',
                    route_str, resolved_file)
                continue

            resolved_routes[route_str] = str(resolved_file)
        return resolved_routes

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

    def set_client_init_messages_getter(self, getter: Callable[[], list[str]]):
        """Register callback used to replay cached state to new WS clients."""
        self._client_init_messages_getter = getter

    def set_trigger_service_caller(self, caller: Callable[[str, float], dict]):
        """Register callback used by HTTP route /api/trigger."""
        self._trigger_service_caller = caller

    def set_html_routes(self, routes: dict[str, str]):
        """Register custom static HTML routes served before static fallback."""
        self._html_routes = dict(routes)

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
