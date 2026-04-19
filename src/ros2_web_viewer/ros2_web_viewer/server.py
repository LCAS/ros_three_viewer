"""FastAPI web server for ros2_web_viewer.

Exposes:
  GET  /api/urdf    → raw URDF XML (204 if not yet received)
  POST /api/trigger → call a std_srvs/Trigger service
    POST /api/register_viewer_topics → subscribe to viewer data topics requested by 3D canvas widgets
  POST /api/register_html_panel_topic → subscribe to a String topic for HTML panel widgets
    GET  /assets/{path:path} → static frontend assets from web/
  GET  /mesh/{pkg}/{path:path} → proxy mesh files from ROS packages
  GET  <configured html routes> → custom HTML files from web/
  WS   /ws          → bidirectional WebSocket (server → client data stream)
"""

import asyncio
import logging
import os
import sys
import time
from pathlib import Path
from typing import Callable, Set

import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, RedirectResponse, Response
from fastapi.staticfiles import StaticFiles
from starlette.exceptions import HTTPException as StarletteHTTPException

log = logging.getLogger('ros2_web_viewer.server')
if not log.handlers:
    _handler = logging.StreamHandler(stream=sys.stderr)
    _handler.setFormatter(logging.Formatter('%(asctime)s [%(levelname)s] %(name)s: %(message)s'))
    log.addHandler(_handler)
log.setLevel(logging.INFO)
log.propagate = True
log.info('Logging initialized for ros2_web_viewer.server')


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
        self._viewer_topic_registrar: Callable[[dict], dict] | None = None
        self._html_panel_topic_registrar: Callable[[str], dict] | None = None
        self._html_routes: dict[str, str] = {}
        self.app = self._build_app()

    # ------------------------------------------------------------------
    # App construction
    # ------------------------------------------------------------------

    def _build_app(self) -> FastAPI:
        app = FastAPI(title='ROS2 Web Viewer', docs_url=None, redoc_url=None)

        @app.middleware('http')
        async def log_http_requests(request: Request, call_next):
            start = time.perf_counter()
            status_code = 500
            try:
                response = await call_next(request)
                status_code = response.status_code
                return response
            finally:
                duration_ms = (time.perf_counter() - start) * 1000.0
                level = logging.INFO if status_code < 400 else logging.WARNING
                log.log(
                    level,
                    'HTTP %s %s -> %d (%.1f ms)',
                    request.method,
                    request.url.path,
                    status_code,
                    duration_ms,
                )

        @app.exception_handler(StarletteHTTPException)
        async def log_http_exception(request: Request, exc: StarletteHTTPException):
            log.warning(
                'HTTP exception %d on %s %s: %s',
                exc.status_code,
                request.method,
                request.url.path,
                exc.detail,
            )
            return JSONResponse(status_code=exc.status_code, content={'detail': exc.detail})

        @app.exception_handler(Exception)
        async def log_unhandled_exception(request: Request, exc: Exception):
            log.exception('Unhandled server error on %s %s', request.method, request.url.path)
            return JSONResponse(status_code=500, content={'detail': 'Internal Server Error'})

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

        @app.post('/api/register_html_panel_topic')
        async def register_html_panel_topic(payload: dict):
            if self._html_panel_topic_registrar is None:
                return JSONResponse(
                    status_code=503,
                    content={'ok': False, 'error': 'HTML panel topic registrar is not configured'},
                )

            topic = str(payload.get('topic', '')).strip()
            loop = asyncio.get_running_loop()
            result = await loop.run_in_executor(
                None,
                lambda: self._html_panel_topic_registrar(topic),
            )
            status_code = 200 if result.get('ok', False) else 400
            return JSONResponse(status_code=status_code, content=result)

        @app.post('/api/register_viewer_topics')
        async def register_viewer_topics(payload: dict):
            if self._viewer_topic_registrar is None:
                return JSONResponse(
                    status_code=503,
                    content={'ok': False, 'error': 'Viewer topic registrar is not configured'},
                )

            loop = asyncio.get_running_loop()
            result = await loop.run_in_executor(
                None,
                lambda: self._viewer_topic_registrar(payload or {}),
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

        def _create_html_route_handler(request_path: str):
            async def static_html_page():
                return FileResponse(request_path, media_type='text/html')
            return static_html_page

        def _create_html_route_redirect_handler(canonical_path: str):
            async def redirect_html_page():
                return RedirectResponse(url=canonical_path, status_code=307)
            return redirect_html_page

        def _create_html_asset_handler(route_path: str, asset_root: Path):
            async def static_html_asset(asset_path: str):
                # Preserve API/assets namespaces and guard against path traversal.
                if not asset_path:
                    return Response(status_code=404)

                candidate = (asset_root / asset_path).resolve()
                if candidate != asset_root and asset_root not in candidate.parents:
                    log.warning(
                        'Blocking path traversal for html route "%s": "%s"',
                        route_path,
                        asset_path,
                    )
                    return Response(status_code=404)
                if not candidate.is_file():
                    return Response(status_code=404)

                return FileResponse(str(candidate))

            return static_html_asset

        log.info('Registering %d custom HTML routes', len(self._html_routes))
        for route_path, file_path in self._resolve_static_html_routes().items():
            log.info('Registering custom HTML route: %s -> %s', route_path, file_path)
            html_file = Path(file_path)
            # For non-root routes, provide a canonical trailing-slash page URL so
            # relative links like "style.css" resolve under that route.
            if route_path != '/':
                canonical_html_path = f'{route_path}/'
                app.add_api_route(
                    route_path,
                    _create_html_route_redirect_handler(canonical_html_path),
                    methods=['GET'],
                )
                app.add_api_route(
                    canonical_html_path,
                    _create_html_route_handler(file_path),
                    methods=['GET'],
                )
                app.add_api_route(
                    f'{route_path}/{{asset_path:path}}',
                    _create_html_asset_handler(route_path, html_file.parent),
                    methods=['GET'],
                )
            else:
                app.add_api_route(route_path, _create_html_route_handler(file_path), methods=['GET'])

        # Static assets are served from /assets; HTML pages come from html_routes.
        app.mount('/assets', StaticFiles(directory=self.web_dir, html=False), name='assets')

        return app

    def _resolve_static_html_routes(self) -> dict[str, str]:
        web_root = Path(self.web_dir).resolve()
        resolved_routes: dict[str, str] = {}
        logging.info('Resolving custom HTML routes with web root "%s"', web_root)
        for route, rel_path in self._html_routes.items():
            log.info('Configuring custom HTML route "%s" -> "%s"', route, rel_path)
            route_str = str(route or '').strip()
            path_str = str(rel_path or '').strip()
            if not route_str.startswith('/'):
                route_str = f'/{route_str}'
            if route_str in ('/ws', '/api', '/api/urdf', '/api/trigger', '/api/register_html_panel_topic', '/api/register_viewer_topics', '/assets'):
                log.warning('Skipping html route "%s": reserved route', route_str)
                continue
            if route_str.startswith('/api/'):
                log.warning('Skipping html route "%s": reserved API namespace', route_str)
                continue
            if route_str.startswith('/assets/'):
                log.warning('Skipping html route "%s": reserved assets namespace', route_str)
                continue

            candidate = Path(path_str)
            if candidate.is_absolute():
                resolved_file = candidate.resolve()
            else:
                resolved_file = (web_root / candidate).resolve()

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

    def set_viewer_topic_registrar(self, registrar: Callable[[dict], dict]):
        """Register callback used by HTTP route /api/register_viewer_topics."""
        self._viewer_topic_registrar = registrar

    def set_html_panel_topic_registrar(self, registrar: Callable[[str], dict]):
        """Register callback used by HTTP route /api/register_html_panel_topic."""
        self._html_panel_topic_registrar = registrar

    def set_html_routes(self, routes: dict[str, str]):
        """Register custom static HTML routes served before static fallback."""
        log.info('Updating HTML routes: %s', routes)
        self._html_routes = dict(routes)
        # Rebuild app so newly configured routes are registered before server start.
        self.app = self._build_app()

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
