import asyncio
from types import SimpleNamespace

from aiohttp import web

from go2_dashboard_bridge.bridge import DashboardBridge


def test_cors_headers_are_added_to_http_errors():
    request = SimpleNamespace(
        method="POST",
        headers={"Origin": "http://localhost:3000"},
    )

    async def reject(_request):
        raise web.HTTPForbidden(
            text='{"error":"Live commands are disabled"}',
            content_type="application/json",
        )

    response = asyncio.run(DashboardBridge._cors_middleware(None, request, reject))

    assert response.status == 403
    assert response.headers["Access-Control-Allow-Origin"] == "http://localhost:3000"
    assert response.headers["Access-Control-Allow-Methods"] == "GET, POST, OPTIONS"

