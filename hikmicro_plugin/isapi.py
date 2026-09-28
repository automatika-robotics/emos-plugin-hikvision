"""Hikvision ISAPI HTTP client (Digest auth).

PTZ is PUTs with small XML bodies to
``/ISAPI/PTZCtrl/channels/<n>/...``; thermometry a GET under
``/ISAPI/Thermal/channels/<n>/...``.
"""

from typing import Optional


class IsapiError(RuntimeError):
    """An ISAPI request failed (transport error or a non-2xx response)."""


class IsapiClient:
    """A thin Digest-auth HTTP client for one camera's ISAPI surface.

    Construction does no I/O; each ``put_xml`` / ``get`` is one synchronous
    request from the launcher process. ``httpx`` is imported on first use.
    """

    def __init__(
        self,
        base_url: str,
        username: str,
        password: str,
        timeout: float = 5.0,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self._username = username
        self._password = password
        self.timeout = timeout
        self._client = None

    def _http(self):
        if self._client is None:
            import httpx

            self._client = httpx.Client(
                auth=httpx.DigestAuth(self._username, self._password),
                timeout=self.timeout,
            )
        return self._client

    def put_xml(self, path: str, xml: str):
        """PUT an XML body to ``path`` (relative to the camera's base URL)."""
        return self._request(
            "PUT",
            path,
            content=xml.encode("utf-8"),
            headers={"Content-Type": "application/xml"},
        )

    def get(self, path: str, params: Optional[dict] = None):
        """GET ``path`` (relative to the camera's base URL)."""
        return self._request("GET", path, params=params)

    def close(self) -> None:
        """Close the underlying HTTP client. Safe to call more than once."""
        if self._client is not None:
            self._client.close()
            self._client = None

    def _request(self, method: str, path: str, **kwargs):
        import httpx

        url = f"{self.base_url}/{path.lstrip('/')}"
        try:
            resp = self._http().request(method, url, **kwargs)
        except httpx.RequestError as exc:  # network / timeout / transport error
            raise IsapiError(f"{method} {path} failed: {exc}") from exc
        if not resp.is_success:
            raise IsapiError(
                f"{method} {path} -> HTTP {resp.status_code}: {resp.text[:200]}"
            )
        return resp


__all__ = ["IsapiClient", "IsapiError"]
