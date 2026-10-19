"""Hikvision ISAPI HTTP client (Digest auth).

PTZ is PUTs with small XML bodies to
``/ISAPI/PTZCtrl/channels/<n>/...``; thermometry a GET under
``/ISAPI/Thermal/channels/<n>/...``.
"""

import re
from typing import Optional

#: A multipart part larger than this is not a thermometry record.
_MAX_PART_BYTES = 1 << 20
_PART_HEADER_END = re.compile(rb"\r?\n\r?\n")
_CONTENT_LENGTH = re.compile(rb"content-length:\s*(\d+)", re.IGNORECASE)


def first_multipart_part(buf: bytes) -> Optional[bytes]:
    """The body of the first complete part in a multipart buffer, else ``None``.

    The part ends after its ``Content-Length`` bytes, or at the next boundary
    line when the header carries no length.
    """
    start = buf.find(b"--")
    if start < 0:
        return None
    header_end = _PART_HEADER_END.search(buf, start)
    if header_end is None:
        return None
    body = buf[header_end.end():]
    length = _CONTENT_LENGTH.search(buf, start, header_end.start())
    if length is not None:
        n = int(length.group(1))
        return body[:n] if len(body) >= n else None
    boundary = re.search(rb"\r?\n--", body)
    return body[: boundary.start()] if boundary else None


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

    def get_first_part(self, path: str, params: Optional[dict] = None) -> str:
        """GET ``path`` and return its body, or the first part of a multipart one.

        Some endpoints answer with a multipart stream that never ends; the
        connection is dropped as soon as one complete part is in.
        """
        import httpx

        url = f"{self.base_url}/{path.lstrip('/')}"
        try:
            with self._http().stream("GET", url, params=params) as resp:
                if not resp.is_success:
                    resp.read()
                    raise IsapiError(
                        f"GET {path} -> HTTP {resp.status_code}: {resp.text[:200]}"
                    )
                if not resp.headers.get("content-type", "").lower().startswith("multipart/"):
                    resp.read()
                    return resp.text
                buf = b""
                for chunk in resp.iter_bytes():
                    buf += chunk
                    part = first_multipart_part(buf)
                    if part is not None:
                        return part.decode("utf-8", errors="replace")
                    if len(buf) > _MAX_PART_BYTES:
                        raise IsapiError(
                            f"GET {path}: no complete part in the first {_MAX_PART_BYTES} bytes"
                        )
        except httpx.RequestError as exc:
            raise IsapiError(f"GET {path} failed: {exc}") from exc
        raise IsapiError(f"GET {path}: the stream ended before a complete part")

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


__all__ = ["IsapiClient", "IsapiError", "first_multipart_part"]
