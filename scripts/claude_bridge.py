"""Local bridge that lets Claude Code run Python inside this Kit app.

Paste into Window > Script Editor and run. Listens on 127.0.0.1 only and
requires a random token written to %TEMP%/kit_claude_bridge.token.
Code runs on Kit's main loop, so USD/omni APIs are safe to use.
Re-running restarts the bridge; run `stop_claude_bridge()` to shut it down.
"""
import asyncio
import builtins
import contextlib
import io
import os
import secrets
import tempfile
import traceback

PORT = 8765
TOKEN_FILE = os.path.join(tempfile.gettempdir(), "kit_claude_bridge.token")
TOKEN = secrets.token_hex(16)
_GLOBALS = {"__name__": "__claude__"}


async def _handle(reader, writer):
    try:
        head = await reader.readuntil(b"\r\n\r\n")
        lines = head.decode("latin-1").split("\r\n")
        headers = {k.strip().lower(): v.strip() for k, _, v in (l.partition(":") for l in lines[1:] if l)}
        body = await reader.readexactly(int(headers.get("content-length", 0)))
        if headers.get("x-token") != TOKEN:
            status, out = "403 Forbidden", "bad token"
        else:
            buf = io.StringIO()
            try:
                with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
                    exec(compile(body.decode("utf-8"), "<claude>", "exec"), _GLOBALS)
            except Exception:
                buf.write(traceback.format_exc())
            status, out = "200 OK", buf.getvalue()
        data = out.encode("utf-8")
        writer.write(
            f"HTTP/1.1 {status}\r\nContent-Type: text/plain; charset=utf-8\r\n"
            f"Content-Length: {len(data)}\r\nConnection: close\r\n\r\n".encode() + data
        )
        await writer.drain()
    except Exception:
        traceback.print_exc()
    finally:
        writer.close()


def stop_claude_bridge():
    server = getattr(builtins, "_claude_bridge_server", None)
    if server:
        server.close()
        builtins._claude_bridge_server = None
        print("Claude bridge stopped")


async def _start():
    stop_claude_bridge()
    builtins._claude_bridge_server = await asyncio.start_server(_handle, "127.0.0.1", PORT)
    with open(TOKEN_FILE, "w") as f:
        f.write(TOKEN)
    print(f"Claude bridge listening on 127.0.0.1:{PORT}")


builtins.stop_claude_bridge = stop_claude_bridge
asyncio.ensure_future(_start())
