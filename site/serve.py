#!/usr/bin/env python3
"""Serve the static benchmark site with caching disabled.

Plain ``python -m http.server`` lets browsers heuristically cache module
scripts and JSON within a session, which makes local development maddening:
after edits, different files can be served from different cached revisions.
This development server sends ``Cache-Control: no-store`` on every response
so a normal reload always reflects the working tree.
"""

from __future__ import annotations

import argparse
import functools
import http.server
import os


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self) -> None:
        self.send_header("Cache-Control", "no-store, must-revalidate")
        self.send_header("Expires", "0")
        super().end_headers()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--directory", default=os.path.dirname(os.path.abspath(__file__)))
    args = parser.parse_args()
    handler = functools.partial(NoCacheHandler, directory=args.directory)
    server = http.server.ThreadingHTTPServer((args.bind, args.port), handler)
    print(f"Serving (no-store) at http://{args.bind}:{args.port}")
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
