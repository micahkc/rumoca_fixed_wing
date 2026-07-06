#!/usr/bin/env python3
"""Small helper CLI for the Rumoca fixed-wing browser workbench."""

from __future__ import annotations

import argparse
import http.server
import importlib.util
import json
import re
import shutil
import socketserver
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SITE = ROOT / "site"
DEFAULT_MODEL = ROOT / "results" / "FixedWingPlantGA.mo"
BUNDLE = SITE / "src" / "fixedwing_ga_model.js"
RUMOCA_PACKAGE = SITE / "node_modules" / "@cognipilot" / "rumoca"
RUMOCA_VENDOR = SITE / "public" / "vendor" / "rumoca"
RUMOCA_BROWSER_FILES = [
    "modelica_language.js",
    "parse_worker.js",
    "rumoca_bind_wasm.js",
    "rumoca_bind_wasm_bg.wasm",
    "rumoca_interactive.js",
    "rumoca_package_meta.json",
    "rumoca_runtime.js",
    "rumoca_worker.js",
]


def js_string(value: str) -> str:
    return json.dumps(value)


def detect_model_name(source: str) -> str:
    matches = re.findall(r"\bmodel\s+(\w+)", source)
    if "RumocaFixedWingFlight" in matches:
        return "RumocaFixedWingFlight"
    if matches:
        return matches[-1]
    raise SystemExit("could not find a Modelica model declaration")


def publish_model(args: argparse.Namespace) -> None:
    model = Path(args.model).resolve()
    source = model.read_text()
    model_name = args.model_name or detect_model_name(source)
    BUNDLE.write_text(
        "// AUTO-GENERATED fixed-wing browser model bundle.\n"
        "// Regenerate with: ./results.py publish-model <model.mo>\n\n"
        f"export const FIXEDWING_GA_SOURCE = {js_string(source)};\n"
        f"export const FIXEDWING_GA_MODEL_NAME = {js_string(model_name)};\n"
    )
    print(f"published {model} as {model_name} -> {BUNDLE.relative_to(ROOT)}")


def vendor_rumoca(_args: argparse.Namespace | None = None) -> None:
    if not RUMOCA_PACKAGE.exists():
        raise SystemExit("missing site/node_modules/@cognipilot/rumoca; run `npm ci` in site/")

    missing = [name for name in RUMOCA_BROWSER_FILES if not (RUMOCA_PACKAGE / name).exists()]
    if missing:
        raise SystemExit(f"Rumoca npm package is missing browser files: {', '.join(missing)}")

    if RUMOCA_VENDOR.exists():
        shutil.rmtree(RUMOCA_VENDOR)
    RUMOCA_VENDOR.mkdir(parents=True)
    for name in RUMOCA_BROWSER_FILES:
        shutil.copy2(RUMOCA_PACKAGE / name, RUMOCA_VENDOR / name)

    package_meta = json.loads((RUMOCA_PACKAGE / "package.json").read_text())
    version = package_meta.get("version", "unknown")
    print(f"vendored @cognipilot/rumoca {version} -> {RUMOCA_VENDOR.relative_to(ROOT)}")


def check_site(_args: argparse.Namespace) -> None:
    vendor_rumoca()
    spec = importlib.util.spec_from_file_location("check_site", SITE / "check_site.py")
    if spec is None or spec.loader is None:
        raise SystemExit("could not load site/check_site.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    status = module.main()
    if status:
        raise SystemExit(status)


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, format: str, *args) -> None:  # noqa: A002
        print(format % args)


def serve_site(args: argparse.Namespace) -> None:
    check_site(argparse.Namespace())
    port = args.port
    handler = lambda *h_args, **h_kwargs: QuietHandler(*h_args, directory=str(SITE), **h_kwargs)
    with socketserver.TCPServer(("127.0.0.1", port), handler) as httpd:
        host, bound_port = httpd.server_address
        print(f"Serving Rumoca fixed-wing workbench at http://{host}:{bound_port}")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nStopped.")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    p_publish = sub.add_parser("publish-model", help="embed a fixed-wing Modelica model for browser use")
    p_publish.add_argument("model", nargs="?", default=str(DEFAULT_MODEL), help="Modelica file to embed")
    p_publish.add_argument("--model-name", default="", help="top-level Modelica model name")
    p_publish.set_defaults(func=publish_model)

    p_check = sub.add_parser("check-site", help="validate the static site bundle")
    p_check.set_defaults(func=check_site)

    p_vendor_rumoca = sub.add_parser("vendor-rumoca", help="copy Rumoca browser files from the pinned npm package")
    p_vendor_rumoca.set_defaults(func=vendor_rumoca)

    p_serve = sub.add_parser("serve-site", help="serve the static workbench locally")
    p_serve.add_argument("--port", type=int, default=0, help="port to bind, default chooses a free port")
    p_serve.set_defaults(func=serve_site)

    # Backwards-compatible alias for old notes/scripts.
    p_workbench = sub.add_parser("serve-modelica-workbench", help="alias for serve-site")
    p_workbench.add_argument("--port", type=int, default=0)
    p_workbench.add_argument("--model", default=str(DEFAULT_MODEL), help="accepted for compatibility; not republished automatically")
    p_workbench.set_defaults(func=serve_site)

    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
