"""Validate the static Rumoca fixed-wing workbench bundle."""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
DATA = ROOT / "public" / "data"
RUMOCA_VENDOR = ROOT / "public" / "vendor" / "rumoca"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise SystemExit(message)


def load_json(name: str):
    path = DATA / name
    require(path.exists(), f"missing site data file: {path}")
    return json.loads(path.read_text())


def main() -> int:
    manifest = load_json("manifest.json")
    method_results = load_json("method_results.json")
    load_json("maneuver_summary.json")
    playback = load_json("playback.json")
    method_traces = load_json("method_traces.json")
    index = (ROOT / "index.html").read_text()
    app = (ROOT / "src" / "app.js").read_text()
    worker = (ROOT / "src" / "modelica_prediction_worker.js").read_text()

    for selector in [
        "scenario-select",
        "model-tabs",
        "leaderboard-body",
        "dataset-body",
        "aircraft-playback",
        "playback-status",
        "playback-segment",
        "timeseries-plots",
        "timeseries-status",
        "modelica-flight-editor",
        "modelica-flight-source",
    ]:
        require(selector in index, f"missing site element #{selector}")

    scenarios = manifest.get("scenarios") or []
    datasets = manifest.get("dataset_registry") or []
    families = set(manifest.get("model_families") or [])
    require(scenarios, "site manifest has no scenarios")
    require(datasets, "site manifest has no datasets")
    require("aircraft6dof" in families, "site manifest must include the 6DOF family")
    require(any(dataset.get("source_type") == "synthetic_simulation" for dataset in datasets), "generated simulation datasets are missing")
    require(any(dataset.get("source_type") == "real_mocap" for dataset in datasets), "real datasets are missing")
    require(method_results, "site has no method results")
    require(playback, "site has no playback trajectories")
    require(isinstance(method_traces, list), "method traces bundle must be a list")
    require("three" in app.lower() and "renderPlayback" in app, "Three.js playback code is missing")
    require("renderTimeseries" in app and "selectedTraceSegments" in app, "time-history trace code is missing")
    require("monaco-editor" in index and "registerModelicaLanguage" in app, "Modelica Monaco editor wiring is missing")
    require("setModelMarkers" in app and "modelicaDiagnostics" in app, "Modelica diagnostic marker wiring is missing")
    require("registerCompletionItemProvider" in app and "registerHoverProvider" in app, "Modelica LSP provider wiring is missing")
    require("public/vendor/rumoca/rumoca_bind_wasm.js" in index, "Rumoca package vendor import map is missing")
    require("public/vendor/rumoca/rumoca_bind_wasm.js" in worker, "prediction worker must use vendored Rumoca package files")
    require("public/wasm" not in index and "public/wasm" not in worker, "site still references copied Rumoca wasm artifacts")
    for name in [
        "modelica_language.js",
        "parse_worker.js",
        "rumoca_bind_wasm.js",
        "rumoca_bind_wasm_bg.wasm",
        "rumoca_interactive.js",
        "rumoca_runtime.js",
        "rumoca_worker.js",
    ]:
        require((RUMOCA_VENDOR / name).exists(), f"missing vendored Rumoca package file: {name}")
    print(f"site ok: {len(scenarios)} scenarios, {len(datasets)} datasets, {len(method_results)} comparison rows")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
