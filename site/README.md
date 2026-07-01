# Rumoca Fixed-Wing Workbench Site

Static browser UI for measured-data playback/comparison, editable Modelica fixed-wing models, keyboard flight, and lockstep waypoint autopilot simulation with Rumoca WASM.

## Serve Locally

```bash
./results.py serve-site
```

## Check The Bundle

```bash
./results.py check-site
```

## Important Files

- `index.html`: workbench shell.
- `src/app.js`: playback, Modelica editor, prediction, keyboard flight, and autopilot UI.
- `src/rumoca_flight.js`: Rumoca WASM/LSP integration.
- `src/modelica_prediction_worker.js`: background prediction worker.
- `public/data/`: measured playback/comparison payloads.
- `public/modelica/CubControl.mo`: default Modelica autopilot.
- `public/wasm/`: local Rumoca browser bundle.
