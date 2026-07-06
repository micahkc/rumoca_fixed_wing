# rumoca_fixed_wing

**Live site:** [https://micahkc.github.io/rumoca_fixed_wing/](https://micahkc.github.io/rumoca_fixed_wing/)

A browser-based fixed-wing Modelica workbench built on [Rumoca](https://github.com/CogniPilot/rumoca) WASM. It lets you:

- Compare measured Sport Cub flight data against an editable Modelica flight model, with in-browser prediction.
- Fly the fixed-wing model interactively with keyboard input.
- Run a lockstep Modelica waypoint autopilot against the plant model, entirely in the browser.

Everything runs client-side as a static site — Modelica compilation and simulation happen in WASM, so no backend is required.

## Repository Layout

- `site/`: the static browser workbench (HTML/JS, served as-is).
- `site/public/data/`: committed playback, flight explorer, method trace, and measured comparison data used by the GUI.
- `site/public/modelica/CubControl.mo`: editable default waypoint autopilot.
- `site/package.json`: pinned browser dependencies, including `@cognipilot/rumoca`.
- `site/public/vendor/rumoca/`: generated Rumoca browser bundle copied from the npm package.
- `site/src/fixedwing_ga_model.js`: embedded default fixed-wing Modelica model (generated, see below).
- `site/src/rumoca_flight.js`: browser-side Rumoca model, autopilot, and LSP helpers.
- `results/FixedWingPlantGA.mo`: current editable fixed-wing plant source.
- `results/sportCubHandTuned.mo`: hand-tuned reference plant source.
- `data/`: compact Sport Cub flight-data NPZ files used for measured-data comparison.
- `results.py`: helper CLI for serving, validating, and updating the site.

## Run Locally

Requires Python 3 and npm. Install the pinned browser dependencies once:

```bash
cd site
npm ci
cd ..
```

```bash
./results.py serve-site
```

Open the printed local URL. The workbench has three tabs:

1. **Flight data** — plays back measured aircraft data; use `Predict here` to run the compiled Modelica model from any point and compare against the measurement.
2. **Keyboard flight** — flies the fixed-wing model directly with keyboard input.
3. **Autopilot** — compiles `CubControl.mo` and the fixed-wing plant, then runs them in lockstep through waypoints.

## Validate the Static Bundle

```bash
./results.py check-site
```

Checks that the static data bundle and Modelica/Rumoca browser wiring are present. This also runs automatically before `serve-site`.

## Update Rumoca Browser Files

```bash
cd site
npm ci
cd ..
./results.py vendor-rumoca
```

The generated `site/public/vendor/rumoca/` directory is copied from the pinned `@cognipilot/rumoca` npm dependency and is not committed.

## Update the Bundled Fixed-Wing Model

```bash
./results.py publish-model results/FixedWingPlantGA.mo
```

Regenerates `site/src/fixedwing_ga_model.js` from a Modelica source file. The browser compiles edited Modelica in-place at runtime, so this is only needed to change the default model shipped with the site.

## Deployment

The site deploys from `site/` via GitHub Actions to GitHub Pages. In the repo settings, set Pages source to **GitHub Actions**; each push to `main` publishes to the URL above.

## License

Apache License 2.0 — see [LICENSE](LICENSE).
