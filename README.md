Website: [https://micahkc.github.io/rumoca_fixed_wing/](https://micahkc.github.io/rumoca_fixed_wing/)

# rumoca_fixed_wing

Minimal Rumoca fixed-wing Modelica workbench.

This repo is focused on the browser GUI for comparing measured Sport Cub flight data against editable Modelica models, flying the fixed-wing model with keyboard input, and running a lockstep Modelica waypoint autopilot through Rumoca WASM.

## What Is Included

- `site/`: the static browser workbench.
- `site/public/data/`: committed playback, flight explorer, method trace, and measured comparison data used by the GUI.
- `site/public/modelica/CubControl.mo`: editable default waypoint autopilot.
- `site/public/wasm/`: local Rumoca WASM/LSP bundle used by the browser.
- `site/src/fixedwing_ga_model.js`: embedded default fixed-wing Modelica model for the browser.
- `site/src/rumoca_flight.js`: browser-side Rumoca model, autopilot, and LSP helpers.
- `results/FixedWingPlantGA.mo`: current editable fixed-wing plant source.
- `results/sportCubHandTuned.mo`: hand-tuned reference plant source.
- `data/`: compact Sport Cub NPZ files retained for measured-data comparison/regeneration work.


## GitHub Pages

The workbench deploys from `site/` with GitHub Actions. In the GitHub repo settings, set Pages to **GitHub Actions** as the source. After the workflow runs, the site is available at:

```text
https://micahkc.github.io/rumoca_fixed_wing/
```

## Run The GUI

```bash
./results.py serve-site
```

Open the printed local URL. The first tab plays measured aircraft data and can run `Predict here` with the compiled Modelica model. The keyboard tab flies the fixed-wing model directly. The autopilot tab compiles `CubControl.mo` and the fixed-wing plant, then runs them lockstep.

## Validate The Static Bundle

```bash
./results.py check-site
```

This checks that the static data bundle and Modelica/Rumoca browser wiring are present.

## Publish A New Fixed-Wing Model To The Browser

```bash
./results.py publish-model results/FixedWingPlantGA.mo
```

This updates `site/src/fixedwing_ga_model.js` from a Modelica source file. The browser still compiles edited Modelica in-place, so this is only needed when changing the default bundled model.

## Project Scope

This is no longer the full `mocap_sysid` benchmark repository. It intentionally does not include the broad method plugin framework, synthetic 6DOF benchmark suite, paper-generation assets, or GA/system-identification training machinery. The repo should stay centered on the GUI, data comparison, Modelica editing, Rumoca WASM execution, keyboard flight, and lockstep autopilot testing.
