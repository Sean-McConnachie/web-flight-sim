# henri-flight-sim

A flight simulator of the Messerschmitt Me-262 A-1a. The flight model builds the
total force and moment from separate components, such as each lifting surface
and each engine. The physics runs at a fixed step and holds no browser code. The
same model therefore runs in the browser and in a Node test harness.

## Run it

```sh
npm install        # install the dependencies, once
npm run dev        # start the Vite dev server
npm run test:unit  # run the unit tests
npm run test:flight  # run the flight tests against the validation targets
npm run lint:ste   # check the prose of every document
```

`npm run typecheck` runs the TypeScript compiler with no emit. `npm run build`
runs the type check and then builds the bundle.

## Running on a hybrid graphics laptop

A laptop with two GPUs can give Chrome one device for the shared images and the
other device for the import. The page then shows nothing. Start Chrome with all
of the work on one device:

```sh
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
google-chrome --use-angle=vulkan --enable-features=Vulkan \
  --enable-unsafe-webgpu --disable-accelerated-2d-canvas http://localhost:5173/
```

`--disable-accelerated-2d-canvas` is part of the same fix. Without it the 2D
canvas draws nothing on this path, and the runway markings do not appear.

## Coordinate frames

- Body frame: `+x` forward out of the nose, `+y` right, `+z` down. Positive lift
  is a negative z force.
- World frame: North-East-Down, with the origin at the runway threshold.
  Altitude above the ground is `-position.z`.
- Render frame: Three.js Y up. Only `src/render/frames.ts` converts between the
  world frame and the render frame.

## Documents

| Document | Content |
| --- | --- |
| `docs/CONVENTIONS.md` | the binding rules for all code and all prose |
| `docs/architecture.md` | module layers and the main loop |
| `docs/flight-model.md` | forces, moments, and the aerodynamic model |
| `docs/aircraft-me262.md` | every aircraft number, with its confidence mark |
| `docs/engine-jumo004.md` | the turbojet model and its limits |
| `docs/controls.md` | input devices, axis map, and key bindings |
| `docs/validation.md` | flight test targets and measured results |

Read `docs/CONVENTIONS.md` before you write any code or any prose in this
repository. It fixes the units, the coordinate frames, the module separation
rule, and the writing style. The `lint:ste` script checks the writing style of
every Markdown file in `docs/` and of this file.

## Status

Early. The documents under `docs/` are skeletons, and the flight test targets in
`docs/validation.md` all read "not measured".
