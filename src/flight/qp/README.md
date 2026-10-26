# OSQP WebAssembly QP kernel

This directory contains the ESM interface and the prebuilt Emscripten module for the official OSQP C solver. The generated module and its `.wasm` file run in Node and browsers through WebAssembly; the runtime does not need Emscripten, WASI, Python, or an npm package.

```js
import { createQP, initQP } from './qp/index.js';

await initQP();
const qp = createQP({ n, m, Pp, Pi, Ap, Ai, settings });
const result = qp.solve({ Px, Ax, q, l, u, warmStart });
qp.dispose();
```

`P` and `A` use CSC storage. `P` must contain only its upper triangle; `Pp` and `Ap` have `n + 1` Int32 entries, and `Pi`/`Ai` are Int32 row indices. Matrix values, `q`, and bounds are Float64. Bounds may use infinities. `warmStart` may be an `n` element primal vector or `{ x, y }` with primal and dual vectors.

The first `solve` calls `osqp_setup` using that solve's actual matrix, vector, and bound values, so OSQP scaling is based on the initial problem. Later calls keep the same solver workspace and use OSQP's matrix and vector update APIs; CSC structure cannot change. Reusing a QP instance avoids repeated setup and symbolic factorization. The current trajectory optimizer reuses it across SQP iterations within one planning call and releases it afterward; it does not yet cache QP workspaces across control ticks. Each solve currently copies the provided vectors into WASM linear memory, and OSQP still performs its numerical factorization/update work as required by changed matrix values.

`status` is OSQP's status string, `statusVal` its integer enum, and residuals/iteration count come from OSQP's solver info. Call and await `initQP()` once before the named synchronous `createQP()` export. Repeated `initQP()` calls share the same asynchronous module load.

## Rebuild

Maintainers need CMake, Ninja or Make, and Emscripten SDK 3.1.70. The solver and QDLDL source trees are vendored under `vendor/osqp/`; the build does not fetch dependencies. Set `EMSDK_ROOT` to the SDK checkout and run:

```sh
EMSDK_ROOT=/path/to/emsdk bash tools/solver/build.sh
```

The script compiles only the C solver and bridge, writing refreshed `osqp_bindings.mjs` and `osqp_bindings.wasm` files here. OSQP and QDLDL versions and source archive hashes are recorded in `vendor/osqp/SOURCE.txt` and `vendor/osqp/QDLDL_SOURCE.txt`; each upstream license is retained beside its source.
