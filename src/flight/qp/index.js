import createBindings from './osqp_bindings.mjs';

let modulePromise;
let activeCreateQP;

const settingIds = new Map([
  ['verbose', 1], ['profiler_level', 2], ['warm_starting', 3], ['scaling', 4],
  ['polishing', 5], ['rho', 6], ['rho_is_vec', 7], ['sigma', 8], ['alpha', 9],
  ['cg_max_iter', 10], ['cg_tol_reduction', 11], ['cg_tol_fraction', 12],
  ['adaptive_rho', 13], ['adaptive_rho_interval', 14],
  ['adaptive_rho_fraction', 15], ['adaptive_rho_tolerance', 16], ['max_iter', 17],
  ['eps_abs', 18], ['eps_rel', 19], ['eps_prim_inf', 20], ['eps_dual_inf', 21],
  ['scaled_termination', 22], ['check_termination', 23], ['check_dualgap', 24],
  ['time_limit', 25], ['delta', 26], ['polish_refine_iter', 27],
]);

const statuses = new Map([
  [1, 'solved'], [2, 'solved inaccurate'], [3, 'primal infeasible'],
  [4, 'primal infeasible inaccurate'], [5, 'dual infeasible'],
  [6, 'dual infeasible inaccurate'], [7, 'maximum iterations reached'],
  [8, 'time limit reached'], [9, 'non convex'], [10, 'interrupted'], [11, 'unsolved'],
]);

function vector(value, name, Type, length) {
  if (value == null) throw new TypeError(`${name} is required`);
  if (!(value instanceof Type)) value = new Type(value);
  if (value.length !== length) throw new RangeError(`${name} must have length ${length}`);
  return value;
}

function finiteVector(value, name, length) {
  value = vector(value, name, Float64Array, length);
  for (let k = 0; k < value.length; k++) {
    if (!Number.isFinite(value[k])) throw new RangeError(`${name}[${k}] must be finite`);
  }
  return value;
}

function boundVector(value, name, length, lower) {
  value = vector(value, name, Float64Array, length);
  const out = Float64Array.from(value);
  for (let k = 0; k < out.length; k++) {
    if (lower && out[k] === -Infinity) out[k] = -1e30;
    else if (!lower && out[k] === Infinity) out[k] = 1e30;
    else if (!Number.isFinite(out[k])) throw new RangeError(`${name}[${k}] is invalid`);
  }
  return out;
}

function checkCsc(p, i, values, rows, cols, name, upper = false) {
  if (p.length !== cols + 1 || p[0] !== 0 || p[cols] !== i.length ||
      i.length !== values.length) throw new RangeError(`${name} has an invalid CSC shape`);
  let previous = 0;
  for (let col = 0; col < cols; col++) {
    const end = p[col + 1];
    if (p[col] !== previous || end < previous || end > i.length)
      throw new RangeError(`${name} column pointers must be monotonic CSC offsets`);
    for (let k = previous; k < end; k++) {
      if (i[k] < 0 || i[k] >= rows || (upper && i[k] > col))
        throw new RangeError(`${name} row index ${i[k]} is out of range`);
    }
    previous = end;
  }
}

function allocate(module, values, Type) {
  const bytes = Math.max(1, values.length) * Type.BYTES_PER_ELEMENT;
  const ptr = module._malloc(bytes);
  if (!ptr) throw new Error(`OSQP WASM could not allocate ${bytes} bytes`);
  if (values.length) {
    const heap = Type === Int32Array ? module.HEAP32 : module.HEAPF64;
    heap.set(values, ptr / Type.BYTES_PER_ELEMENT);
  }
  return ptr;
}

export async function initQP() {
  if (!modulePromise) {
    modulePromise = createBindings({
      locateFile: () => new URL('./osqp_bindings.wasm', import.meta.url).href,
    }).catch((error) => {
      modulePromise = undefined;
      throw error;
    });
  }
  const module = await modulePromise;

  function createQP({ n, m, Pp, Pi, Ap, Ai, settings = {} }) {
    if (!Number.isInteger(n) || n < 1 || !Number.isInteger(m) || m < 0)
      throw new RangeError('n must be positive and m must be non-negative integers');
    Pp = vector(Pp, 'Pp', Int32Array, n + 1);
    Pi = vector(Pi, 'Pi', Int32Array, Pi?.length);
    Ap = vector(Ap, 'Ap', Int32Array, n + 1);
    Ai = vector(Ai, 'Ai', Int32Array, Ai?.length);
    checkCsc(Pp, Pi, Pi, n, n, 'P', true);
    checkCsc(Ap, Ai, Ai, m, n, 'A');

    const keys = [];
    const values = [];
    for (const [name, value] of Object.entries(settings)) {
      const id = settingIds.get(name);
      if (!id) throw new TypeError(`Unsupported OSQP setting: ${name}`);
      if (typeof value !== 'number' && typeof value !== 'boolean')
        throw new TypeError(`OSQP setting ${name} must be numeric or boolean`);
      if (typeof value === 'number' && !Number.isFinite(value))
        throw new TypeError(`OSQP setting ${name} must be finite`);
      keys.push(id);
      values.push(Number(value));
    }

    const ids = new Int32Array(keys);
    const settingValues = new Float64Array(values);
    let handle = 0;

    let disposed = false;
    const ensureLive = () => {
      if (disposed) throw new Error('QP instance has been disposed');
    };
    return {
      solve({ Px, Ax, q, l, u, warmStart }) {
        ensureLive();
        Px = finiteVector(Px, 'Px', Pi.length);
        Ax = finiteVector(Ax, 'Ax', Ai.length);
        q = finiteVector(q, 'q', n);
        l = boundVector(l, 'l', m, true);
        u = boundVector(u, 'u', m, false);
        for (let k = 0; k < m; k++) {
          if (l[k] > u[k]) throw new RangeError(`l[${k}] must not exceed u[${k}]`);
        }
        let warmX;
        let warmY;
        if (warmStart instanceof Float64Array || Array.isArray(warmStart)) {
          warmX = vector(warmStart, 'warmStart', Float64Array, n);
        } else if (warmStart != null) {
          if (warmStart.x != null) warmX = vector(warmStart.x, 'warmStart.x', Float64Array, n);
          if (warmStart.y != null) warmY = vector(warmStart.y, 'warmStart.y', Float64Array, m);
        }
        const solvePtrs = [];
        const putSolve = (data) => {
          const ptr = allocate(module, data, Float64Array);
          solvePtrs.push(ptr);
          return ptr;
        };
        const xPtr = putSolve(new Float64Array(n));
        try {
          const pxPtr = putSolve(Px);
          const axPtr = putSolve(Ax);
          const qPtr = putSolve(q);
          const lPtr = putSolve(l);
          const uPtr = putSolve(u);
          const firstSolve = !handle;
          if (firstSolve) {
            const setupPtrs = [];
            const putSetup = (data, Type) => {
              const ptr = allocate(module, data, Type);
              setupPtrs.push(ptr);
              return ptr;
            };
            try {
              handle = module._qp_create(n, m, Pi.length, Ai.length,
                putSetup(Pp, Int32Array), putSetup(Pi, Int32Array), pxPtr,
                putSetup(Ap, Int32Array), putSetup(Ai, Int32Array), axPtr,
                qPtr, lPtr, uPtr, putSetup(ids, Int32Array),
                putSetup(settingValues, Float64Array), ids.length);
            } finally {
              for (const ptr of setupPtrs) module._free(ptr);
            }
            if (!handle) throw new Error('OSQP rejected the problem data or settings during setup');
          }
          const result = module._qp_solve(handle, firstSolve ? 0 : 1,
            pxPtr, axPtr, qPtr, lPtr, uPtr, warmX ? putSolve(warmX) : 0,
            warmY ? putSolve(warmY) : 0, xPtr);
          if (result !== 0) throw new Error(`OSQP update/solve failed with code ${result}`);
          const x = module.HEAPF64.slice(xPtr / Float64Array.BYTES_PER_ELEMENT,
            xPtr / Float64Array.BYTES_PER_ELEMENT + n);
          const statusVal = module._qp_status(handle);
          return {
            x,
            status: statuses.get(statusVal) ?? `unknown status ${statusVal}`,
            statusVal,
            iterations: module._qp_iterations(handle),
            primalResidual: module._qp_primal_residual(handle),
            dualResidual: module._qp_dual_residual(handle),
          };
        } finally {
          for (const ptr of solvePtrs) module._free(ptr);
        }
      },
      dispose() {
        if (!disposed) {
          if (handle) module._qp_destroy(handle);
          disposed = true;
        }
      },
    };
  }

  activeCreateQP = createQP;
  return { createQP };
}

export function createQP(options) {
  if (!activeCreateQP) throw new Error('Call and await initQP() before createQP()');
  return activeCreateQP(options);
}
