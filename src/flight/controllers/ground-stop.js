// Initial driving controller: full service brake with existing actuator lag and ABS.
// No throttle, steering manoeuvre, trajectory optimization or RCS compensation.
import { NA, DRIVE_IDX } from '../contract.js';

export function createGroundStop() {
  let stopSpeed = 0.5;
  return {
    name: 'ground-stop', stopped: false,
    reset({ task } = {}) {
      stopSpeed = task?.success?.stop_speed ?? 0.5;
      this.stopped = false;
    },
    act(s) {
      const out = new Float64Array(NA);
      if (!s.armed) return out;
      this.stopped = Math.hypot(s.v[0], s.v[2]) <= stopSpeed;
      out[DRIVE_IDX.brake] = 1; // 100% means full wheel braking, never RCS.
      return out; // steer and every valve remain zero, also while holding at rest.
    },
  };
}
