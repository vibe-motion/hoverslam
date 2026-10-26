// Landing mission supervisor, separate from both flight planning and ground control.
// Touchdown selects driving; bounce cannot restart flight without an upper-level request.
export class LandingModes {
  constructor({ onChange, onDeparture } = {}) {
    this.onChange = onChange; this.onDeparture = onDeparture; this.reset();
  }
  reset() {
    this.mode = 'flight'; this.previousPhase = null;
    this.transitions = 0; this.departures = 0;
  }
  setMode(mode) {
    if (mode !== 'flight' && mode !== 'ground') throw new Error(`Unknown control mode: ${mode}`);
    if (mode === this.mode) return;
    this.mode = mode; this.transitions++;
    this.onChange?.(mode);
  }
  observe(s) {
    // The soft-body rig starts calling act only after arming, so its first sample can already be airborne.
    const departed = s.phase === 'air' && this.previousPhase !== 'air';
    if (departed && this.mode === 'flight') {
      this.departures++;
      this.onDeparture?.(); // one event, not a per-flight-tick actuator reset
    }
    if (s.armed && s.phase === 'ground' && this.previousPhase !== 'ground') this.setMode('ground');
    this.previousPhase = s.phase;
    return departed;
  }
}
