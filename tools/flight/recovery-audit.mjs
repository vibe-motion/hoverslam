// Audit actually recorded commands and current-model thrust margins; no simulation or controller modification.
// node tools/flight/recovery-audit.mjs input=<trajectory.json|recording.sim> [out=report.json]
import fs from 'node:fs';
import zlib from 'node:zlib';
import { parseArgs } from '../lib/args.mjs';
import { unpackRecording } from '../../src/softbody/recording.js';
import { Propulsion } from '../../src/flight/propulsion.js';
import { MassModel } from '../../src/flight/massprops.js';
import { PROPULSION, WORLD } from '../../src/core/spec.js';
const a = parseArgs();
if (!a.input) throw new Error('input=<trajectory.json|recording.sim> required');
let log, flight;
if (a.input.endsWith('.sim')) {
  const raw = zlib.gunzipSync(fs.readFileSync(a.input));
  const { meta } = unpackRecording(raw.buffer.slice(raw.byteOffset, raw.byteOffset + raw.length));
  log = meta.propulsion; flight = meta.flight;
} else { log = JSON.parse(fs.readFileSync(a.input, 'utf8')); flight = log; }
const limits = flight.commandLimits;
if (!limits?.sharedIgnition) throw new Error('recording has no shared ignition command limits');
const bottom = PROPULSION.nozzles.flatMap((n,i) => n.group === 'bottom' ? [i] : []);
const prop = new Propulsion(), mass = new MassModel();
const events = flight.events || [], armed = events.find(e => e.type === 'armed')?.t ?? 0;
const touch = events.find(e => e.type === 'touchdown')?.t ?? Infinity;
let violations = 0, burnRows = 0, coastRows = 0, switches = 0, previous = null;
let minSteadyTwr = Infinity, minActualTwr = Infinity, minAvailableTwr = Infinity;
let minGas = Infinity, maxGas = 0;
const intervals = [], restarts = [];
let lastShutdownTime = null;
for (let k=0;k<log.t.length;k++) {
  const t=log.t[k], u=log.u_cmd[k];
  if (t < armed - 1e-6 || t >= touch) continue;
  const on=u.some(v=>v>1e-7), mg=log.tank[k][0];
  const valid = on ? bottom.every(i=>u[i]>=limits.minThrottle-1e-7) && u.every(v=>v>=0 && v<=1+1e-7) : u.every(v=>Math.abs(v)<=1e-7);
  if (!valid) violations++;
  if (on) burnRows++; else coastRows++;
  if (!on && previous === true) lastShutdownTime = t;
  if (on && previous === false && lastShutdownTime !== null) restarts.push({ t, offSeconds: t-lastShutdownTime });
  if (on !== previous) { if(previous!==null || on) switches++; intervals.push({t,mode:on?'burn':'coast'}); previous=on; }
  minGas=Math.min(minGas,mg); maxGas=Math.max(maxGas,mg);
  prop.m=mg; prop.T=log.tank[k][1]; prop.p=log.tank[k][2];
  const weight=(log.kit ? log.kit.total0-log.kit.gas0+mg : mass.at(mg).M)*WORLD.gravity;
  const floor=new Float64Array(prop.n); for(const i of bottom) floor[i]=limits.minThrottle;
  prop.valve.set(floor); prop.step(floor,0);
  minAvailableTwr=Math.min(minAvailableTwr,bottom.reduce((s,i)=>s+prop.thrust[i],0)/weight);
  if(on) {
    prop.valve.set(u); prop.step(u,0);
    minSteadyTwr=Math.min(minSteadyTwr,bottom.reduce((s,i)=>s+prop.thrust[i],0)/weight);
    minActualTwr=Math.min(minActualTwr,bottom.reduce((s,i)=>s+log.thrust[k][i],0)/weight);
  }
}
const report={input:a.input,limits,metrics:flight.metrics,commandViolations:violations,burnRows,coastRows,switches,intervals,restarts,
  airborneWindowSeconds:touch-armed, switchingRateHz:Number.isFinite(touch)?switches/(touch-armed):null,
  gasRangeKg:[minGas,maxGas],minimumAvailableBottomSteadyTwr:minAvailableTwr,minimumCommandedBottomSteadyTwr:minSteadyTwr,
  minimumMeasuredBottomTwrDuringBurn:minActualTwr,
  passes:burnRows>0&&violations===0&&minAvailableTwr>1,
  interpretation:'Steady thrust uses recorded gas state and original propulsion model. Measured thrust includes ignition lag and need not exceed weight during transitions. TWR uses bottom thrust magnitude, not world-vertical projection.'};
if(a.out) fs.writeFileSync(a.out,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report,null,2));
