// Smooth, contact-free 6-DOF prediction model for the two-level optimizer. This is a planning approximation, not a
// plant replacement: it uses a 0.01 s (or smaller) semi-implicit translational / midpoint angular substep, omits
// contact, arming/phase events and damage,
// and intentionally sets the propulsion command deadband to zero so closed-valve commands retain useful gradients.
// Valve lag, regulator/blowdown and nozzle physics still come from the reference Propulsion and MassModel classes.
import { WORLD, VEHICLE, PROPULSION, GENERATED } from '../../core/spec.js';
import { NU as CONTRACT_NU } from '../contract.js';
import { Propulsion } from '../propulsion.js';
import { MassModel } from '../massprops.js';

export const NX = 24;
export const ND = 23;
export const NU = CONTRACT_NU;

const G = WORLD.gravity, RHO = WORLD.air.density, CDA = VEHICLE.aero.CdA;
const HMAX = 0.01;
const normQ = q => {
  const n = Math.hypot(q[0], q[1], q[2], q[3]);
  if (!(n > 0) || !Number.isFinite(n)) { q[0] = q[1] = q[2] = 0; q[3] = 1; }
  else for (let i = 0; i < 4; i++) q[i] /= n;
  return q;
};
function quatMul(a, b, o) {
  const x=a[0], y=a[1], z=a[2], w=a[3], X=b[0], Y=b[1], Z=b[2], W=b[3];
  o[0]=w*X+x*W+y*Z-z*Y; o[1]=w*Y-x*Z+y*W+z*X; o[2]=w*Z+x*Y-y*X+z*W; o[3]=w*W-x*X-y*Y-z*Z;
  return o;
}
function quatMatrix(q, r) {
  const x=q[0], y=q[1], z=q[2], w=q[3];
  r[0]=1-2*(y*y+z*z); r[1]=2*(x*y-z*w); r[2]=2*(x*z+y*w);
  r[3]=2*(x*y+z*w); r[4]=1-2*(x*x+z*z); r[5]=2*(y*z-x*w);
  r[6]=2*(x*z-y*w); r[7]=2*(y*z+x*w); r[8]=1-2*(x*x+y*y);
  return r;
}
function deltaQuat(d, o) {
  const a=Math.hypot(d[0],d[1],d[2]);
  if (a < 1e-12) { o[0]=0.5*d[0]; o[1]=0.5*d[1]; o[2]=0.5*d[2]; o[3]=1; }
  else { const k=Math.sin(0.5*a)/a; o[0]=k*d[0]; o[1]=k*d[1]; o[2]=k*d[2]; o[3]=Math.cos(0.5*a); }
  return o;
}

/**
 * One-stage smooth flight predictor. State packing is p,v,q[x,y,z,w],body omega, gas mass, valve openings.
 * Attitude perturbations use a right-multiplied quaternion and therefore a body-frame tangent vector.
 */
export class FlightModel {
  constructor() {
    // A private shallow spec copy removes the plant's command deadband without changing shared spec or plant physics.
    const smoothSpec = { ...PROPULSION, valve: { ...PROPULSION.valve, deadband: 0 } };
    this.prop = new Propulsion(smoothSpec, WORLD.air.pressure);
    this.mass = new MassModel(PROPULSION, GENERATED.massprops);
    this._mp = this._massOut(); this._mp2 = this._massOut();
    this._dq = new Float64Array(4); this._R = new Float64Array(9);
  }

  _massOut() { return { M: 0, com: new Float64Array(3), I: new Float64Array(6), Iinv: new Float64Array(6) }; }
  pack(s) {
    const x = new Float64Array(NX), q = s.q;
    x.set(s.p,0); x.set(s.v,3); x.set(q,6); x.set(s.w,10);
    x[13] = Math.max(0, +s.tank.m || 0);
    for (let i=0;i<NU;i++) x[14+i] = s.valve?.[i] ?? 0;
    normQ(x.subarray(6,10));
    return x;
  }
  state(x, t = 0) {
    const mg=Math.max(0,x[13]), mp=this.mass.at(mg,this._mp), tank=this._tank(mg);
    return { t, p: Float64Array.from(x.subarray(0,3)), v: Float64Array.from(x.subarray(3,6)),
      q: normQ(Float64Array.from(x.subarray(6,10))), w: Float64Array.from(x.subarray(10,13)),
      comB: Float64Array.from(mp.com), tank: { m: mg, T: tank.T, p: tank.p },
      valve: Float64Array.from(x.subarray(14,24)), armed: true, phase: 'air' };
  }
  _tank(m) {
    const rho=m/this.prop.V, p=this.prop.K*rho**this.prop.spec.tank.polytropic_n;
    return { p, T: rho>0 ? p/(rho*this.prop.R) : this.prop.spec.tank.T0 };
  }

  step(x, u, dt, out = new Float64Array(NX), forcedSubsteps = 0) {
    out.set(x);
    if (!(dt > 0)) { normQ(out.subarray(6,10)); return out; }
    const n=forcedSubsteps || Math.max(1,Math.ceil(dt/HMAX)), h=dt/n;
    const p=out.subarray(0,3), v=out.subarray(3,6), q=out.subarray(6,10), w=out.subarray(10,13), valve=out.subarray(14,24);
    let gas=Math.max(0,out[13]);
    const prop=this.prop, nozzle=PROPULSION.nozzles, R=this._R;
    const thrustBody=new Float64Array(3), torqueBody=new Float64Array(3), drag=new Float64Array(3), accW=new Float64Array(3), comOld=new Float64Array(3), cmd=new Float64Array(NU);
    for (let k=0;k<n;k++) {
      const mp=this.mass.at(gas,this._mp); comOld.set(mp.com);
      prop.m=gas;
      const tank=this._tank(gas); prop.p=tank.p; prop.T=tank.T; prop.valve.set(valve);
      for(let i=0;i<NU;i++) { const z=Number.isFinite(+u[i])?+u[i]:0; cmd[i]=Math.max(0,Math.min(1,z)); }
      prop.step(cmd,h); valve.set(prop.valve);
      const Rb=quatMatrix(q,R); thrustBody.fill(0); torqueBody.fill(0);
      for(let i=0;i<NU;i++) {
        const z=nozzle[i], e=z.exhaust, el=Math.hypot(e[0],e[1],e[2]), f=prop.thrust[i];
        const fx=-e[0]/el*f, fy=-e[1]/el*f, fz=-e[2]/el*f;
        thrustBody[0]+=fx; thrustBody[1]+=fy; thrustBody[2]+=fz;
        const rx=z.exit[0]-mp.com[0], ry=z.exit[1]-mp.com[1], rz=z.exit[2]-mp.com[2];
        torqueBody[0]+=ry*fz-rz*fy; torqueBody[1]+=rz*fx-rx*fz; torqueBody[2]+=rx*fy-ry*fx;
      }
      const vbx=Rb[0]*v[0]+Rb[3]*v[1]+Rb[6]*v[2], vby=Rb[1]*v[0]+Rb[4]*v[1]+Rb[7]*v[2], vbz=Rb[2]*v[0]+Rb[5]*v[1]+Rb[8]*v[2];
      drag[0]=-0.5*RHO*CDA[0]*Math.abs(vbx)*vbx; drag[1]=-0.5*RHO*CDA[1]*Math.abs(vby)*vby; drag[2]=-0.5*RHO*CDA[2]*Math.abs(vbz)*vbz;
      thrustBody[0]+=drag[0]; thrustBody[1]+=drag[1]; thrustBody[2]+=drag[2];
      const fx=Rb[0]*thrustBody[0]+Rb[1]*thrustBody[1]+Rb[2]*thrustBody[2];
      const fy=Rb[3]*thrustBody[0]+Rb[4]*thrustBody[1]+Rb[5]*thrustBody[2];
      const fz=Rb[6]*thrustBody[0]+Rb[7]*thrustBody[1]+Rb[8]*thrustBody[2];
      v[0]+=h*fx/mp.M; v[1]+=h*(fy/mp.M-G); v[2]+=h*fz/mp.M;
      p[0]+=h*v[0]; p[1]+=h*v[1]; p[2]+=h*v[2];
      this._angularAccel(w,mp.I,mp.Iinv,torqueBody,accW);
      const wm0=w[0]+0.5*h*accW[0], wm1=w[1]+0.5*h*accW[1], wm2=w[2]+0.5*h*accW[2];
      this._angularAccel([wm0,wm1,wm2],mp.I,mp.Iinv,torqueBody,accW);
      w[0]+=h*accW[0]; w[1]+=h*accW[1]; w[2]+=h*accW[2];
      const qx=q[0],qy=q[1],qz=q[2],qw=q[3],hh=0.5*h,wx=w[0],wy=w[1],wz=w[2];
      q[0]=qx+hh*(qw*wx+qy*wz-qz*wy); q[1]=qy+hh*(qw*wy+qz*wx-qx*wz);
      q[2]=qz+hh*(qw*wz+qx*wy-qy*wx); q[3]=qw+hh*(-qx*wx-qy*wy-qz*wz); normQ(q);
      const nextGas=Math.max(0,prop.m), mp2=this.mass.at(nextGas,this._mp2);
      if(nextGas!==gas) {
        quatMatrix(q,Rb); const dx=mp2.com[0]-comOld[0],dy=mp2.com[1]-comOld[1],dz=mp2.com[2]-comOld[2];
        p[0]+=Rb[0]*dx+Rb[1]*dy+Rb[2]*dz; p[1]+=Rb[3]*dx+Rb[4]*dy+Rb[5]*dz; p[2]+=Rb[6]*dx+Rb[7]*dy+Rb[8]*dz;
        const cx=w[1]*dz-w[2]*dy,cy=w[2]*dx-w[0]*dz,cz=w[0]*dy-w[1]*dx;
        v[0]+=Rb[0]*cx+Rb[1]*cy+Rb[2]*cz; v[1]+=Rb[3]*cx+Rb[4]*cy+Rb[5]*cz; v[2]+=Rb[6]*cx+Rb[7]*cy+Rb[8]*cz;
      }
      gas=nextGas;
    }
    out[13]=gas; normQ(q); return out;
  }
  _angularAccel(w,I,Ii,tau,o) {
    const lx=I[0]*w[0]+I[3]*w[1]+I[4]*w[2], ly=I[3]*w[0]+I[1]*w[1]+I[5]*w[2], lz=I[4]*w[0]+I[5]*w[1]+I[2]*w[2];
    const gx=tau[0]-(w[1]*lz-w[2]*ly), gy=tau[1]-(w[2]*lx-w[0]*lz), gz=tau[2]-(w[0]*ly-w[1]*lx);
    o[0]=Ii[0]*gx+Ii[3]*gy+Ii[4]*gz; o[1]=Ii[3]*gx+Ii[1]*gy+Ii[5]*gz; o[2]=Ii[4]*gx+Ii[5]*gy+Ii[2]*gz; return o;
  }
  plus(x, d, out = new Float64Array(NX)) {
    out.set(x);
    for(let i=0;i<3;i++){out[i]+=d[i];out[3+i]+=d[3+i];out[10+i]+=d[9+i];}
    out[13]=Math.max(0,out[13]+d[12]); for(let i=0;i<NU;i++) out[14+i]+=d[13+i];
    // Right/body boxplus: q' = q ⊗ Exp(delta_theta_body).
    deltaQuat(d.subarray(6,9),this._dq); quatMul(x.subarray(6,10),this._dq,out.subarray(6,10)); normQ(out.subarray(6,10)); return out;
  }
  minus(x, reference, out = new Float64Array(ND)) {
    for(let i=0;i<3;i++){out[i]=x[i]-reference[i];out[3+i]=x[3+i]-reference[3+i];out[9+i]=x[10+i]-reference[10+i];}
    out[12]=x[13]-reference[13]; for(let i=0;i<NU;i++) out[13+i]=x[14+i]-reference[14+i];
    // Right/body boxminus: delta_theta = Log(q_reference^-1 ⊗ q), with the short quaternion representative.
    const a=reference.subarray(6,10), b=x.subarray(6,10), ax=-a[0],ay=-a[1],az=-a[2],aw=a[3];
    const rx=aw*b[0]+ax*b[3]+ay*b[2]-az*b[1], ry=aw*b[1]-ax*b[2]+ay*b[3]+az*b[0];
    const rz=aw*b[2]+ax*b[1]-ay*b[0]+az*b[3], rw=aw*b[3]-ax*b[0]-ay*b[1]-az*b[2];
    const sign=rw<0?-1:1, n=Math.hypot(rx,ry,rz), angle=2*Math.atan2(n,Math.abs(rw));
    const k=n>1e-12?sign*angle/n:2*sign;
    out[6]=k*rx;out[7]=k*ry;out[8]=k*rz; return out;
  }
  linearize(x,u,dt) {
    const n=Math.max(1,Math.ceil(dt/HMAX)), next=this.step(x,u,dt,new Float64Array(NX),n), A=new Float64Array(ND*ND), B=new Float64Array(ND*NU), D=new Float64Array(ND);
    const h=1e-5, dx=new Float64Array(ND), xp=new Float64Array(NX), xm=new Float64Array(NX), yp=new Float64Array(NX), ym=new Float64Array(NX);
    for(let j=0;j<ND;j++) {
      let hp=h, hm=h;
      if(j===12&&x[13]<=h) hm=0;
      if(j>=13) { const valve=x[14+j-13]; if(valve<=h) hm=0; else if(valve>=1-h) hp=0; }
      dx.fill(0); dx[j]=hp; if(hp) this.plus(x,dx,xp); else xp.set(x);
      dx[j]=-hm; if(hm) this.plus(x,dx,xm); else xm.set(x);
      if(hp) this.step(xp,u,dt,yp,n); else yp.set(next);
      if(hm) this.step(xm,u,dt,ym,n); else ym.set(next);
      const dp=this.minus(yp,next), dm=this.minus(ym,next), den=hp+hm;
      for(let i=0;i<ND;i++) A[i*ND+j]=den?(dp[i]-dm[i])/den:0;
    }
    const up=Float64Array.from(u), um=Float64Array.from(u);
    for(let j=0;j<NU;j++) {
      let hp=h, hm=h; if(u[j]<=h) hm=0; else if(u[j]>=1-h) hp=0;
      up[j]=u[j]+hp; um[j]=u[j]-hm;
      if(hp) this.step(x,up,dt,yp,n); else yp.set(next);
      if(hm) this.step(x,um,dt,ym,n); else ym.set(next);
      const dp=this.minus(yp,next), dm=this.minus(ym,next), den=hp+hm;
      for(let i=0;i<ND;i++) B[i*NU+j]=den?(dp[i]-dm[i])/den:0;
      up[j]=u[j]; um[j]=u[j];
    }
    const hd=Math.min(1e-5,Math.max(1e-7,dt*1e-3));
    this.step(x,u,dt+hd,yp,n); this.step(x,u,Math.max(0,dt-hd),ym,n);
    const den=(dt+hd)-Math.max(0,dt-hd), dp=this.minus(yp,next), dm=this.minus(ym,next);
    for(let i=0;i<ND;i++) D[i]=(dp[i]-dm[i])/den;
    return { next, A, B, D };
  }
}
