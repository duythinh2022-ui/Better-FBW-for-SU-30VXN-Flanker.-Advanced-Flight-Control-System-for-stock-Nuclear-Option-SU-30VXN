//! Su-30 FCS: the control allocator's solve and the surface grid kernels, native.
//!
//! A port of the F-22E Strike Raptor FCS 4.1.0 core (its `native/fcs_core`) to this jet's own `Allocator.cs` and
//! `SurfaceEffector.cs`, expression for expression. Same rules as the F-22E's (see that crate's header and native/README.md):
//! same operand types, order and float <-> double conversions as the C# source, no transcendental function except the
//! kernels' one `atan2`, no fused multiply-add, no reordering. The C# code stays the reference and the fallback.
//!
//! What this jet's allocator has that the F-22E's does not (all in both `lib.rs` and the plugin's `FcsNative.cs`):
//! * the velocity-frame lateral residual (Revoker 0.4.0: `slipFrame`, `slipSin`, `slipCos` in `Allocator.E`);
//! * the coordinate pass's Newton candidate also for the yaw nozzles (`CoordNewton == 1 && (TvcEffector || TvcYawEffector)`):
//!   `EffN.kind` 3 = TvcYawEffector;
//! * the table rebuild's normal force per grid point (`SurfaceEffector.FN`, `lastFN = (rootInv * fTot).y`): mode 0 of the
//!   grid kernel writes it to `GridIn.out_fn` when that is not null.

#![allow(non_snake_case)]
#![allow(clippy::needless_range_loop, clippy::too_many_arguments, clippy::excessive_precision)]

use core::ops::{Add, Sub};

/// Bumped whenever the layout or the meaning of the structures below changes. The plugin refuses any other value.
// Su-30 0.5.6: the coupled asymmetric-load check's LP (CoupledCap.cs), from the Ternion 0.3.0 core
pub mod cpl;
pub use cpl::*;

pub const ABI: i32 = 5005006;

pub const MAX_EFF: usize = 16;
pub const MAX_N: usize = 32;

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct V3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl V3 {
    pub const ZERO: V3 = V3 { x: 0.0, y: 0.0, z: 0.0 };

    // UnityEngine.Vector3: operator *(Vector3, float), operator /(Vector3, float), Dot, sqrMagnitude
    #[inline(always)]
    fn mul(self, d: f32) -> V3 {
        V3 { x: self.x * d, y: self.y * d, z: self.z * d }
    }
    #[inline(always)]
    fn div(self, d: f32) -> V3 {
        V3 { x: self.x / d, y: self.y / d, z: self.z / d }
    }
    #[inline(always)]
    fn dot(a: V3, b: V3) -> f32 {
        a.x * b.x + a.y * b.y + a.z * b.z
    }
    #[inline(always)]
    fn sqr(self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }
}

impl Add for V3 {
    type Output = V3;
    #[inline(always)]
    fn add(self, b: V3) -> V3 {
        V3 { x: self.x + b.x, y: self.y + b.y, z: self.z + b.z }
    }
}

impl Sub for V3 {
    type Output = V3;
    #[inline(always)]
    fn sub(self, b: V3) -> V3 {
        V3 { x: self.x - b.x, y: self.y - b.y, z: self.z - b.z }
    }
}

/// One effector, as `Allocator.Solve` reads it (640 bytes, every field 4 bytes wide).
#[repr(C)]
#[derive(Clone, Copy)]
pub struct EffN {
    pub active: i32,
    /// 1 = TvcEffector, 2 = SurfaceEffector of kind Stabilator, 3 = TvcYawEffector, 0 = any other effector
    pub kind: i32,
    /// index of `partner` in this array, -1 when it has none (or its partner is not in the list)
    pub partner: i32,
    pub roll_group: i32,
    pub pitch_group: i32,
    pub two_horizon: i32,
    pub n: i32,
    pub node_on: i32,
    pub node_k: i32,
    pub min: f32,
    pub max: f32,
    pub now: f32,
    pub cmd: f32,
    pub rate: f32,
    pub reach_tau: f32,
    pub mu: f32,
    pub lam: f32,
    pub lo: f32,
    pub hi: f32,
    pub pair_sign: f32,
    pub sym_cost: f32,
    pub sym_target: f32,
    pub anti_cost: f32,
    pub anchor: f32,
    pub band: f32,
    pub anchor_k: f32,
    /// Effector.RegN, evaluated by the plugin
    pub reg_n: f32,
    /// Effector.Step, evaluated by the plugin
    pub step: f32,
    pub node_c: f32,
    pub node_d: V3,
    pub grid: [f32; MAX_N],
    pub m: [V3; MAX_N],
}

/// The allocator's configuration, the target, and what a solve leaves behind (200 bytes).
#[repr(C)]
#[derive(Clone, Copy)]
pub struct AllocIo {
    /// Allocator.LmFactor
    pub lm_factor: f64,
    /// effs.Count
    pub n: i32,
    pub gn_iterations: i32,
    pub paired_trim_search: i32,
    pub fight_stab_split_only: i32,
    pub reach_pass: i32,
    pub two_horizon: i32,
    pub two_horizon_path: i32,
    pub coord_newton: i32,
    pub axis_w: V3,
    pub reg_scale: f32,
    pub fight_k: f32,
    pub fight_w: [f32; 4],
    pub pitch_fight_k: f32,
    pub pitch_fight_sign: f32,
    /// i00 i01 i02 i10 i11 i12 i20 i21 i22
    pub inv: [f32; 9],
    pub fight_target_min: f32,
    pub pair_horizon: f32,
    pub target: V3,
    /// in: the allocator's lastCost before the call; out: after it
    pub last_cost: f32,
    /// in / out: resDbg
    pub res_dbg: V3,
    /// in / out: lastNeededPick
    pub last_needed_pick: i32,
    /// out: how many times each counter was incremented by this solve
    pub reach_moves: i32,
    pub two_horizon_bound: i32,
    pub coord_newton_moves: i32,
    pub paired_moves: i32,
    /// out: SolveCore runs in this call (diagnostics)
    pub core_solves: i32,
    /// Allocator.slipFrame (0 / 1), slipSin, slipCos: the velocity-frame lateral residual
    pub slip_frame: i32,
    pub slip_sin: f32,
    pub slip_cos: f32,
    pub reserved: i32,
}

// ---- UnityEngine.Mathf, as the game's UnityEngine.CoreModule implements it
#[inline(always)]
fn maxf(a: f32, b: f32) -> f32 {
    if a > b {
        a
    } else {
        b
    }
}

#[inline(always)]
fn minf(a: f32, b: f32) -> f32 {
    if a < b {
        a
    } else {
        b
    }
}

#[inline(always)]
fn clampf(value: f32, min: f32, max: f32) -> f32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

#[inline(always)]
fn absf(f: f32) -> f32 {
    f.abs()
}

/// Mathf.Sqrt(f) = (float)Math.Sqrt((double)f)
#[inline(always)]
fn sqrtf(f: f32) -> f32 {
    (f as f64).sqrt() as f32
}

/// C# `(int)float` on x64 (cvttss2si): truncation, and 0x80000000 for NaN or out of range, where Rust's `as` would
/// saturate. An index built from that then fails its bounds check here just as it throws in C#.
#[inline(always)]
fn f2i(f: f32) -> i32 {
    #[cfg(target_arch = "x86_64")]
    unsafe {
        use core::arch::x86_64::{_mm_cvttss_si32, _mm_set_ss};
        _mm_cvttss_si32(_mm_set_ss(f))
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        if f.is_nan() || f >= 2147483648.0 || f < -2147483648.0 {
            i32::MIN
        } else {
            f as i32
        }
    }
}

#[inline(always)]
fn ix(i: i32) -> usize {
    // a negative index wraps to a huge one and fails the bounds check (C#: IndexOutOfRangeException)
    i as u32 as usize
}

/// Effector.MAt
#[inline(always)]
fn m_at(e: &EffN, c: f32) -> V3 {
    if e.n < 2 {
        return V3::ZERO;
    }
    let num = (c - e.min) / e.step;
    if num <= 0.0 {
        return e.m[0];
    }
    if num >= (e.n - 1) as f32 {
        return e.m[ix(e.n - 1)];
    }
    let num2 = f2i(num);
    let num3 = num - num2 as f32;
    let a = e.m[ix(num2)];
    let b = e.m[ix(num2 + 1)];
    let mut m = a + (b - a).mul(num3);
    if e.node_on != 0 && num2 == e.node_k {
        let g0 = e.min + num2 as f32 * e.step;
        let w = if c <= e.node_c {
            if e.node_c > g0 {
                (c - g0) / (e.node_c - g0)
            } else {
                1.0
            }
        } else if g0 + e.step > e.node_c {
            (g0 + e.step - c) / (g0 + e.step - e.node_c)
        } else {
            1.0
        };
        m = m + e.node_d.mul(w);
    }
    m
}

/// Allocator.BudgetExcess
#[inline(always)]
fn budget_excess(e: &EffN, c: f32) -> f32 {
    let d = c - e.anchor;
    if d > e.band {
        return d - e.band;
    }
    if d < 0.0 - e.band {
        return d + e.band;
    }
    0.0
}

/// Allocator.BudgetCost
#[inline(always)]
fn budget_cost(e: &EffN, c: f32) -> f32 {
    if e.anchor_k <= 0.0 {
        return 0.0;
    }
    let ex = budget_excess(e, c) / e.reg_n;
    e.anchor_k * ex * ex
}

const FIGHT_GROUPS: usize = 3;

struct Solver<'a> {
    io: &'a mut AllocIo,
    e: &'a mut [EffN],
    sw: V3,           // Mathf.Sqrt(axisWeight.*)
    sq_pfk: f64,      // Math.Sqrt(pitchFightK)
    sq_fk: [f64; 4],  // Math.Sqrt(fightK * fightW[g])
    hint: f64,        // fightSignHint
    cnt: usize,
    act: [usize; MAX_EFF],
    x: [f64; MAX_EFF],
    prev: [f64; MAX_EFF],
    dx: [f64; MAX_EFF],
    h: [[f64; MAX_EFF]; MAX_EFF],
    rhs: [f64; MAX_EFF],
    jc: [V3; MAX_EFF],
    pidx: [i32; MAX_EFF],
    r0: [f64; MAX_EFF],
    grp: [i32; MAX_EFF],
    jg: [f64; MAX_EFF],
    pgrp: [i32; MAX_EFF],
    y0: [f64; MAX_EFF],
    jp: [f64; MAX_EFF],
    pg_r: [f64; 3],
    pg_on: [bool; 3],
    gr_r: [f64; FIGHT_GROUPS + 1],
    gr_on: [bool; FIGHT_GROUPS + 1],
    // the reach pass
    rch_n: usize,
    rch_e: [usize; MAX_EFF],
    rch_dest: [f32; MAX_EFF],
    rch_lo: [f32; MAX_EFF],
    rch_hi: [f32; MAX_EFF],
    rch_cmd0: [f32; MAX_EFF],
}

impl<'a> Solver<'a> {
    fn new(io: &'a mut AllocIo, e: &'a mut [EffN]) -> Self {
        let sw = V3 { x: sqrtf(io.axis_w.x), y: sqrtf(io.axis_w.y), z: sqrtf(io.axis_w.z) };
        let sq_pfk = (io.pitch_fight_k as f64).sqrt();
        let mut sq_fk = [0.0f64; 4];
        for g in 0..4 {
            sq_fk[g] = ((io.fight_k * io.fight_w[g]) as f64).sqrt();
        }
        Solver {
            io,
            e,
            sw,
            sq_pfk,
            sq_fk,
            hint: 0.0,
            cnt: 0,
            act: [0; MAX_EFF],
            x: [0.0; MAX_EFF],
            prev: [0.0; MAX_EFF],
            dx: [0.0; MAX_EFF],
            h: [[0.0; MAX_EFF]; MAX_EFF],
            rhs: [0.0; MAX_EFF],
            jc: [V3::ZERO; MAX_EFF],
            pidx: [-1; MAX_EFF],
            r0: [0.0; MAX_EFF],
            grp: [0; MAX_EFF],
            jg: [0.0; MAX_EFF],
            pgrp: [0; MAX_EFF],
            y0: [0.0; MAX_EFF],
            jp: [0.0; MAX_EFF],
            pg_r: [0.0; 3],
            pg_on: [false; 3],
            gr_r: [0.0; FIGHT_GROUPS + 1],
            gr_on: [false; FIGHT_GROUPS + 1],
            rch_n: 0,
            rch_e: [0; MAX_EFF],
            rch_dest: [0.0; MAX_EFF],
            rch_lo: [0.0; MAX_EFF],
            rch_hi: [0.0; MAX_EFF],
            rch_cmd0: [0.0; MAX_EFF],
        }
    }

    #[inline(always)]
    fn ae(&self, i: usize) -> &EffN {
        &self.e[self.act[i]]
    }

    /// act[i].MAt(c)
    #[inline(always)]
    fn mat(&self, i: usize, c: f32) -> V3 {
        m_at(&self.e[self.act[i]], c)
    }

    /// Allocator.E (with the velocity-frame lateral residual)
    #[inline(always)]
    fn E(&self, m: V3) -> V3 {
        let i = &self.io.inv;
        let num = i[0] * m.x + i[1] * m.y + i[2] * m.z;
        let num2 = i[3] * m.x + i[4] * m.y + i[5] * m.z;
        let num3 = i[6] * m.x + i[7] * m.y + i[8] * m.z;
        if self.io.slip_frame != 0 {
            let e_v = num * self.io.slip_cos + num3 * self.io.slip_sin;
            let e_n = num * self.io.slip_sin - num3 * self.io.slip_cos;
            return V3 { x: e_v * self.sw.x, y: num2 * self.sw.y, z: e_n * self.sw.z };
        }
        V3 { x: num * self.sw.x, y: num2 * self.sw.y, z: num3 * self.sw.z }
    }

    /// Allocator.Cost
    #[inline(always)]
    fn cost(&self, err: V3) -> f32 {
        self.E(err).sqr()
    }

    /// Allocator.Reg
    #[inline(always)]
    fn reg(&self, e: &EffN, c: f32, p: f32) -> f32 {
        let num = e.reg_n;
        let num2 = c / num;
        let num3 = (c - p) / num;
        self.io.reg_scale * (e.mu * num2 * num2 + e.lam * num3 * num3) + budget_cost(e, c)
    }

    /// Allocator.PairReg
    #[inline(always)]
    fn pair_reg(&self, i: usize) -> f32 {
        let num = self.pidx[i];
        if num < 0 {
            return 0.0;
        }
        let num = num as usize;
        let effector = self.ae(i);
        let effector2 = self.ae(num);
        let num2 = effector.reg_n;
        let num3 = effector.pair_sign * (self.x[i] as f32) / num2;
        let num4 = effector2.pair_sign * (self.x[num] as f32) / num2;
        let num5 = 0.5f32 * (num3 - num4) - 0.5f32 * (effector.pair_sign - effector2.pair_sign) * effector.sym_target / num2;
        let num6 = 0.5f32 * (num3 + num4);
        self.io.reg_scale * (effector.sym_cost * num5 * num5 + effector.anti_cost * num6 * num6)
    }

    /// Allocator.PairRegWith
    #[inline(always)]
    fn pair_reg_with(&mut self, j: usize, c: f32) -> f32 {
        let num = self.x[j];
        self.x[j] = c as f64;
        let mut num2 = 0.0f32;
        for i in 0..self.cnt {
            if self.pidx[i] >= 0 && (i == j || self.pidx[i] as usize == j) {
                num2 += self.pair_reg(i);
            }
        }
        self.x[j] = num;
        num2
    }

    /// Allocator.StabSplitMode
    #[inline(always)]
    fn stab_split_mode(&self, i: usize) -> bool {
        self.io.fight_stab_split_only != 0 && self.grp[i] == 1 && self.pidx[i] >= 0 && self.grp[self.pidx[i] as usize] == 1
    }

    /// Allocator.GroupRoll
    fn group_roll(&self, g: i32) -> f64 {
        let i00 = self.io.inv[0] as f64;
        let mut s = 0.0f64;
        for i in 0..self.cnt {
            if self.grp[i] == g {
                if self.stab_split_mode(i) {
                    let j = self.pidx[i] as usize;
                    let sym = 0.5 * (self.ae(i).pair_sign as f64 * self.x[i] - self.ae(j).pair_sign as f64 * self.x[j]);
                    let xs = (self.ae(i).pair_sign as f64 * sym) as f32;
                    s += i00 * (self.mat(i, self.x[i] as f32).x as f64 - self.mat(i, xs).x as f64);
                    continue;
                }
                s += i00 * (self.mat(i, self.x[i] as f32).x as f64 - self.r0[i]);
            }
        }
        s
    }

    /// Allocator.NetSign
    fn net_sign(&self) -> f64 {
        if self.hint != 0.0 {
            return self.hint;
        }
        let mut t = 0.0f64;
        for g in 1..=FIGHT_GROUPS as i32 {
            t += self.group_roll(g);
        }
        if t >= 0.0 {
            1.0
        } else {
            -1.0
        }
    }

    /// Allocator.FightCost
    fn fight_cost(&self) -> f32 {
        if self.io.fight_k <= 0.0 {
            return 0.0;
        }
        let sg = self.net_sign();
        let mut t = 0.0f64;
        for g in 1..=FIGHT_GROUPS {
            let a = self.group_roll(g as i32);
            if a * sg < 0.0 {
                t += self.io.fight_w[g] as f64 * a * a;
            }
        }
        (self.io.fight_k as f64 * t) as f32
    }

    /// Allocator.FightCostWith
    fn fight_cost_with(&mut self, j: usize, c: f32) -> f32 {
        if self.io.fight_k <= 0.0 || self.grp[j] == 0 {
            return 0.0;
        }
        let i00 = self.io.inv[0] as f64;
        let d = i00 * (self.mat(j, c).x as f64 - self.mat(j, self.x[j] as f32).x as f64);
        let mut tot = 0.0f64;
        let split_j = self.stab_split_mode(j);
        let xj = self.x[j];
        if split_j {
            self.x[j] = c as f64;
        }
        for g in 1..=FIGHT_GROUPS {
            let v = self.group_roll(g as i32) + if g as i32 == self.grp[j] && !split_j { d } else { 0.0 };
            self.gr_r[g] = v;
            tot += self.gr_r[g];
        }
        self.x[j] = xj;
        let sg = if self.hint != 0.0 {
            self.hint
        } else if tot >= 0.0 {
            1.0
        } else {
            -1.0
        };
        let mut t = 0.0f64;
        for g in 1..=FIGHT_GROUPS {
            if self.gr_r[g] * sg < 0.0 {
                t += self.io.fight_w[g] as f64 * self.gr_r[g] * self.gr_r[g];
            }
        }
        (self.io.fight_k as f64 * t) as f32
    }

    /// Allocator.GroupPitch
    fn group_pitch(&self, g: i32) -> f64 {
        let i11 = self.io.inv[4] as f64;
        let mut s = 0.0f64;
        for i in 0..self.cnt {
            if self.pgrp[i] == g {
                s += i11 * (self.mat(i, self.x[i] as f32).y as f64 - self.y0[i]);
            }
        }
        s
    }

    /// Allocator.PFTerm
    #[inline(always)]
    fn pf_term(&self, a: f64, b: f64) -> f64 {
        let k = self.io.pitch_fight_sign as f64;
        if a * k <= 0.0 || b * k >= 0.0 {
            return 0.0;
        }
        let sg = if a + b >= 0.0 { 1.0 } else { -1.0 };
        (if a * sg < 0.0 { a * a } else { 0.0 }) + (if b * sg < 0.0 { b * b } else { 0.0 })
    }

    /// Allocator.PitchFightCost
    fn pitch_fight_cost(&self) -> f32 {
        if self.io.pitch_fight_k <= 0.0 {
            return 0.0;
        }
        (self.io.pitch_fight_k as f64 * self.pf_term(self.group_pitch(1), self.group_pitch(2))) as f32
    }

    /// Allocator.PFPair
    fn pf_pair(&mut self, s: usize, cs: f32, t: usize, ct: f32) -> f32 {
        if self.io.pitch_fight_k <= 0.0 {
            return 0.0;
        }
        let xs = self.x[s];
        let xt = self.x[t];
        self.x[s] = cs as f64;
        self.x[t] = ct as f64;
        let r = self.pitch_fight_cost();
        self.x[s] = xs;
        self.x[t] = xt;
        r
    }

    /// Allocator.PitchFightCostWith
    fn pitch_fight_cost_with(&self, j: usize, c: f32) -> f32 {
        if self.io.pitch_fight_k <= 0.0 || self.pgrp[j] == 0 {
            return 0.0;
        }
        let i11 = self.io.inv[4] as f64;
        let d = i11 * (self.mat(j, c).y as f64 - self.mat(j, self.x[j] as f32).y as f64);
        let a = self.group_pitch(1) + if self.pgrp[j] == 1 { d } else { 0.0 };
        let b = self.group_pitch(2) + if self.pgrp[j] == 2 { d } else { 0.0 };
        (self.io.pitch_fight_k as f64 * self.pf_term(a, b)) as f32
    }

    /// Allocator.Total
    fn total(&self) -> f32 {
        let mut zero = V3::ZERO;
        let mut num = 0.0f32;
        for i in 0..self.cnt {
            zero = zero + self.mat(i, self.x[i] as f32);
            num += self.reg(self.ae(i), self.x[i] as f32, self.prev[i] as f32) + self.pair_reg(i);
        }
        self.cost(zero - self.io.target) + num + self.fight_cost() + self.pitch_fight_cost()
    }

    /// Allocator.MomentSum
    fn moment_sum(&self) -> V3 {
        let mut t = V3::ZERO;
        for i in 0..self.cnt {
            t = t + self.mat(i, self.x[i] as f32);
        }
        t
    }

    /// the five-term cost of effector j at deflection c that the scan and the Newton candidate compare:
    /// Cost(m - vector4) + Reg + PairRegWith + FightCostWith + PitchFightCostWith
    #[inline(always)]
    fn cand(&mut self, j: usize, m: V3, vector4: V3, c: f32, p: f32) -> f32 {
        let a = self.cost(m - vector4);
        let b = self.reg(self.ae(j), c, p);
        let cc = self.pair_reg_with(j, c);
        let d = self.fight_cost_with(j, c);
        let ee = self.pitch_fight_cost_with(j, c);
        a + b + cc + d + ee
    }

    /// Allocator.SolveCore
    fn solve_core(&mut self) {
        self.io.core_solves += 1;
        let n_all = self.e.len();
        let mut count = 0usize;
        for i in 0..n_all {
            if self.e[i].active != 0 {
                self.act[count] = i;
                count += 1;
            }
        }
        self.cnt = count;
        if count == 0 {
            return;
        }
        for i in 0..count {
            let ef = self.ae(i);
            let xi = clampf(ef.cmd, ef.lo, ef.hi) as f64;
            let pi = ef.cmd as f64;
            self.x[i] = xi;
            self.prev[i] = pi;
        }
        let fight_k = self.io.fight_k;
        let pitch_fight_k = self.io.pitch_fight_k;
        for j in 0..count {
            let ef = *self.ae(j);
            // act.IndexOf(act[j].partner)
            let mut pi = -1i32;
            if ef.partner >= 0 {
                let p = ef.partner as usize;
                for k in 0..count {
                    if self.act[k] == p {
                        pi = k as i32;
                        break;
                    }
                }
            }
            self.pidx[j] = pi;
            self.grp[j] = if fight_k > 0.0 { ef.roll_group } else { 0 };
            self.r0[j] = m_at(&ef, 0.0).x as f64;
            self.pgrp[j] = if pitch_fight_k > 0.0 { ef.pitch_group } else { 0 };
            self.y0[j] = m_at(&ef, 0.0).y as f64;
        }
        let i00 = self.io.inv[0] as f64;
        let i11 = self.io.inv[4] as f64;
        let target = self.io.target;
        self.hint = 0.0;
        if fight_k > 0.0 && self.io.fight_target_min > 0.0 {
            let mut tr = i00 * target.x as f64;
            for jh in 0..count {
                tr -= i00 * self.r0[jh];
            }
            if (tr * 57.29578f64).abs() > self.io.fight_target_min as f64 {
                self.hint = if tr >= 0.0 { 1.0 } else { -1.0 };
            }
        }
        let reg_scale = self.io.reg_scale;
        let lm_factor = self.io.lm_factor;
        let mut num = self.total();
        for _k in 0..self.io.gn_iterations {
            let mut zero = V3::ZERO;
            for l in 0..count {
                zero = zero + self.mat(l, self.x[l] as f32);
            }
            let vector = self.E(zero - target);
            for m in 0..count {
                let effector = *self.ae(m);
                let num2 = 0.5f32 * effector.step;
                let num3 = maxf(effector.min, self.x[m] as f32 - num2);
                let num4 = minf(effector.max, self.x[m] as f32 + num2);
                self.jc[m] = if num4 > num3 {
                    self.E(m_at(&effector, num4) - m_at(&effector, num3)).div(num4 - num3)
                } else {
                    V3::ZERO
                };
                self.jp[m] = if self.pgrp[m] != 0 && num4 > num3 {
                    self.sq_pfk * i11 * ((m_at(&effector, num4).y - m_at(&effector, num3).y) as f64) / ((num4 - num3) as f64)
                } else {
                    0.0
                };
                self.jg[m] = if self.grp[m] != 0 && num4 > num3 {
                    self.sq_fk[ix(self.grp[m])] * i00 * ((m_at(&effector, num4).x - m_at(&effector, num3).x) as f64) / ((num4 - num3) as f64)
                } else {
                    0.0
                };
                if self.grp[m] != 0 && num4 > num3 && self.stab_split_mode(m) {
                    let xm = self.x[m];
                    self.x[m] = num4 as f64;
                    let g4 = self.group_roll(self.grp[m]);
                    self.x[m] = num3 as f64;
                    let g3 = self.group_roll(self.grp[m]);
                    self.x[m] = xm;
                    self.jg[m] = self.sq_fk[ix(self.grp[m])] * (g4 - g3) / ((num4 - num3) as f64);
                }
            }
            if pitch_fight_k > 0.0 {
                let pa = self.group_pitch(1);
                let pb = self.group_pitch(2);
                let sg_p = if pa + pb >= 0.0 { 1.0 } else { -1.0 };
                let pfs = self.io.pitch_fight_sign as f64;
                let pat = pa * pfs > 0.0 && pb * pfs < 0.0;
                self.pg_on[1] = pat && pa * sg_p < 0.0;
                self.pg_on[2] = pat && pb * sg_p < 0.0;
                self.pg_r[1] = if self.pg_on[1] { self.sq_pfk * pa } else { 0.0 };
                self.pg_r[2] = if self.pg_on[2] { self.sq_pfk * pb } else { 0.0 };
            } else {
                self.pg_on[1] = false;
                self.pg_on[2] = false;
            }
            if fight_k > 0.0 {
                let sg_f = self.net_sign();
                for g in 1..=FIGHT_GROUPS {
                    let gr = self.group_roll(g as i32);
                    self.gr_on[g] = gr * sg_f < 0.0;
                    self.gr_r[g] = if self.gr_on[g] { self.sq_fk[g] * gr } else { 0.0 };
                }
            }
            for n in 0..count {
                let effector2 = *self.ae(n);
                let num5 = effector2.reg_n;
                let num6 = 1.0 / ((num5 * num5) as f64);
                for num7 in 0..count {
                    self.h[n][num7] = V3::dot(self.jc[n], self.jc[num7]) as f64;
                    if self.pgrp[n] != 0 && self.pgrp[n] == self.pgrp[num7] && self.pg_on[ix(self.pgrp[n])] {
                        self.h[n][num7] += self.jp[n] * self.jp[num7];
                    }
                    if self.grp[n] != 0 && self.grp[n] == self.grp[num7] && self.gr_on[ix(self.grp[n])] {
                        self.h[n][num7] += self.jg[n] * self.jg[num7];
                    }
                }
                let num8 = lm_factor * self.h[n][n] + 1E-09f64;
                let num9 = reg_scale as f64;
                self.h[n][n] += num9 * ((effector2.mu + effector2.lam) as f64) * num6 + num8;
                self.rhs[n] = 0.0
                    - (V3::dot(self.jc[n], vector) as f64
                        + num9 * (effector2.mu as f64 * num6 * self.x[n] + effector2.lam as f64 * num6 * (self.x[n] - self.prev[n])));
                if self.grp[n] != 0 && self.gr_on[ix(self.grp[n])] {
                    self.rhs[n] -= self.jg[n] * self.gr_r[ix(self.grp[n])];
                }
                if self.pgrp[n] != 0 && self.pg_on[ix(self.pgrp[n])] {
                    self.rhs[n] -= self.jp[n] * self.pg_r[ix(self.pgrp[n])];
                }
                if effector2.anchor_k > 0.0 {
                    let bx = budget_excess(&effector2, self.x[n] as f32);
                    if bx != 0.0 {
                        self.h[n][n] += effector2.anchor_k as f64 * num6;
                        self.rhs[n] -= effector2.anchor_k as f64 * num6 * bx as f64;
                    }
                }
            }
            for num10 in 0..count {
                let num11 = self.pidx[num10];
                if num11 >= 0 {
                    let num11 = num11 as usize;
                    let effector3 = *self.ae(num10);
                    let effector4 = *self.ae(num11);
                    let num12 = effector3.reg_n;
                    let num13 = 1.0 / ((num12 * num12) as f64);
                    let num14 = effector3.pair_sign as f64;
                    let num15 = effector4.pair_sign as f64;
                    let num16 = 0.25 * reg_scale as f64 * effector3.sym_cost as f64 * num13;
                    let num17 = 0.25 * reg_scale as f64 * effector3.anti_cost as f64 * num13;
                    let num18 = num14 * self.x[num10] - num15 * self.x[num11] - (num14 - num15) * effector3.sym_target as f64;
                    let num19 = num14 * self.x[num10] + num15 * self.x[num11];
                    self.h[num10][num10] += num16 + num17;
                    self.h[num11][num11] += num16 + num17;
                    self.h[num10][num11] += (0.0 - num16 + num17) * num14 * num15;
                    self.h[num11][num10] += (0.0 - num16 + num17) * num14 * num15;
                    self.rhs[num10] -= num16 * num14 * num18 + num17 * num14 * num19;
                    self.rhs[num11] -= (0.0 - num16) * num15 * num18 + num17 * num15 * num19;
                }
            }
            if !solve_linear(&self.h, &self.rhs, &mut self.dx, count) {
                break;
            }
            let mut num20 = 1.0f64;
            let mut flag = false;
            let mut array = [0.0f64; MAX_EFF];
            let mut num21 = 0;
            while num21 < 4 && !flag {
                for num22 in 0..count {
                    array[num22] = self.x[num22];
                }
                for num23 in 0..count {
                    let ef = self.ae(num23);
                    let v = clampf((array[num23] + num20 * self.dx[num23]) as f32, ef.lo, ef.hi);
                    self.x[num23] = v as f64;
                }
                let num24 = self.total();
                if num24 < num {
                    num = num24;
                    flag = true;
                } else {
                    for num25 in 0..count {
                        self.x[num25] = array[num25];
                    }
                }
                num21 += 1;
                num20 *= 0.5;
            }
            if !flag {
                break;
            }
        }
        let mut vector2 = V3::ZERO;
        for num26 in 0..count {
            vector2 = vector2 + self.mat(num26, self.x[num26] as f32);
        }
        let coord_newton = self.io.coord_newton;
        for num27 in 0..count {
            let effector5 = *self.ae(num27);
            let num28 = self.x[num27] as f32;
            let p = self.prev[num27] as f32;
            let vector3 = vector2 - m_at(&effector5, num28);
            let vector4 = target - vector3;
            let mut num29 = self.cand(num27, m_at(&effector5, num28), vector4, num28, p);
            let mut num30 = num28;
            let mut num31 = -1i32;
            for num32 in 0..effector5.n {
                let num33 = effector5.grid[ix(num32)];
                if !(num33 < effector5.lo - 0.0001f32) && !(num33 > effector5.hi + 0.0001f32) {
                    let num34 = self.cand(num27, effector5.m[ix(num32)], vector4, num33, p);
                    if num34 < num29 - 1E-09f32 {
                        num29 = num34;
                        num30 = num33;
                        num31 = num32;
                    }
                }
            }
            if num31 > 0 && num31 < effector5.n - 1 {
                let c = effector5.grid[ix(num31 - 1)];
                let c2 = effector5.grid[ix(num31 + 1)];
                let num35 = self.cand(num27, effector5.m[ix(num31 - 1)], vector4, c, p);
                let num36 = self.cand(num27, effector5.m[ix(num31 + 1)], vector4, c2, p);
                let num37 = num35 - 2.0f32 * num29 + num36;
                if num37 > 1E-12f32 {
                    let num38 = clampf(
                        effector5.grid[ix(num31)] + clampf(0.5f32 * (num35 - num36) / num37, -0.5f32, 0.5f32) * effector5.step,
                        effector5.lo,
                        effector5.hi,
                    );
                    let num39 = self.cand(num27, m_at(&effector5, num38), vector4, num38, p);
                    if num39 < num29 {
                        num29 = num39;
                        num30 = num38;
                    }
                }
            }
            if coord_newton == 2 || (coord_newton == 1 && (effector5.kind == 1 || effector5.kind == 3)) {
                let xc = self.x[num27] as f32;
                let h_n = 0.25f32 * effector5.step;
                let a_n = maxf(effector5.lo, xc - h_n);
                let b_n = minf(effector5.hi, xc + h_n);
                if b_n > a_n + 1E-05f32 {
                    let g_n = self.E(m_at(&effector5, b_n) - m_at(&effector5, a_n)).div(b_n - a_n);
                    let r_n = self.E(m_at(&effector5, xc) - vector4);
                    let rn2 = effector5.reg_n * effector5.reg_n;
                    let den_n = V3::dot(g_n, g_n) as f64 + (reg_scale * (effector5.mu + effector5.lam) / rn2) as f64;
                    if den_n > 1E-12f64 {
                        let gr_n = V3::dot(g_n, r_n) as f64 + (reg_scale * (effector5.mu * xc + effector5.lam * (xc - p)) / rn2) as f64;
                        let c_n = clampf((xc as f64 - gr_n / den_n) as f32, effector5.lo, effector5.hi);
                        let cost_n = self.cand(num27, m_at(&effector5, c_n), vector4, c_n, p);
                        if cost_n < num29 {
                            #[allow(unused_assignments)]
                            {
                                num29 = cost_n;
                            }
                            num30 = c_n;
                            self.io.coord_newton_moves += 1;
                        }
                    }
                }
            }
            self.x[num27] = num30 as f64;
            vector2 = vector3 + m_at(&effector5, num30);
        }
        if self.io.paired_trim_search != 0 {
            self.paired_search(&mut vector2);
        }
        self.io.res_dbg = self.moment_sum() - target;
        for num40 in 0..count {
            let k = self.act[num40];
            let c = clampf(self.x[num40] as f32, self.e[k].lo, self.e[k].hi);
            self.e[k].cmd = c;
            self.x[num40] = c as f64;
        }
        self.io.last_cost = self.total();
    }

    /// Allocator.PairedSearch
    fn paired_search(&mut self, total: &mut V3) {
        let count = self.cnt;
        let target = self.io.target;
        let mut it = usize::MAX;
        for i in 0..count {
            if self.ae(i).kind == 1 {
                it = i;
                break;
            }
        }
        if it == usize::MAX {
            return;
        }
        let tv = *self.ae(it);
        if tv.hi - tv.lo < 0.05f32 {
            return;
        }
        let t_lo = m_at(&tv, tv.lo);
        let t_hi = m_at(&tv, tv.hi);
        let k_y = (t_hi.y - t_lo.y) / (tv.hi - tv.lo);
        if absf(k_y) < 1E-06f32 {
            return;
        }
        let pair_horizon = self.io.pair_horizon;
        for s in 0..count {
            let se = *self.ae(s);
            if se.kind != 2 {
                continue;
            }
            let xs = self.x[s] as f32;
            let xt = self.x[it] as f32;
            let m_s = m_at(&se, xs);
            let m_t = m_at(&tv, xt);
            let others = *total - m_s - m_t;
            let ps = self.prev[s] as f32;
            let pt = self.prev[it] as f32;
            let mut best = {
                let a = self.cost(*total - target);
                let b = self.reg(&se, xs, ps);
                let c = self.pair_reg_with(s, xs);
                let d = self.reg(&tv, xt, pt);
                let e = self.fight_cost_with(s, xs);
                let f = self.pf_pair(s, xs, it, xt);
                a + b + c + d + e + f
            };
            let mut bs = xs;
            let mut bt = xt;
            let mut b_ms = m_s;
            let mut b_mt = m_t;
            for g in 0..se.n {
                let gs = se.grid[ix(g)];
                if gs < se.lo - 0.0001f32
                    || gs > se.hi + 0.0001f32
                    || absf(gs - se.now) > maxf(se.step, pair_horizon * maxf(1.0f32, se.rate))
                {
                    continue;
                }
                let ms = se.m[ix(g)];
                let gt = clampf(xt + (m_s.y - ms.y) / k_y, tv.lo, tv.hi);
                let mt = m_at(&tv, gt);
                let c = {
                    let a = self.cost(others + ms + mt - target);
                    let b = self.reg(&se, gs, ps);
                    let c = self.pair_reg_with(s, gs);
                    let d = self.reg(&tv, gt, pt);
                    let e = self.fight_cost_with(s, gs);
                    let f = self.pf_pair(s, gs, it, gt);
                    a + b + c + d + e + f
                };
                if c < best - 1E-09f32 {
                    best = c;
                    bs = gs;
                    bt = gt;
                    b_ms = ms;
                    b_mt = mt;
                }
            }
            if bs != xs {
                self.x[s] = bs as f64;
                self.x[it] = bt as f64;
                *total = others + b_ms + b_mt;
                self.io.paired_moves += 1;
            }
        }
    }

    /// Allocator.ReachFrom
    fn reach_from(&mut self) {
        let n_all = self.e.len();
        self.rch_n = 0;
        for i in 0..n_all {
            let e = &self.e[i];
            if e.active == 0 || e.reach_tau <= 0.0 {
                continue;
            }
            let reach = maxf(1.0f32, e.rate) * e.reach_tau;
            if absf(e.cmd - e.now) <= reach {
                continue;
            }
            let k = self.rch_n;
            self.rch_e[k] = i;
            self.rch_dest[k] = e.cmd;
            self.rch_lo[k] = e.lo;
            self.rch_hi[k] = e.hi;
            self.rch_n += 1;
        }
        if self.rch_n == 0 {
            return;
        }
        self.io.reach_moves += 1;
        for i in 0..n_all {
            self.e[i].cmd = self.rch_cmd0[i];
        }
        for i in 0..self.rch_n {
            let dest = self.rch_dest[i];
            let e = &mut self.e[self.rch_e[i]];
            let reach = maxf(1.0f32, e.rate) * e.reach_tau;
            let at = e.now + clampf(dest - e.now, -reach, reach);
            e.cmd = at;
            e.hi = at;
            e.lo = at;
        }
        self.solve_core();
        for i in 0..self.rch_n {
            let e = &mut self.e[self.rch_e[i]];
            e.lo = self.rch_lo[i];
            e.hi = self.rch_hi[i];
            e.cmd = self.rch_dest[i];
        }
    }

    #[inline(always)]
    fn in_rch(&self, i: usize) -> bool {
        for k in 0..self.rch_n {
            if self.rch_e[k] == i {
                return true;
            }
        }
        false
    }

    /// Allocator.SolveInner, its sequential path (the plugin's worker-thread reach plan gives the same result by construction)
    fn solve_inner(&mut self) {
        let n_all = self.e.len();
        let mut any = false;
        if self.io.reach_pass != 0 {
            for e in self.e.iter() {
                if e.active != 0 && e.reach_tau > 0.0 {
                    any = true;
                    break;
                }
            }
        }
        if !any {
            self.solve_core();
            return;
        }
        for i in 0..n_all {
            self.rch_cmd0[i] = self.e[i].cmd;
        }
        self.solve_core();
        if self.io.two_horizon == 0 {
            self.reach_from();
            return;
        }
        let j_d = self.io.last_cost;
        self.reach_from();
        if self.rch_n == 0 {
            self.io.last_needed_pick = 0;
            return;
        }
        let mut any_th = false;
        for k in 0..self.rch_n {
            if self.e[self.rch_e[k]].two_horizon != 0 {
                any_th = true;
            }
        }
        if !any_th {
            self.io.last_needed_pick = 0;
            return;
        }
        self.io.last_needed_pick = 1;
        let mut j_r = self.io.last_cost;
        let mut th_r = [0.0f32; MAX_EFF];
        for i in 0..n_all {
            th_r[i] = self.e[i].cmd;
        }
        let target = self.io.target;
        if self.io.two_horizon_path != 0 {
            let mut oth = V3::ZERO;
            for i in 0..n_all {
                let e = &self.e[i];
                if e.active != 0 && !self.in_rch(i) {
                    oth = oth + m_at(e, e.cmd);
                }
            }
            let mut acc = 0.0f32;
            for k in 1..=8i32 {
                let f = k as f32 / 8.0f32;
                let mut tot = oth;
                for i in 0..self.rch_n {
                    let e = &self.e[self.rch_e[i]];
                    tot = tot + m_at(e, e.now + f * (e.cmd - e.now));
                }
                acc += self.cost(tot - target);
            }
            j_r = maxf(j_r, acc / 8.0f32);
        }
        let mut th_lo = [0.0f32; MAX_EFF];
        let mut th_hi = [0.0f32; MAX_EFF];
        for i in 0..n_all {
            let cmd0 = self.rch_cmd0[i];
            let e = &mut self.e[i];
            th_lo[i] = e.lo;
            th_hi[i] = e.hi;
            e.cmd = cmd0;
            if e.active != 0 && e.reach_tau > 0.0 && e.two_horizon != 0 {
                let reach = maxf(1.0f32, e.rate) * e.reach_tau;
                let lo = maxf(e.lo, minf(e.now, e.hi) - reach);
                let hi = minf(e.hi, maxf(e.now, e.lo) + reach);
                if lo <= hi {
                    e.lo = lo;
                    e.hi = hi;
                }
            }
        }
        self.solve_core();
        let j_b = self.io.last_cost;
        for i in 0..n_all {
            self.e[i].lo = th_lo[i];
            self.e[i].hi = th_hi[i];
        }
        if 2.0f32 * j_b <= j_r + j_d {
            self.io.two_horizon_bound += 1;
            return;
        }
        for i in 0..n_all {
            self.e[i].cmd = th_r[i];
        }
    }
}

/// Allocator.SolveLinear (Gaussian elimination with partial pivoting on copies; false when singular)
fn solve_linear(a0: &[[f64; MAX_EFF]; MAX_EFF], b0: &[f64; MAX_EFF], out_x: &mut [f64; MAX_EFF], n: usize) -> bool {
    let mut array = *a0;
    let mut array2 = *b0;
    for i in 0..n {
        let mut num = i;
        let mut num2 = array[i][i].abs();
        for j in (i + 1)..n {
            if array[j][i].abs() > num2 {
                num2 = array[j][i].abs();
                num = j;
            }
        }
        if num2 < 1E-18f64 {
            return false;
        }
        if num != i {
            for k in 0..n {
                let num3 = array[i][k];
                array[i][k] = array[num][k];
                array[num][k] = num3;
            }
            let num4 = array2[i];
            array2[i] = array2[num];
            array2[num] = num4;
        }
        for l in (i + 1)..n {
            let num5 = array[l][i] / array[i][i];
            if num5 != 0.0 {
                for m in i..n {
                    array[l][m] -= num5 * array[i][m];
                }
                array2[l] -= num5 * array2[i];
            }
        }
    }
    for num6 in (0..n).rev() {
        let mut num7 = array2[num6];
        for num8 in (num6 + 1)..n {
            num7 -= array[num6][num8] * out_x[num8];
        }
        out_x[num6] = num7 / array[num6][num6];
    }
    true
}

// ---------------------------------------------------------------------------------------------------------------------
// 4.1.0: the grid kernels
// ---------------------------------------------------------------------------------------------------------------------

/// UnityEngine.Quaternion (x, y, z, w).
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Q4 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl Q4 {
    /// UnityEngine.Quaternion: operator *(Quaternion, Vector3), expression for expression
    #[inline(always)]
    fn rot(self, p: V3) -> V3 {
        let num = self.x * 2.0f32;
        let num2 = self.y * 2.0f32;
        let num3 = self.z * 2.0f32;
        let num4 = self.x * num;
        let num5 = self.y * num2;
        let num6 = self.z * num3;
        let num7 = self.x * num2;
        let num8 = self.x * num3;
        let num9 = self.y * num3;
        let num10 = self.w * num;
        let num11 = self.w * num2;
        let num12 = self.w * num3;
        V3 {
            x: (1.0f32 - (num5 + num6)) * p.x + (num7 - num12) * p.y + (num8 + num11) * p.z,
            y: (num7 + num12) * p.x + (1.0f32 - (num4 + num6)) * p.y + (num9 - num10) * p.z,
            z: (num8 - num11) * p.x + (num9 + num10) * p.y + (1.0f32 - (num4 + num5)) * p.z,
        }
    }
}

/// UnityEngine.Vector3.Cross
#[inline(always)]
fn v3_cross(a: V3, b: V3) -> V3 {
    V3 { x: a.y * b.z - a.z * b.y, y: a.z * b.x - a.x * b.z, z: a.x * b.y - a.y * b.x }
}

/// UnityEngine.Vector3.magnitude: (float)Math.Sqrt(x * x + y * y + z * z)
#[inline(always)]
fn v3_mag(a: V3) -> f32 {
    ((a.x * a.x + a.y * a.y + a.z * a.z) as f64).sqrt() as f32
}

/// UnityEngine.Vector3: operator -(Vector3)
#[inline(always)]
fn v3_neg(a: V3) -> V3 {
    V3 { x: -a.x, y: -a.y, z: -a.z }
}

/// UnityEngine.Mathf.Atan2: (float)Math.Atan2(y, x)
#[inline(always)]
fn atan2f(y: f32, x: f32) -> f32 {
    (y as f64).atan2(x as f64) as f32
}

/// The airfoil tables' length (AirfoilTable: 128 points, index = angle * 20.37185 + 64).
pub const FOIL_N: usize = 128;

/// AirfoilTable.Read. `Err` for a NaN index: the C# code throws there (an array index from `(int)NaN`), so the plugin
/// has to run its own loop and throw as it always did.
#[inline(always)]
unsafe fn foil_read(t: *const f32, idx: f32) -> Result<f32, ()> {
    if idx <= 0.0f32 {
        return Ok(*t);
    }
    if idx >= 127.0f32 {
        return Ok(*t.add(127));
    }
    if idx.is_nan() {
        return Err(());
    }
    let num = idx as i32;
    let num2 = idx - (num as f32);
    let a = *t.add(num as usize);
    let b = *t.add(num as usize + 1);
    Ok(a + (b - a) * num2)
}

/// AirfoilTable.Get, the tabulated branch (the plugin keeps a `generic` airfoil in C#: that branch is sin / cos).
#[inline(always)]
unsafe fn foil_get(cl: *const f32, cd: *const f32, a: f32) -> Result<(f32, f32), ()> {
    let idx = a * 20.37185f32 + 64.0f32;
    Ok((foil_read(cl, idx)?, foil_read(cd, idx)?))
}

/// One surface, one airflow, every deflection of its grid (248 bytes). The plugin fills it per call; the arrays are its
/// own (pinned for the call).
#[repr(C)]
pub struct GridIn {
    /// per grid point (SurfaceEffector.GridGeom): Quaternion.Inverse(q), q * Vector3.right, com + q * col - cg
    pub g_inv: *const Q4,
    pub g_right: *const V3,
    pub g_arm: *const V3,
    /// per grid point, host section: Quaternion.Inverse(r1), r1 * Vector3.right, Camber.Delta's zero test
    pub gh_inv: *const Q4,
    pub gh_right: *const V3,
    pub gh_zero: *const u8,
    /// the surface's airfoil table and its host's (FOIL_N floats each)
    pub cl: *const f32,
    pub cd: *const f32,
    pub hcl: *const f32,
    pub hcd: *const f32,
    /// mode 0: receives the n moments
    pub out_m: *mut V3,
    /// mode 0: receives the n normal forces (SurfaceEffector.FN: (rootInv * fTot).y); null = not wanted
    pub out_fn: *mut f32,
    pub n: i32,
    /// hostK > 0
    pub host: i32,
    /// hv.sqrMagnitude < 1E-06f
    pub hv_small: i32,
    pub reserved: i32,
    /// the surface's airflow, airflow / speed, 0.5 rho v^2 S eff
    pub v: V3,
    pub vhat: V3,
    pub num: f32,
    /// the host's airflow, its Vector3.normalized, its 0.5 rho v^2 S eff
    pub hv: V3,
    pub hvn: V3,
    pub hq: f32,
    /// hostCom + hostRot * hostCol - cg, and the host's undeflected force (Camber.Foil)
    pub host_arm: V3,
    pub host_base: V3,
    pub root_inv: Q4,
    /// mode 1: IaInv[0,0], IaInv[0,2], IaInv[2,0], IaInv[2,2]
    pub ia: [f32; 4],
    /// mode 1: pitch min, pitch max, roll accel min, max, yaw accel min, max over the grid
    pub out: [f32; 6],
}

/// SurfaceEffector.Eval / EvalFlow for grid point `l`, from the geometry of GridGeom: the flow's part only. Also returns
/// Eval's lastFN, (rootInv * fTot).y (0 where Eval returns early).
#[inline(always)]
unsafe fn grid_point(g: &GridIn, l: usize) -> Result<(V3, f32), ()> {
    let v_flow = g.v;
    let vector = (*g.g_inv.add(l)).rot(v_flow);
    let (c_l, c_d) = foil_get(g.cl, g.cd, atan2f(vector.y, vector.z))?;
    let vector2 = v3_cross(v_flow, *g.g_right.add(l));
    let magnitude = v3_mag(vector2);
    if magnitude < 0.0001f32 {
        return Ok((V3::ZERO, 0.0f32));
    }
    let num = g.num;
    // -vector2 / magnitude * (CL * num) - vhat * (CD * num)
    let rhs2 = v3_neg(vector2).div(magnitude).mul(c_l * num) - g.vhat.mul(c_d * num);
    let mut m_f = v3_cross(*g.g_arm.add(l), rhs2);
    let mut f_tot = rhs2;
    if g.host != 0 {
        let mut d_h = V3::ZERO;
        if *g.gh_zero.add(l) == 0 {
            // Camber.FoilPre(hv, Inverse(r1), r1 * right, hostFoil, hq, hv.normalized, small) - hostBase
            let hv = g.hv;
            let vl = (*g.gh_inv.add(l)).rot(hv);
            let (cl, cd) = foil_get(g.hcl, g.hcd, atan2f(vl.y, vl.z))?;
            let n = v3_cross(hv, *g.gh_right.add(l));
            let m = v3_mag(n);
            let f = if m < 1E-06f32 || g.hv_small != 0 {
                V3::ZERO
            } else {
                v3_neg(n).div(m).mul(cl * g.hq) - g.hvn.mul(cd * g.hq)
            };
            d_h = f - g.host_base;
        }
        m_f = m_f + v3_cross(g.host_arm, d_h);
        f_tot = f_tot + d_h;
    }
    // Eval: lastFN = (ctx.rootInv * fTot).y
    let fn_ = g.root_inv.rot(f_tot).y;
    // Effector.ToAero
    let vector = g.root_inv.rot(m_f);
    Ok((V3 { x: 0.0f32 - vector.z, y: 0.0f32 - vector.x, z: vector.y }, fn_))
}

/// UnityEngine.Mathf.Min / Max (two floats)
#[inline(always)]
fn mathf_min(a: f32, b: f32) -> f32 {
    if a < b {
        a
    } else {
        b
    }
}

#[inline(always)]
fn mathf_max(a: f32, b: f32) -> f32 {
    if a > b {
        a
    } else {
        b
    }
}

unsafe fn grid_run(g: &mut GridIn, mode: i32) -> Result<(), ()> {
    let n = g.n as usize;
    if mode == 0 {
        for l in 0..n {
            let (m, fn_) = grid_point(g, l)?;
            *g.out_m.add(l) = m;
            if !g.out_fn.is_null() {
                *g.out_fn.add(l) = fn_;
            }
        }
        return Ok(());
    }
    // the sweep's capacity loop (FcsController.AsymSweepPoint)
    let mut num8 = f32::MAX;
    let mut num9 = f32::MIN;
    let mut num10 = f32::MAX;
    let mut num11 = f32::MIN;
    let mut p_min_s = f32::MAX;
    let mut p_max_s = f32::MIN;
    for l in 0..n {
        let (m2, _) = grid_point(g, l)?;
        p_min_s = mathf_min(p_min_s, m2.y);
        p_max_s = mathf_max(p_max_s, m2.y);
        // AccRoll, AccYaw
        let b = (g.ia[0] * m2.x + g.ia[1] * m2.z) * 57.29578f32;
        let b2 = (g.ia[2] * m2.x + g.ia[3] * m2.z) * 57.29578f32;
        num8 = mathf_min(num8, b);
        num9 = mathf_max(num9, b);
        num10 = mathf_min(num10, b2);
        num11 = mathf_max(num11, b2);
    }
    g.out = [p_min_s, p_max_s, num8, num9, num10, num11];
    Ok(())
}

/// Size of GridIn, for the plugin's layout check.
#[no_mangle]
pub extern "C" fn su30_fcs_grid_size() -> i32 {
    core::mem::size_of::<GridIn>() as i32
}

/// The grid kernels. mode 0: `out_m[0..n]` = the surface's moment at each grid deflection (the table rebuild);
/// mode 1: `out` = the min / max the asymmetric-load sweep keeps. 0 = done; negative = nothing written may be used and
/// the plugin runs its C# loop: -1 a panic, -2 bad arguments, -7 a NaN airfoil index (the C# code throws there).
///
/// # Safety
/// `g` must point at one valid, writable `GridIn` whose arrays hold `n` elements (`FOIL_N` for the tables).
#[no_mangle]
pub unsafe extern "C" fn su30_fcs_grid(g: *mut GridIn, mode: i32) -> i32 {
    if g.is_null() {
        return -2;
    }
    let g = &mut *g;
    if g.n < 1 || g.n as usize > 4 * MAX_N || (mode != 0 && mode != 1) {
        return -2;
    }
    if g.g_inv.is_null() || g.g_right.is_null() || g.g_arm.is_null() || g.cl.is_null() || g.cd.is_null() {
        return -2;
    }
    if g.host != 0 && (g.gh_inv.is_null() || g.gh_right.is_null() || g.gh_zero.is_null() || g.hcl.is_null() || g.hcd.is_null()) {
        return -2;
    }
    if mode == 0 && g.out_m.is_null() {
        return -2;
    }
    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| grid_run(g, mode)));
    match r {
        Ok(Ok(())) => 0,
        Ok(Err(())) => -7,
        Err(_) => -1,
    }
}

/// What the plugin checks before it trusts the library.
#[no_mangle]
pub extern "C" fn su30_fcs_abi() -> i32 {
    ABI
}

/// (size of EffN << 16) | size of AllocIo, for the plugin's layout check.
#[no_mangle]
pub extern "C" fn su30_fcs_sizes() -> i32 {
    ((core::mem::size_of::<EffN>() as i32) << 16) | (core::mem::size_of::<AllocIo>() as i32)
}

fn validate(io: &AllocIo, effs: &[EffN]) -> i32 {
    let n = effs.len() as i32;
    for e in effs {
        if e.n < 0 || e.n as usize > MAX_N {
            return -3;
        }
        if e.partner < -1 || e.partner >= n {
            return -4;
        }
        if e.roll_group < 0 || e.roll_group > FIGHT_GROUPS as i32 || e.pitch_group < 0 || e.pitch_group > 2 {
            return -5;
        }
    }
    if io.gn_iterations < 0 || io.gn_iterations > 64 {
        return -6;
    }
    0
}

/// Allocator.Solve. `effs` points at `io.n` effectors; their `cmd` fields hold the answer on return (0), `io` the
/// cost, the residual and the counters. A negative return means nothing may be used and the caller solves in C#:
/// -1 a panic (an index the C# code would have thrown on), -2 bad sizes, -3..-6 fields out of range.
///
/// # Safety
/// `io` must point at one valid `AllocIo` and `effs` at `io.n` valid `EffN`, both writable and not aliased for the call.
#[no_mangle]
pub unsafe extern "C" fn su30_fcs_solve(io: *mut AllocIo, effs: *mut EffN) -> i32 {
    if io.is_null() || effs.is_null() {
        return -2;
    }
    let io = &mut *io;
    if io.n < 0 || io.n as usize > MAX_EFF {
        return -2;
    }
    let effs = core::slice::from_raw_parts_mut(effs, io.n as usize);
    let v = validate(io, effs);
    if v != 0 {
        return v;
    }
    io.reach_moves = 0;
    io.two_horizon_bound = 0;
    io.coord_newton_moves = 0;
    io.paired_moves = 0;
    io.core_solves = 0;
    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let mut s = Solver::new(io, effs);
        s.solve_inner();
    }));
    match r {
        Ok(()) => 0,
        Err(_) => -1,
    }
}

/// Pure-Rust entry for tests and benchmarks.
pub fn solve(io: &mut AllocIo, effs: &mut [EffN]) {
    io.n = effs.len() as i32;
    io.reach_moves = 0;
    io.two_horizon_bound = 0;
    io.coord_newton_moves = 0;
    io.paired_moves = 0;
    io.core_solves = 0;
    let mut s = Solver::new(io, effs);
    s.solve_inner();
}
