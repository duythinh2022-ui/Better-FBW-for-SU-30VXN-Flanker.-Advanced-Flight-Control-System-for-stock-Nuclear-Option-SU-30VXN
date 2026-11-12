//! Su-30 FCS 0.5.6: the coupled asymmetric-load check's LP (`CoupledCap.cs`, class `CplPoint`), native. From the Ternion FCS 0.3.0 core;
//! `side` (CplIo.side, was `reserved`) added for the one-side Reserve (Su-30 0.5.0's `Reserve(..., side)`; 2 = feasibility only).
//!
//! A port of `CplPoint.Reserve`, `LateralFeasible`, `PitchHeld` and their simplex (`Phase1`, `DropArtificials`, `Phase2`, `Run`,
//! `Pivot`), expression for expression: the same float expressions rounded to float before they widen to double, the same
//! double operations in the same order, the same comparisons and tolerances, the same pivoting rule. No maths library call.
//! The C# code is the reference; on a runtime with strict 32-bit float arithmetic (the bench) the two return the same bits.
//!
//! The caller owns every array (the `CplPoint`'s own, pinned for the call), so nothing is allocated here and a call is as
//! thread-safe as the C# method it replaces (one `CplPoint` per sweep point).

/// CplPoint.MaxCol / MaxEff / IterCap
pub const CPL_MAX_COL: usize = 420;
pub const CPL_MAX_EFF: usize = 18;
pub const CPL_ITER_CAP: i32 = 400;
/// CplPoint.W0: the widest tableau row (points + slack, two artificials, the right-hand side)
pub const CPL_W0: usize = CPL_MAX_COL + 1 + 2 + 1;

/// One call (FcsNative.CplIo). `op`: 0 = Reserve, 1 = LateralFeasible, 2 = PitchHeld.
#[repr(C)]
pub struct CplIo {
    /// the points' roll, pitch and yaw accelerations (CplPoint.R / Q / Y), `n_col` of them
    pub r: *const f32,
    pub q: *const f32,
    pub y: *const f32,
    /// CplPoint.effStart (`n_eff + 1` entries) and effOf (`n_col` entries)
    pub eff_start: *const i32,
    pub eff_of: *const i32,
    /// the workspace: T and T1 ((MaxEff + 3) * W0 doubles each), basis and basis1 (MaxEff + 2), a0 / a1 / cc (MaxCol + 1)
    pub t: *mut f64,
    pub t1: *mut f64,
    pub basis: *mut i32,
    pub basis1: *mut i32,
    pub a0: *mut f64,
    pub a1: *mut f64,
    pub cc: *mut f64,
    pub n_col: i32,
    pub n_eff: i32,
    pub op: i32,
    /// Reserve: sgnA; PitchHeld: dir
    pub sgn: i32,
    pub sin_a: f32,
    pub cos_a: f32,
    pub b_r: f32,
    pub b_q: f32,
    pub b_y: f32,
    /// out. Reserve: rp, rn (NaN unless ok); PitchHeld: rp = the figure (NaN if the load cannot be held)
    pub rp: f32,
    pub rn: f32,
    /// out: the method's bool (Reserve, LateralFeasible); PitchHeld: 1
    pub ok: i32,
    /// out: what the call adds to CplPoint.Iters
    pub iters: i32,
    /// Reserve only: 0 = both sides; +1 / -1 = that side only (the other figure NaN); 2 = phase 1 only (`ok` = feasible, both NaN)
    pub side: i32,
}

struct Lp<'a> {
    r: &'a [f32],
    q: &'a [f32],
    y: &'a [f32],
    eff_start: &'a [i32],
    eff_of: &'a [i32],
    t: &'a mut [f64],
    t1: &'a mut [f64],
    basis: &'a mut [i32],
    basis1: &'a mut [i32],
    a0: &'a mut [f64],
    a1: &'a mut [f64],
    cc: &'a mut [f64],
    n_col: usize,
    n_eff: usize,
    n: usize,
    m: usize,
    w: usize,
    iters: i32,
}

impl<'a> Lp<'a> {
    // CplPoint.Reserve
    fn reserve(&mut self, sin_a: f32, cos_a: f32, b_r: f32, b_q: f32, b_y: f32, sgn_a: i32, side: i32) -> (bool, f32, f32) {
        let n_col = self.n_col;
        // columns: the points, then the pitch slack (its own column, outside every surface)
        self.n = n_col + 1;
        for j in 0..n_col {
            self.a0[j] = self.q[j] as f64;
            self.a1[j] = (sin_a * self.r[j] - cos_a * self.y[j]) as f64;
            self.cc[j] = (-(cos_a * self.r[j] + sin_a * self.y[j])) as f64;
        }
        self.a0[n_col] = if sgn_a >= 0 { 1.0 } else { -1.0 };
        self.a1[n_col] = 0.0;
        self.cc[n_col] = 0.0;
        let b0: f64 = (-b_q) as f64;
        let b1: f64 = (-(sin_a * b_r - cos_a * b_y)) as f64;
        let lin: f64 = (cos_a * b_r + sin_a * b_y) as f64;
        if !self.phase1(b0, b1) {
            return (false, f32::NAN, f32::NAN);
        }
        if side == 2 {
            return (true, f32::NAN, f32::NAN);
        }
        if side != 0 {
            let (ok_s, obj_s) = self.phase2(if side > 0 { 1.0 } else { -1.0 });
            if !ok_s {
                return (false, f32::NAN, f32::NAN);
            }
            if side > 0 {
                return (true, (-obj_s + lin) as f32, f32::NAN);
            }
            return (true, f32::NAN, (-obj_s - lin) as f32);
        }
        let len = (self.m + 1) * self.w;
        self.t1[..len].copy_from_slice(&self.t[..len]);
        let m = self.m;
        self.basis1[..m].copy_from_slice(&self.basis[..m]);
        let (ok_p, obj_p) = self.phase2(1.0);
        if !ok_p {
            return (false, f32::NAN, f32::NAN);
        }
        self.t[..len].copy_from_slice(&self.t1[..len]);
        self.basis[..m].copy_from_slice(&self.basis1[..m]);
        let (ok_n, obj_n) = self.phase2(-1.0);
        if !ok_n {
            return (false, f32::NAN, f32::NAN);
        }
        let rp = (-obj_p + lin) as f32;
        let rn = (-obj_n - lin) as f32;
        (true, rp, rn)
    }

    // CplPoint.LateralFeasible
    fn lateral_feasible(&mut self, sin_a: f32, cos_a: f32, b_r: f32, b_y: f32) -> bool {
        let n_col = self.n_col;
        self.n = n_col + 1;
        for j in 0..n_col {
            self.a0[j] = 0.0;
            self.a1[j] = (sin_a * self.r[j] - cos_a * self.y[j]) as f64;
            self.cc[j] = 0.0;
        }
        self.a0[n_col] = 1.0; // the first row is free (0 = slack)
        self.a1[n_col] = 0.0;
        self.cc[n_col] = 0.0;
        self.phase1(0.0, (-(sin_a * b_r - cos_a * b_y)) as f64)
    }

    // CplPoint.PitchHeld
    fn pitch_held(&mut self, dir: i32, b_r: f32, b_q: f32, b_y: f32) -> f32 {
        let n_col = self.n_col;
        self.n = n_col;
        for j in 0..n_col {
            self.a0[j] = self.r[j] as f64;
            self.a1[j] = self.y[j] as f64;
            self.cc[j] = (if dir < 0 { self.q[j] } else { 0f32 - self.q[j] }) as f64;
        }
        if !self.phase1((-b_r) as f64, (-b_y) as f64) {
            return f32::NAN;
        }
        let (ok, obj) = self.phase2(1.0);
        if !ok {
            return f32::NAN;
        }
        ((if dir < 0 { obj } else { 0.0 - obj }) + b_q as f64) as f32
    }

    // CplPoint.Phase1
    fn phase1(&mut self, b0: f64, b1: f64) -> bool {
        let n = self.n;
        let n_eff = self.n_eff;
        let n_col = self.n_col;
        self.m = n_eff + 2;
        self.w = n + 3;
        let m = self.m;
        let w = self.w;
        let rhs = w - 1;
        let ra = n_eff;
        let rb = n_eff + 1;
        for v in self.t[..(m + 1) * w].iter_mut() {
            *v = 0.0;
        }
        let mut s0 = b0;
        let mut s1 = b1;
        for e in 0..n_eff {
            let mut best = self.eff_start[e] as usize;
            let end = self.eff_start[e + 1] as usize;
            let mut j = self.eff_start[e] as usize + 1;
            while j < end {
                if self.cc[j] < self.cc[best] {
                    best = j;
                }
                j += 1;
            }
            self.basis[e] = best as i32;
            let ro = e * w;
            let mut j = self.eff_start[e] as usize;
            while j < end {
                self.t[ro + j] = 1.0;
                j += 1;
            }
            self.t[ro + rhs] = 1.0;
            s0 -= self.a0[best];
            s1 -= self.a1[best];
        }
        let g0: f64 = if s0 < 0.0 { -1.0 } else { 1.0 };
        let g1: f64 = if s1 < 0.0 { -1.0 } else { 1.0 };
        let r0o = ra * w;
        let r1o = rb * w;
        for j in 0..n {
            let e: i32 = if j < n_col { self.eff_of[j] } else { -1 };
            let sub0 = if e >= 0 { self.a0[self.basis[e as usize] as usize] } else { 0.0 };
            let sub1 = if e >= 0 { self.a1[self.basis[e as usize] as usize] } else { 0.0 };
            self.t[r0o + j] = g0 * (self.a0[j] - sub0);
            self.t[r1o + j] = g1 * (self.a1[j] - sub1);
        }
        self.t[r0o + n] = 1.0;
        self.t[r1o + n + 1] = 1.0;
        self.t[r0o + rhs] = g0 * s0;
        self.t[r1o + rhs] = g1 * s1;
        self.basis[ra] = n as i32;
        self.basis[rb] = (n + 1) as i32;
        let b_scale = 1.0 + b0.abs() + b1.abs();
        if s0.abs() + s1.abs() <= 1e-9 * b_scale {
            // already feasible at the starting basis
            return self.drop_artificials();
        }
        let zo = m * w;
        for j in 0..n {
            self.t[zo + j] = -(self.t[r0o + j] + self.t[r1o + j]);
        }
        self.t[zo + n] = 0.0;
        self.t[zo + n + 1] = 0.0;
        self.t[zo + rhs] = -(self.t[r0o + rhs] + self.t[r1o + rhs]);
        if self.run(n + 2) == 2 {
            return false;
        }
        if -self.t[zo + rhs] > 1e-7 * b_scale {
            return false;
        }
        self.drop_artificials()
    }

    // CplPoint.DropArtificials
    fn drop_artificials(&mut self) -> bool {
        let n = self.n;
        for i in 0..self.m {
            if self.basis[i] < n as i32 {
                continue;
            }
            let ro = i * self.w;
            for j in 0..n {
                if self.t[ro + j].abs() > 1e-9 {
                    self.pivot(i, j);
                    break;
                }
            }
        }
        true
    }

    // CplPoint.Phase2: minimise sgn * cc . x from the present (feasible) basis
    fn phase2(&mut self, sgn: f64) -> (bool, f64) {
        let n = self.n;
        let m = self.m;
        let w = self.w;
        let zo = m * w;
        let rhs = w - 1;
        for j in 0..w {
            self.t[zo + j] = if j < n { sgn * self.cc[j] } else { 0.0 };
        }
        for i in 0..m {
            let bj = self.basis[i];
            let cb = if bj < n as i32 { sgn * self.cc[bj as usize] } else { 0.0 };
            if cb != 0.0 {
                let ro = i * w;
                for j in 0..w {
                    self.t[zo + j] -= cb * self.t[ro + j];
                }
            }
        }
        if self.run(n) == 2 {
            return (false, 0.0);
        }
        (true, -self.t[zo + rhs])
    }

    // CplPoint.Run
    fn run(&mut self, n_allowed: usize) -> i32 {
        let m = self.m;
        let w = self.w;
        let zo = m * w;
        let rhs = w - 1;
        let mut it: i32 = 0;
        loop {
            it += 1;
            if it > CPL_ITER_CAP {
                self.iters += it;
                return 2;
            }
            let bland = it > 80;
            let mut e: i32 = -1;
            let mut best: f64 = -1e-9;
            for j in 0..n_allowed {
                let rc = self.t[zo + j];
                if rc < best {
                    e = j as i32;
                    if bland {
                        break;
                    }
                    best = rc;
                }
            }
            if e < 0 {
                self.iters += it;
                return 0;
            }
            let e = e as usize;
            let mut r: i32 = -1;
            let mut ratio: f64 = f64::MAX;
            for i in 0..m {
                let a = self.t[i * w + e];
                if a > 1e-10 {
                    let q = self.t[i * w + rhs] / a;
                    if q < ratio - 1e-12 || (q < ratio + 1e-12 && r >= 0 && self.basis[i] < self.basis[r as usize]) {
                        ratio = q;
                        r = i as i32;
                    }
                }
            }
            if r < 0 {
                self.iters += it;
                return 2;
            }
            self.pivot(r as usize, e);
        }
    }

    // CplPoint.Pivot
    fn pivot(&mut self, r: usize, e: usize) {
        let m = self.m;
        let w = self.w;
        let ro = r * w;
        let pv = self.t[ro + e];
        for j in 0..w {
            self.t[ro + j] /= pv;
        }
        for i in 0..=m {
            if i == r {
                continue;
            }
            let io = i * w;
            let f = self.t[io + e];
            if f == 0.0 {
                continue;
            }
            for j in 0..w {
                self.t[io + j] -= f * self.t[ro + j];
            }
        }
        self.basis[r] = e as i32;
    }
}

unsafe fn cpl_run(io: &mut CplIo) {
    let n_col = io.n_col as usize;
    let n_eff = io.n_eff as usize;
    let tl = (CPL_MAX_EFF + 3) * CPL_W0;
    let mut lp = Lp {
        r: core::slice::from_raw_parts(io.r, n_col),
        q: core::slice::from_raw_parts(io.q, n_col),
        y: core::slice::from_raw_parts(io.y, n_col),
        eff_start: core::slice::from_raw_parts(io.eff_start, n_eff + 1),
        eff_of: core::slice::from_raw_parts(io.eff_of, n_col),
        t: core::slice::from_raw_parts_mut(io.t, tl),
        t1: core::slice::from_raw_parts_mut(io.t1, tl),
        basis: core::slice::from_raw_parts_mut(io.basis, CPL_MAX_EFF + 2),
        basis1: core::slice::from_raw_parts_mut(io.basis1, CPL_MAX_EFF + 2),
        a0: core::slice::from_raw_parts_mut(io.a0, CPL_MAX_COL + 1),
        a1: core::slice::from_raw_parts_mut(io.a1, CPL_MAX_COL + 1),
        cc: core::slice::from_raw_parts_mut(io.cc, CPL_MAX_COL + 1),
        n_col,
        n_eff,
        n: 0,
        m: 0,
        w: 0,
        iters: 0,
    };
    match io.op {
        0 => {
            let (ok, rp, rn) = lp.reserve(io.sin_a, io.cos_a, io.b_r, io.b_q, io.b_y, io.sgn, io.side);
            io.ok = ok as i32;
            io.rp = rp;
            io.rn = rn;
        }
        1 => {
            io.ok = lp.lateral_feasible(io.sin_a, io.cos_a, io.b_r, io.b_y) as i32;
            io.rp = f32::NAN;
            io.rn = f32::NAN;
        }
        _ => {
            io.rp = lp.pitch_held(io.sgn, io.b_r, io.b_q, io.b_y);
            io.rn = f32::NAN;
            io.ok = 1;
        }
    }
    io.iters = lp.iters;
}

/// Size of CplIo, for the plugin's layout check.
#[no_mangle]
pub extern "C" fn su30_fcs_cpl_size() -> i32 {
    core::mem::size_of::<CplIo>() as i32
}

/// The coupled check's LP: one of CplPoint.Reserve / LateralFeasible / PitchHeld (`io.op`). 0 = `io`'s outputs hold the
/// answer; negative = nothing in them may be used and the plugin runs its C# method: -1 a panic (an index the C# code would
/// have thrown on), -2 bad arguments.
///
/// # Safety
/// `io` must point at one valid, writable `CplIo` whose arrays have the sizes its fields' comments give.
#[no_mangle]
pub unsafe extern "C" fn su30_fcs_cpl(io: *mut CplIo) -> i32 {
    if io.is_null() {
        return -2;
    }
    let io = &mut *io;
    if io.n_col < 0 || io.n_col as usize > CPL_MAX_COL || io.n_eff < 0 || io.n_eff as usize > CPL_MAX_EFF || io.op < 0 || io.op > 2 || io.side < -1 || io.side > 2 {
        return -2;
    }
    if io.r.is_null() || io.q.is_null() || io.y.is_null() || io.eff_start.is_null() || io.eff_of.is_null() || io.t.is_null() || io.t1.is_null()
        || io.basis.is_null() || io.basis1.is_null() || io.a0.is_null() || io.a1.is_null() || io.cc.is_null()
    {
        return -2;
    }
    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| cpl_run(io)));
    match r {
        Ok(()) => 0,
        Err(_) => -1,
    }
}
