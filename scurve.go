package trajectory

import "math"

// planSCurve builds the time-optimal jerk-limited (7-phase) profile.
//
// The canonical phase sequence is
//
//	 0: +J  jerk-up acceleration      (a: 0 -> +A_a)
//	 1:  0  constant acceleration     (a = +A_a)
//	 2: -J  jerk-down acceleration    (a: +A_a -> 0)
//	[   cruise at the peak velocity   ]
//	 3: -J  jerk-down deceleration    (a: 0 -> -A_d)
//	 4:  0  constant deceleration     (a = -A_d)
//	 5: +J  jerk-up deceleration      (a: -A_d -> 0)
//
// Each phase is allowed to degenerate to zero length: when the velocity
// change is too small to ever reach AMax under JMax, phases 1 and 4 vanish
// (pure double-jerk ramps); when the path is too short to reach VMax, the
// cruise vanishes and the peak velocity itself drops below VMax. All
// degenerations are detected via the duration formulas (durations are
// clamped at zero, never negative) and zero-length phases are skipped
// everywhere downstream.
func planSCurve(L, v0, vEnd float64, lim Limits) (*Profile, error) {
	aMax := lim.AMax
	jMax := lim.JMax
	vM := lim.VMax

	// Minimum distance needed merely to turn v0 into vEnd (peak velocity
	// equal to the larger endpoint velocity).
	dMin, _ := sCurveRampDistance(math.Max(v0, vEnd), v0, vEnd, aMax, jMax)
	if L < dMin-Eps {
		return nil, infeasibleError("s-curve", L, dMin)
	}

	// Durations assuming the profile reaches VMax.
	dRamp, t := sCurveRampDistance(vM, v0, vEnd, aMax, jMax)

	p := &Profile{
		Kind: "s-curve",
		V0:   v0, VEnd: vEnd, VMax: vM, AMax: aMax, JMax: jMax, L: L,
	}

	if L >= dRamp-Eps {
		// VMax reached.
		p.Reach = true
		p.vPeakS = vM
		p.t = t
		p.tCr = nonNeg((L - dRamp) / vM)
		return p, nil
	}

	// VMax unreachable: no cruise; find the switching velocity vPeak < VMax
	// where acceleration crosses zero. For a candidate vPeak the phase
	// durations follow the same formulas; total ramp distance is continuous
	// and strictly increasing in vPeak, so bisection solves it exactly
	// regardless of which phases happen to degenerate.
	lo, hi := math.Max(v0, vEnd), vM
	var tt [6]float64
	for i := 0; i < 100; i++ {
		mid := (lo + hi) / 2
		d, cand := sCurveRampDistance(mid, v0, vEnd, aMax, jMax)
		if d > L {
			hi = mid
		} else {
			lo = mid
			tt = cand
		}
	}
	vPeak := (lo + hi) / 2
	// Recompute durations at the converged peak for the stored profile.
	_, tt = sCurveRampDistance(vPeak, v0, vEnd, aMax, jMax)

	p.Reach = false
	p.vPeakS = vPeak
	p.t = tt
	p.tCr = 0
	return p, nil
}

// halfRamp returns the three phase durations of one jerk-limited velocity
// ramp from vLo to vHi: jerk time tj (each of the two jerk phases) and
// constant-acceleration time tc. When the required velocity change cannot
// reach aMax, the constant-acceleration duration is exactly zero (the
// middle phase degenerates).
func halfRamp(vLo, vHi, aMax, jMax float64) (tj, tc float64) {
	dv := vHi - vLo
	if dv <= Eps {
		return 0, 0
	}
	tjAccelLimited := aMax / jMax
	tjVelocityLimited := math.Sqrt(dv / jMax)
	if tjVelocityLimited < tjAccelLimited {
		// Double-jerk ramp: the two jerk phases alone supply exactly dv;
		// the constant-acceleration phase degenerates to zero. Set it
		// explicitly so no floating-point residue survives.
		return tjVelocityLimited, 0
	}
	tj = tjAccelLimited
	peakA := jMax * tj
	// Velocity gained during the two jerk phases equals jMax*tj^2; whatever
	// remains is covered by the constant-acceleration phase.
	tc = nonNeg((dv - jMax*tj*tj) / peakA)
	return tj, tc
}

// sCurveRampDistance integrates the six ramp phases (no cruise) that take
// v0 -> vPeak -> vEnd and returns the covered distance and the phase
// durations. Integration uses the exact constant-jerk propagation
// formulas, so acceleration is continuous across every phase boundary.
func sCurveRampDistance(vPeak, v0, vEnd, aMax, jMax float64) (float64, [6]float64) {
	var t [6]float64
	t1, t2 := halfRamp(v0, vPeak, aMax, jMax)
	t4, t5 := halfRamp(vEnd, vPeak, aMax, jMax)
	t[0], t[1], t[2] = t1, t2, t1
	t[3], t[4], t[5] = t4, t5, t4

	// Acceleration half: start (s=0, v=v0, a=0), jerks +J, 0, -J.
	s, v, a := 0.0, v0, 0.0
	s, v, a = propagate(s, v, a, +jMax, t[0])
	s, v, a = propagate(s, v, a, 0, t[1])
	s, v, a = propagate(s, v, a, -jMax, t[2])
	// Deceleration half: jerks -J, 0, +J, ending at vEnd.
	s, v, a = propagate(s, v, a, -jMax, t[3])
	s, v, a = propagate(s, v, a, 0, t[4])
	s, v, a = propagate(s, v, a, +jMax, t[5])
	return s, t
}

// propagate advances the state for dt under constant jerk j:
//
//	a1 = a0 + j·dt
//	v1 = v0 + a0·dt + ½j·dt²
//	s1 = s0 + v0·dt + ½a0·dt² + ⅙j·dt³
func propagate(s, v, a, j, dt float64) (float64, float64, float64) {
	s += v*dt + 0.5*a*dt*dt + j*dt*dt*dt/6
	v += a*dt + 0.5*j*dt*dt
	a += j * dt
	return s, v, a
}

// sCurvePhases returns the ordered non-degenerate phases as (jerk, duration)
// pairs including the cruise phase.
func sCurvePhases(p *Profile) []phaseJK {
	all := make([]phaseJK, 0, 7)
	jerks := []float64{+p.JMax, 0, -p.JMax, 0 /* cruise */, -p.JMax, 0, +p.JMax}
	durs := []float64{p.t[0], p.t[1], p.t[2], p.tCr, p.t[3], p.t[4], p.t[5]}
	for i := range durs {
		if durs[i] > Eps {
			all = append(all, phaseJK{j: jerks[i], dt: durs[i]})
		}
	}
	return all
}

type phaseJK struct {
	j  float64
	dt float64
}

func sampleSCurve(p *Profile, t float64) State {
	T := p.Duration()
	t = clampTime(t, T)
	s, v, a := 0.0, p.V0, 0.0
	remaining := t
	for _, ph := range sCurvePhases(p) {
		if remaining <= ph.dt+Eps {
			s, v, a = propagate(s, v, a, ph.j, remaining)
			return State{T: t, S: s, V: v, A: a}
		}
		s, v, a = propagate(s, v, a, ph.j, ph.dt)
		remaining -= ph.dt
	}
	return State{T: t, S: p.L, V: p.VEnd, A: 0}
}

func jerkSCurve(p *Profile, t float64) float64 {
	t = clampTime(t, p.Duration())
	cum := 0.0
	for _, ph := range sCurvePhases(p) {
		if t < cum+ph.dt-Eps {
			return ph.j
		}
		cum += ph.dt
	}
	return 0
}

func sCurveSegments(p *Profile) []Segment {
	names := []string{
		"jerk-up acceleration", "constant acceleration", "jerk-down acceleration",
		"cruise",
		"jerk-down deceleration", "constant deceleration", "jerk-up deceleration",
	}
	jerks := []float64{+p.JMax, 0, -p.JMax, 0, -p.JMax, 0, +p.JMax}
	durs := []float64{p.t[0], p.t[1], p.t[2], p.tCr, p.t[3], p.t[4], p.t[5]}
	var segs []Segment
	cum := 0.0
	for i := range durs {
		if durs[i] > Eps {
			segs = append(segs, Segment{
				Name: names[i], T0: cum, T1: cum + durs[i],
				Duration: durs[i], Jerk: jerks[i],
			})
			cum += durs[i]
		}
	}
	return segs
}
