package planner

import "math"

// PlanSCurve plans a time-optimal S-curve (jerk-limited) velocity
// profile from rest to rest over distance L under the limits in lim.
//
// The full profile has up to seven phases:
//
//	+jerk, const accel, -jerk, cruise, -jerk, const decel, +jerk
//
// Two kinds of degeneration are handled explicitly:
//
//  1. Short paths: the velocity limit is unreachable, the cruise phase
//     vanishes and the peak velocity is solved analytically.
//  2. Extreme velocity/acceleration/jerk combinations: when the peak
//     velocity is below Amax^2/Jmax the jerk limit prevents the
//     acceleration from ever reaching Amax, so the constant-acceleration
//     phases degenerate to zero length and are skipped. No negative- or
//     zero-duration segment is ever emitted, and acceleration stays
//     continuous at every phase switch (it is exactly what the jerk
//     limit buys).
//
// It returns ErrInfeasible for non-positive distances or invalid limits.
func PlanSCurve(L float64, lim Limits) ([]JerkSeg, error) {
	if L <= 0 || !lim.Valid() {
		return nil, ErrInfeasible
	}
	vmax, amax, jmax := lim.Vmax, lim.Amax, lim.Jmax

	// Velocity at which the acceleration limit is just barely reached
	// during a jerk-limited ramp: vA = Amax^2 / Jmax.
	vA := amax * amax / jmax

	// minDistance returns the distance of a rest->v->rest S-curve.
	minDistance := func(v float64) float64 {
		tj, ta := rampTimes(v, amax, jmax)
		// The velocity profile of the accel phase is symmetric in time,
		// so its distance is v*(2*tj+ta)/2; decel mirrors it.
		return v * (2*tj + ta)
	}

	var vp, tv float64
	if L >= minDistance(vmax) {
		vp = vmax
		tv = (L - minDistance(vmax)) / vmax
	} else {
		// Velocity limit unreachable: solve minDistance(vp) = L.
		// lcrit separates the "Amax reached" regime from the
		// "constant-acceleration phase degenerated" regime.
		lcrit := minDistance(vA)
		if L >= lcrit {
			// vp >= vA: L = vp^2/Amax + vp*vA/Amax with vA = Amax^2/Jmax,
			// i.e. vp^2 + vA*vp - L*Amax = 0 (quadratic in vp).
			vp = 0.5 * (math.Sqrt(vA*vA+4*L*amax) - vA)
		} else {
			// vp < vA: L = 2*vp^(3/2)/sqrt(Jmax).
			vp = math.Pow(0.5*L*math.Sqrt(jmax), 2.0/3.0)
		}
		tv = 0
	}

	tj, ta := rampTimes(vp, amax, jmax)

	var segs []JerkSeg
	t := 0.0
	push := func(dur, j float64) {
		if dur <= segEps { // skip degenerate phases (e.g. ta == 0)
			return
		}
		segs = append(segs, JerkSeg{T0: t, Dur: dur, J: j})
		t += dur
	}
	push(tj, jmax)  // acceleration ramp-up
	push(ta, 0)     // constant acceleration (may be degenerate)
	push(tj, -jmax) // acceleration ramp-down
	push(tv, 0)     // cruise (may be degenerate)
	push(tj, -jmax) // deceleration ramp-up
	push(ta, 0)     // constant deceleration (may be degenerate)
	push(tj, jmax)  // deceleration ramp-down
	return segs, nil
}

// rampTimes returns the jerk-phase duration tj and the
// constant-acceleration-phase duration ta needed to swing the velocity
// by dv with peak acceleration at most amax under jerk limit jmax.
// When dv < amax^2/jmax the acceleration limit is not reached and
// ta == 0 (the constant-acceleration phase degenerates).
func rampTimes(dv, amax, jmax float64) (tj, ta float64) {
	if dv >= amax*amax/jmax {
		return amax / jmax, dv/amax - amax/jmax
	}
	return math.Sqrt(dv / jmax), 0
}

// SampleJerk integrates an S-curve profile (constant-jerk segments)
// into uniformly spaced samples starting from rest at position s0.
// Acceleration is carried across segments, which keeps it continuous
// at every phase switch by construction.
func SampleJerk(segs []JerkSeg, s0, dt float64) []Sample {
	if dt <= 0 {
		dt = 1e-3
	}
	s, v, a := s0, 0.0, 0.0
	samples := []Sample{{T: 0, S: s}}
	t := 0.0
	for _, seg := range segs {
		end := seg.T0 + seg.Dur
		for t < end-segEps {
			h := math.Min(dt, end-t)
			s = s + v*h + 0.5*a*h*h + seg.J*h*h*h/6
			v = v + a*h + 0.5*seg.J*h*h
			a = a + seg.J*h
			t += h
			// Skip emitting samples for degenerately small steps: they
			// only amplify floating-point noise in finite-difference
			// jerk checks downstream.
			if h > 1e-9 {
				samples = append(samples, Sample{T: t, S: s, V: v, A: a, J: seg.J})
			}
		}
	}
	return samples
}
