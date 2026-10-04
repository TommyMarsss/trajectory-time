package planner

import "math"

// PlanTrapezoid plans a time-optimal trapezoidal velocity profile that
// moves from velocity v0 to velocity v1 over distance L under the
// velocity and acceleration limits in lim.
//
// When L is too short to accelerate to lim.Vmax and decelerate back to
// v1, the profile degenerates to a triangular one: it accelerates to a
// peak velocity vp < lim.Vmax and immediately decelerates, with no
// cruise phase. The planner never emits a negative-duration cruise
// segment; degenerate phases are omitted entirely.
//
// It returns ErrInfeasible when the distance is too short to even
// connect v0 to v1 at the acceleration limit.
func PlanTrapezoid(L, v0, v1 float64, lim Limits) ([]AccelSeg, error) {
	if L < 0 {
		return nil, ErrInfeasible
	}
	if lim.Vmax <= 0 || lim.Amax <= 0 {
		return nil, ErrInfeasible
	}
	if v0 < 0 || v1 < 0 || v0 > lim.Vmax || v1 > lim.Vmax {
		return nil, ErrInfeasible
	}
	a := lim.Amax

	// Feasibility: the distance must at least allow connecting v0 to v1.
	if minD := math.Abs(v1*v1-v0*v0) / (2 * a); L < minD-segEps {
		return nil, ErrInfeasible
	}

	// Distance needed to ramp v0 -> Vmax and Vmax -> v1.
	dUp := (lim.Vmax*lim.Vmax - v0*v0) / (2 * a)
	dDown := (lim.Vmax*lim.Vmax - v1*v1) / (2 * a)

	var tUp, tCruise, tDown, vPeak float64
	if dUp+dDown <= L {
		// Trapezoid: the velocity limit is reached, cruise fills the rest.
		vPeak = lim.Vmax
		tUp = (lim.Vmax - v0) / a
		tDown = (lim.Vmax - v1) / a
		tCruise = (L - dUp - dDown) / lim.Vmax
	} else {
		// Triangle: Vmax is unreachable; solve for the peak velocity.
		// L = (vp^2 - v0^2)/(2a) + (vp^2 - v1^2)/(2a)
		vPeak = math.Sqrt((2*a*L + v0*v0 + v1*v1) / 2)
		tUp = (vPeak - v0) / a
		tDown = (vPeak - v1) / a
		tCruise = 0
	}

	var segs []AccelSeg
	t := 0.0
	push := func(dur, acc float64) {
		if dur <= segEps { // skip degenerate zero/negative-length phases
			return
		}
		segs = append(segs, AccelSeg{T0: t, Dur: dur, A: acc})
		t += dur
	}
	push(tUp, a)
	push(tCruise, 0)
	push(tDown, -a)
	return segs, nil
}

// SampleAccel integrates a trapezoidal profile (constant-acceleration
// segments) into uniformly spaced samples starting from rest position
// s0 with velocity v0. Jerk is reported as 0 inside segments; the
// acceleration discontinuities at segment boundaries are inherent to
// the trapezoidal model.
func SampleAccel(segs []AccelSeg, s0, v0, dt float64) []Sample {
	if dt <= 0 {
		dt = 1e-3
	}
	s, v := s0, v0
	samples := []Sample{{T: 0, S: s, V: v}}
	t := 0.0
	for _, seg := range segs {
		end := seg.T0 + seg.Dur
		// The sampler assumes segments are contiguous from t=0.
		for t < end-segEps {
			h := math.Min(dt, end-t)
			s = s + v*h + 0.5*seg.A*h*h
			v = v + seg.A*h
			t += h
			// Skip emitting samples for degenerately small steps: they
			// only amplify floating-point noise in finite-difference
			// checks downstream.
			if h > 1e-9 {
				samples = append(samples, Sample{T: t, S: s, V: v, A: seg.A})
			}
		}
	}
	return samples
}
