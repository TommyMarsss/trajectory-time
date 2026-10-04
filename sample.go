package trajectory

// State is a kinematic sample along the path at time T.
// S is arc length, V is path velocity and A is path acceleration
// (A = dV/dt; jerk is its time derivative).
type State struct {
	T float64
	S float64
	V float64
	A float64
}

func clampTime(t, total float64) float64 {
	if t < 0 {
		return 0
	}
	if t > total {
		return total
	}
	return t
}

// Sample returns the kinematic state at time t. Times outside [0, Duration]
// are clamped to the endpoints.
func (p *Profile) Sample(t float64) State {
	if p.Kind == "s-curve" {
		return sampleSCurve(p, t)
	}
	return sampleTrapezoid(p, t)
}

func sampleTrapezoid(p *Profile, t float64) State {
	T := p.Duration()
	t = clampTime(t, T)

	t1 := p.tAcc
	t2 := p.tAcc + p.tCruise
	s1 := (p.vPeak*p.vPeak - p.V0*p.V0) / (2 * p.AMax)
	s2 := s1 + p.vPeak*p.tCruise

	switch {
	case t <= t1+Eps:
		return State{T: t, S: p.V0*t + p.AMax*t*t/2, V: p.V0 + p.AMax*t, A: p.AMax}
	case t <= t2+Eps:
		tau := t - t1
		return State{T: t, S: s1 + p.vPeak*tau, V: p.vPeak, A: 0}
	default:
		tau := t - t2
		s := s2 + p.vPeak*tau - p.AMax*tau*tau/2
		return State{T: t, S: s, V: p.vPeak - p.AMax*tau, A: -p.AMax}
	}
}

// JerkAt returns the commanded jerk at time t. Trapezoid profiles carry
// infinite jerk in theory; the method reports the sign-changing impulses at
// ramp boundaries only for completeness, callers should not rely on it.
func (p *Profile) JerkAt(t float64) float64 {
	if p.Kind != "s-curve" {
		return 0
	}
	return jerkSCurve(p, clampTime(t, p.Duration()))
}

// SampleN returns n+1 uniformly spaced samples over [0, Duration], endpoints
// included.
func (p *Profile) SampleN(n int) []State {
	if n < 1 {
		n = 1
	}
	out := make([]State, n+1)
	T := p.Duration()
	for i := 0; i <= n; i++ {
		out[i] = p.Sample(T * float64(i) / float64(n))
	}
	return out
}

// Segment is a human-readable description of one phase of a profile.
type Segment struct {
	Name     string
	T0, T1   float64
	Duration float64
	Jerk     float64
}

// Segments returns the non-degenerate phases of the profile in time order.
// Zero-length phases are skipped (they are the degenerations required by
// extreme limit combinations).
func (p *Profile) Segments() []Segment {
	if p.Kind != "s-curve" {
		var segs []Segment
		add := func(name string, d float64, jerk float64) {
			if d > Eps {
				segs = append(segs, Segment{Name: name, Duration: d, Jerk: jerk})
			}
		}
		add("acceleration", p.tAcc, 0)
		add("cruise", p.tCruise, 0)
		add("deceleration", p.tDec, 0)
		for i := range segs {
			if i == 0 {
				segs[i].T0 = 0
			} else {
				segs[i].T0 = segs[i-1].T1
			}
			segs[i].T1 = segs[i].T0 + segs[i].Duration
		}
		return segs
	}
	return sCurveSegments(p)
}
