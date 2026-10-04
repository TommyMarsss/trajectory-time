package planner

import (
	"math"
	"testing"
)

const tol = 1e-9

func sumDurAccel(segs []AccelSeg) float64 {
	t := 0.0
	for _, s := range segs {
		t += s.Dur
	}
	return t
}

func sumDurJerk(segs []JerkSeg) float64 {
	t := 0.0
	for _, s := range segs {
		t += s.Dur
	}
	return t
}

// Trapezoid on a long path: Vmax is reached and a cruise phase exists.
func TestTrapezoidReachesVmax(t *testing.T) {
	lim := Limits{Vmax: 2, Amax: 1, Jmax: 1}
	segs, err := PlanTrapezoid(10, 0, 0, lim)
	if err != nil {
		t.Fatal(err)
	}
	if len(segs) != 3 {
		t.Fatalf("expected 3 phases (accel/cruise/decel), got %d: %+v", len(segs), segs)
	}
	// dUp = dDown = 2, cruise distance = 6, times 2/3/2 -> total 7.
	if got := sumDurAccel(segs); math.Abs(got-7) > tol {
		t.Fatalf("total time = %v, want 7", got)
	}
	if segs[1].A != 0 || math.Abs(segs[1].Dur-3) > tol {
		t.Fatalf("cruise phase = %+v, want 3s at zero accel", segs[1])
	}
	samples := SampleAccel(segs, 0, 0, 0.001)
	if got := samples[len(samples)-1].S; math.Abs(got-10) > 1e-6 {
		t.Fatalf("final position = %v, want 10", got)
	}
	if v := Validate(samples, lim, 1e-9, false); len(v) != 0 {
		t.Fatalf("unexpected violations: %+v", v)
	}
}

// Trapezoid on a short path: Vmax unreachable, must degenerate to a
// triangle with no cruise phase (and no negative cruise duration).
func TestTrapezoidTriangular(t *testing.T) {
	lim := Limits{Vmax: 10, Amax: 1, Jmax: 1}
	segs, err := PlanTrapezoid(1, 0, 0, lim)
	if err != nil {
		t.Fatal(err)
	}
	if len(segs) != 2 {
		t.Fatalf("expected 2 phases (triangle), got %d: %+v", len(segs), segs)
	}
	for _, s := range segs {
		if s.Dur <= 0 {
			t.Fatalf("non-positive segment duration: %+v", s)
		}
	}
	// vp = sqrt(a*L) = 1, total = 2*sqrt(L/a) = 2.
	if got := sumDurAccel(segs); math.Abs(got-2) > tol {
		t.Fatalf("total time = %v, want 2", got)
	}
	samples := SampleAccel(segs, 0, 0, 0.0005)
	peak := 0.0
	for _, sm := range samples {
		peak = math.Max(peak, sm.V)
	}
	if peak >= lim.Vmax {
		t.Fatalf("peak velocity %v should stay below Vmax %v", peak, lim.Vmax)
	}
	if math.Abs(peak-1) > 1e-6 {
		t.Fatalf("peak velocity = %v, want 1", peak)
	}
	if got := samples[len(samples)-1].S; math.Abs(got-1) > 1e-6 {
		t.Fatalf("final position = %v, want 1", got)
	}
}

// Distance too short to connect the boundary velocities: infeasible.
func TestTrapezoidInfeasible(t *testing.T) {
	lim := Limits{Vmax: 10, Amax: 1, Jmax: 1}
	// Reaching v1=2 from rest needs (2^2)/(2*1) = 2 m; only 0.5 given.
	if _, err := PlanTrapezoid(0.5, 0, 2, lim); err != ErrInfeasible {
		t.Fatalf("err = %v, want ErrInfeasible", err)
	}
	if _, err := PlanTrapezoid(1, 5, 0, lim); err != ErrInfeasible {
		t.Fatalf("err = %v, want ErrInfeasible", err)
	}
}

// S-curve on a long path: all seven phases present, theory-checked time.
func TestSCurveFullProfile(t *testing.T) {
	lim := Limits{Vmax: 2, Amax: 1, Jmax: 1}
	segs, err := PlanSCurve(20, lim)
	if err != nil {
		t.Fatal(err)
	}
	if len(segs) != 7 {
		t.Fatalf("expected 7 phases, got %d: %+v", len(segs), segs)
	}
	// tj = 1, ta = 2/1-1 = 1, ramp distance = 2*(2*1+1) = 6,
	// tv = (20-6)/2 = 7, total = 4*1 + 2*1 + 7 = 13.
	if got := sumDurJerk(segs); math.Abs(got-13) > tol {
		t.Fatalf("total time = %v, want 13", got)
	}
	samples := SampleJerk(segs, 0, 0.001)
	if got := samples[len(samples)-1].S; math.Abs(got-20) > 1e-6 {
		t.Fatalf("final position = %v, want 20", got)
	}
	if v := Validate(samples, lim, 1e-6, true); len(v) != 0 {
		t.Fatalf("unexpected violations: %+v", v)
	}
}

// S-curve with extreme limits: Vmax < Amax^2/Jmax, so the jerk limit
// prevents ever reaching Amax and the constant-acceleration phases
// degenerate to zero length and must be skipped.
func TestSCurveDegenerateConstAccel(t *testing.T) {
	lim := Limits{Vmax: 10, Amax: 5, Jmax: 1}
	segs, err := PlanSCurve(2, lim)
	if err != nil {
		t.Fatal(err)
	}
	// vp = (L*sqrt(J)/2)^(2/3) = 1, tj = 1, ta = 0, tv = 0:
	// only the four jerk phases survive.
	if len(segs) != 4 {
		t.Fatalf("expected 4 phases (const-accel and cruise degenerated), got %d: %+v", len(segs), segs)
	}
	for _, s := range segs {
		if s.Dur <= 0 {
			t.Fatalf("non-positive segment duration: %+v", s)
		}
		if s.J == 0 {
			t.Fatalf("zero-jerk phase should have been skipped: %+v", s)
		}
	}
	if got := sumDurJerk(segs); math.Abs(got-4) > tol {
		t.Fatalf("total time = %v, want 4", got)
	}
	samples := SampleJerk(segs, 0, 0.0005)
	peakA := 0.0
	for _, sm := range samples {
		peakA = math.Max(peakA, math.Abs(sm.A))
	}
	if peakA >= lim.Amax {
		t.Fatalf("peak accel %v should stay below Amax %v (jerk-limited)", peakA, lim.Amax)
	}
	if got := samples[len(samples)-1].S; math.Abs(got-2) > 1e-6 {
		t.Fatalf("final position = %v, want 2", got)
	}
	if v := Validate(samples, lim, 1e-6, true); len(v) != 0 {
		t.Fatalf("unexpected violations: %+v", v)
	}
}

// S-curve on a medium path: Amax reached but Vmax not, so only the
// cruise phase degenerates.
func TestSCurveNoCruise(t *testing.T) {
	lim := Limits{Vmax: 10, Amax: 2, Jmax: 2}
	segs, err := PlanSCurve(5, lim)
	if err != nil {
		t.Fatal(err)
	}
	if len(segs) != 6 {
		t.Fatalf("expected 6 phases (cruise degenerated), got %d: %+v", len(segs), segs)
	}
	// vA = 2, vp = (sqrt(4+40)-2)/2 = (sqrt(44)-2)/2, tj = 1,
	// ta = vp/2 - 1, total = 4*tj + 2*ta.
	vp := 0.5 * (math.Sqrt(44) - 2)
	want := 4*1 + 2*(vp/2-1)
	if got := sumDurJerk(segs); math.Abs(got-want) > tol {
		t.Fatalf("total time = %v, want %v", got, want)
	}
	samples := SampleJerk(segs, 0, 0.0005)
	if got := samples[len(samples)-1].S; math.Abs(got-5) > 1e-6 {
		t.Fatalf("final position = %v, want 5", got)
	}
	if v := Validate(samples, lim, 1e-6, true); len(v) != 0 {
		t.Fatalf("unexpected violations: %+v", v)
	}
}

// The validator must localize injected violations in time and type.
func TestValidateDetectsInjectedViolations(t *testing.T) {
	lim := Limits{Vmax: 2, Amax: 1, Jmax: 2}
	segs, err := PlanSCurve(20, lim)
	if err != nil {
		t.Fatal(err)
	}
	base := SampleJerk(segs, 0, 0.001)
	if v := Validate(base, lim, 1e-6, true); len(v) != 0 {
		t.Fatalf("baseline trajectory should be clean, got %+v", v)
	}
	mid := len(base) / 2

	// Velocity violation.
	s := append([]Sample(nil), base...)
	s[mid].V = lim.Vmax * 1.5
	v := Validate(s, lim, 1e-6, true)
	if len(v) == 0 || v[0].Type != ViolateVelocity || math.Abs(v[0].T-base[mid].T) > 1e-12 {
		t.Fatalf("velocity injection not localized: %+v", v)
	}

	// Acceleration violation.
	s = append([]Sample(nil), base...)
	s[mid].A = lim.Amax * 2
	v = Validate(s, lim, 1e-6, true)
	foundA := false
	for _, x := range v {
		if x.Type == ViolateAcceleration && math.Abs(x.T-base[mid].T) < 1e-12 {
			foundA = true
		}
	}
	if !foundA {
		t.Fatalf("acceleration injection not localized: %+v", v)
	}

	// Acceleration discontinuity -> jerk violation at the jump.
	s = append([]Sample(nil), base...)
	for i := mid; i < len(s); i++ {
		s[i].A += 0.9 * lim.Amax
	}
	v = Validate(s, lim, 1e-6, true)
	foundJ := false
	for _, x := range v {
		if x.Type == ViolateJerk && math.Abs(x.T-base[mid].T) < 1e-12 {
			foundJ = true
		}
	}
	if !foundJ {
		t.Fatalf("jerk injection not localized: %+v", v)
	}
}

// Total duration must match the theoretical optimum for long and short
// paths under both profile models.
func TestTotalDurationMatchesTheory(t *testing.T) {
	cases := []struct {
		name string
		L    float64
		lim  Limits
		want float64
	}{
		{"trap-long", 10, Limits{Vmax: 2, Amax: 1, Jmax: 1}, 7},
		{"trap-short", 1, Limits{Vmax: 10, Amax: 1, Jmax: 1}, 2},
		{"scurve-long", 20, Limits{Vmax: 2, Amax: 1, Jmax: 1}, 13},
		{"scurve-short", 2, Limits{Vmax: 10, Amax: 5, Jmax: 1}, 4},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			p, err := NewPath([]float64{0, c.L}, []float64{0, 0})
			if err != nil {
				t.Fatal(err)
			}
			kind := Trapezoid
			if c.name[:6] == "scurve" {
				kind = SCurve
			}
			tr, err := PlanPath(p, c.lim, kind, 0.0005)
			if err != nil {
				t.Fatal(err)
			}
			if math.Abs(tr.TotalTime-c.want) > 1e-9 {
				t.Fatalf("total time = %v, want %v", tr.TotalTime, c.want)
			}
		})
	}
}

// Stitched multi-segment paths: position/velocity continuous at every
// stitch, S-curve acceleration continuous too, no constraint violated.
func TestStitchedPathContinuity(t *testing.T) {
	lim := Limits{Vmax: 2, Amax: 1, Jmax: 1}
	p, err := NewPath([]float64{0, 3, 3, 6}, []float64{0, 0, 2, 2})
	if err != nil {
		t.Fatal(err)
	}
	tr, err := PlanPath(p, lim, SCurve, 0.001)
	if err != nil {
		t.Fatal(err)
	}
	if v := Validate(tr.Samples, lim, 1e-6, true); len(v) != 0 {
		t.Fatalf("stitched S-curve violates constraints: %+v", v[:3])
	}
	// Velocity vanishes at the stitch arc lengths (3 and 5).
	for _, sJunction := range []float64{3, 5} {
		best := math.Inf(1)
		for _, sm := range tr.Samples {
			if math.Abs(sm.S-sJunction) < 0.01 {
				best = math.Min(best, math.Abs(sm.V))
			}
		}
		if best > 1e-6 {
			t.Fatalf("velocity at junction s=%v not zero: min |v| = %v", sJunction, best)
		}
	}
	if got := tr.Samples[len(tr.Samples)-1].S; math.Abs(got-p.Length()) > 1e-6 {
		t.Fatalf("final arc length = %v, want %v", got, p.Length())
	}
}

// Property sweep: for a grid of distances and limits, plans must never
// contain non-positive durations, must cover exactly the distance and
// must satisfy every constraint.
func TestPlanSweepNeverIllegal(t *testing.T) {
	lims := []Limits{
		{Vmax: 2, Amax: 1, Jmax: 1},
		{Vmax: 1.5, Amax: 1.2, Jmax: 2},
		{Vmax: 10, Amax: 5, Jmax: 1},
		{Vmax: 0.5, Amax: 0.1, Jmax: 0.05},
	}
	for _, lim := range lims {
		for _, L := range []float64{0.01, 0.1, 0.37, 1, 2.9, 5, 20, 100} {
			ts, err := PlanTrapezoid(L, 0, 0, lim)
			if err != nil {
				t.Fatalf("trapezoid L=%v lim=%+v: %v", L, lim, err)
			}
			for _, s := range ts {
				if s.Dur <= 0 {
					t.Fatalf("trapezoid L=%v lim=%+v: non-positive dur %+v", L, lim, s)
				}
			}
			sa := SampleAccel(ts, 0, 0, 0.001)
			if got := sa[len(sa)-1].S; math.Abs(got-L) > 1e-6 {
				t.Fatalf("trapezoid L=%v lim=%+v: covered %v", L, lim, got)
			}
			if v := Validate(sa, lim, 1e-6, false); len(v) != 0 {
				t.Fatalf("trapezoid L=%v lim=%+v: violations %+v", L, lim, v[0])
			}

			js, err := PlanSCurve(L, lim)
			if err != nil {
				t.Fatalf("scurve L=%v lim=%+v: %v", L, lim, err)
			}
			for _, s := range js {
				if s.Dur <= 0 {
					t.Fatalf("scurve L=%v lim=%+v: non-positive dur %+v", L, lim, s)
				}
			}
			sj := SampleJerk(js, 0, 0.001)
			if got := sj[len(sj)-1].S; math.Abs(got-L) > 1e-6 {
				t.Fatalf("scurve L=%v lim=%+v: covered %v", L, lim, got)
			}
			if v := Validate(sj, lim, 1e-6, true); len(v) != 0 {
				t.Fatalf("scurve L=%v lim=%+v: violations %+v", L, lim, v[0])
			}
		}
	}
}
