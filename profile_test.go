package trajectory

import (
	"math"
	"testing"
)

const tol = 1e-7

func approxEq(t *testing.T, name string, got, want, eps float64) {
	t.Helper()
	if math.Abs(got-want) > eps {
		t.Errorf("%s = %.9f, want %.9f (diff %.3e)", name, got, want, got-want)
	}
}

// ---------------------------------------------------------------- trapezoid

func TestTrapezoidReachable(t *testing.T) {
	// v0=vEnd=0, VMax=10, a=2, L=120:
	// ramp distance = 2*100/4 = 50, tAcc=tDec=5, cruise 70/10=7, T=17.
	p, err := Plan(120, 0, 0, Limits{VMax: 10, AMax: 2})
	if err != nil {
		t.Fatal(err)
	}
	if p.Kind != "trapezoid" || !p.Reach {
		t.Fatalf("expected reachable trapezoid, got kind=%s reach=%v", p.Kind, p.Reach)
	}
	approxEq(t, "tAcc", p.tAcc, 5, tol)
	approxEq(t, "tCruise", p.tCruise, 7, tol)
	approxEq(t, "tDec", p.tDec, 5, tol)
	approxEq(t, "T", p.Duration(), 17, tol)
	segs := p.Segments()
	if len(segs) != 3 {
		t.Fatalf("want 3 segments, got %d", len(segs))
	}
	assertEndpointAndLimits(t, p, true)
	// Velocity must actually hit the limit during the cruise.
	maxV := maxVelocity(p)
	approxEq(t, "max v", maxV, 10, 1e-6)
}

func TestTrapezoidTriangularShortPath(t *testing.T) {
	// L=12: ramp distance to 10 would be 50 > 12 -> triangular.
	p, err := Plan(12, 0, 0, Limits{VMax: 10, AMax: 2})
	if err != nil {
		t.Fatal(err)
	}
	if p.Reach {
		t.Fatal("VMax must not be reached on a short path")
	}
	if p.tCruise != 0 {
		t.Fatalf("cruise duration must be exactly zero in triangular case, got %.6f (no negative cruise allowed)", p.tCruise)
	}
	if len(p.Segments()) != 2 {
		t.Fatalf("triangular profile has 2 segments, got %d", len(p.Segments()))
	}
	// v* = sqrt(2*2*12/2) = sqrt(24)
	vWant := math.Sqrt(24)
	approxEq(t, "vPeak", p.vPeak, vWant, 1e-9)
	if p.vPeak >= p.VMax {
		t.Fatalf("peak velocity %.6f must be below VMax %.6f", p.vPeak, p.VMax)
	}
	approxEq(t, "T", p.Duration(), 2*vWant/2, 1e-9)
	assertEndpointAndLimits(t, p, true)
}

func TestTrapezoidTriangularAsymmetricEndpoints(t *testing.T) {
	// v0=2, vEnd=1, a=2, L=5: distance to reach VMax is far above L.
	p, err := Plan(5, 2, 1, Limits{VMax: 10, AMax: 2})
	if err != nil {
		t.Fatal(err)
	}
	if p.Reach || p.tCruise != 0 {
		t.Fatalf("expected triangular, reach=%v cruise=%.4f", p.Reach, p.tCruise)
	}
	// Closed form v* = sqrt((2aL+v0²+vEnd²)/2).
	vWant := math.Sqrt((20 + 4 + 1) / 2.0)
	approxEq(t, "vPeak", p.vPeak, vWant, 1e-9)
	assertEndpointAndLimits(t, p, true)
}

func TestTrapezoidInfeasible(t *testing.T) {
	// To drop from v=10 to 0 with a=2 needs at least 25 length.
	_, err := Plan(1, 10, 0, Limits{VMax: 10, AMax: 2})
	if err == nil {
		t.Fatal("expected infeasibility error")
	}
	var ie *InfeasibleError
	if !asError(err, &ie) {
		t.Fatalf("want *InfeasibleError, got %T: %v", err, err)
	}
	if ie.MinLength != 25 {
		t.Errorf("MinLength = %v, want 25", ie.MinLength)
	}
}

func TestTrapezoidJustTouchesLimit(t *testing.T) {
	// L exactly equal to the ramp distance: cruise is zero but the limit is
	// reached at exactly one instant; no negative duration anywhere.
	p, err := Plan(50, 0, 0, Limits{VMax: 10, AMax: 2})
	if err != nil {
		t.Fatal(err)
	}
	if !p.Reach || p.tCruise != 0 {
		t.Fatalf("expected touch-limit profile, reach=%v cruise=%.4f", p.Reach, p.tCruise)
	}
	for i, d := range []float64{p.tAcc, p.tCruise, p.tDec} {
		if d < 0 {
			t.Fatalf("duration %d negative: %v", i, d)
		}
	}
	assertEndpointAndLimits(t, p, true)
}

// ------------------------------------------------------------------- s-curve

func TestSCurveFullSevenPhases(t *testing.T) {
	// v0=vEnd=0, VMax=10, a=2, j=10, L=120.
	// tj = a/j = 0.2; tc = (10 - 10*0.04)/(10*0.2) = 4.8.
	// ramp distance per half: vLo=0: j tj^3 + 1.5 j tj² tc + 0.5 j tj tc²
	//   = 10*.008 + 15*.04*4.8 + 5*.2*23.04 = 0.08+2.88+23.04 = 26.
	// total ramps 52; cruise (120-52)/10 = 6.8; T = 2*(0.4+4.8)+6.8 = 17.2.
	p, err := Plan(120, 0, 0, Limits{VMax: 10, AMax: 2, JMax: 10})
	if err != nil {
		t.Fatal(err)
	}
	if !p.Reach {
		t.Fatal("expected VMax reached")
	}
	want := [6]float64{0.2, 4.8, 0.2, 0.2, 4.8, 0.2}
	for i := range want {
		approxEq(t, "phase "+string(rune('0'+i)), p.t[i], want[i], 1e-9)
	}
	approxEq(t, "cruise", p.tCr, 6.8, 1e-9)
	approxEq(t, "T", p.Duration(), 17.2, 1e-9)
	if len(p.Segments()) != 7 {
		t.Fatalf("want 7 non-degenerate segments, got %d", len(p.Segments()))
	}
	assertEndpointAndLimits(t, p, true)
	assertAccelerationContinuity(t, p)
}

func TestSCurveShortPathNoCruise(t *testing.T) {
	p, err := Plan(6, 0, 0, Limits{VMax: 10, AMax: 2, JMax: 10})
	if err != nil {
		t.Fatal(err)
	}
	if p.Reach {
		t.Fatal("VMax unreachable on short path")
	}
	if p.tCr != 0 {
		t.Fatalf("cruise must vanish, got %.6f", p.tCr)
	}
	if p.vPeakS >= p.VMax {
		t.Fatalf("peak %.6f >= VMax", p.vPeakS)
	}
	for i, d := range p.t {
		if d < 0 {
			t.Fatalf("phase %d has negative duration %v", i, d)
		}
	}
	assertEndpointAndLimits(t, p, true)
	assertAccelerationContinuity(t, p)
}

func TestSCurveDegenerateConstantAccel(t *testing.T) {
	// a=8, j=20, threshold dv* = a²/j = 3.2.
	// accelerating half dv=2 < 3.2 -> tc must be exactly 0 (degenerate).
	// decelerating half dv=4 > 3.2 -> tc > 0 must survive.
	p, err := Plan(40, 8, 6, Limits{VMax: 10, AMax: 8, JMax: 20})
	if err != nil {
		t.Fatal(err)
	}
	if p.t[1] != 0 {
		t.Errorf("accelerating constant-accel phase must degenerate to 0, got %.6f", p.t[1])
	}
	if p.t[4] <= 0 {
		t.Errorf("decelerating constant-accel phase must survive, got %.6f", p.t[4])
	}
	// No negative durations anywhere.
	for i, d := range p.t {
		if d < 0 {
			t.Fatalf("phase %d negative: %v", i, d)
		}
	}
	// Expected accelerating jerk time sqrt(2/20) and the ramp must close.
	approxEq(t, "tj acc", p.t[0], math.Sqrt(0.1), 1e-9)
	assertEndpointAndLimits(t, p, true)
	assertAccelerationContinuity(t, p)

	// Segments() must skip the zero-length phase.
	names := map[string]bool{}
	for _, s := range p.Segments() {
		if s.Duration <= 0 {
			t.Fatalf("reported non-positive segment %q: %v", s.Name, s.Duration)
		}
		names[s.Name] = true
	}
	if names["constant acceleration"] {
		t.Error("degenerate 'constant acceleration' phase must be skipped in Segments()")
	}
	if !names["constant deceleration"] {
		t.Error("non-degenerate 'constant deceleration' phase must be present")
	}
}

func TestSCurveBothHalvesDegenerate(t *testing.T) {
	// Tiny velocity change on a short path: both constant-acceleration
	// phases vanish AND there is no cruise. v0=vEnd=0.5 and the reached
	// peak (~0.536 for L=0.06) stays below 0.5+a²/j = 1.125, so both
	// halves are pure double-jerk ramps.
	p, err := Plan(0.06, 0.5, 0.5, Limits{VMax: 10, AMax: 5, JMax: 40})
	if err != nil {
		t.Fatal(err)
	}
	if p.t[1] != 0 || p.t[4] != 0 {
		t.Fatalf("both tc must be zero, got %.4f %.4f", p.t[1], p.t[4])
	}
	if p.tCr != 0 {
		t.Fatalf("cruise must be zero, got %.4f", p.tCr)
	}
	if p.t[0] <= 0 || p.Duration() <= 0 {
		t.Fatalf("jerk phases and total duration must be positive, tj=%.6f T=%.6f", p.t[0], p.Duration())
	}
	for _, s := range p.Segments() {
		if s.Duration <= 0 {
			t.Fatalf("non-positive segment %q", s.Name)
		}
	}
	assertEndpointAndLimits(t, p, true)
	assertAccelerationContinuity(t, p)
}

func TestSCurveInfeasible(t *testing.T) {
	_, err := Plan(0.001, 8, 0, Limits{VMax: 10, AMax: 2, JMax: 10})
	if err == nil {
		t.Fatal("expected infeasibility error")
	}
}

// ----------------------------------------------- duration vs theory, sweeps

func TestDurationMatchesTheorySweep(t *testing.T) {
	cfgs := []struct {
		name             string
		v0, vE, vM, a, j float64
		lengths          []float64
	}{
		{"trap", 0, 0, 10, 2, 0, []float64{0.1, 1, 4.9, 5, 12, 49.9999, 50, 51, 120, 1000}},
		{"trap-endpoints", 2, 1, 10, 2, 0, []float64{0.8, 2, 5, 50, 300}},
		{"scurve", 0, 0, 10, 2, 10, []float64{0.2, 1, 3, 6, 51.9, 52, 53, 120, 500}},
		{"scurve-tight-j", 0, 0, 10, 2, 1, []float64{5, 20, 80, 100, 300}},
		{"scurve-small-dv", 8, 6, 10, 8, 20, []float64{4.5, 5, 10, 40, 80}},
		{"scurve-endpoints", 1.5, 0.5, 10, 3, 12, endpointLengths()},
	}
	for _, cfg := range cfgs {
		for _, L := range cfg.lengths {
			lim := Limits{VMax: cfg.vM, AMax: cfg.a, JMax: cfg.j}
			p, err := Plan(L, cfg.v0, cfg.vE, lim)
			if err != nil {
				t.Fatalf("%s L=%v: unexpected error %v", cfg.name, L, err)
			}
			th := TheoreticalDuration(p)
			if math.Abs(p.Duration()-th) > 1e-7*math.Max(1, th) {
				t.Errorf("%s L=%v: planner T=%.9f theory T=%.9f", cfg.name, L, p.Duration(), th)
			}
			// Durations must never be negative in any phase.
			for i, d := range p.t {
				if d < -1e-9 {
					t.Errorf("%s L=%v phase %d negative: %v", cfg.name, L, i, d)
				}
			}
			if p.tCruise < 0 || p.tCr < 0 {
				t.Errorf("%s L=%v negative cruise", cfg.name, L)
			}
			assertEndpointAndLimits(t, p, false)
		}
	}
}

func endpointLengths() []float64 {
	return []float64{0.6, 1, 3, 10, 60}
}

// --------------------------------------------- injected violation detection

func TestDetectInjectedVelocityViolation(t *testing.T) {
	p, _ := Plan(120, 0, 0, Limits{VMax: 10, AMax: 2})
	states := p.SampleN(1000)
	cruise := p.Segments()[1]
	for i := range states {
		if states[i].T > cruise.T0 && states[i].T < cruise.T1 {
			states[i].V = 11.5
		}
	}
	rep := ValidateTrajectory(states, Limits{VMax: 10, AMax: 2}, false, ValidationTolerances{})
	if rep.OK {
		t.Fatal("violation must be detected")
	}
	var got *Violation
	for i := range rep.Violations {
		if rep.Violations[i].Type == ViolationVelocity {
			got = &rep.Violations[i]
		}
	}
	if got == nil {
		t.Fatalf("want velocity violation, got %+v", rep.Violations)
	}
	if got.Value < 11.5-1e-9 || got.Limit != 10 {
		t.Errorf("bad violation payload: %+v", got)
	}
	if got.T0 <= cruise.T0 || got.T1 >= cruise.T1 || got.T < got.T0 || got.T > got.T1 {
		t.Errorf("violation time bracket wrong: %+v cruise=[%v,%v]", got, cruise.T0, cruise.T1)
	}
}

func TestDetectInjectedAccelerationAndJerkViolation(t *testing.T) {
	p, _ := Plan(120, 0, 0, Limits{VMax: 10, AMax: 2, JMax: 10})
	states := p.SampleN(1200)
	T := p.Duration()
	for i := range states {
		u := states[i].T / T
		if u > 0.2 && u < 0.3 {
			states[i].A = 2.9
		}
	}
	rep := ValidateTrajectory(states, Limits{VMax: 10, AMax: 2, JMax: 10}, true, ValidationTolerances{})
	kinds := map[ViolationType]int{}
	for _, v := range rep.Violations {
		kinds[v.Type]++
		if v.Type == ViolationAccel && (math.Abs(v.Value-2.9) > 1e-9) {
			t.Errorf("accel peak value wrong: %v", v.Value)
		}
	}
	if kinds[ViolationAccel] == 0 {
		t.Error("acceleration violation not detected")
	}
	if kinds[ViolationJerk] < 2 {
		t.Errorf("two jerk spikes expected at window edges, got %d", kinds[ViolationJerk])
	}
}

func TestCleanProfilesPassValidation(t *testing.T) {
	cfgs := []struct {
		L, v0, vE, vM, a, j float64
	}{
		{120, 0, 0, 10, 2, 0}, {12, 0, 0, 10, 2, 0}, {0.8, 2, 1, 10, 2, 0},
		{120, 0, 0, 10, 2, 10}, {6, 0, 0, 10, 2, 10},
		{40, 8, 6, 10, 8, 20}, {300, 0, 0, 10, 2, 1},
		{3, 1.5, 0.5, 10, 3, 12}, {0.5, 0.5, 0.5, 10, 5, 40},
	}
	for _, c := range cfgs {
		p, err := Plan(c.L, c.v0, c.vE, Limits{VMax: c.vM, AMax: c.a, JMax: c.j})
		if err != nil {
			t.Fatalf("L=%v j=%v: %v", c.L, c.j, err)
		}
		rep := ValidateProfile(p, 0, ValidationTolerances{})
		if !rep.OK {
			t.Errorf("clean profile flagged: L=%v j=%v violations=%v", c.L, c.j, rep.Violations)
		}
	}
}

// -------------------------------------------------------- geometric binding

func TestTrajectoryBinding(t *testing.T) {
	pl := NewPolyline([]Point{{0, 0}, {3, 0}, {3, 4}}) // length 7
	if pl.Length() != 7 {
		t.Fatalf("polyline length = %v, want 7", pl.Length())
	}
	tr, err := PlanPath(pl, 0, 0, Limits{VMax: 10, AMax: 4})
	if err != nil {
		t.Fatal(err)
	}
	end := tr.At(tr.Duration())
	approxEq(t, "end s", end.S, 7, 1e-7)
	approxEq(t, "end x", end.Pos.X, 3, 1e-7)
	approxEq(t, "end y", end.Pos.Y, 4, 1e-7)
	start := tr.At(0)
	if start.Pos != (Point{0, 0}) {
		t.Errorf("start = %+v", start.Pos)
	}
}

// --------------------------------------------------------------- utilities

func assertEndpointAndLimits(t *testing.T, p *Profile, dense bool) {
	t.Helper()
	T := p.Duration()
	end := p.Sample(T)
	start := p.Sample(0)
	if math.Abs(end.S-p.L) > 1e-6*math.Max(1, p.L) {
		t.Errorf("terminal position %.9f != L %.9f", end.S, p.L)
	}
	if math.Abs(end.V-p.VEnd) > 1e-6*math.Max(1, p.VMax) {
		t.Errorf("terminal velocity %.9f != vEnd %.9f", end.V, p.VEnd)
	}
	if math.Abs(start.V-p.V0) > 1e-9 {
		t.Errorf("initial velocity %.9f != v0 %.9f", start.V, p.V0)
	}
	if math.Abs(start.S) > 1e-9 {
		t.Errorf("initial position %.9f != 0", start.S)
	}
	n := 4000
	if !dense {
		n = 2000
	}
	for i := 0; i <= n; i++ {
		tm := T * float64(i) / float64(n)
		st := p.Sample(tm)
		if st.V > p.VMax+1e-7 {
			t.Fatalf("velocity %.9f > VMax at t=%.6f", st.V, tm)
		}
		if st.V < -1e-9 {
			t.Fatalf("negative velocity %.9f at t=%.6f", st.V, tm)
		}
		if math.Abs(st.A) > p.AMax+1e-7 {
			t.Fatalf("|a|=%.9f > AMax at t=%.6f", math.Abs(st.A), tm)
		}
		if p.Kind == "s-curve" {
			if math.Abs(st.A) > p.AMax+1e-7 {
				t.Fatalf("s-curve |a| at t=%.6f", tm)
			}
		}
	}
	// Monotonic non-decreasing position.
	prev := 0.0
	for i := 0; i <= n; i++ {
		st := p.Sample(T * float64(i) / float64(n))
		if st.S < prev-1e-9 {
			t.Fatalf("position decreased at t fraction %d", i)
		}
		prev = st.S
	}
}

// assertAccelerationContinuity verifies that acceleration takes the same
// value on both sides of every S-curve phase boundary (jerk may switch
// sign; acceleration itself must not jump). The finite-difference offset
// naturally accumulates at most J·δ, so the tolerance scales with the jerk
// limit rather than assuming the boundary is sampled exactly.
func assertAccelerationContinuity(t *testing.T, p *Profile) {
	t.Helper()
	T := p.Duration()
	const delta = 1e-7
	tol := 3*p.JMax*delta + 1e-6
	for _, seg := range p.Segments() {
		if seg.T0 <= delta || seg.T0 >= T-delta {
			continue
		}
		aL := p.Sample(seg.T0 - delta).A
		aR := p.Sample(seg.T0 + delta).A
		if math.Abs(aL-aR) > tol {
			t.Errorf("acceleration jump at t=%.6f: %.9f -> %.9f (tol %.2e)", seg.T0, aL, aR, tol)
		}
	}
	// Jerk bounded everywhere (evaluate jerk sign via segment jerks and the
	// achieved peak acceleration).
	for _, seg := range p.Segments() {
		if p.JMax > 0 && math.Abs(seg.Jerk) > p.JMax+1e-9 {
			t.Errorf("jerk %.3f exceeds JMax in segment %q", seg.Jerk, seg.Name)
		}
	}
}

func maxVelocity(p *Profile) float64 {
	m := 0.0
	for i := 0; i <= 4000; i++ {
		v := p.Sample(p.Duration() * float64(i) / 4000).V
		if v > m {
			m = v
		}
	}
	return m
}

func asError(err error, target **InfeasibleError) bool {
	if e, ok := err.(*InfeasibleError); ok {
		*target = e
		return true
	}
	return false
}
