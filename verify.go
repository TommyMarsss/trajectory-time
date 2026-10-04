package trajectory

import (
	"fmt"
	"math"
)

// ViolationType identifies which kinematic constraint was broken.
type ViolationType string

const (
	ViolationVelocity ViolationType = "velocity"
	ViolationAccel    ViolationType = "acceleration"
	ViolationJerk     ViolationType = "jerk"
	ViolationEndpoint ViolationType = "endpoint"
)

// Violation pinpoints one constraint breach. T is the time of the worst
// sample, T0/T1 bracket the full contiguous run of offending samples,
// Value is the observed quantity and Limit the bound that was exceeded.
type Violation struct {
	Type   ViolationType
	T      float64
	Value  float64
	Limit  float64
	T0, T1 float64
	// Detail is set only for structural (non-kinematic) violations.
	Detail string
}

func (v Violation) String() string {
	switch v.Type {
	case ViolationEndpoint:
		return fmt.Sprintf("endpoint violation at t=%.6g: value %.6g, expected %.6g", v.T, v.Value, v.Limit)
	default:
		return fmt.Sprintf("%s limit exceeded in t∈[%.6g, %.6g]: peak |value|=%.6g at t=%.6g (limit %.6g)",
			v.Type, v.T0, v.T1, v.Value, v.T, v.Limit)
	}
}

// ValidationReport is the result of checking a curve against its limits.
type ValidationReport struct {
	OK         bool
	Violations []Violation
}

// ValidationTolerances controls the slack allowed before a sample counts as
// a violation. Zero values select the defaults (1e-6 relative).
type ValidationTolerances struct {
	V, A, J float64
}

func (tol ValidationTolerances) orDefault(lim Limits) (float64, float64, float64) {
	av := tol.V
	if av <= 0 {
		av = 1e-6 * max(1, lim.VMax)
	}
	aa := tol.A
	if aa <= 0 {
		aa = 1e-6 * max(1, lim.AMax)
	}
	aj := tol.J
	if aj <= 0 && lim.JMax > 0 {
		aj = 1e-6 * max(1, lim.JMax)
	}
	return av, aa, aj
}

// ValidateProfile densely checks a planned profile against its own limits.
// step is the nominal sampling resolution; when <= 0 a resolution of
// Duration/2000 is used. All phase boundary times are sampled explicitly.
func ValidateProfile(p *Profile, step float64, tol ValidationTolerances) ValidationReport {
	T := p.Duration()
	if step <= 0 {
		step = T / 2000
	}
	n := int(math.Ceil(T/step)) + 1
	tSet := map[float64]struct{}{}
	states := make([]State, 0, n+16)
	add := func(t float64) {
		t = clampTime(t, T)
		key := math.Round(t / 1e-12)
		if _, ok := tSet[key]; ok {
			return
		}
		tSet[key] = struct{}{}
		states = append(states, p.Sample(t))
	}
	add(0)
	for i := 1; i < n; i++ {
		add(T * float64(i) / float64(n))
	}
	for _, seg := range p.Segments() {
		add(seg.T0)
		add(seg.T1)
	}
	add(T)

	lim := Limits{VMax: p.VMax, AMax: p.AMax, JMax: p.JMax}
	rep := ValidateTrajectory(states, lim, p.Kind == "s-curve", tol)

	// Endpoint / completeness checks.
	if s0 := p.Sample(0); math.Abs(s0.S) > 1e-7*max(1, p.L) ||
		math.Abs(s0.V-p.V0) > 1e-7*max(1, p.VMax) {
		rep.Violations = append(rep.Violations, Violation{
			Type: ViolationEndpoint, T: 0, T0: 0, T1: 0,
			Value: s0.V, Limit: p.V0,
		})
	}
	if sE := p.Sample(T); math.Abs(sE.S-p.L) > 1e-6*max(1, p.L) ||
		math.Abs(sE.V-p.VEnd) > 1e-6*max(1, p.VMax) {
		rep.Violations = append(rep.Violations, Violation{
			Type: ViolationEndpoint, T: T, T0: T, T1: T,
			Value: sE.V, Limit: p.VEnd,
		})
	}
	rep.OK = len(rep.Violations) == 0
	return rep
}

// violationRun accumulates one contiguous run of offending samples.
type violationRun struct {
	typ              ViolationType
	t0, t1           float64
	tWorst           float64
	excessWorst, val float64
	limit            float64
}

// ValidateTrajectory checks an arbitrarily sampled, time-ordered curve
// against velocity/acceleration/jerk limits. It is the entry point for both
// planned profiles and externally supplied curves (including deliberately
// corrupted ones): violations are reported with their exact time range,
// peak time, observed value and constraint type.
//
// Jerk is estimated by central differences of acceleration; when
// jerkLimited is false jerk is not checked (trapezoid profiles legitimately
// contain impulsive jerk at ramp switches).
func ValidateTrajectory(states []State, lim Limits, jerkLimited bool, tol ValidationTolerances) ValidationReport {
	tv, ta, tj := tol.orDefault(lim)
	var rep ValidationReport

	if len(states) == 0 {
		rep.Violations = []Violation{{
			Type: ViolationEndpoint, Detail: "no samples",
		}}
		rep.OK = false
		return rep
	}
	if len(states) < 2 {
		rep.Violations = []Violation{{
			Type: ViolationEndpoint, T: states[0].T,
			Detail: "need at least two samples",
		}}
		rep.OK = false
		return rep
	}

	var runs []violationRun
	active := map[ViolationType]*violationRun{}
	finish := func(typ ViolationType) {
		if r := active[typ]; r != nil {
			runs = append(runs, *r)
			delete(active, typ)
		}
	}
	record := func(typ ViolationType, t, value, excess, bound float64) {
		r := active[typ]
		if r == nil {
			r = &violationRun{typ: typ, t0: t, t1: t, tWorst: t, excessWorst: excess, val: value, limit: bound}
			active[typ] = r
			return
		}
		r.t1 = t
		if excess > r.excessWorst {
			r.excessWorst, r.tWorst, r.val = excess, t, value
		}
	}

	for _, st := range states {
		switch {
		case st.V > lim.VMax+tv:
			record(ViolationVelocity, st.T, st.V, st.V-lim.VMax, lim.VMax)
		case st.V < -tv:
			record(ViolationVelocity, st.T, st.V, -st.V, lim.VMax)
		default:
			finish(ViolationVelocity)
		}

		switch {
		case st.A > lim.AMax+ta:
			record(ViolationAccel, st.T, st.A, st.A-lim.AMax, lim.AMax)
		case st.A < -lim.AMax-ta:
			record(ViolationAccel, st.T, st.A, -st.A-lim.AMax, lim.AMax)
		default:
			finish(ViolationAccel)
		}
	}
	finish(ViolationVelocity)
	finish(ViolationAccel)

	if jerkLimited && lim.JMax > 0 {
		gap := 2.5 * firstDt(states)
		for i := 1; i < len(states)-1; i++ {
			dt := states[i+1].T - states[i-1].T
			if dt <= 0 {
				continue
			}
			j := (states[i+1].A - states[i-1].A) / dt
			t := states[i].T
			if math.Abs(j) > lim.JMax+tj {
				r := active[ViolationJerk]
				if r != nil && t-r.t1 > gap {
					finish(ViolationJerk)
				}
				record(ViolationJerk, t, j, math.Abs(j)-lim.JMax, lim.JMax)
			} else {
				finish(ViolationJerk)
			}
		}
		finish(ViolationJerk)
	}

	for _, r := range runs {
		rep.Violations = append(rep.Violations, Violation{
			Type: r.typ, T: r.tWorst, Value: r.val, Limit: r.limit, T0: r.t0, T1: r.t1,
		})
	}
	rep.OK = len(rep.Violations) == 0
	return rep
}

func firstDt(states []State) float64 {
	if len(states) < 2 {
		return 0
	}
	return states[1].T - states[0].T
}
