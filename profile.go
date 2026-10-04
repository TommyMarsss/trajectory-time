// Package trajectory implements time-optimal velocity planning along a fixed
// geometric path.
//
// Planning is performed on the scalar arc-length coordinate s of the path.
// Given start/end velocities and symmetric limits on velocity, acceleration
// and jerk, the planner returns the fastest possible velocity profile:
//
//   - Trapezoid profiles when only velocity and acceleration are bounded
//     (constant-acceleration ramps plus an optional cruise segment),
//   - 7-segment S-curve profiles when jerk is bounded as well (constant-jerk
//     transitions, constant-acceleration ramps and an optional cruise segment).
//
// Both planners handle the short-path case: if the path is too short to ever
// reach the velocity limit, the profile degenerates to a triangular
// (trapezoid) respectively peak-limited (S-curve) profile instead of
// producing a negative cruise/constant-acceleration duration.
package trajectory

import (
	"errors"
	"math"
)

// Eps is the tolerance used to decide that a segment duration has degenerated
// to zero. Segment durations are computed from physical quantities; values at
// or below this threshold are treated as exact degenerations and skipped.
const Eps = 1e-9

// Profile describes a fully resolved time-optimal velocity profile along a
// path of arc length L.
type Profile struct {
	Kind string // "trapezoid" or "s-curve"

	V0    float64 // start velocity
	VEnd  float64 // end velocity
	VMax  float64 // velocity limit
	AMax  float64 // acceleration limit (symmetric)
	JMax  float64 // jerk limit (0 for trapezoid profiles)
	L     float64 // path length
	Reach bool    // whether the velocity limit is actually reached

	// Trapezoid durations (seconds); tCruise == 0 means a triangular profile.
	tAcc, tCruise, tDec float64
	// Velocity at the acceleration/deceleration switch for triangular
	// profiles; equals VMax when the limit is reached.
	vPeak float64

	// S-curve durations of the six ramp phases (cruise is separate in tCr):
	//
	//	0 jerk-up acceleration     3 jerk-down deceleration
	//	1 constant acceleration    4 constant deceleration
	//	2 jerk-down acceleration   5 jerk-up deceleration
	//
	// Any phase may be zero when the corresponding extremum is not reached.
	t [6]float64
	// Peak velocity reached in the unreachable-limit S-curve case.
	vPeakS float64
	tCr    float64 // cruise duration for S-curves
}

// Limits collects the kinematic limits of a planning request. JMax == 0
// disables jerk limiting (trapezoid profile).
type Limits struct {
	VMax float64
	AMax float64
	JMax float64
}

var (
	errNonPositive = errors.New("trajectory: length, VMax and AMax must be positive")
	errJerk        = errors.New("trajectory: JMax must be non-negative")
	errVelocity    = errors.New("trajectory: V0 and VEnd must lie in [0, VMax]")
)

func validate(L float64, v0, vEnd float64, lim Limits) error {
	if L <= 0 || lim.VMax <= 0 || lim.AMax <= 0 {
		return errNonPositive
	}
	if lim.JMax < 0 {
		return errJerk
	}
	if v0 < -Eps || vEnd < -Eps || v0 > lim.VMax+1e-7*max(1, lim.VMax) || vEnd > lim.VMax+1e-7*max(1, lim.VMax) {
		return errVelocity
	}
	return nil
}

// Plan computes the time-optimal profile for travelling arc length L from
// velocity v0 to velocity vEnd under the given limits.
func Plan(L, v0, vEnd float64, lim Limits) (*Profile, error) {
	v0 = math.Max(0, v0)
	vEnd = math.Max(0, vEnd)
	if err := validate(L, v0, vEnd, lim); err != nil {
		return nil, err
	}
	if lim.JMax == 0 {
		return planTrapezoid(L, v0, vEnd, lim)
	}
	return planSCurve(L, v0, vEnd, lim)
}

// Duration returns the total travel time of the profile.
func (p *Profile) Duration() float64 {
	if p.Kind == "trapezoid" {
		return p.tAcc + p.tCruise + p.tDec
	}
	d := p.tCr
	for _, t := range p.t {
		d += t
	}
	return d
}
