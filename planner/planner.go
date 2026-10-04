// Package planner implements time-optimal velocity planning along a
// geometric path under velocity, acceleration and jerk limits.
//
// Two profile models are provided:
//
//   - Trapezoidal profile (PlanTrapezoid): piecewise-constant acceleration.
//     Degenerates to a triangular profile (no cruise phase) when the path is
//     too short to reach the velocity limit.
//   - S-curve profile (PlanSCurve): piecewise-constant jerk, giving a
//     continuous acceleration signal. Acceleration segments degenerate to
//     zero length when the jerk limit prevents reaching the acceleration
//     limit; such segments are identified and skipped.
//
// Profiles for individual path segments are stitched together by PlanPath,
// which plans each straight segment between waypoints with zero boundary
// velocities (the path comes to rest at every corner).
package planner

import "errors"

// Limits bundles the kinematic constraints of the trajectory.
type Limits struct {
	Vmax float64 // velocity limit (m/s), > 0
	Amax float64 // acceleration limit (m/s^2), > 0
	Jmax float64 // jerk limit (m/s^3), > 0; only used by the S-curve planner
}

// Valid reports whether all limits are strictly positive.
func (l Limits) Valid() bool {
	return l.Vmax > 0 && l.Amax > 0 && l.Jmax > 0
}

// Sample is one point of a sampled trajectory.
type Sample struct {
	T float64 // time (s)
	S float64 // arc-length position along the path (m)
	V float64 // velocity (m/s)
	A float64 // acceleration (m/s^2)
	J float64 // jerk (m/s^3); 0 inside trapezoidal segments
}

// AccelSeg is a constant-acceleration segment of a trapezoidal profile.
type AccelSeg struct {
	T0  float64 // start time (s)
	Dur float64 // duration (s), always > 0 in a well-formed plan
	A   float64 // constant acceleration (m/s^2)
}

// JerkSeg is a constant-jerk segment of an S-curve profile.
type JerkSeg struct {
	T0  float64 // start time (s)
	Dur float64 // duration (s), always > 0 in a well-formed plan
	J   float64 // constant jerk (m/s^3)
}

// ErrInfeasible is returned when the boundary velocities cannot be
// connected within the given distance under the acceleration limit.
var ErrInfeasible = errors.New("planner: infeasible boundary conditions for distance and limits")

// segEps is the tolerance below which a computed segment duration is
// treated as zero and the segment is skipped. This is what keeps
// degenerate S-curve phases (and trapezoid phases) from producing
// illegal negative- or zero-duration segments.
const segEps = 1e-12
