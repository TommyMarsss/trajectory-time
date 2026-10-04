package planner

import "math"

// ViolationType identifies which constraint a trajectory violates.
type ViolationType string

const (
	ViolateVelocity     ViolationType = "velocity"
	ViolateAcceleration ViolationType = "acceleration"
	ViolateJerk         ViolationType = "jerk"
)

// Violation pinpoints one constraint violation in a sampled trajectory.
type Violation struct {
	T     float64       // time of the offending sample (s)
	Index int           // sample index
	Type  ViolationType // which constraint is violated
	Value float64       // measured magnitude
	Limit float64       // allowed limit
}

// Validate checks a sampled trajectory against the kinematic limits and
// reports every violation with its time and constraint type.
//
// Velocity and acceleration are checked per sample. Jerk is checked by
// finite-differencing the acceleration samples, which also catches
// acceleration discontinuities at stitched segment boundaries (a jump
// in acceleration shows up as a huge finite-difference jerk). Set
// checkJerk to false for trapezoidal profiles, whose acceleration is
// discontinuous by design.
//
// tol is an absolute tolerance added to every limit.
func Validate(samples []Sample, lim Limits, tol float64, checkJerk bool) []Violation {
	var out []Violation
	for i, sm := range samples {
		if sm.V > lim.Vmax+tol {
			out = append(out, Violation{T: sm.T, Index: i, Type: ViolateVelocity, Value: sm.V, Limit: lim.Vmax})
		}
		if math.Abs(sm.A) > lim.Amax+tol {
			out = append(out, Violation{T: sm.T, Index: i, Type: ViolateAcceleration, Value: math.Abs(sm.A), Limit: lim.Amax})
		}
		if checkJerk && i > 0 {
			dt := sm.T - samples[i-1].T
			if dt > 0 {
				j := (sm.A - samples[i-1].A) / dt
				if math.Abs(j) > lim.Jmax+tol {
					out = append(out, Violation{T: sm.T, Index: i, Type: ViolateJerk, Value: math.Abs(j), Limit: lim.Jmax})
				}
			}
		}
	}
	return out
}
