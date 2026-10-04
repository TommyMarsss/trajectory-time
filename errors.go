package trajectory

import "fmt"

// InfeasibleError reports that no profile exists for the request because the
// path is too short to reconcile the endpoint velocities with the
// acceleration limit (and, for S-curves, the jerk limit).
type InfeasibleError struct {
	Kind      string
	Length    float64
	MinLength float64
}

func (e *InfeasibleError) Error() string {
	return fmt.Sprintf("trajectory: %s profile infeasible: path length %.6g is below the required minimum %.6g",
		e.Kind, e.Length, e.MinLength)
}

func infeasibleError(kind string, L, dMin float64) error {
	return &InfeasibleError{Kind: kind, Length: L, MinLength: dMin}
}
