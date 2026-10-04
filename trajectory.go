package trajectory

// Trajectory binds a geometric path and the time-optimal velocity profile
// planned along its arc length.
type Trajectory struct {
	Path    Path
	Profile *Profile
}

// PlanPath plans the time-optimal profile for the whole given path.
func PlanPath(path Path, v0, vEnd float64, lim Limits) (*Trajectory, error) {
	p, err := Plan(path.Length(), v0, vEnd, lim)
	if err != nil {
		return nil, err
	}
	return &Trajectory{Path: path, Profile: p}, nil
}

// Duration returns the total motion time.
func (tr *Trajectory) Duration() float64 { return tr.Profile.Duration() }

// Pose is the spatial and kinematic state at one time instant.
type Pose struct {
	State
	Pos Point
	Tan Point
}

// At evaluates the full pose at time t.
func (tr *Trajectory) At(t float64) Pose {
	st := tr.Profile.Sample(t)
	pos, tan := tr.Path.At(st.S)
	return Pose{State: st, Pos: pos, Tan: tan}
}
