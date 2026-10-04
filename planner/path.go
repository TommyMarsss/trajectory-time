package planner

import (
	"errors"
	"math"
)

// Path is a 2D polyline parameterized by arc length.
type Path struct {
	X, Y  []float64 // waypoint coordinates
	cum   []float64 // cumulative arc length at each waypoint
	total float64
}

// NewPath builds a path from waypoint coordinates (at least 2 points,
// consecutive points must be distinct).
func NewPath(x, y []float64) (Path, error) {
	if len(x) != len(y) || len(x) < 2 {
		return Path{}, errors.New("planner: path needs >= 2 waypoints with matching coordinates")
	}
	p := Path{X: x, Y: y, cum: make([]float64, len(x))}
	for i := 1; i < len(x); i++ {
		d := math.Hypot(x[i]-x[i-1], y[i]-y[i-1])
		if d <= 0 {
			return Path{}, errors.New("planner: duplicate consecutive waypoints")
		}
		p.cum[i] = p.cum[i-1] + d
	}
	p.total = p.cum[len(p.cum)-1]
	return p, nil
}

// Length returns the total arc length of the path.
func (p Path) Length() float64 { return p.total }

// SegmentLengths returns the arc length of each straight segment.
func (p Path) SegmentLengths() []float64 {
	out := make([]float64, len(p.cum)-1)
	for i := range out {
		out[i] = p.cum[i+1] - p.cum[i]
	}
	return out
}

// PointAt returns the (x, y) position and heading at arc length s
// (clamped to the path).
func (p Path) PointAt(s float64) (x, y, theta float64) {
	if s <= 0 {
		dx, dy := p.X[1]-p.X[0], p.Y[1]-p.Y[0]
		return p.X[0], p.Y[0], math.Atan2(dy, dx)
	}
	if s >= p.total {
		n := len(p.X)
		dx, dy := p.X[n-1]-p.X[n-2], p.Y[n-1]-p.Y[n-2]
		return p.X[n-1], p.Y[n-1], math.Atan2(dy, dx)
	}
	// Binary search for the segment containing s.
	lo, hi := 0, len(p.cum)-1
	for lo+1 < hi {
		mid := (lo + hi) / 2
		if p.cum[mid] <= s {
			lo = mid
		} else {
			hi = mid
		}
	}
	segLen := p.cum[hi] - p.cum[lo]
	f := (s - p.cum[lo]) / segLen
	dx, dy := p.X[hi]-p.X[lo], p.Y[hi]-p.Y[lo]
	return p.X[lo] + f*dx, p.Y[lo] + f*dy, math.Atan2(dy, dx)
}

// ProfileKind selects the velocity profile model.
type ProfileKind string

const (
	Trapezoid ProfileKind = "trapezoid"
	SCurve    ProfileKind = "scurve"
)

// Trajectory is a fully planned, sampled motion along a path.
type Trajectory struct {
	Kind      ProfileKind
	Path      Path
	Samples   []Sample // S is the global arc length along the whole path
	TotalTime float64
}

// PlanPath plans a time-optimal trajectory along the whole path. Every
// straight segment between waypoints is planned with zero boundary
// velocities (the motion stops at each corner) and the per-segment
// profiles are stitched in time and arc length. Position and velocity
// are therefore continuous across stitches; for the S-curve model
// acceleration is continuous as well (it is zero at every stitch).
func PlanPath(p Path, lim Limits, kind ProfileKind, dt float64) (Trajectory, error) {
	tr := Trajectory{Kind: kind, Path: p}
	sOffset := 0.0
	for _, L := range p.SegmentLengths() {
		var samples []Sample
		var err error
		switch kind {
		case Trapezoid:
			var segs []AccelSeg
			segs, err = PlanTrapezoid(L, 0, 0, lim)
			if err == nil {
				samples = SampleAccel(segs, sOffset, 0, dt)
			}
		case SCurve:
			var segs []JerkSeg
			segs, err = PlanSCurve(L, lim)
			if err == nil {
				samples = SampleJerk(segs, sOffset, dt)
			}
		default:
			err = errors.New("planner: unknown profile kind")
		}
		if err != nil {
			return Trajectory{}, err
		}
		tOffset := tr.TotalTime
		for i, sm := range samples {
			if i == 0 && len(tr.Samples) > 0 {
				continue // drop duplicated stitch sample
			}
			sm.T += tOffset
			tr.Samples = append(tr.Samples, sm)
		}
		tr.TotalTime = tr.Samples[len(tr.Samples)-1].T
		sOffset += L
	}
	return tr, nil
}
