package trajectory

import "math"

// Point is a position in the plane.
type Point struct {
	X, Y float64
}

// Path is a fixed geometric curve parametrized by arc length. The planner
// computes a scalar velocity profile along this coordinate; playback maps
// arc length back to a point on the curve.
type Path interface {
	// Length is the total arc length.
	Length() float64
	// At returns the point and unit tangent at arc length s (s is clamped
	// to [0, Length]).
	At(s float64) (Point, Point)
}

// Polyline is a piecewise-linear path through the given vertices.
type Polyline struct {
	pts    []Point
	cum    []float64 // cumulative arc length at each vertex
	length float64
}

// NewPolyline builds a polyline; adjacent duplicate vertices are allowed and
// contribute zero length.
func NewPolyline(pts []Point) *Polyline {
	if len(pts) < 2 {
		panic("trajectory: a polyline needs at least two points")
	}
	pl := &Polyline{pts: make([]Point, len(pts)), cum: make([]float64, len(pts))}
	copy(pl.pts, pts)
	for i := 1; i < len(pts); i++ {
		pl.cum[i] = pl.cum[i-1] + dist(pts[i-1], pts[i])
	}
	pl.length = pl.cum[len(pts)-1]
	return pl
}

func (pl *Polyline) Length() float64 { return pl.length }

func (pl *Polyline) At(s float64) (Point, Point) {
	if pl.length <= 0 {
		return pl.pts[0], Point{1, 0}
	}
	s = clampRange(s, 0, pl.length)
	// Locate the segment (binary search on cumulative lengths).
	lo, hi := 0, len(pl.cum)-1
	for hi-lo > 1 {
		mid := (lo + hi) / 2
		if pl.cum[mid] <= s {
			lo = mid
		} else {
			hi = mid
		}
	}
	segLen := pl.cum[lo+1] - pl.cum[lo]
	var u float64
	var tan Point
	if segLen <= Eps {
		u = 0
		// Zero-length segment: borrow the next non-degenerate tangent.
		tan = pl.tangentAfter(lo)
	} else {
		u = (s - pl.cum[lo]) / segLen
		dx, dy := pl.pts[lo+1].X-pl.pts[lo].X, pl.pts[lo+1].Y-pl.pts[lo].Y
		tan = Point{dx / segLen, dy / segLen}
	}
	p := Point{
		X: pl.pts[lo].X + u*(pl.pts[lo+1].X-pl.pts[lo].X),
		Y: pl.pts[lo].Y + u*(pl.pts[lo+1].Y-pl.pts[lo].Y),
	}
	return p, tan
}

// tangentAfter returns the tangent of the first non-degenerate segment at or
// after index i.
func (pl *Polyline) tangentAfter(i int) Point {
	for k := i; k+1 < len(pl.pts); k++ {
		l := pl.cum[k+1] - pl.cum[k]
		if l > Eps {
			return Point{(pl.pts[k+1].X - pl.pts[k].X) / l, (pl.pts[k+1].Y - pl.pts[k].Y) / l}
		}
	}
	for k := i; k > 0; k-- {
		l := pl.cum[k] - pl.cum[k-1]
		if l > Eps {
			return Point{(pl.pts[k].X - pl.pts[k-1].X) / l, (pl.pts[k].Y - pl.pts[k-1].Y) / l}
		}
	}
	return Point{1, 0}
}

// Points returns a copy of the polyline vertices.
func (pl *Polyline) Points() []Point {
	out := make([]Point, len(pl.pts))
	copy(out, pl.pts)
	return out
}

func dist(a, b Point) float64 {
	dx, dy := a.X-b.X, a.Y-b.Y
	return math.Hypot(dx, dy)
}

func clampRange(x, lo, hi float64) float64 {
	if x < lo {
		return lo
	}
	if x > hi {
		return hi
	}
	return x
}
