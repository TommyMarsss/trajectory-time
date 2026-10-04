package trajectory

import "math"

// TheoreticalDuration recomputes the optimal travel time from the textbook
// closed-form equations instead of the planner's phase bookkeeping. It is an
// independent cross-check: the planner output and this value must agree.
//
// Trapezoid model (bang-zero-bang acceleration):
//
//	dAcc = (vM²-v0²)/(2a), dDec = (vM²-vEnd²)/(2a)
//	reachable:   T = (vM-v0)/a + (vM-vEnd)/a + (L-dAcc-dDec)/vM
//	triangular:  v* = sqrt((2aL+v0²+vEnd²)/2), T = (2v*-v0-vEnd)/a
//
// S-curve model: each velocity ramp of size dv spends
//
//	tj = min(a/j, sqrt(dv/j)) in each jerk phase,
//	tc = (dv - j·tj²)/(j·tj)  in the constant-acceleration phase
//
// (tc == 0 is the degenerate double-jerk ramp). Half-ramp duration is
// 2tj+tc and its covered distance is
//
//	vLo(2tj+tc) + j·tj³ + (3/2)j·tj²·tc + (1/2)j·tj·tc².
//
// The unreachable-limit case solves sHalf(v0,v*) + sHalf(vEnd,v*) = L by
// bisection on v* (same root, independently integrated).
func TheoreticalDuration(p *Profile) float64 {
	if p.Kind != "s-curve" {
		return theoryTrapezoid(p.L, p.V0, p.VEnd, p.VMax, p.AMax)
	}
	return theorySCurve(p.L, p.V0, p.VEnd, p.VMax, p.AMax, p.JMax)
}

func theoryTrapezoid(L, v0, vEnd, vM, a float64) float64 {
	dAcc := (vM*vM - v0*v0) / (2 * a)
	dDec := (vM*vM - vEnd*vEnd) / (2 * a)
	if L >= dAcc+dDec-Eps {
		return (vM-v0)/a + (vM-vEnd)/a + (L-dAcc-dDec)/vM
	}
	vStar := math.Sqrt((2*a*L + v0*v0 + vEnd*vEnd) / 2)
	return (2*vStar - v0 - vEnd) / a
}

func theoryHalf(vLo, vHi, a, j float64) (dist, dt float64) {
	dv := vHi - vLo
	if dv <= Eps {
		return 0, 0
	}
	tj := math.Min(a/j, math.Sqrt(dv/j))
	tc := math.Max(0, (dv-j*tj*tj)/(j*tj))
	dt = 2*tj + tc
	dist = vLo*dt + j*tj*tj*tj + 1.5*j*tj*tj*tc + 0.5*j*tj*tc*tc
	return dist, dt
}

func theorySCurve(L, v0, vEnd, vM, a, j float64) float64 {
	dAcc, tAcc := theoryHalf(v0, vM, a, j)
	dDec, tDec := theoryHalf(vEnd, vM, a, j)
	if L >= dAcc+dDec-Eps {
		return tAcc + tDec + (L-dAcc-dDec)/vM
	}
	lo, hi := math.Max(v0, vEnd), vM
	for i := 0; i < 100; i++ {
		mid := (lo + hi) / 2
		d1, _ := theoryHalf(v0, mid, a, j)
		d2, _ := theoryHalf(vEnd, mid, a, j)
		if d1+d2 > L {
			hi = mid
		} else {
			lo = mid
		}
	}
	vStar := (lo + hi) / 2
	_, t1 := theoryHalf(v0, vStar, a, j)
	_, t2 := theoryHalf(vEnd, vStar, a, j)
	return t1 + t2
}
