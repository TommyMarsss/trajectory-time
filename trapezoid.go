package trajectory

import "math"

// planTrapezoid builds the time-optimal bang-acceleration profile.
//
// Structure: accelerate at +AMax, optionally cruise at VMax, decelerate at
// -AMax. When the path is too short to reach VMax the cruise segment and the
// plateau at VMax disappear and the profile becomes triangular: accelerate
// to a single switching velocity vPeak < VMax, then decelerate.
func planTrapezoid(L, v0, vEnd float64, lim Limits) (*Profile, error) {
	a := lim.AMax
	vM := lim.VMax

	// Minimum distance required just to change v0 into vEnd with |accel| = a.
	dMin := math.Abs(v0*v0-vEnd*vEnd) / (2 * a)
	if L < dMin-Eps {
		return nil, infeasibleError("trapezoid", L, dMin)
	}

	p := &Profile{
		Kind: "trapezoid",
		V0:   v0, VEnd: vEnd, VMax: vM, AMax: a, L: L,
	}

	// Distance needed to accelerate v0 -> VMax and decelerate VMax -> vEnd.
	dRamp := ((vM*vM - v0*v0) + (vM*vM - vEnd*vEnd)) / (2 * a)

	if L >= dRamp-Eps {
		// Trapezoid: the velocity limit is reached (tCruise == 0 means the
		// profile merely touches VMax at one instant).
		p.Reach = true
		p.vPeak = vM
		p.tAcc = nonNeg((vM - v0) / a)
		p.tDec = nonNeg((vM - vEnd) / a)
		p.tCruise = nonNeg((L - dRamp) / vM)
		return p, nil
	}

	// Triangular profile: solve for the single switching velocity.
	//
	//	L = (vPeak^2-v0^2)/(2a) + (vPeak^2-vEnd^2)/(2a)
	vPeak := math.Sqrt((2*a*L + v0*v0 + vEnd*vEnd) / 2)
	if vPeak > vM {
		vPeak = vM // numeric guard; the branch above guarantees vPeak < VMax
	}
	p.Reach = false
	p.vPeak = vPeak
	p.tAcc = nonNeg((vPeak - v0) / a)
	p.tDec = nonNeg((vPeak - vEnd) / a)
	p.tCruise = 0
	return p, nil
}

// nonNeg clamps tiny negative rounding artefacts to zero. Values that are
// genuinely negative (beyond tolerance) are returned so downstream
// arithmetic cannot silently hide a real modelling error.
func nonNeg(x float64) float64 {
	if x < 0 && x > -1e-7 {
		return 0
	}
	return x
}
