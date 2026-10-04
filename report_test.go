package trajectory

import (
	"bytes"
	"encoding/json"
	"regexp"
	"testing"
)

func TestDetectJerkOnlyViolation(t *testing.T) {
	// A jump inside the acceleration envelope (|a| <= AMax) must still be
	// caught as a jerk violation without raising an acceleration violation.
	p, _ := Plan(120, 0, 0, Limits{VMax: 10, AMax: 2, JMax: 10})
	states := p.SampleN(1200)
	T := p.Duration()
	for i := range states {
		u := states[i].T / T
		if u > 0.45 && u < 0.55 {
			states[i].A = 1.5
		}
	}
	rep := ValidateTrajectory(states, Limits{VMax: 10, AMax: 2, JMax: 10}, true, ValidationTolerances{})
	kinds := map[ViolationType]int{}
	for _, v := range rep.Violations {
		kinds[v.Type]++
	}
	if kinds[ViolationAccel] != 0 {
		t.Errorf("no acceleration breach expected, got %d", kinds[ViolationAccel])
	}
	if kinds[ViolationJerk] < 2 {
		t.Errorf("jerk edges must be detected, got %d", kinds[ViolationJerk])
	}
}

func TestTrapezoidSkipsJerkCheck(t *testing.T) {
	// A trapezoid has impulsive jerk at ramp switches; with jerkLimited
	// false those must not be reported as violations.
	p, _ := Plan(120, 0, 0, Limits{VMax: 10, AMax: 2})
	rep := ValidateProfile(p, 0, ValidationTolerances{})
	for _, v := range rep.Violations {
		if v.Type == ViolationJerk {
			t.Errorf("trapezoid must not be checked for jerk, got %v", v)
		}
	}
}

func TestEmptyAndShortSamples(t *testing.T) {
	if r := ValidateTrajectory(nil, Limits{VMax: 1, AMax: 1}, false, ValidationTolerances{}); r.OK {
		t.Error("empty samples must not validate")
	}
	if r := ValidateTrajectory([]State{{T: 0}}, Limits{VMax: 1, AMax: 1}, false, ValidationTolerances{}); r.OK {
		t.Error("single sample must not validate")
	}
}

func TestReportIsSelfContained(t *testing.T) {
	p, err := Plan(40, 8, 6, Limits{VMax: 10, AMax: 8, JMax: 20})
	if err != nil {
		t.Fatal(err)
	}
	tr := &Trajectory{Path: NewPolyline([]Point{{0, 0}, {10, 0}, {20, 0}, {40, 0}}), Profile: p}
	sd := BuildScenarioData("x", "x", "x", tr, 200)
	var buf bytes.Buffer
	if err := WriteReport(&buf, []ScenarioData{sd}); err != nil {
		t.Fatal(err)
	}
	html := buf.String()

	for _, pat := range []string{`src=`, `href=`, `http://`, `https://`, `@import`} {
		if regexp.MustCompile(pat).MatchString(html) {
			t.Errorf("report contains forbidden external reference: %s", pat)
		}
	}
	re := regexp.MustCompile(`<script id="trajectory-data" type="application/json">(.*?)</script>`)
	m := re.FindStringSubmatch(html)
	if m == nil {
		t.Fatal("embedded data block not found")
	}
	var got []ScenarioData
	if err := json.Unmarshal([]byte(m[1]), &got); err != nil {
		t.Fatalf("embedded JSON invalid: %v", err)
	}
	if len(got) != 1 || len(got[0].Samples) != 201 || len(got[0].Path) != 4 {
		t.Fatalf("embedded payload wrong: %+v", got)
	}
}
