package trajectory

import (
	"encoding/json"
	"io"
	"text/template"
)

// ScenarioData is one fully resolved scenario embedded into the static HTML
// report: parameters, segment table, dense time samples, the path geometry
// and (optionally) constraint violations found by the validator.
type ScenarioData struct {
	ID         string          `json:"id"`
	Name       string          `json:"name"`
	Note       string          `json:"note"`
	Kind       string          `json:"kind"`
	Reach      bool            `json:"reach"`
	Corrupted  bool            `json:"corrupted"`
	V0         float64         `json:"v0"`
	VEnd       float64         `json:"vEnd"`
	VMax       float64         `json:"vMax"`
	AMax       float64         `json:"aMax"`
	JMax       float64         `json:"jMax"`
	Length     float64         `json:"length"`
	Duration   float64         `json:"duration"`
	TheoryTime float64         `json:"theoryTime"`
	Segments   []SegmentJSON   `json:"segments"`
	Samples    [][4]float64    `json:"samples"` // [t, s, v, a]
	Path       [][2]float64    `json:"path"`    // polyline vertices
	Violations []ViolationJSON `json:"violations"`
}

// SegmentJSON describes one non-degenerate phase.
type SegmentJSON struct {
	Name     string  `json:"name"`
	T0       float64 `json:"t0"`
	T1       float64 `json:"t1"`
	Duration float64 `json:"duration"`
	Jerk     float64 `json:"jerk"`
}

// ViolationJSON is one detected constraint breach.
type ViolationJSON struct {
	Type  string  `json:"type"`
	T     float64 `json:"t"`
	Value float64 `json:"value"`
	Limit float64 `json:"limit"`
	T0    float64 `json:"t0"`
	T1    float64 `json:"t1"`
}

// BuildScenarioData densely samples a trajectory and packages it for the
// report. sampleCount <= 0 selects 600 samples.
func BuildScenarioData(id, name, note string, tr *Trajectory, sampleCount int) ScenarioData {
	if sampleCount <= 0 {
		sampleCount = 600
	}
	p := tr.Profile
	sd := ScenarioData{
		ID: id, Name: name, Note: note,
		Kind: p.Kind, Reach: p.Reach,
		V0: p.V0, VEnd: p.VEnd, VMax: p.VMax, AMax: p.AMax, JMax: p.JMax,
		Length: p.L, Duration: p.Duration(),
		TheoryTime: TheoreticalDuration(p),
	}
	for _, seg := range p.Segments() {
		sd.Segments = append(sd.Segments, SegmentJSON{
			Name: seg.Name, T0: seg.T0, T1: seg.T1,
			Duration: seg.Duration, Jerk: seg.Jerk,
		})
	}
	T := p.Duration()
	sd.Samples = make([][4]float64, sampleCount+1)
	for i := 0; i <= sampleCount; i++ {
		t := T * float64(i) / float64(sampleCount)
		st := p.Sample(t)
		sd.Samples[i] = [4]float64{st.T, st.S, st.V, st.A}
	}
	if pl, ok := tr.Path.(*Polyline); ok {
		for _, q := range pl.Points() {
			sd.Path = append(sd.Path, [2]float64{q.X, q.Y})
		}
	}
	sd.Violations = []ViolationJSON{}
	return sd
}

// AttachViolations runs the validator over arbitrary (possibly corrupted)
// samples and attaches the detected breaches to the scenario data.
func (sd *ScenarioData) AttachViolations(samples []State, jerkLimited bool) {
	lim := Limits{VMax: sd.VMax, AMax: sd.AMax, JMax: sd.JMax}
	rep := ValidateTrajectory(samples, lim, jerkLimited, ValidationTolerances{})
	sd.Violations = []ViolationJSON{}
	sd.Corrupted = len(rep.Violations) > 0
	sd.Samples = make([][4]float64, len(samples))
	for i, st := range samples {
		sd.Samples[i] = [4]float64{st.T, st.S, st.V, st.A}
	}
	sd.Duration = samples[len(samples)-1].T
	for _, v := range rep.Violations {
		sd.Violations = append(sd.Violations, ViolationJSON{
			Type: string(v.Type), T: v.T, Value: v.Value, Limit: v.Limit,
			T0: v.T0, T1: v.T1,
		})
	}
}

// WriteReport writes the single self-contained static HTML file: all data is
// inlined as JSON, all rendering is vanilla JavaScript on <canvas>. No
// external resources, frameworks or services are used.
func WriteReport(w io.Writer, scenarios []ScenarioData) error {
	data, err := json.Marshal(scenarios)
	if err != nil {
		return err
	}
	tmpl := template.Must(template.New("report").Parse(reportHTML))
	return tmpl.Execute(w, string(data))
}
