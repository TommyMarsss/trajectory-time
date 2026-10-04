// Command plan generates a single self-contained HTML report showing
// the planned trajectories (position/velocity/acceleration over time)
// with an animated replay along the path. No server, no dependencies:
// the output file embeds all data and logic.
package main

import (
	_ "embed"
	"encoding/json"
	"flag"
	"fmt"
	"log"
	"os"
	"strings"

	"github.com/TommyMarsss/trajectory-time/planner"
)

//go:embed report_template.html
var template string

type reportData struct {
	Limits   planner.Limits         `json:"limits"`
	Path     pathJSON               `json:"path"`
	Profiles map[string]profileJSON `json:"profiles"`
}

type pathJSON struct {
	X []float64 `json:"x"`
	Y []float64 `json:"y"`
}

type profileJSON struct {
	Total float64 `json:"total"`
	Dt    float64 `json:"dt"`
	// Each sample is [t, s, v, a, j].
	Samples [][]float64 `json:"samples"`
}

func pack(tr planner.Trajectory, dt float64) profileJSON {
	p := profileJSON{Total: tr.TotalTime, Dt: dt, Samples: make([][]float64, len(tr.Samples))}
	for i, sm := range tr.Samples {
		p.Samples[i] = []float64{sm.T, sm.S, sm.V, sm.A, sm.J}
	}
	return p
}

func main() {
	out := flag.String("o", "report.html", "output HTML file")
	dt := flag.Float64("dt", 0.005, "sampling period (s)")
	flag.Parse()

	// Demo path: a long straight, a short hop (triangular profile) and
	// a medium straight, exercising every degeneration branch.
	path, err := planner.NewPath(
		[]float64{0, 4, 4, 6.5},
		[]float64{0, 0, 0.6, 0.6},
	)
	if err != nil {
		log.Fatal(err)
	}
	lim := planner.Limits{Vmax: 1.5, Amax: 1.2, Jmax: 2.0}

	data := reportData{
		Limits:   lim,
		Path:     pathJSON{X: path.X, Y: path.Y},
		Profiles: map[string]profileJSON{},
	}
	for _, kind := range []planner.ProfileKind{planner.Trapezoid, planner.SCurve} {
		tr, err := planner.PlanPath(path, lim, kind, *dt)
		if err != nil {
			log.Fatalf("plan %s: %v", kind, err)
		}
		checkJerk := kind == planner.SCurve
		if v := planner.Validate(tr.Samples, lim, 1e-6, checkJerk); len(v) > 0 {
			log.Fatalf("planned %s trajectory violates constraints: %+v", kind, v[0])
		}
		data.Profiles[string(kind)] = pack(tr, *dt)
		fmt.Printf("%-10s total time %.3f s over %.3f m (%d samples)\n",
			kind, tr.TotalTime, path.Length(), len(tr.Samples))
	}

	js, err := json.Marshal(data)
	if err != nil {
		log.Fatal(err)
	}
	html := strings.Replace(template, "/*__DATA__*/", string(js), 1)
	if err := os.WriteFile(*out, []byte(html), 0o644); err != nil {
		log.Fatal(err)
	}
	fmt.Printf("wrote %s\n", *out)
}
