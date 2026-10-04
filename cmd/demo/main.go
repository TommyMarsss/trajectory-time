// Command demo generates the self-contained static HTML report
// (report.html) showcasing trapezoid and S-curve time-optimal profiles,
// their degenerations and the constraint-violation detector.
//
// Usage:
//
//	go run ./cmd/demo            # writes report.html
//	go run ./cmd/demo out.html   # custom output path
package main

import (
	"math"
	"os"

	tra "github.com/TommyMarsss/trajectory-time"
)

func main() {
	out := "report.html"
	if len(os.Args) > 1 {
		out = os.Args[1]
	}

	var scenarios []tra.ScenarioData

	// 1. Trapezoid, velocity limit reached (long path).
	scenarios = append(scenarios, makeScenario(
		"trap-reach", "梯形 · 可达限速（长路径）",
		"路径足够长：+AMax 加速到 VMax，匀速巡航，最后 -AMax 减速到终点速度。",
		120, 0, 0, tra.Limits{VMax: 10, AMax: 2},
	))

	// 2. Trapezoid, velocity limit unreachable (triangular).
	scenarios = append(scenarios, makeScenario(
		"trap-triangle", "梯形 · 不可达限速（短路径退化为三角形）",
		"路径过短，无法加速到 VMax 再减速：匀速段消失，速度在单个切换点 v*<VMax 处由升转降。",
		12, 0, 0, tra.Limits{VMax: 10, AMax: 2},
	))

	// 3. S-curve, everything reached.
	scenarios = append(scenarios, makeScenario(
		"s-reach", "S 曲线 · 达到限速（七段完整）",
		"完整七段：加加速→匀加速→减加速→匀速→减减速→匀减速→加减速；相邻段加速度连续，jerk 仅取 ±JMax 或 0。",
		120, 0, 0, tra.Limits{VMax: 10, AMax: 2, JMax: 10},
	))

	// 4. S-curve, VMax unreachable (short path).
	scenarios = append(scenarios, makeScenario(
		"s-short", "S 曲线 · 短路径（达不到限速）",
		"路径过短：巡航段消失，峰值速度 v*<VMax；若速度变化也很小，恒加速段同时退化为零长度。",
		6, 0, 0, tra.Limits{VMax: 10, AMax: 2, JMax: 10},
	))

	// 5. S-curve, jerk-dominated: constant-accel phases degenerate.
	scenarios = append(scenarios, makeScenario(
		"s-no-const-acc", "S 曲线 · jerk 主导（恒加速段退化为零）",
		"速度增量很小（Δv<a²/J）：tj=√(Δv/J)<a/J，恒加速段 tc=0。本例加速半坡 Δv=2 退化、减速半坡 Δv=4 不退化，两半退化段数不同。",
		40, 8, 6, tra.Limits{VMax: 10, AMax: 8, JMax: 20},
	))

	// 6. S-curve with very tight jerk (long constant-acceleration phases).
	scenarios = append(scenarios, makeScenario(
		"s-tight-jerk", "S 曲线 · 强 jerk 约束（长恒加速段）",
		"JMax 很小：jerk 过渡段很长、恒加速段按耦合关系缩短但仍存在，体现速度/加速度/jerk 三约束耦合。",
		300, 0, 0, tra.Limits{VMax: 10, AMax: 2, JMax: 1},
	))

	// 7. Non-zero endpoint velocities, asymmetric-ish triangular S-curve.
	scenarios = append(scenarios, makeScenario(
		"s-endpoints", "S 曲线 · 非零端点速度（短路径退化）",
		"起点/终点速度不同且路径短：两个半斜坡的退化段数可以不同，拼接点加速度仍为零、速度唯一。",
		3, 1.5, 0.5, tra.Limits{VMax: 10, AMax: 3, JMax: 12},
	))

	// 8. Injected velocity violation.
	scenarios = append(scenarios, injectVelocityViolation())

	// 9. Injected acceleration + jerk violation.
	scenarios = append(scenarios, injectAccelJerkViolation())

	f, err := os.Create(out)
	if err != nil {
		panic(err)
	}
	defer f.Close()
	if err := tra.WriteReport(f, scenarios); err != nil {
		panic(err)
	}
}

// makePath builds a wavy polyline whose arc length equals L.
func makePath(L float64) *tra.Polyline {
	const n = 120
	raw := make([]tra.Point, n+1)
	for i := 0; i <= n; i++ {
		u := float64(i) / n
		x := u
		y := 0.22*math.Sin(2*math.Pi*1.3*u) + 0.08*math.Sin(2*math.Pi*3.1*u+0.7)
		raw[i] = tra.Point{X: x, Y: y}
	}
	total := tra.NewPolyline(raw).Length()
	k := L / total
	pts := make([]tra.Point, len(raw))
	for i, p := range raw {
		pts[i] = tra.Point{X: p.X * k, Y: p.Y * k}
	}
	return tra.NewPolyline(pts)
}

func makeScenario(id, name, note string, L, v0, vEnd float64, lim tra.Limits) tra.ScenarioData {
	tr, err := tra.PlanPath(makePath(L), v0, vEnd, lim)
	if err != nil {
		panic(err)
	}
	sd := tra.BuildScenarioData(id, name, note, tr, 800)
	rep := tra.ValidateProfile(tr.Profile, 0, tra.ValidationTolerances{})
	if !rep.OK {
		for _, v := range rep.Violations {
			sd.Violations = append(sd.Violations, tra.ViolationJSON{
				Type: string(v.Type), T: v.T, Value: v.Value, Limit: v.Limit, T0: v.T0, T1: v.T1,
			})
		}
		sd.Corrupted = true
	}
	return sd
}

// injectVelocityViolation starts from a valid trapezoid and pushes the
// cruise plateau above the velocity limit; the sampler keeps s(t)
// consistent so the breach is purely in v(t).
func injectVelocityViolation() tra.ScenarioData {
	L := 120.0
	lim := tra.Limits{VMax: 10, AMax: 2}
	p, err := tra.Plan(L, 0, 0, lim)
	if err != nil {
		panic(err)
	}
	sd := tra.BuildScenarioData("bad-velocity", "注入违例 · 速度超限（巡航段被抬高）",
		"在合法梯形曲线上人为把巡航速度抬高到 VMax 以上，检测器应定位速度违例的时间段与峰值时刻。",
		mustTraj(L, p, lim), 800)

	// Rebuild samples with the cruise-window velocity scaled to 1.18x.
	states := p.SampleN(800)
	cruise := p.Segments()[1]
	for i := range states {
		if states[i].T > cruise.T0 && states[i].T < cruise.T1 {
			states[i].V *= 1.18
		}
	}
	sd.Violations = nil
	sd.AttachViolations(states, false)
	return sd
}

// injectAccelJerkViolation takes a valid S-curve and overwrites the
// acceleration over a middle window above AMax with a sharp jump, producing
// both an acceleration breach and jerk spikes at the window edges.
func injectAccelJerkViolation() tra.ScenarioData {
	L := 120.0
	lim := tra.Limits{VMax: 10, AMax: 2, JMax: 10}
	p, err := tra.Plan(L, 0, 0, lim)
	if err != nil {
		panic(err)
	}
	sd := tra.BuildScenarioData("bad-accel", "注入违例 · 加速度跳变（同时触发 jerk 违例）",
		"在合法 S 曲线中段把加速度强行抬到 AMax 以上且引入跳变；检测器应分别报出加速度违例与其边缘的 jerk 违例时刻。",
		mustTraj(L, p, lim), 800)

	states := p.SampleN(800)
	T := p.Duration()
	// Widen the constant-acceleration plateau artificially.
	for i := range states {
		u := states[i].T / T
		if u > 0.18 && u < 0.30 {
			states[i].A = lim.AMax * 1.45
		}
	}
	sd.Violations = nil
	sd.AttachViolations(states, true)
	return sd
}

func mustTraj(L float64, p *tra.Profile, lim tra.Limits) *tra.Trajectory {
	_ = lim
	return &tra.Trajectory{Path: makePath(L), Profile: p}
}
