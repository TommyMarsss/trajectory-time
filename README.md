# trajectory-time — 沿路径的时间最优速度规划（Go，仅标准库）

给定一条几何路径与速度 / 加速度 / jerk（加速度变化率）限制，计算沿路径弧长
坐标的**时间最优**速度曲线（梯形或 S 曲线），并验证整条轨迹满足全部约束。
附带一个**单文件静态 HTML 报告**（数据与逻辑全部内嵌，无服务、无第三方库、
无前端框架），在浏览器中直接打开即可查看位置 / 速度 / 加速度 / jerk 曲线并
逐帧回放沿路径的运动。

## 快速开始

```bash
go test ./...                 # 全部自动化测试
go run ./cmd/demo report.html # 生成静态报告，浏览器直接打开 report.html
```

作为库使用：

```go
import tra "github.com/TommyMarsss/trajectory-time"

path := tra.NewPolyline([]tra.Point{{0, 0}, {3, 0}, {3, 4}})
tr, err := tra.PlanPath(path, 0, 0, tra.Limits{VMax: 10, AMax: 2})            // 梯形
tr, err  = tra.PlanPath(path, 0, 0, tra.Limits{VMax: 10, AMax: 2, JMax: 10}) // S 曲线

rep := tra.ValidateProfile(tr.Profile, 0, tra.ValidationTolerances{})
for _, v := range rep.Violations { fmt.Println(v) }

pose := tr.At(3.5) // t=3.5s 时的 s, v, a 与平面位置/切向
```

## 文件结构

| 文件 | 内容 |
|---|---|
| `profile.go` | `Profile` / `Limits`、参数校验、入口 `Plan`、总时长 |
| `trapezoid.go` | 梯形（bang-zero-bang）规划，含三角形退化 |
| `scurve.go` | jerk 受限的 6 斜坡段 + 巡航规划、恒 jerk 精确积分、采样 |
| `sample.go` | 任意时刻状态采样、分段枚举（跳过零长度段） |
| `verify.go` | 约束检测器：速度 / 加速度 / jerk / 端点条件违例定位 |
| `theory.go` | 独立的闭式理论最优时长（与规划器交叉验证） |
| `path.go` / `trajectory.go` | 折线几何路径、路径↔弧长绑定 |
| `report.go` / `report_html.go` | 单文件静态 HTML 报告生成（内嵌 JSON + 原生 JS Canvas） |
| `cmd/demo/main.go` | 9 个场景（含 2 个人为注入违例）与报告生成入口 |

## 模型与公式

规划在标量弧长坐标 s 上进行：v = ṡ，a = v̇，j = ȧ。所有限幅对称。

### 1. 梯形速度曲线（仅 v、a 受限）

结构为 `+AMax 加速 → 匀速 VMax → -AMax 减速`。

- 加速段距离 `dAcc=(VMax²-v0²)/(2AMax)`，减速段距离
  `dDec=(VMax²-vEnd²)/(2AMax)`。
- **路径足够长（L ≥ dAcc+dDec）**：三段梯形，巡航时长
  `tCr=(L-dAcc-dDec)/VMax`，永不产生负时长。
- **路径过短（L < dAcc+dDec）**：正确退化为**三角形**——巡航段时长恒为
  0，速度在唯一切换点
  `v* = √((2AMax·L+v0²+vEnd²)/2) < VMax`
  处由升转降，`tAcc=(v*-v0)/AMax`、`tDec=(v*-vEnd)/AMax`。
  若错误地仍按梯形计算会得到负的巡航时长；本实现用分支显式排除。
- **恰好相切**：L = dAcc+dDec 时巡航时长为 0 但标记为“达到限速”（单点
  接触）。
- **不可行**：连单纯改变端点速度所需的最小距离
  `|v0²-vEnd²|/(2AMax)` 都不够时返回 `*InfeasibleError`（含所需最小长度）。

### 2. S 曲线（v、a、j 同时受限）

完整结构为 6 个斜坡段 + 巡航（俗称 7 段）：

```
+J 加加速 → 0 匀加速 → -J 减加速 | 匀速 | -J 减减速 → 0 匀减速 → +J 加减速
```

每个半斜坡（速度由 vLo 升到 vHi，Δv=vHi-vLo）的参数：

```
tj = min(AMax/JMax, √(Δv/JMax))             # 每个 jerk 段的时长
tc = max(0, (Δv - JMax·tj²)/(JMax·tj))      # 恒加速段时长
```

- 当 `√(Δv/JMax) ≥ AMax/JMax`（即 `Δv ≥ AMax²/JMax`）时加速度能打到
  AMax，tj=AMax/JMax，tc>0（三段完整）。
- 当 **`Δv < AMax²/JMax`** 时加速度还没到 AMax 就必须回落：
  **tc 恒等于 0，恒加速段退化为零长度**，只剩一对恒 jerk 段。该分支被
  显式识别并置零（不保留浮点残差），`Segments()` 与采样一律跳过零段。
- **路径过短到不了 VMax**：巡航段为零，切换峰值 v\*<VMax 由对
  “两半斜坡距离之和 = L”做二分求解（距离函数对 v\* 连续单调，100 次二分
  到机器精度），即使两半斜坡退化方式不同也成立。
- 段间状态用精确的恒 jerk 递推传播：

  ```
  a' = a + j·dt;  v' = v + a·dt + ½j·dt²;  s' = s + v·dt + ½a·dt² + ⅙j·dt³
  ```

  每段结束时 a 恰好落在 0 或 ±峰值，因此**相邻段切换处加速度连续**
  （只有 jerk 允许在 ±JMax/0 之间切换）。

## 约束耦合的处理

速度、加速度、jerk 三个限制不是独立的：

- 同样的 Δv，在小 jerk 下需要更长的过渡距离，可能因此够不到 VMax
  （巡航段消失）；
- 能否达到 AMax 取决于 `Δv` 与 `AMax²/JMax` 的比较（tc 是否退化）；
- 两半斜坡的端点速度不同，其退化段数可以不同（报告中
  “jerk 主导”场景：加速半坡 tc=0、减速半坡 tc=0.1）。

规划器统一以“先按能达到极值算距离 → 距离不足则求切换峰值 → 每段时长按
公式钳为非负 → 零段跳过”的顺序处理这些耦合，不产生任何负时长。

## 约束验证（含人为注入违例的定位）

`ValidateTrajectory(states, lim, jerkLimited, tol)` 接受任意时间有序采样
（不仅是本库规划结果），输出 `[]Violation`，每条包含：

- 违例**类型**：`velocity` / `acceleration` / `jerk` / `endpoint`；
- 违例**时间区间** [T0,T1]（连续违例样本合并为一段）、峰值时刻 T；
- 观测值与限幅值。

jerk 由加速度中心差分估计；梯形曲线理论上在切换点有冲量 jerk，因此
`jerkLimited=false` 时不检查 jerk。`ValidateProfile` 另加端点条件
（s(0)=0、s(T)=L、端点速度匹配）与相位边界加密采样。

报告内置两个注入场景：

1. 把合法梯形巡航速度抬高 18% → 定位到速度违例区间与峰值；
2. 把合法 S 曲线中段加速度抬到 1.45·AMax 并制造跳变 → 同时定位加速度
   违例段与其两侧两个 jerk 尖峰。

## 最优性的验证方式

`theory.go` 用**与规划器完全独立的闭式公式**重新计算最优总时长（梯形
直接代入；S 曲线达到限速时闭式、未达到时独立二分），每个场景与每组
扫掠长度都要求二者一致（相对误差 ≤ 1e-7）。测试覆盖从“刚过最小可行
长度”到“远长于斜坡距离”的长 / 短路径，逐点检查 0≤v≤VMax、|a|≤AMax、
|j|≤JMax、位置单调且端点闭合。

## 静态 HTML 报告

- 单个 `report.html`：场景数据以 JSON 内嵌于
  `<script type="application/json">`，全部绘图用原生 Canvas 与
  JavaScript，无任何 `<script src>` / 外链 CSS / 字体 / 框架 / 图形库，
  不启动服务，`file://` 直接打开。
- 位置 / 速度 / 加速度（S 曲线另加 jerk）四张时间曲线图共享十字光标，
  悬停联动回放；红色虚线为限幅、红色三角为违例峰值。
- 右侧为沿二维折线路径的逐帧回放（播放 / 暂停、0.25×–4× 变速、拖动）、
  分段表（点击跳转）、违例列表（点击定位）。
- 支持浅色 / 深色（跟随系统或手动切换）。
- 可选深链：`report.html#sc=2&theme=light&t=0.45`（场景序号、主题、
  归一化时间）。

## 已知局限

1. **限幅对称且标量**：v≥0（单向运动），加速 / 减速共用同一 AMax、JMax；
   不支持非对称加速限幅或沿路径变化的速度限幅（如曲率限速 v≤√(aLat/κ)）。
2. **无路径曲率约束**：折线几何仅用于可视化映射，拐角未做向心加速度 /
   侧向 jerk 限速；真实弯道需把曲率相关限速并入弧长规划（前向传播）。
3. **时间最优的 jerk 策略取 bang-bang（±JMax 或 0）**：对本模型（固定
   上下界、无加加速度平方代价）是时间最优的，但不保证能耗 / 跟踪误差等
   其它指标最优。
4. **验证依赖采样密度**：jerk 由中心差分估计，宽度小于采样间隔的窄脉冲
   可能被平滑；检测窄脉冲应提高采样密度（`ValidateProfile` 的 step
   参数）。分段边界在规划曲线上已强制加密。
5. **不做停留 / 等待段与反向运动**，端点速度必须在 [0, VMax] 内。
6. 数值上对长度 / 限幅的量级使用相对容差（默认 1e-6）；极端量级（>1e9
   或 <1e-9 的混合单位）可能需要传入自定义 `ValidationTolerances`。
