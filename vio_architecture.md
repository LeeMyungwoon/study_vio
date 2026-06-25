# Basalt-Centric 6-Camera Generalized-Camera VIO/SLAM Architecture

이 문서는 서로 다른 yaw 방향을 바라보는 **6개의 단안카메라와 1개의 IMU**를 하나의 rigid rig로 묶어, Basalt 계열의 sparse direct feature tracking과 sliding-window optimization을 기반으로 구현하는 최종 VIO/SLAM 아키텍처다.

대상 카메라 구성은 다음과 같다.

```text
카메라 수:
  6 monocular cameras

기구 배치:
  pitch는 거의 동일
  yaw는 서로 다름
  인접 카메라끼리 FoV 15~20° overlap
  전체적으로 360°에 가까운 surround-view ring

센서 융합:
  6 cameras + 1 IMU
  one rigid body
  one estimator
```

현재 설계의 핵심은 다음이다.

```text
Local VIO backbone:
  Basalt sparse patch tracking
  + Basalt square-root sliding-window optimization / marginalization

Multi-camera formulation:
  6개의 카메라를 6개의 독립 VIO로 보지 않는다.
  하나의 generalized camera rig로 보고 timestamp당 body state를 하나만 둔다.

Temporal tracking:
  각 카메라 내부에서는 Basalt식 coarse-to-fine direct patch optical flow
  IMU pose prediction + landmark depth/covariance로 초기 위치를 예측

Cross-camera tracking:
  실제로 overlap하는 인접 6개 edge에서만 수행
  descriptor-free geometry-guided projection / epipolar search
  + Basalt식 direct patch alignment

Landmark representation:
  rig 전체 global landmark ID
  host frame-camera pair 기반 inverse depth + covariance
  immature / mature 상태 분리
  camera seam을 넘는 long track + landmark rehosting

Bounded compute:
  rig-wide tracking budget B_track
  보통 더 작은 optimization budget B_opt
  camera/azimuth/elevation 최소 quota
  robust information / max-logdet selection

Dynamic robustness:
  DynaVINS++의 weighted parallax + ATLS + BCC + SSR
  단, Basalt의 square-root prior를 오염시키지 않도록
  BCC 통과 전에는 marginalization / NFR export를 금지

Global SLAM:
  local VIO에는 descriptor를 사용하지 않음
  rig keyframe에서만 저주기로 descriptor를 생성
  intra-camera + inter-camera loop closure
  Basalt NFR + global BA / pose graph

Downstream output:
  high-rate body pose / velocity / bias / gravity / covariance
  sparse landmarks + depth covariance + static confidence
  rejected-track dynamic hints + landmark free-space rays
  Occupancy Network temporal alignment와 향후 tight coupling에 사용
```

여러 논문/레포에서 가져오는 핵심 아이디어는 다음과 같다.

| Source | 가져오는 핵심 아이디어 | 이 문서에서 쓰는 위치 | 그대로 쓰지 않는 부분 |
|---|---|---|---|
| [Basalt](https://github.com/VladyslavUsenko/basalt) | sparse patch tracking, host-frame inverse-depth landmark, one sliding-window VIO, square-root marginalization, NFR mapping | local tracking, backend, marginalization, global mapping의 골격 | stereo 중심 가정, single/stereo camera topology |
| [VILENS-MC](https://arxiv.org/abs/2109.05975) / [Newer College usage](https://ori-drs.github.io/newer-college-dataset/usage/) | 모든 카메라를 동시에 쓰는 factor graph, Cross-Camera Feature Tracking(CCFT), fixed overall feature budget, informative feature selection | camera seam handoff, rig-wide landmark identity, B_track/B_opt | projected point의 nearest detected feature만 연결하는 association, camera별 독립 SFS |
| [MAVIS](https://arxiv.org/abs/2309.08142) / [OpenMAVIS](https://github.com/MAVIS-SLAM/OpenMAVIS) | partially overlapped multi-camera local-map projection, exact `SE_2(3)` IMU preintegration, IMU intrinsic compensation, frame-drop/time-offset/mid-exposure handling, intra/inter-camera loop closure | IMU factor, timing model, degraded operation, global SLAM | local VIO의 매-frame descriptor matching, stereo-dependent initialization, tracking 영상 histogram equalization, ORB-SLAM3 backend 직접 병합 |
| [DynaVINS++](https://arxiv.org/abs/2410.15373) | weighted parallax, Adaptive Truncated Least Squares(ATLS), Bias Consistency Check(BCC), Stable State Recovery(SSR) | dynamic-aware keyframe, robust BA, stable commit protocol | VINS-Fusion state/marginalization 코드를 그대로 이식하는 것 |
| [Enhancing VIO Robustness and Accuracy, 2025](https://doi.org/10.3390/robotics14060071) | Basalt의 IMU-guided feature prediction, multi-camera optical-flow 확장, adaptive detector threshold 문제의식 | temporal initial guess, Basalt 코드 수정 참고, load-aware detection | parent-child camera tree, 고정 사각 overlap만으로 association, 모든 feature에 평균 depth, 단순 feature-count threshold |
| *Visual-Inertial SLAM with Multiple Cameras*, LMU 2020 | optical-flow VIO와 descriptor mapping 분리, VIO marginalization 정보를 NFR factor로 mapper에 전달 | local VIO / global mapper의 주기 분리, NFR export | 카메라마다 독립 Basalt-SLAM을 돌린 뒤 map merging하는 구조 |
| [MCVIO](https://arxiv.org/abs/2109.12030) | 각 카메라의 temporal tracks를 하나의 tightly-coupled backend로 결합, embedded GPU frontend | overlap 없이도 multi-camera tight coupling이 성립한다는 baseline, 병렬 frontend | VINS 계열 backend를 최종 estimator로 사용하는 것 |
| [Redesigning SLAM for Arbitrary Multi-Camera Systems](https://arxiv.org/abs/2003.02014) | arbitrary 2~6 camera, information-theoretic keyframe selection, sensor-agnostic initialization | rig-level keyframe / initialization 설계 검증 | SVO 계열 전체 pipeline |
| [OpenVINS](https://docs.openvins.com/) | arbitrary camera와 timing/intrinsic calibration, sparse KLT frontend, bounded feature update | 비교 baseline, camera dropout/asynchrony 설계 참고 | EKF/MSCKF를 최종 backend로 사용하는 것 |

특히 네 핵심 논문의 역할은 정확히 분리한다.

```text
Basalt:
  최종 estimator의 수치적 / 자료구조적 골격
  patch tracking + reprojection BA + square-root marginalization + NFR

VILENS-MC:
  camera seam을 넘는 landmark continuity
  fixed overall feature budget
  informative feature selection의 문제 정의

MAVIS:
  multi-camera local-map projection과 실제 센서 운용 세부사항
  exact SE_2(3) preintegration
  frame drop / time offset / mid-exposure
  intra/inter-camera global loop closure

DynaVINS++:
  dynamic residual의 영향 차단
  잘못된 visual optimum이 IMU bias까지 오염시키는지 검사
  불안정 state를 stable snapshot으로 되돌리는 recovery protocol
```

이 네 가지는 서로 대체 관계가 아니다.

```text
Basalt        = estimator backbone
VILENS-MC     = multi-camera tracking / budget policy
MAVIS         = sensor handling / exact IMU / global multi-camera SLAM
DynaVINS++    = robust optimization / state recovery
```

따라서 최종 시스템은 여러 VIO를 병렬로 붙인 ensemble이 아니라 다음의 의도된 hybrid다.

> **Basalt의 하나의 sliding-window VIO 안에 VILENS-MC식 CCFT와 fixed budget, MAVIS식 exact IMU·timing·multi-camera loop closure, DynaVINS++식 ATLS·BCC·SSR을 넣은 6-camera generalized-camera VI-SLAM.**

---

## 0. Target System and Whole Architecture

카메라 순서는 body yaw 기준으로 다음처럼 정의한다.

```text
C0: Front
C1: Front-Left
C2: Rear-Left
C3: Rear
C4: Rear-Right
C5: Front-Right
```

인접 overlap graph:

```text
C0 <-> C1 <-> C2 <-> C3 <-> C4 <-> C5
 ^                                      |
 |______________________________________|
```

수식으로는:

\[
\mathcal E_{\mathrm{overlap}}
=
\{(0,1),(1,2),(2,3),(3,4),(4,5),(5,0)\}.
\]

전체 camera pair는 15개지만 cross-camera tracking은 실제 overlap edge 6개에서만 한다. 각 edge는 양방향으로 사용할 수 있으므로 구현상 directed edge 12개를 만들 수 있지만, 물리 topology는 undirected ring 6개다.

가장 중요한 상태 설계:

```text
잘못된 구조:
  C0 VIO + C1 VIO + ... + C5 VIO
  -> pose 6개를 나중에 fuse

최종 구조:
  하나의 body state x_k
  + camera별 고정 extrinsic T_BC_i
  + 6개 camera observation residual
  + one IMU factor chain
```

시각 `k`의 상태는 다음과 같다.

\[
\mathbf x_k=
\left(
\mathbf T_{WB_k},
\mathbf v_{W_k},
\mathbf b_{g_k},
\mathbf b_{a_k}
\right).
\]

카메라 pose는 별도 최적화 state가 아니라 body pose와 calibrated extrinsic으로 결정한다.

\[
\mathbf T_{WC_i,k}
=
\mathbf T_{WB_k}\mathbf T_{BC_i}.
\]

따라서 카메라 사이에서 같은 feature를 한 번도 연결하지 못해도, 각 camera residual이 동일한 local body-state window / trajectory를 제약하므로 이미 tight-coupled multi-camera VIO다. CCFT는 tight coupling의 성립 조건이 아니라 다음을 개선하는 보강 모듈이다.

```text
CCFT의 역할:
  camera seam에서 track lifetime 연장
  overlap의 duplicate landmark 감소
  동일 timestamp multi-view depth 개선
  더 긴 temporal constraint 형성

CCFT가 담당하지 않는 것:
  multi-camera tight coupling 자체의 성립
  모든 landmark의 depth 초기화
  descriptor 기반 global relocalization
```

전체 구조:

```text
+--------------------------------------------------------------+
| 6 hardware-synchronized monocular cameras + 1 IMU            |
+-----------------------------+--------------------------------+
                              |
                              v
+--------------------------------------------------------------+
| Sensor Calibration / Time Model                              |
| intrinsics, T_BC_i, camera-IMU delay, global-shutter         |
| mid-exposure, IMU intrinsic compensation                     |
+-----------------------------+--------------------------------+
                              |
                              v
+--------------------------------------------------------------+
| Rig FrameBundle Builder                                      |
| one reference body time + up to 6 optional camera frames     |
| frame drop 허용, 늦은 camera 때문에 전체 bundle 정지 금지    |
+-----------------------------+--------------------------------+
                              |
               +--------------+--------------+
               |                             |
               v                             v
+-----------------------------+   +-----------------------------+
| Per-camera Temporal Tracker |   | Rig Local-Map Projector     |
| Basalt direct patch flow     |   | visible mature landmarks   |
| IMU/depth initial guess      |   | -> current cameras         |
+--------------+--------------+   +--------------+--------------+
               |                             |
               +--------------+--------------+
                              |
                              v
+--------------------------------------------------------------+
| Adjacent-Camera CCFT                                         |
| ring edge only                                               |
| mature: direct projection + covariance-aware patch refine    |
| immature: inverse-depth interval -> epipolar curve search    |
+-----------------------------+--------------------------------+
                              |
                              v
+--------------------------------------------------------------+
| Unified Rig Landmark Manager                                 |
| global ID / immature-mature / inverse depth / covariance     |
| observation confidence q / static confidence s / rehosting   |
+-----------------------------+--------------------------------+
                              |
                              v
+--------------------------------------------------------------+
| Rig-Centric Feature Budget                                   |
| B_track: frontend maintained tracks                          |
| B_opt: backend selected landmarks, B_opt <= B_track          |
| camera + azimuth/elevation quotas + robust max-logdet        |
+-----------------------------+--------------------------------+
                              |
                              v
+--------------------------------------------------------------+
| DynaVINS++ Robustness Layer                                  |
| static-weighted angular parallax                             |
| ATLS alternating weights/state                              |
+-----------------------------+--------------------------------+
                              |
                              v
+--------------------------------------------------------------+
| One Basalt Square-Root Sliding-Window Backend                |
| all-camera reprojection + exact SE_2(3) IMU + prior          |
+-----------------------------+--------------------------------+
                              |
                              v
+--------------------------------------------------------------+
| BCC -> Stable Commit Protocol                                |
| pass: marginalize / NFR export                               |
| fail: restore -> diagnostics -> RetryPolicy -> bounded retry |
+-----------------------------+--------------------------------+
                              |
             +----------------+----------------+
             |                                 |
             v                                 v
+-----------------------------+   +-----------------------------+
| High-rate Local Odometry    |   | Low-rate Global SLAM        |
| IMU-rate propagated output  |   | keyframe-only descriptors   |
| smooth odom->base           |   | intra/inter camera loops    |
+-----------------------------+   | NFR + global BA             |
                                  +--------------+--------------+
                                                 |
                                                 v
                                  +-----------------------------+
                                  | map->odom correction        |
                                  | persistent global map       |
                                  +-----------------------------+
```

최종 목표:

- 한 방향이 textureless·overexposed·occluded여도 다른 yaw camera가 body pose를 계속 제약한다.
- 카메라 수가 6개라고 backend landmark 수와 BA 비용이 6배 증가하지 않는다.
- local VIO의 high-frequency tracking에는 descriptor를 사용하지 않는다.
- camera seam에서는 landmark ID가 끊기지 않되, 불확실한 cross-camera association을 억지로 합치지 않는다.
- dynamic object가 image 대부분을 차지해도 visual residual이 pose뿐 아니라 IMU bias와 marginalization prior를 오염시키는 것을 막는다.
- local `odom`은 제어에 적합하게 연속적이고, global loop correction은 `map->odom`에 반영한다.
- Occupancy Network가 사용할 pose·gravity·landmark·dynamic/free-space evidence를 명확한 interface로 내보낸다.

---

## 1. Coordinate Frames, State and Time Convention

### 1.1 Coordinate frames

```text
W:
  VIO world frame
  initialization 시 gravity-aligned
  yaw와 global position은 gauge freedom

B:
  body / IMU reference frame
  estimator의 pose state가 정의되는 frame

C_i:
  i번째 camera optical frame, i in [0,5]

P_i:
  i번째 image pixel domain
```

변환 convention은 문서 전체에서 다음으로 고정한다.

```text
T_AB:
  B frame에 표현된 점을 A frame으로 변환

p_A = T_AB * p_B
```

따라서:

\[
\mathbf p_{C_i}
=
\mathbf T_{C_iB}
\mathbf T_{BW}
\mathbf p_W.
\]

코드 전체에서 `T_BC`와 `T_CB`를 혼용하지 않는다. calibration 파일, projection 함수, Jacobian 단위 테스트는 이 convention을 기준으로 한다.

### 1.2 Estimator state

window state:

\[
\mathcal X_{\text{nav}}
=
\left\{
\mathbf T_{WB_k},
\mathbf v_{W_k},
\mathbf b_{g_k},
\mathbf b_{a_k}
\right\}_{k\in\mathcal W}.
\]

landmark state:

\[
\mathcal X_{\text{lm}}
=
\{\rho_\ell\}_{\ell\in\mathcal L},
\]

여기서 `rho_l`은 host frame-camera pair에서 정의되는 inverse depth다.

운용 중 기본적으로 고정하는 calibration state:

```text
T_BC_i             camera extrinsic
camera intrinsics  projection parameters
Delta t_i^clk      camera timestamp clock to IMU/body-clock offset
Delta t_mid        timestamp convention to exposure-midpoint correction
global shutter     rolling_shutter_readout_i = 0
M_g, M_a            IMU scale / misalignment matrices
```

최종 운용 estimator에서 calibration을 모두 자유 state로 두지 않는다. offline calibration 후 고정하는 것이 기본이며, calibration mode에서만 strong prior를 둔 작은 correction을 허용한다.

global shutter에서 `rolling_shutter_readout_i = 0`이라는 말은 row별 pose skew가 없다는 뜻이다. sensor transport latency, timestamp pipeline delay, camera-IMU clock offset이 0이라는 뜻이 아니며, 이들은 `Delta t_i^{clk}`와 exposure midpoint correction에 포함한다. 같은 지연을 `Delta t_i^{clk}`와 `Delta t_mid`에 중복 저장하지 않는다.

### 1.3 Camera observation time

camera `i`의 hardware timestamp를 `\tau_{k,i}`라 하고, 실제 exposure center의 body-clock time을 다음처럼 둔다.

\[
t_{k,i}
=
\tau_{k,i}
+
\Delta t_i^{\mathrm{clk}}
+
\Delta t^{\mathrm{mid}}_{k,i}.
\]

여기서 `Delta t_i^{clk}`는 camera timestamp clock에서 IMU/body clock으로 가는 fixed offset이고, `Delta t^{mid}_{k,i}`는 timestamp가 exposure start/end/driver arrival 중 어디를 의미하는지에 따른 exposure-midpoint correction이다. hardware timestamp가 이미 exposure midpoint의 body-clock time이면 두 항은 0으로 collapse한다. `CameraFrame.exposure_midpoint`는 위 변환을 이미 적용한 최종 body-clock exposure center이며, downstream residual은 raw `sensor_timestamp`에 offset을 다시 더하지 않는다. bundle reference `t_k`는 선택된 rig reference exposure midpoint이며, 구현에서는 `delta t_{k,i}=t_{k,i}-t_k`만 residual attach / interpolation 결정에 사용한다.

최종 target hardware는 global shutter camera이므로 같은 image의 모든 pixel은 동일한 `t_{k,i}`를 사용한다. 즉 rolling-shutter row-time residual은 기본 estimator에 넣지 않는다.

각 visual residual은 명목상 동일 bundle에 속하더라도 `t_{k,i}`의 body pose를 사용한다. 하드웨어 동기화가 있어도 exposure midpoint와 sensor transport 지연은 0이라고 가정하지 않는다.

중요한 점은 `T_WB(t_{k,i})`가 독립 pose state가 아니라는 것이다. observation time이 bundle reference time과 다르면, 다음 두 계약 중 하나를 명시적으로 선택한다.

```text
default synchronized global-shutter path:
  rig reference time t_k를 exposure midpoint에 맞춘다.
  per-frame time jitter가 만드는 predicted reprojection error가 gate 안이면 residual은 해당 active state(s)에 직접 연결한다.
  남은 independent timestamp jitter는 projection covariance S_time에 넣는다.

interpolated observation path:
  predicted reprojection error가 gate를 넘거나 partial bundle/drop handling이 필요하면
  t_o를 active window 안의 두 state 사이에 두고
  IMU preintegration 기반 interpolation operator로 residual을 연결한다.
```

direct attach 여부는 raw time threshold만으로 결정하지 않는다. 빠른 yaw 회전이나 근거리 translation에서는 작은 `|t_{k,i}-t_k|`도 큰 pixel error를 만들 수 있으므로, 다음 motion-aware gate를 사용한다.

`S_time`은 fixed calibration offset prior가 아니라, fixed calibration을 적용한 뒤 남는 per-frame timestamp jitter / trigger jitter / metadata quantization uncertainty를 나타낸다. `Delta t_i^{clk}`의 calibration uncertainty는 같은 camera의 여러 residual에 공통으로 걸리는 systematic nuisance이므로, 일반 VIO residual covariance에 독립 노이즈처럼 반복해서 넣지 않는다. 그 uncertainty가 무시하기 어렵다면 calibration mode에서 correction state와 strong prior로 다루거나 offline calibration을 다시 수행한다.

\[
\mathbf S_{\mathrm{time},o}
=
\mathbf J_{\tau,o}
\mathbf P_{\tau,o}
\mathbf J_{\tau,o}^{\top}.
\]

단일 current projection처럼 target observation time만 들어가는 residual에서는

\[
\mathbf J_{\tau,o}
=
\left[
\frac{\partial \hat{\mathbf u}_o}{\partial t_o}
\right],
\qquad
\mathbf P_{\tau,o}
=
\left[
\sigma^{2,\mathrm{jit}}_{t,o}
\right].
\]

host inverse-depth reprojection처럼 host exposure time과 target exposure time이 모두 들어가는 residual에서는 둘을 동시에 넣는다.

\[
\mathbf J_{\tau,o}
=
\left[
\frac{\partial \hat{\mathbf u}_o}{\partial t_h}
\quad
\frac{\partial \hat{\mathbf u}_o}{\partial t_o}
\right],
\qquad
\mathbf P_{\tau,o}
=
\begin{bmatrix}
\sigma^{2,\mathrm{jit}}_{t,h} & \sigma^{\mathrm{jit}}_{t,ho} \\
\sigma^{\mathrm{jit}}_{t,ho} & \sigma^{2,\mathrm{jit}}_{t,o}
\end{bmatrix}.
\]

같은 trigger 또는 같은 software timestamp path를 공유하면 `sigma^{jit}_{t,ho}=0`이라고 가정하지 않는다. independent per-frame timestamp jitter라고 검증된 경우에만 off-diagonal을 0으로 둔다.

```text
direct attach:
  trace(S_time,o) 또는 worst-axis pixel sigma가 threshold 이하
  그리고 Mahalanobis gate에서 S_time,o를 measurement covariance에 포함

interpolate:
  S_time,o가 gate를 넘음
  또는 exposure time이 reference state와 다른 active interval에 있음
```

interpolated path의 pose는 surrounding window state와 IMU preintegration으로 만든 미분 가능한 interpolation / propagation operator를 사용한다.

\[
\mathbf T_{WB}(t_o)
=
\mathcal I_T
\left(
\mathbf x_a,
\mathbf x_b,
\mathcal P_{a\rightarrow o},
\mathcal P_{o\rightarrow b}
\right),
\qquad
t_a \le t_o \le t_b.
\]

visual residual의 state Jacobian은 이 operator를 통해 연결된다.

\[
\mathbf J_o
=
\mathbf J_\pi
\mathbf J_T
\left[
\frac{\partial \mathbf T_{WB}(t_o)}{\partial \delta\mathbf x_a},
\frac{\partial \mathbf T_{WB}(t_o)}{\partial \delta\mathbf x_b}
\right].
\]

`Delta t_i^{clk}`와 timestamp-to-midpoint convention은 최종 운용에서는 fixed calibration이다. 따라서 일반 VIO residual의 optimization Jacobian에는 time-offset correction state에 대한 Jacobian block을 넣지 않는다. `S_time`에는 fixed offset prior가 아니라 observation-local jitter만 반영한다. calibration mode에서만 strong prior를 둔 `delta Delta t_i^{clk}` correction state를 추가하고, 그때만 time-offset Jacobian을 factor에 포함한다.

`t_o`가 active window의 최신 state보다 미래에 있으면 미래 state를 암묵적으로 참조하지 않는다. 구현은 state node를 `t_o` 이후까지 생성하고 optimize하거나, 해당 observation 처리를 한 bundle 지연시키거나, deadline 정책에 따라 drop한다.

따라서 timestamp 보정은 frozen propagated pose에 residual을 붙이는 근사가 아니라, sliding-window state와 같은 linearization / marginalization convention 안에서 처리한다.

미래에 rolling-shutter camera를 지원한다면 `t_o`를 row별 `t_{k,i,r}`로 바꾸고 row-time Jacobian, covariance gate, patch sampling time을 모두 함께 추가해야 한다. 현재 문서의 최종 구현 대상에서는 제외한다.

### 1.4 Output frames

```text
odom:
  local VIO가 생성하는 continuous frame
  loop closure가 있어도 odom->base는 갑자기 점프하지 않음

map:
  global mapping / loop closure가 정정하는 frame

TF target:
  map -> odom -> base_link(B)
```

local VIO의 목적은 가장 최근 pose의 저지연·연속성이다. global SLAM의 목적은 장기 drift 제거다. 두 목적을 같은 pose stream에 강제로 섞지 않는다.

---

## 2. Overall Architecture

추천 전체 처리 순서:

```text
IMU stream
-> intrinsic compensation
-> high-rate propagation / preintegration

6 camera streams
-> frame timestamp correction
-> photometric linearization / vignetting correction
-> batched pyramid / gradient
-> rig-aware detection
-> per-camera temporal direct tracking
-> local-map reprojection
-> adjacent-camera CCFT
-> landmark lifecycle update
-> dynamic/static confidence update
-> B_track -> robust B_opt selection
-> one multi-camera visual-inertial BA
-> BCC
   pass -> commit -> rehost -> marginalize -> NFR export
   fail -> stable snapshot restore -> diagnostics
        -> RetryPolicy -> rebuild tentative graph -> bounded retry
-> optimized local state + IMU-rate propagated state

rig keyframe stream
-> selected-camera descriptor extraction
-> BoW / place retrieval
-> generalized-camera geometric verification
-> intra/inter-camera loop constraint
-> NFR + global BA / pose graph
-> map->odom correction
```

해당 구조에서 “frontend”와 “backend”를 정확히 구분한다.

```text
Frontend:
  image preprocessing
  detector
  temporal patch tracking
  cross-camera patch tracking
  triangulation
  landmark data association
  quality / static confidence
  feature budget

Local backend:
  IMU preintegration
  all-camera reprojection residual
  ATLS alternating optimization
  BCC / SSR
  square-root marginalization

Global backend:
  keyframe descriptor / place recognition
  loop geometric verification
  NFR factors
  pose graph or global bundle adjustment
```

### 2.1 Core estimator modules and supporting runtime modules

Core estimation modules:

| Module | 역할 |
|---|---|
| `RigTemporalTracker` | camera별 Basalt direct patch tracking |
| `RigProjectionTracker` | mature local landmark를 보일 수 있는 camera에 재투영 |
| `CrossCameraTracker` | adjacent edge의 CCFT / epipolar direct search |
| `RigLandmarkManager` | global landmark ID, inverse depth, covariance, rehosting |
| `RigFeatureBudgeter` | B_track/B_opt, coverage quota, robust information selection |
| `RobustWeightManager` | `s_l`, `q_o`, ATLS weight와 truncation range |
| `MultiCameraVioEstimator` | one body-state sliding-window optimization |
| `StableStateManager` | BCC, snapshot, SSR, stable commit |
| `NfrExporter` | stable marginalization information을 global mapper로 전달 |
| `RigLoopCloser` | keyframe-only intra/inter-camera loop closure |

Supporting runtime modules:

| Module | 역할 |
|---|---|
| `SensorTimeSynchronizer` | camera/IMU timestamp, deadline, partial bundle |
| `PhotometricCalibrator` | response/vignetting/exposure normalization |
| `CameraOverlapGraph` | depth-dependent overlap/visibility candidates |
| `ImuPropagator` | IMU-rate low-latency output |
| `HealthMonitor` | camera별/rig 전체 tracking 상태와 degraded mode |
| `LoadController` | latency에 따라 detector / B_track / B_opt를 조정 |
| `OccupancyBridge` | pose, gravity, landmarks, rays, feedback factors interface |

예측 state와 runtime control을 구분해야 한다.

```text
Estimated states:
  pose / velocity / gyro bias / accel bias
  inverse-depth landmarks
  optional calibration corrections (calibration mode only)

Robust latent variables:
  landmark static confidence s_l
  observation tracking confidence q_o
  ATLS auxiliary weights

Runtime controls:
  camera deadline
  detector threshold / grid quota
  CCFT candidate budget
  B_track / B_opt
  loop descriptor budget
```

`B_track`와 `B_opt`는 estimator state가 아니라 compute-control 변수다. `s_l`과 ATLS weight는 visual residual에 영향을 주는 robust latent variable지만, world geometry 자체를 의미하지 않는다.

---

## 3. Sensor Rig Calibration and Preprocessing

### 3.1 Camera intrinsics

각 camera는 독립적인 projection model을 갖는다.

```text
지원 후보:
  pinhole + radial-tangential
  Kannala-Brandt fisheye
  Double Sphere
  Unified / Extended Unified
```

Basalt가 지원하는 camera model interface를 그대로 활용하되, 모든 projection / unprojection / Jacobian은 camera ID에 따라 선택한다.

```text
pi_i(p_C)        -> pixel u_i
pi_i^{-1}(u_i)   -> unit bearing b_C_i
J_pi_i            projection Jacobian
```

모든 카메라의 pitch가 같다는 기구 설계는 초기값과 overlap topology를 단순화할 뿐, projection model에서 pitch를 동일 상수로 hard-code하는 근거가 아니다. 실제 조립 오차를 포함한 full `SE(3)` extrinsic을 사용한다.

### 3.2 Camera-to-body extrinsics

필수:

```text
T_BC_0 ... T_BC_5
```

권장 calibration 방식:

```text
1. camera intrinsics를 충분한 depth / FOV coverage로 추정
2. 6 cameras + IMU를 하나의 continuous-time calibration problem으로 joint calibration
3. time offset과 extrinsic을 함께 추정
4. 모든 adjacent overlap edge에서 cross-camera reprojection 검증
5. calibration covariance / residual report 저장
```

최종 운용:

```text
offline precise calibration:
  mandatory

startup calibration check:
  static reprojection / gravity / time consistency

runtime online full extrinsic optimization:
  금지 (기본)

slow correction with strong prior:
  calibration mode에서만 선택
```

이유:

- 6개 camera extrinsic을 VIO state와 동시에 자유롭게 풀면 state dimension과 gauge/observability 문제가 증가한다.
- 15~20°의 좁은 overlap에서는 모든 extrinsic component를 온라인으로 안정적으로 관측하기 어렵다.
- cross-camera association 오차와 extrinsic 오차를 estimator가 서로 대신 설명할 수 있다.

### 3.3 Time calibration

필수 항목:

```text
camera hardware timestamp
camera timestamp clock -> IMU/body-clock offset Delta t_i^clk
timestamp convention -> exposure midpoint correction Delta t_mid
per-frame timestamp / trigger jitter model
global-shutter validation (rolling-shutter readout = 0)
frame sequence / dropped-frame detection
```

MAVIS의 핵심 교훈은 “multi-camera 장치가 nominally synchronized”라는 사실만으로 시간 오차가 사라지지 않는다는 것이다. bandwidth 증가로 일부 frame이 누락되거나 camera별 exposure 시각이 달라질 수 있으므로, 남은 camera observation을 계속 사용하고 각 관측의 실제 global-shutter exposure midpoint로 body pose를 보간해야 한다.

검증 항목에서 `rolling-shutter readout = 0`은 row별 exposure offset이 없다는 뜻이다. hardware timestamp가 exposure midpoint를 직접 의미하지 않으면 transport / driver / trigger delay는 `Delta t_i^{clk}`에, timestamp가 exposure start/end/arrival 중 무엇을 의미하는지는 `Delta t_mid`에 한 번만 반영한다. 반복 residual covariance에는 frame마다 달라지는 timestamp / trigger jitter만 넣고, fixed offset calibration prior를 독립 noise처럼 매 observation에 넣지 않는다.

### 3.4 Photometric calibration

Basalt식 direct patch tracking은 camera 간 밝기 차이에 민감하다.

오프라인:

```text
camera response function
vignetting / lens shading
black level
gain / exposure metadata scale
```

운용 이미지 branch:

```text
Tracking branch:
  raw 또는 linearized grayscale
  response / vignetting correction
  affine brightness model
  매 frame 비선형 histogram equalization 금지

Detection branch:
  corner detection 안정화가 필요할 때만
  제한적 CLAHE / contrast-normalized image 허용
  검출 좌표는 tracking branch의 원본/선형 영상으로 전달
```

MAVIS의 histogram equalization은 ORB feature 수를 늘리는 데 유효할 수 있지만, frame마다 달라지는 비선형 intensity mapping은 direct photometric consistency를 파괴할 수 있다. 두 branch를 분리하면 두 목적이 충돌하지 않는다.

### 3.5 IMU calibration

필수:

```text
noise density
random walk
scale factor
axis misalignment
gyro/accel cross-axis matrix
optional g-sensitivity
saturation range
```

MAVIS식 IMU intrinsic compensation을 exact preintegration 이전에 적용한다.

\[
\boldsymbol\omega
=
\mathbf M_g(\boldsymbol\omega_m-\mathbf b_g-\mathbf n_g),
\]

\[
\mathbf a
=
\mathbf M_a(\mathbf a_m-\mathbf b_a-\mathbf n_a).
\]

IMU model이 달라지면 preintegration residual, covariance, bias Jacobian, BCC 기준을 모두 같은 model로 다시 맞춰야 한다.

### 3.6 Calibration validation gates

배포 전 자동 검증:

```text
intrinsic reprojection RMS / spatial residual map
adjacent-camera cross-reprojection error
camera-IMU temporal residual sweep
static gravity direction consistency
IMU Allan variance consistency
rotation-only sequence의 optical-flow prediction error
translation sequence의 landmark-depth projection error
```

runtime calibration-health signal:

```text
adjacent edge마다:
  median CCFT reprojection residual
  photometric alignment success rate
  forward-backward error

특정 edge만 지속적으로 나쁘면:
  해당 edge CCFT를 disable
  두 camera의 independent temporal tracks는 유지
  전체 VIO는 계속 동작
```

---

## 4. Rig FrameBundle and Camera Topology

### 4.1 FrameBundle

하나의 `FrameBundle`은 하나의 body reference time과 최대 6개의 camera observation을 가진다.

```cpp
struct FrameBundle {
    int64_t bundle_id;
    TimeNs reference_time;
    std::array<std::optional<CameraFrame>, 6> frames;
    ImuPrediction predicted_state;
    BundleHealth health;
};
```

`CameraFrame`:

```cpp
struct TimestampUncertainty {
    double jitter_std_sec;             // per-frame jitter only, not fixed offset prior
    int64_t shared_trigger_group_id;   // -1 if independent
};

struct CameraFrame {
    CameraId camera_id;
    TimeNs sensor_timestamp;           // raw camera/driver timestamp
    TimeNs exposure_midpoint;          // final body-clock exposure center used by residuals
    TimestampUncertainty timestamp_uncertainty;
    ImageView linear_gray;
    ImagePyramid pyramid;
    GradientPyramid gradients;
    ExposureMetadata exposure;
};
```

`exposure_midpoint`는 `sensor_timestamp`, `timestamp_to_body_offset_sec`, timestamp convention, exposure metadata를 모두 적용한 결과다. downstream tracker/backend는 이 값에 calibration offset을 다시 더하지 않는다. auto exposure나 driver metadata 때문에 midpoint correction이 frame마다 달라지면 `ExposureMetadata`에서 계산해 `CameraFrame.exposure_midpoint`에 반영한다.

### 4.2 Partial bundle

다음은 정상적인 입력으로 취급한다.

```text
C0 valid
C1 valid
C2 dropped
C3 valid
C4 valid
C5 valid
```

이 경우:

- C2 observation만 없다.
- IMU factor와 나머지 5개 camera residual은 그대로 사용한다.
- C2의 기존 landmark는 immediately delete하지 않고 visibility/missed-count 상태로 유지한다.
- C2 때문에 전체 bundle을 폐기하거나 긴 IMU integration을 강제하지 않는다.

단, bundle builder는 무한정 늦은 camera를 기다리지 않는다.

```text
camera deadline:
  one-frame latency budget 안의 bounded wait

late frame:
  다음 bundle에 잘못 삽입하지 않고 drop / diagnostic

IMU:
  camera보다 우선 보존
  bounded queue overflow 시 image를 먼저 drop
```

### 4.3 Overlap graph

```cpp
struct CameraOverlapEdge {
    CameraId a;
    CameraId b;
    SE3d T_b_a;
    DepthDependentOverlapMask mask_a_to_b;
    DepthDependentOverlapMask mask_b_to_a;
    EdgeHealth health;
};
```

고정 사각형 overlap box는 detector candidate를 줄이는 coarse mask로만 사용할 수 있다. 실제 cross-camera projection은 다음을 사용한다.

```text
camera model
full T_CjCi
landmark depth / inverse-depth interval
pose/time uncertainty
```

overlap 영역은 depth에 따라 변한다. 따라서 offline mask는 여러 depth bin에서 ray를 다른 camera로 투영해 만든다.

```text
near mask:
  large parallax / occlusion을 고려한 보수적 영역

mid mask:
  일반 CCFT candidate 영역

far / infinity mask:
  rotation-dominant common FoV 영역
```

### 4.4 Same-pitch ring의 이점과 한계

이점:

- overlap이 주로 image 좌우 seam에 형성된다.
- body azimuth 기준으로 camera 순서를 안정적으로 정의할 수 있다.
- cylindrical rig grid와 seed ownership이 단순하다.
- relative rotation 초기값이 주로 yaw이므로 patch warp 초기화가 쉬워진다.

한계:

- 실제 extrinsic은 yaw-only가 아니다.
- camera center가 거의 같고 yaw만 다르면 same-time triangulation baseline이 거의 없다.
- 15~20° overlap은 36° overlap을 사용한 VILENS-MC 장치보다 좁다.
- 따라서 CCFT는 강건성을 높이는 보조 constraint이며, estimator가 살아남기 위한 단일 기반으로 삼지 않는다.

---

## 5. Image Pyramid, Detection and Photometric Tracking Input

### 5.1 Batched preprocessing

6개의 image pyramid와 gradient를 camera별 독립 할당으로 만들되, 실행은 batch / parallel로 묶는다.

```text
linearized grayscale
-> undistortion은 필요 시 lookup / camera model 내부 sampling
-> Gaussian pyramid
-> Ix / Iy gradient pyramid
-> detector response
```

full panorama stitching은 사용하지 않는다.

```text
panorama를 사용하지 않는 이유:
  seam interpolation이 patch photometry를 변형
  projection uncertainty가 image resampling에 숨겨짐
  near-field parallax와 occlusion을 하나의 panorama가 표현하지 못함
  한 camera drop이 전체 panorama geometry를 흔듦
  generalized camera의 정확한 ray origin을 잃음
```

### 5.2 Detector

기본 후보:

```text
FAST / AGAST / Shi-Tomasi
```

Basalt의 detector와 patch tracker를 최대한 유지한다. detector가 만드는 것은 descriptor가 아니라 “track을 시작할 수 있는 corner 위치 + reference patch”다.

카메라마다 독립적으로 feature 수를 꽉 채우지 않는다. detector는 Section 11의 rig cylindrical grid와 Section 12의 B_track budget을 따른다.

### 5.3 Patch representation

각 seed는 다음을 저장한다.

```text
reference image pyramid level
reference patch intensities
reference patch gradients / Hessian
host camera / frame
photometric affine parameters
initial inverse-depth interval
```

cross-camera patch appearance 차이를 위해 target observation마다 affine brightness model을 둔다.

\[
r_p
=
I_j(\mathbf W(\mathbf p))
-
\left(e^{a_{ij}}I_i(\mathbf p)+b_{ij}\right).
\]

여기서 `a_ij`, `b_ij`는 global camera calibration parameter가 아니라 해당 patch alignment에서 추정하거나 exposure metadata로 초기화하는 local nuisance parameter다.

### 5.4 Detector enhancement와 tracking 영상 분리

```text
Detector image:
  optional local contrast normalization

Tracker image:
  linear photometric domain

금지:
  CLAHE 결과 patch와 raw 결과 patch를 직접 비교
```

corner를 잘 찾는 것과 brightness constancy를 지키는 것은 서로 다른 문제다. detector-only enhancement로 둘을 분리한다.

---

## 6. Per-Camera Temporal Direct Tracking

각 camera의 시간 방향 tracking은 Basalt식 sparse direct optical flow를 사용한다.

```text
C_i(k-1) reference patch
        |
        +-- IMU propagated body motion
        +-- camera extrinsic
        +-- landmark inverse depth / covariance
        v
predicted pixel in C_i(k)
        |
        v
coarse-to-fine patch alignment
        |
        v
photometric / forward-backward / geometry validation
```

### 6.1 Mature landmark initial guess

세계 좌표의 mature landmark `p_W_l`을 현재 camera에 투영한다.

\[
\hat{\mathbf u}_{k,i,\ell}
=
\pi_i
\left(
\mathbf T_{C_iB}
\mathbf T_{BW}(t_{k,i})
\mathbf p_{W,\ell}
\right).
\]

이 initial guess는 다음 불확실성을 반영한다.

```text
pose prediction covariance
landmark covariance
camera extrinsic calibration covariance
per-frame camera timestamp jitter
```

projection covariance:

\[
\mathbf S_{u}
=
\mathbf J_x\mathbf P_x\mathbf J_x^\top
+
\mathbf J_\ell\mathbf P_\ell\mathbf J_\ell^\top
+
\mathbf S_{\mathrm{calib}}
+
\mathbf S_{\mathrm{time}}.
\]

search radius는 고정 pixel이 아니라 `S_u`의 confidence ellipse로 결정한다.

frontend search radius에는 필요하면 fixed calibration uncertainty에서 온 보수적 margin을 별도로 더할 수 있다. 하지만 backend residual covariance에서 fixed time offset prior를 independent timestamp jitter처럼 반복 적용하지 않는다는 Section 1.3의 원칙은 유지한다.

### 6.2 Immature track initial guess

아직 depth가 안정적이지 않은 track은 다음을 결합한다.

```text
IMU rotation prediction
last optical-flow velocity
inverse-depth interval
camera motion direction
```

rotation-only infinity warp:

\[
\hat{\mathbf u}^{\infty}_{k,i}
=
\pi_i
\left(
\mathbf R_{C_iB}
\mathbf R_{B_kB_{k-1}}
\mathbf R_{BC_i}
\pi_i^{-1}(\mathbf u_{k-1,i})
\right).
\]

translation은 depth interval이 만드는 search region으로 반영한다. 평균 depth 하나를 모든 feature에 적용하지 않는다.

### 6.3 2025 Basalt extension의 IMU prediction에서 채택/수정하는 점

채택:

```text
이전 pixel을 그대로 initial guess로 두지 않음
IMU로 다음 pose를 예측해 optical flow의 수렴 basin 안에서 시작
빠른 회전에서 pyramid search 실패 감소
```

수정:

```text
논문:
  triangulated landmarks의 average depth를 feature prediction에 사용

최종 구조:
  mature landmark -> landmark별 inverse depth
  immature track   -> inverse-depth interval / rotation-only warp
```

평균 depth는 순수 회전에서는 image displacement가 depth에 거의 무관하므로 잘 작동한다. 그러나 translation이 있으면 근거리와 원거리 parallax가 달라진다. 따라서 ground robot의 전진·회전 복합 운동에는 feature별 depth uncertainty가 필요하다.

### 6.4 IMU prediction이 해결하지 않는 것

IMU initial guess는 motion blur를 제거하지 않는다.

```text
해결:
  frame-to-frame displacement가 커서 optimizer가 잘못된 위치에서 시작하는 문제

해결하지 못함:
  exposure 중 texture가 실제로 사라진 blur
  saturation
  low-light shot noise
```

따라서 blur 대응은 다음의 조합이다.

```text
short exposure / synchronized exposure control
IMU initial guess
larger coarse pyramid
patch quality rejection
다른 yaw camera의 redundancy
```

### 6.5 Patch alignment

권장:

```text
coarse-to-fine pyramid
translation 또는 small affine warp
Gauss-Newton / inverse compositional update
subpixel convergence
per-patch affine brightness
```

종료 조건:

```text
max iteration
update norm
photometric cost reduction
Hessian condition number
image boundary
```

### 6.6 Temporal tracking validation

최소 gate:

```text
1. patch gradient / Hessian eigenvalue
2. pyramid-level convergence
3. normalized photometric residual
4. forward-backward consistency
5. predicted covariance ellipse
6. image boundary / valid mask
7. track acceleration plausibility
```

forward-backward:

\[
\|\mathbf u_{k-1}-\hat{\mathbf u}_{k-1}^{\mathrm{back}}\|
<\tau_{\mathrm{fb}}.
\]

threshold는 pyramid level과 projection covariance에 따라 조절한다.

---

## 7. Camera Visibility and Depth-Dependent Overlap Graph

### 7.1 Visibility query

mature landmark는 매 frame 6개 image 전체를 검색하지 않는다.

```text
1. landmark를 current body frame으로 변환
2. camera frustum / projection validity 검사
3. occlusion / viewing-angle / image-border 검사
4. 관측 가능한 camera set V_l 계산
5. current owner camera + adjacent camera만 patch alignment
```

\[
\mathcal V_{k,\ell}
=
\left\{
 i\;|\;
\pi_i(\mathbf p_{C_i})\in\Omega_i,
\;z_{C_i}>0,
\;\alpha_{i,\ell}<\alpha_{\max}
\right\}.
\]

15~20° overlap에서는 일반적으로 `|V_l|`이 1 또는 2다. 따라서 landmark마다 6-camera all-search가 필요 없다.

### 7.2 Overlap mask 생성

각 edge `(i,j)`에 대해 source pixel ray를 여러 inverse-depth bin으로 target camera에 투영한다.

\[
\mathbf u_j(\rho)
=
\pi_j
\left(
\mathbf T_{C_jC_i}
\frac{\pi_i^{-1}(\mathbf u_i)}{\rho}
\right).
\]

다음 depth interval을 고려한다.

```text
near:
  robot 근거리 obstacle / strong parallax

mid:
  일반 landmark depth

far:
  infinity-like rotation overlap
```

mask는 candidate pruning용이다. mask 안에 있다는 이유만으로 correspondence를 만들지 않는다.

### 7.3 Ring graph가 parent-child tree보다 적합한 이유

parent-child tree에서는 camera 하나가 최대 하나의 parent를 갖고 feature가 순차적으로 전달된다. 그러나 ring에서는 각 camera가 양쪽 이웃과 overlap한다.

```text
C0의 이웃:
  C1, C5

C3의 이웃:
  C2, C4
```

parent-child로 강제하면:

- 마지막 edge `(C5,C0)`를 자연스럽게 표현하기 어렵다.
- 처리 순서에 따라 duplicate/누락이 달라진다.
- parent failure가 child tracking에 전파된다.
- 양방향 seam handoff를 대칭적으로 관리할 수 없다.

최종 구조의 graph edge는 독립적이며 camera tracking은 edge 성공 여부와 무관하게 유지된다.

### 7.4 Narrow overlap policy

15~20° overlap에서는 모든 feature를 cross-camera track하려 하지 않는다.

```text
CCFT candidate:
  overlap strip에 들어온 track
  mature 또는 depth interval이 충분히 좁은 track
  patch quality가 좋은 track
  expected target view angle이 허용 범위인 track

CCFT 제외:
  image center에서 seam과 먼 track
  depth interval이 지나치게 넓음
  severe occlusion 가능성
  repetitive texture
  camera edge health degraded
```

CCFT compute는 별도 `B_xcam` budget으로 제한한다.

---

## 8. Unified Rig Landmark Representation and Lifecycle

### 8.1 Global landmark identity

landmark ID는 camera별 namespace가 아니다.

```text
잘못된 표현:
  C1 feature 32
  C2 feature 19
  -> 서로 다른 landmark로 고정

최종 표현:
  Landmark 105
    observation: C1(k-2)
    observation: C1(k-1)
    observation: C1(k)
    observation: C2(k)
    observation: C2(k+1)
```

동일 physical point가 camera seam을 넘어가도 ID를 유지한다.

### 8.2 Host-frame inverse depth

landmark `l`은 host frame-camera pair `(h_l,c_l)`에서 정의한다.

\[
\mathbf p_{C_{c_l}}
=
\frac{1}{\rho_l}
\pi_{c_l}^{-1}(\mathbf u_{h_l,c_l,l}).
\]

세계 점:

\[
\mathbf p_{W,l}
=
\mathbf T_{WB}(t_{h_l,c_l})
\mathbf T_{BC_{c_l}}
\frac{\pi_{c_l}^{-1}(\mathbf u_{h_l,c_l,l})}{\rho_l}.
\]

여기서 `t_{h_l,c_l}`은 host camera observation의 global-shutter exposure midpoint다. host frame index만으로 pose time을 암묵적으로 대체하지 않는다.

### 8.3 Immature tracklet

```cpp
struct ImmatureTracklet {
    LandmarkId id;
    FrameCameraId host;
    Vec2 host_pixel;
    PatchReference patch;
    InverseDepthInterval rho_interval;
    std::vector<FeatureObservation> observations;
    TrackQuality quality;
};
```

특징:

- exact depth state로 BA에 넣기 전 단계다.
- temporal observations로 epipolar geometry를 축적한다.
- cross-camera depth가 불확실하면 두 track을 억지로 merge하지 않는다.
- interval이 충분히 줄고 triangulation condition이 통과하면 mature로 승격한다.

### 8.4 Mature landmark

```cpp
struct MatureLandmark {
    LandmarkId id;
    FrameCameraId host;
    Vec2 host_pixel;
    double inverse_depth;
    double inverse_depth_variance;
    PatchReference reference_patch;
    std::vector<FeatureObservation> observations;
    double static_confidence;
    LandmarkStatus status;
};
```

### 8.5 Observation-level state

```cpp
struct FeatureObservation {
    FrameId frame_id;
    CameraId camera_id;
    TimeNs exposure_time;
    Vec2 pixel;
    Mat2 pixel_covariance;
    float tracking_confidence;  // q_o
    float photometric_error;
    float forward_backward_error;
    bool cross_camera;
};
```

### 8.6 Static confidence와 tracking confidence 분리

landmark-level:

\[
s_l\in[0,1]
\]

```text
의미:
  physical landmark가 static world에 속할 가능성

공유:
  camera와 timestamp가 달라도 동일 landmark에 공통
```

observation-level:

\[
q_o\in[0,1]
\]

```text
의미:
  특정 camera/timestamp의 patch alignment가 맞을 가능성

영향:
  blur, occlusion, exposure, edge distortion, CCFT mismatch
```

visual observation의 기본 신뢰도:

\[
w_o^{\mathrm{base}}=s_l q_o.
\]

한 camera에서 patch가 실패했다고 landmark 전체를 dynamic으로 만들지 않는다. 반대로 landmark가 moving object에 있으면 모든 camera observation의 `s_l`을 낮춘다.

### 8.7 Landmark lifecycle

```text
DETECTED
  -> IMMATURE_TEMPORAL
  -> TRIANGULATION_CANDIDATE
  -> MATURE
  -> MULTI_CAMERA_TRACKED
  -> REHOSTED (0회 이상)
  -> MARGINALIZED_LOCAL / EXPORTED_GLOBAL
  -> RETIRED
```

실패 branch:

```text
IMMATURE + insufficient parallax -> keep interval or timeout
bad patch / repeated FB failure  -> RETIRED
suspected false CCFT merge       -> split / rollback provisional link
low static confidence            -> backend downweight / reject
```

### 8.8 Provisional cross-camera association

cross-camera merge는 즉시 영구 확정하지 않는다.

```text
PROVISIONAL:
  single CCFT observation
  backend에는 낮은 initial weight

CONFIRMED:
  다음 temporal observation 또는 reverse-cycle 확인
  reprojection / photometric consistency 유지

REJECTED:
  후속 관측이 모순
  기존 source/target track을 다시 분리
```

nearest-corner association 하나가 global landmark graph를 영구 오염시키는 것을 막는다.

### 8.9 Landmark rehosting

host keyframe이 window 밖으로 나가기 전에 남아 있는 좋은 observation으로 host를 바꾼다.

새 host 선택 score:

```text
triangulation conditioning
viewing angle
patch gradient
photometric stability
inverse-depth covariance
window 안에서 예상 잔존 시간
```

rehosting은 global landmark ID를 바꾸지 않는다.

변경:

```text
host frame-camera pair
host bearing
inverse depth
inverse-depth covariance
reference patch
```

유지:

```text
global landmark ID
observation history
static confidence
map association
```

rehosting이 없으면 CCFT로 track lifetime을 늘려도 최초 host marginalization 시 landmark가 끊긴다.

---

## 9. Enhanced Cross-Camera Feature Tracking (CCFT)

VILENS-MC의 핵심인 CCFT를 채택하되, 최종 association은 Basalt direct patch 방식으로 강화한다.

### 9.1 VILENS-MC 원본과 최종 구조의 차이

| 단계 | VILENS-MC 개념 | 최종 구조 |
|---|---|---|
| candidate | known extrinsic + IMU + landmark depth | 동일 + projection covariance + depth-dependent overlap graph |
| target association | projected 위치와 가장 가까운 detected feature | source patch를 target image에 직접 align |
| depth unknown | 이전 landmark / pose prediction 활용 | inverse-depth interval이 만드는 epipolar curve 검색 |
| outlier | robust cost가 잘못된 association 완화 | frontend multi-gate + provisional association + ATLS |
| topology | 4-camera configured rig | 6-camera ring edge |
| budget | fixed overall budget, camera별 SFS | rig-wide constrained budget |

### 9.2 Mature landmark cross-camera projection

source camera `i`의 landmark를 adjacent target camera `j`에 투영할 때도 depth는 항상 host frame-camera pair의 `rho_l`에서 나온다. current source pixel `u_i`가 host가 아닌데 여기에 `1/rho_l`을 직접 곱하면 잘못된 3D point가 된다.

먼저 host observation으로 landmark world point를 만든다.

\[
\mathbf p_{W,l}
=
\mathbf T_{WB}(t_{h_l,c_l})
\mathbf T_{BC_{c_l}}
\frac{\pi_{c_l}^{-1}(\mathbf u_{h_l,c_l,l})}{\rho_l}.
\]

그 다음 target camera observation time으로 투영한다.

\[
\hat{\mathbf u}_{k,j,l}
=
\pi_j
\left(
\mathbf T_{C_jB}
\mathbf T_{BW}(t_{k,j})
\mathbf p_{W,l}
\right).
\]

source observation `C_i(k)`는 patch appearance, forward-backward cycle, residual validation에 사용한다. 그러나 source가 현재 host가 아니면 source pixel은 depth parameterization의 기준이 아니다.

host가 source camera이고 host exposure time과 source exposure time이 같은 특수한 경우에만 위 식은 다음 상대 camera projection으로 축약된다.

\[
\hat{\mathbf u}_{j}
=
\pi_j
\left(
\mathbf T_{C_jC_i}
\frac{\pi_i^{-1}(\mathbf u_i)}{\rho_l}
\right).
\]

이 구분이 camera time offset / global-shutter mid-exposure compensation을 포함한 최종식이다.

### 9.3 Direct patch refinement

```text
predicted target pixel
-> covariance ellipse 안의 initial search
-> pyramid coarse alignment
-> affine brightness compensation
-> subpixel direct refinement
-> local corner Hessian 확인
```

target camera에서 사전에 descriptor나 대응 corner를 모두 만들 필요가 없다. direct alignment가 target observation pixel을 만든다. 단, target 위치의 gradient가 landmark observation으로 쓰기에 충분한지는 별도 검사한다.

### 9.4 Immature landmark epipolar search

immature track도 host frame-camera pair와 inverse-depth interval을 가진다. 따라서 epipolar curve는 arbitrary current source pixel이 아니라 host bearing에서 생성한다. 최근 source observation은 patch appearance와 interval refinement에 쓰되, rehosting 전까지 `rho`의 기준을 바꾸지 않는다.

\[
\mathcal C_{h\rightarrow j}(\mathbf u_{h,c_h})
=
\left\{
\pi_j
\left(
\mathbf T_{C_jB}
\mathbf T_{BW}(t_{k,j})
\mathbf T_{WB}(t_{h,c_h})
\mathbf T_{BC_{c_h}}
\frac{\pi_{c_h}^{-1}(\mathbf u_{h,c_h})}{\rho}
\right)
\;\middle|\;
\rho\in[\rho_{\min},\rho_{\max}]
\right\}.
\]

fisheye/generalized camera에서는 straight epipolar line이 아니라 curve일 수 있다.

검색:

```text
sample inverse depth, not uniform pixel distance
-> projected curve samples
-> coarse photometric score
-> best basin
-> 1D depth refinement
-> 2D subpixel patch refinement
```

### 9.5 Cross-camera patch warp

camera optical axes가 다르므로 pure translation patch model만으로 충분하지 않을 수 있다.

최종 우선순위:

```text
1. bearing / relative rotation 기반 first-order affine warp
2. mature landmark 주변 local plane이 있으면 plane-induced warp
3. small patch + pyramid로 residual viewpoint difference 흡수
4. excessive view-angle change면 association 거부
```

15~20° overlap은 target camera seam 근처에서 source point가 target camera의 중심 방향에 가까워지는 handoff 구간이므로, 전체 yaw 차이보다 실제 local viewing-angle difference를 검사한다.

### 9.6 Association validation

필수 gate:

```text
positive depth
projection inside valid image
patch Hessian
photometric residual
NCC 또는 zero-mean patch score (보조)
forward-backward camera cycle
Mahalanobis reprojection gate
triangulation angle / depth covariance improvement
temporal confirmation
```

Mahalanobis gate:

\[
\mathbf e^\top\mathbf S_u^{-1}\mathbf e
<\chi^2_{2,\alpha}.
\]

camera cycle:

```text
C_i -> C_j direct alignment
C_j -> C_i reverse projection/alignment
source pixel로 돌아오는지 확인
```

### 9.7 Handoff는 ownership transfer가 아니다

Overlap에서 두 camera observation을 동시에 유지한다.

```text
Landmark l:
  C1(k-2)
  C1(k-1)
  C1(k)
  C2(k)      <- simultaneous overlap observation
  C2(k+1)
```

C2 observation을 만들었다고 C1 observation을 삭제하지 않는다. C1에서 자연히 visibility가 끝나면 C2에서 같은 ID로 이어진다.

동일 timestamp의 두 observation은 같은 body state와 같은 landmark depth를 제약한다.

### 9.8 CCFT가 실패해도 VIO가 유지되는 이유

```text
C1 track와 C2 track가 seam에서 연결되지 않음
-> duplicate landmark가 생길 수 있음
-> 하지만 두 landmark의 residual 모두 same body state에 들어감
-> multi-camera VIO는 계속 tight-coupled
```

따라서 narrow overlap에서 CCFT recall을 높이기 위해 false association을 늘리는 것보다 precision을 우선한다.

---

## 10. Triangulation and Depth Initialization

### 10.1 Primary depth source: temporal multi-view

기본 depth 초기화는 같은 camera의 시간에 따른 parallax다.

```text
C_i(k-m) -> ... -> C_i(k)
body translation / rotation
-> multi-view triangulation
```

장점:

- adjacent camera baseline에 의존하지 않는다.
- overlap이 좁아도 동작한다.
- 모든 camera에서 독립적으로 depth를 만들 수 있다.

### 10.2 Same-time cross-camera triangulation

다음 조건에서만 보조로 사용한다.

```text
camera centers 사이 physical baseline이 충분
bearing intersection angle이 충분
extrinsic covariance가 작음
CCFT association confidence가 높음
```

삼각측량 angle:

\[
\theta_l
=
\cos^{-1}
\left(
\hat{\mathbf b}_i^\top
\mathbf R_{C_iC_j}
\hat{\mathbf b}_j
\right).
\]

`theta_l`이 작으면 depth variance가 크므로 mature로 승격하지 않는다.

### 10.3 Yaw overlap과 stereo baseline은 다르다

```text
카메라가 다른 yaw를 봄:
  common FoV / handoff를 제공

카메라 center가 떨어져 있음:
  triangulation baseline을 제공
```

camera center가 거의 같은데 optical axis만 다르면 same-time metric depth 정보는 거의 없다. overlap이 있다는 사실만으로 stereo가 성립하지 않는다.

### 10.4 Inverse-depth interval update

immature track은 각 관측으로 inverse-depth posterior를 줄인다.

```text
initial broad interval
-> temporal/cross-camera epipolar photometric score
-> robust interval intersection / probabilistic update
-> covariance threshold 통과
-> mature landmark
```

false precision을 막기 위해:

- baseline이 작으면 covariance를 강제로 줄이지 않는다.
- pure rotation 구간에서는 depth가 관측되지 않는다는 것을 유지한다.
- calibration/time uncertainty를 triangulation covariance에 포함한다.

### 10.5 Degenerate motion

```text
pure rotation:
  bearing tracking은 가능
  depth update는 거의 불가능

constant velocity + distant scene:
  scale/depth condition이 약함

planar scene:
  translation 방향에 따라 depth conditioning 저하
```

IMU가 pose prediction을 돕더라도 visual depth observability를 대신 만들지는 않는다.

---

## 11. Rig-Centric Spatial Feature Distribution

카메라별 image grid만 사용하면 동일 yaw sector에 feature가 중복되고, texture가 많은 방향이 budget을 독점할 수 있다.

### 11.1 Body-frame bearing

camera feature bearing을 body frame으로 변환한다.

\[
\mathbf b_B
=
\mathbf R_{BC_i}\pi_i^{-1}(\mathbf u_i).
\]

azimuth / elevation:

\[
\phi=\operatorname{atan2}(b_y,b_x),
\]

\[
\theta
=
\operatorname{atan2}
\left(
 b_z,\sqrt{b_x^2+b_y^2}
\right).
\]

### 11.2 Cylindrical rig grid

```text
azimuth:
  -pi ~ +pi를 N_phi sector로 분할

elevation:
  camera vertical FOV를 N_theta band로 분할
```

feature coverage는 camera image coordinate가 아니라 `(phi,theta)` cell에서 평가한다.

```text
           elevation
               ^
 -180° -------------------------------- +180° azimuth
        C3 C4 C5 C0 C1 C2 (개념적 순환)
```

### 11.3 Overlap seed ownership

새 feature seed를 overlap 양쪽 camera에서 동시에 만들면 duplicate가 증가한다.

새 seed owner를 다음 score로 선택한다.

```text
optical axis에 더 가까움
image border에서 더 멂
gradient / corner response가 큼
blur가 작음
exposure가 정상
해당 azimuth sector의 track이 부족
camera health가 좋음
```

중요한 구분:

```text
new seed:
  overlap에서 owner camera 하나만 생성

existing mature landmark:
  두 camera observation 모두 허용
```

중복 seed 억제와 multi-view observation 유지가 모순되지 않는 이유다.

### 11.4 Per-camera minimum quota

rig-wide budget을 사용해도 각 camera에 최소 quota를 둔다.

이유:

- front camera가 texture-rich하다고 rear camera의 독립 시야를 버리면 안 된다.
- 서로 다른 yaw 방향은 rotation/translation observability에 다른 정보를 준다.
- 특정 camera가 다시 필요해질 때 track pool이 0이면 recovery가 느리다.

단, camera가 dropped/black/saturated면 quota를 억지로 채우지 않는다. quota는 “valid camera에 대한 minimum target”이지 잘못된 feature를 만들라는 명령이 아니다.

---

## 12. Two-Stage Rig-Wide Feature Budget

VILENS-MC의 fixed overall feature budget을 두 단계로 확장한다.

```text
B_track:
  frontend가 유지 / 추적하는 전체 candidate 수

B_opt:
  local BA에 실제로 넣는 landmark 수

관계:
  B_opt <= B_track
```

일반 운용에서는 `B_opt`가 `B_track`보다 작지만, 초기화 직후나 저특징 상황에서는 두 값이 같아질 수 있다.

추가 budget:

```text
B_xcam:
  frame당 CCFT candidate / patch align 수

B_loop:
  rig keyframe당 descriptor 수
```

### 12.1 Tracking budget

frontend selection score 예시:

\[
S_l^{\mathrm{track}}
=
\lambda_g S_{\mathrm{gradient}}
+
\lambda_a S_{\mathrm{age}}
+
\lambda_c S_{\mathrm{coverage}}
+
\lambda_d S_{\mathrm{depth}}
+
\lambda_h S_{\mathrm{health}}
-
\lambda_b S_{\mathrm{blur}}.
\]

hard constraints:

```text
valid camera minimum quota
azimuth sector minimum quota
elevation band minimum quota
cell maximum density
near/far landmark balance
long-track preservation
```

### 12.2 Optimization budget

각 landmark의 robust pose information은 landmark depth가 active variable인지에 따라 다르게 계산한다.

landmark position / inverse depth를 고정된 measurement처럼 취급하면 다음 naive form이 가능해 보인다.

\[
\sum_{o\in\mathcal O_l}
s_l q_o
\mathbf J_o^\top
\boldsymbol\Sigma_o^{-1}
\mathbf J_o.
\]

하지만 최종 backend에서는 `rho_l`도 함께 optimize하므로 이 값을 그대로 pose information으로 쓰면 depth uncertainty를 무시해 과대평가할 수 있다. 따라서 candidate landmark별 raw visual Jacobian을 pose block과 inverse-depth block으로 나누고, measurement covariance와 selection-time weights를 한 번만 적용한다.

\[
\mathbf J_l
=
\left[
\mathbf J_{x,l}
\quad
\mathbf J_{\rho,l}
\right],
\qquad
\mathbf H_l
=
\mathbf J_l^\top
\mathbf W_l
\mathbf J_l.
\]

여기서 `J_l`은 raw Jacobian convention이고,

\[
\mathbf W_l
=
\operatorname{diag}(s_l q_o \eta_o^{\mathrm{pre}})
\otimes
\boldsymbol\Sigma_o^{-1}.
\]

여기서 `s_l`은 landmark-level static confidence, `q_o`는 observation-level tracking confidence이고, `eta_o^{pre}`는 selection 전 history / retry downweight다. `eta_o^{pre}`는 `s_l` 또는 `q_o`를 다시 포함하지 않으며, current optimization에서 아직 계산되지 않은 ATLS auxiliary weight도 아니다.

```text
allowed for eta_pre:
  previous stable commit의 residual / ATLS history 중 s_l, q_o에 이미 반영되지 않은 항
  RetryPolicy blacklist / downweight
  delayed semantic / occupancy weak prior 중 s_l에 이미 반영되지 않은 항

handled outside eta_pre:
  tracking confidence q_o
  static confidence s_l

not allowed for eta_pre:
  이번 B_opt 선택 뒤에야 계산되는 current ATLS auxiliary weight
```

동등하게 whitened Jacobian을 쓰는 구현에서는

\[
\bar{\mathbf J}_o
=
\sqrt{s_l q_o \eta_o^{\mathrm{pre}}}
\mathbf L_o
\mathbf J_o,
\qquad
\mathbf L_o^\top \mathbf L_o=\boldsymbol\Sigma_o^{-1}
\]

로 만들고 `H_l = \bar J_l^\top \bar J_l`만 사용한다. raw `J`에 `W`를 곱하는 convention과 whitened `J` convention을 동시에 적용하지 않는다.

max-logdet selection에는 landmark를 제거한 pose-only Schur information을 사용한다.

\[
\boldsymbol\Lambda_l^{\mathrm{robust}}
=
\mathbf H_{xx,l}
-
\mathbf H_{x\rho,l}
\left(
\mathbf H_{\rho\rho,l}
+
\lambda_\rho
\right)^{-1}
\mathbf H_{\rho x,l}.
\]

Basalt식 nullspace projection / square-root landmark elimination을 사용하는 구현에서는 위 Schur form과 동등한 nullspace-projected pose information을 사용한다. baseline이 작거나 depth가 underconstrained인 landmark는 `H_{\rho\rho}`가 약하므로 false high information을 주지 않도록 damping과 conditioning gate를 둔다.

선택 문제:

\[
\mathcal S^*
=
\arg\max_{|\mathcal S|\le B_{\mathrm{opt}}}
\log\det
\left(
\boldsymbol\Lambda_{\mathrm{prior}}
+
\sum_{l\in\mathcal S}
\boldsymbol\Lambda_l^{\mathrm{robust}}
+
\epsilon\mathbf I
\right).
\]

제약:

```text
camera minimum contribution
azimuth/elevation coverage
near/far depth diversity
mature long-track minimum
cross-camera landmark minimum (조건 충족 시)
```

### 12.3 Why robust information must precede SFS

동적 객체 위 feature는 순간적으로 큰 motion과 큰 Jacobian을 가져 naive max-logdet에서 높은 점수를 받을 수 있다.

잘못된 순서:

```text
정보량만 보고 B_opt 선택
-> dynamic feature가 budget 독점
-> 나중에 제거
-> 이미 좋은 static feature를 버림
```

최종 순서:

```text
tracking quality / static confidence 추정
-> eta_pre 구성(previous-history / RetryPolicy, s_l/q_o와 중복 금지)
-> robust information matrix 구성
-> constrained max-logdet selection
-> selected visual factors에 current ATLS alternating optimization 적용
```

따라서 feature selection과 ATLS 사이에는 순환 의존성이 없다. ATLS는 선택된 factor들의 current residual을 보고 auxiliary weight를 갱신하고, 그 결과는 다음 commit 이후 `s_l`, `q_o`, 또는 `eta_pre` history 중 정확히 하나의 경로로만 돌아온다.

### 12.4 VILENS-MC SFS에서 수정하는 점

VILENS-MC는 informative feature subset을 통해 fixed budget을 유지하는 핵심 방향을 제공한다. 최종 구조는 다음을 강화한다.

```text
VILENS-MC:
  camera별 SFS 적용

최종:
  rig 전체 information matrix에서 선택
  단 camera / spherical sector quota로 collapse 방지
```

camera별 SFS만 하면 각 camera가 비슷한 방향의 redundant constraint를 반복 선택할 수 있다. rig-wide selection은 body pose 전체에 대한 complementary information을 직접 본다.

### 12.5 Approximate solver

정확한 combinatorial selection은 비싸므로:

```text
lazy greedy max-logdet
rank-one Cholesky update
pre-cluster by cylindrical cell
mandatory quota 먼저 삽입
remaining slots를 information gain으로 채움
```

feature selection 비용이 BA 절감보다 커지지 않도록 `B_track` 후보를 먼저 제한한다.

### 12.6 Budget adaptation under load

```text
latency 정상:
  configured B_track / B_opt 유지

frontend overload:
  new detection 감소
  low-value immature track 먼저 제거
  mature long-track 보존

backend overload:
  B_opt 감소
  window iteration 제한
  camera/sector quota는 유지

severe overload:
  CCFT candidate B_xcam 감소
  local VIO temporal tracks는 우선 유지
```

카메라 수가 늘어도 backend 비용은 주로 다음으로 제한된다.

\[
\mathrm{Cost}_{\mathrm{backend}}
\approx
f(N_{\mathrm{window}},B_{\mathrm{opt}},N_{\mathrm{iter}}),
\]

6배 camera 수가 곧 6배 BA 크기를 의미하지 않게 만든다.

---

## 13. Dynamic-Environment Robustness: DynaVINS++ Integration

DynaVINS++의 핵심은 단순 dynamic mask가 아니다.

```text
1. weighted parallax
2. ATLS로 threshold 밖 dynamic/outlier visual residual의 영향을 truncate
3. visual error가 pose와 IMU bias를 잘못 끌고 갔는지 BCC로 검사
4. unstable이면 이전 stable state로 SSR
```

### 13.1 Feature static confidence

landmark static confidence `s_l`은 다음 evidence로 갱신한다.

```text
ATLS residual history
IMU-predicted motion과의 불일치
multi-camera observation consistency
track acceleration / epipolar consistency
optional semantic dynamic probability
optional Occupancy Flow feedback (weak prior only)
```

semantic mask는 hard delete가 아니다.

```text
사람/차량 class인데 실제로 정지:
  geometry가 일관되면 temporarily static일 수 있음

static class인데 움직임:
  geometry가 불일치하면 dynamic
```

최종 판단은 geometry/IMU consistency가 우선이다.

### 13.2 Rig-level weighted angular parallax

camera pixel parallax를 직접 평균하지 않는다. body-frame bearing으로 변환하고 rotation을 보상한다.

\[
\mathbf b_{B,k,l}
=
\mathbf R_{BC_i}
\pi_i^{-1}(\mathbf u_{k,i,l}).
\]

reference keyframe `r`에서 current `k`까지 rotation-compensated angular parallax:

\[
\theta_l
=
\cos^{-1}
\left(
\hat{\mathbf b}_{B,k,l}^\top
\mathbf R_{B_kB_r}
\hat{\mathbf b}_{B,r,l}
\right).
\]

weighted rig parallax:

\[
P_{\mathrm{rig}}
=
\frac{\sum_l s_l\bar q_l\theta_l}
{\sum_l s_l\bar q_l+\epsilon}.
\]

moving object feature가 keyframe selection을 왜곡하는 것을 줄인다.

### 13.3 Adaptive Truncated Least Squares (ATLS)

local visual residual은 일반 Huber를 여러 겹 쌓는 대신 ATLS를 주 robust loss로 사용한다.

개념:

```text
small residual:
  정상 static observation으로 사용

large residual beyond adaptive truncation:
  gradient를 0으로 만들어 BA에서 영향 제거
```

Black-Rangarajan duality 기반 alternating scheme:

```text
1. current state에서 visual residual 계산
2. adaptive truncation range 갱신
3. auxiliary / feature weight 갱신
4. weight 고정 후 square-root BA
5. residual 재계산
6. 2~3회 또는 수렴까지 반복
```

square-root system에 넣을 때:

\[
\begin{aligned}
w_o
&=
s_l q_o a_o^{\mathrm{ATLS}},
\\
\tilde{\mathbf r}_o
&=
\sqrt{w_o}\mathbf r_o,
\\
\tilde{\mathbf J}_o
&=
\sqrt{w_o}\mathbf J_o.
\end{aligned}
\]

`a_o^{ATLS}`는 current residual에서 ATLS alternating step이 계산하는 auxiliary weight다. `s_l`과 `q_o`는 current ATLS auxiliary weight가 아니며, backend row scaling에서 한 번만 곱한다. `eta_pre`는 selection용 history weight이므로 이미 `s_l` 또는 `q_o`로 반영된 evidence를 다시 곱하지 않는다.

```text
backend visual residual:
  ATLS

frontend patch optimization:
  photometric robustification / gate 허용

loop edge:
  switchable constraint 또는 별도 robust kernel
```

`Huber + Geman-McClure + ATLS`를 같은 local visual factor에 중첩하지 않는다. 어느 module이 outlier를 담당하는지 불명확해지고 threshold 해석이 무너진다.

### 13.4 Adaptive truncation range

truncation range는 고정 pixel threshold 하나가 아니다.

반영:

```text
current feature residual distribution
IMU preintegration uncertainty
motion aggressiveness
camera projection covariance
cross-camera 여부
```

빠른 운동에서는 IMU prediction 자체가 더 불확실할 수 있으므로 range를 지나치게 좁혀 static feature까지 버리지 않는다. 반대로 abruptly dynamic object가 나타난 뒤 BCC가 실패하면 range를 더 보수적으로 좁힌다.

### 13.5 Bias Consistency Check (BCC)

문제:

```text
moving object가 image 대부분을 차지
-> 잘못된 visual optimum
-> pose가 dynamic object motion을 따라감
-> optimizer가 IMU residual을 맞추기 위해 b_g / b_a를 비정상적으로 변경
-> 다음 IMU propagation까지 오염
```

BCC는 optimized pose chain과 optimized bias가 IMU model 관점에서 일관적인지 검사한다.

검사 항목:

```text
bias update Mahalanobis norm
preintegration residual 변화
pose increment와 bias-corrected IMU delta의 consistency
bias random-walk prior 대비 jump
```

예:

\[
\delta\mathbf b^\top
\mathbf P_b^{-1}
\delta\mathbf b
<\chi^2_{d,\alpha}.
\]

threshold는 arbitrary bias magnitude보다 estimator covariance / IMU noise model을 기준으로 둔다.

### 13.6 Stable State Recovery (SSR)

optimization 전에 stable snapshot을 저장한다.

```cpp
struct StableEstimatorSnapshot {
    SlidingWindowStates states;
    LandmarkStates landmarks;
    RobustWeights weights;
    SquareRootPrior marginal_prior;
    LinearizationPoints linearization_points;
    PreintegrationCache imu_factors;
    RigLandmarkGraph associations;
};
```

flow:

```text
snapshot stable state
-> ATLS + VI optimization
-> BCC
   pass:
     state commit
     rehosting
     marginalization
     NFR export
   fail:
     snapshot 전체 restore
     failure diagnostics로 retry policy 생성
     raw bundle에서 factor graph 재생성
     suspect association blacklist / provisional merge 제외
     truncation range 축소
     bounded re-optimization
```

retry policy는 restored landmark graph를 사후 수정하는 명령이 아니다. 실패한 tentative graph에서 얻은 diagnostics를 기록한 뒤, stable snapshot 위에 같은 raw measurements를 다시 ingest할 때 어떤 observation을 제외하거나 낮은 weight로 둘지 결정하는 입력이다.

### 13.7 Why BCC must precede marginalization

잘못된 순서:

```text
optimize unstable state
-> marginalization prior 생성
-> BCC fail
-> state vector만 복구
```

이 경우 unstable state와 dynamic residual 정보가 square-root prior에 이미 들어갔다. state 숫자만 되돌려도 prior 오염은 남는다.

최종 invariant:

> **BCC를 통과한 state만 stable commit하고, stable commit된 window만 marginalize하며, 그 이후에만 NFR factor를 export한다.**

### 13.8 Total visual occlusion

모든 `s_l` 또는 effective weight가 거의 0이면 fake visual update를 만들지 않는다.

```text
short duration:
  IMU propagation only
  covariance 증가

recovery:
  새 static tracks 확보
  current propagated pose로 visual alignment
  robust reinitialization / window rebuild

long duration:
  LOST 또는 DEGRADED_IMU_ONLY 상태 명시
```

confidence를 유지한 채 IMU-only를 VIO output처럼 가장하지 않는다.

---

## 14. One Basalt Square-Root Multi-Camera Sliding-Window Backend

### 14.1 State vector

\[
\mathcal X
=
\mathcal X_{\mathrm{nav}}
\cup
\mathcal X_{\mathrm{lm}}.
\]

```text
navigation state per window time:
  T_WB
  v_W
  b_g
  b_a

landmark state:
  host-camera inverse depth rho_l

fixed calibration:
  T_BC_i
  intrinsics_i
  camera time model
```

calibration mode에서만 다음 correction state를 선택적으로 추가한다.

```text
small delta T_BC_i with strong prior
small delta timestamp_to_body_offset_i with strong prior
```

### 14.2 Objective function

\[
\min_{\mathcal X}
\left[
\|\mathbf r_{\mathrm{prior}}\|^2
+
\sum_{k}\|\mathbf r_{\mathrm{imu},k}\|^2_{\Sigma_{I,k}}
+
\sum_{o\in\mathcal O_{\mathrm{selected}}}
s_l q_o
\rho_{\mathrm{ATLS}}
\left(
\|\mathbf r_o^{\mathrm{vis}}\|^2_{\Sigma_o}
\right)
\right].
\]

실제 square-root solve에서는 Section 13.3의 Black-Rangarajan auxiliary form을 사용해 whitened pixel residual `||r||^2_{\Sigma_o}` 기준으로 `a_o^{ATLS}`를 갱신하고, 고정된 inner iteration마다 `w_o=s_l q_o a_o^{ATLS}`로 residual/Jacobian row를 whiten한다. `s_l`과 `q_o`는 ATLS truncation threshold를 바꾸는 항이 아니라 loss 밖의 base confidence weight다. objective 식의 `rho_ATLS`와 row weight `a_o^{ATLS}`를 별도 robust kernel 두 개처럼 중첩하지 않는다.

backend는 photometric bundle adjustment가 아니다.

```text
frontend:
  direct patch photometric alignment로 pixel correspondence 생성

backend:
  feature reprojection residual로 pose / depth 최적화
```

이 구분은 Basalt의 핵심 구조와 일치한다.

### 14.3 Multi-camera reprojection residual

landmark `l`의 host가 frame `h`, camera `c_h`이고 target observation이 frame `k`, camera `c_t`라면, host와 target의 실제 exposure midpoint를 각각 다음처럼 둔다.

\[
t_h = t_{h,c_h},
\qquad
t_o = t_{k,c_t}.
\]

\[
\hat{\mathbf u}_{k,c_t,l}
=
\pi_{c_t}
\left(
\mathbf T_{C_{c_t}B}
\mathbf T_{BW}(t_o)
\mathbf T_{WB}(t_h)
\mathbf T_{BC_{c_h}}
\frac{\pi_{c_h}^{-1}(\mathbf u_{h,c_h,l})}{\rho_l}
\right).
\]

residual:

\[
\mathbf r_{k,c_t,l}^{\mathrm{vis}}
=
\mathbf u_{k,c_t,l}
-
\hat{\mathbf u}_{k,c_t,l}.
\]

`T_WB(t_h)`와 `T_WB(t_o)`는 Section 1.3의 time contract를 따른다. synchronized global-shutter default path에서는 host와 target 양쪽 time jitter가 만든 motion-induced `S_time`이 gate 안에 있으면 해당 active state(s)에 직접 연결한다. 이 gate를 넘거나 partial bundle 처리가 필요하면 미분 가능한 interpolation / propagation operator로 active window state에 연결한다. 어느 경우에도 이 pose들은 별도 independent pose node가 아니며, frozen propagated pose도 아니다.

이 residual의 measurement covariance에는 target timestamp jitter만 넣지 않는다. host inverse-depth parameterization이 `T_WB(t_h)`를 통과하므로 `J_tau=[d u_hat / d t_h, d u_hat / d t_o]`와 joint timestamp-jitter covariance `P_tau`를 사용한다.

동일 landmark가 same bundle의 두 camera에서 관측되면 두 residual이 같은 local body-state trajectory와 같은 inverse-depth parameter에 연결된다. exposure midpoint가 같으면 같은 `T_WB(t)`를 공유하고, 다르면 같은 window state들에 대한 interpolation Jacobian을 공유한다.

### 14.4 Tight coupling의 정확한 의미

다음 두 조건이 핵심이다.

```text
1. 모든 camera visual residual이 동일 body-state window에 들어감
2. visual state와 IMU factor를 하나의 objective에서 joint optimize
```

필요하지 않은 조건:

```text
모든 feature가 cross-camera matched
모든 camera가 같은 landmark를 관측
전형적인 stereo pair
```

per-camera temporal tracking이 독립이어도 backend state가 하나면 tight-coupled다.

### 14.5 Square-root optimization

Basalt의 square-root formulation을 유지한다.

장점:

```text
normal equation H=J^T J를 직접 만들 때보다 수치 안정성 향상
rank deficiency / gauge에 더 안정적
marginalization에서 square-root prior 유지
float precision과 ill-conditioned geometry에 유리
```

ATLS weight는 Jacobian/residual row scaling으로 적용하므로 square-root QR 구조와 양립한다.

### 14.6 Landmark elimination

landmark inverse-depth는 Schur complement 또는 Basalt의 nullspace / square-root elimination 구조로 제거한다.

feature budget의 목적은 단순 residual 개수뿐 아니라:

```text
landmark elimination 비용
fill-in
linear solve 비용
marginal prior 크기
```

를 제한하는 것이다.

### 14.7 Gauge and gravity

VIO world에서:

```text
observable:
  metric scale
  roll / pitch (gravity)
  relative 6DoF motion

unobservable gauge:
  global position
  global yaw
```

global mapping / loop closure는 gauge를 임의 anchor로 고정한다. BCC나 NFR에서 unobservable direction을 absolute truth처럼 검사하지 않는다.

### 14.8 First-estimate / linearization consistency

marginalized variables의 Jacobian linearization point를 임의로 갱신하면 prior consistency가 깨진다.

최종 원칙:

```text
active factors:
  current estimate에서 relinearize

marginalization prior:
  저장된 linearization point / perturbation convention 유지

SSR:
  state만 아니라 linearization point와 prior를 함께 restore
```

---

## 15. Exact `SE_2(3)` IMU Preintegration and Propagation

### 15.1 Role in the final architecture

MAVIS의 exact preintegration은 Basalt backend 위에 두 번째 IMU factor를 추가하는 것이 아니다.

```text
잘못된 구조:
  Basalt preintegration factor
  + MAVIS exact preintegration factor
  -> 같은 IMU measurement를 이중 사용

최종 구조:
  Basalt state / solver / marginalization
  + MAVIS식 exact SE_2(3) preintegration implementation으로 factor 교체
```

### 15.2 Why exact preintegration

일반적인 discrete approximation은 측정 interval 동안 회전과 translation/velocity coupling을 근사한다. 빠른 회전과 긴 적분 구간에서는 position·velocity delta 오차가 커질 수 있다.

exact `SE_2(3)` formulation은 pose/velocity를 확장 group에서 일관되게 적분해:

```text
aggressive rotation
camera frame drop으로 integration interval 증가
low camera rate
```

에서 prediction과 frontend initial guess를 개선한다.

### 15.3 Required interface

```cpp
struct PreintegratedImuFactor {
    TimeNs t0;
    TimeNs t1;
    SE23d delta;
    Matrix15 covariance;
    Matrix bias_jacobian;
    Vector3 linearization_bg;
    Vector3 linearization_ba;
};
```

최소 제공:

```text
delta rotation
delta velocity
delta position
covariance
bias Jacobians
measurement duration
validity / saturation flag
```

### 15.4 Convention matching

MAVIS 수식을 Basalt에 이식할 때 반드시 다시 맞출 것:

```text
left vs right perturbation
T_WB vs T_BW
body vs world velocity
bias subtraction sign
gravity sign
state ordering
covariance ordering
Jacobian block ordering
```

논문 수식을 복사하고 compile이 되는 것이 검증이 아니다. finite-difference Jacobian과 Monte-Carlo covariance test가 필요하다.

### 15.5 BCC consistency

BCC는 실제 backend가 사용하는 preintegration residual과 동일한 model로 계산한다.

```text
금지:
  backend = exact SE_2(3)
  BCC = VINS-Mono approximate preintegration formula
```

서로 다른 model을 쓰면 정상 state를 inconsistent로 판단하거나 반대로 오류를 놓칠 수 있다.

### 15.6 High-rate propagation

optimization은 camera bundle rate에서 수행하고 output은 IMU rate로 propagate한다.

```text
last optimized state
+ incoming IMU
-> propagated T_WB / v / covariance
-> high-rate odometry output
```

optimized state가 갱신되면 propagation base를 재설정한다. local output의 discontinuity를 줄이기 위해 state correction을 일관된 timestamp에서 적용한다.

### 15.7 IMU saturation and discontinuity

```text
saturation:
  measurement flag
  covariance inflation
  affected interval의 BCC threshold 조정

missing IMU:
  interpolation을 가장하지 않음
  integration invalid / degraded 상태

out-of-order IMU:
  bounded reorder buffer 또는 drop diagnostic
```

visual factor가 IMU 결손을 완전히 대신한다고 가정하지 않는다.

---

## 16. Stable Marginalization, Rehosting and Non-Linear Factor Recovery

### 16.1 Commit order

최종 transaction order:

```text
A. stable snapshot 생성
B. new bundle factors 구성
C. ATLS alternating optimization
D. BCC
E1. pass:
      commit optimized state
      landmark rehosting
      marginalization candidate 결정
      square-root marginalization
      NFR extraction/export
E2. fail:
      snapshot restore
      diagnostics로 retry policy 생성
      raw bundle에서 tentative factors 재생성
      suspect associations 제외 / weight 감소
      truncation range 축소
      bounded re-optimize
```

### 16.2 Rehost before marginalization

host frame을 제거하기 전에 surviving observation으로 rehost한다.

```text
1. marginalization될 host landmark 찾기
2. remaining window의 candidate host score 계산
3. inverse-depth reparameterization
4. residual/Jacobian consistency 검증
5. old host state marginalize
```

### 16.3 Square-root prior

marginalization 결과는 square-root form으로 저장한다.

```text
R_prior * delta_x - r_prior
```

SSR snapshot에는 다음이 모두 있어야 한다.

```text
R_prior
r_prior
variable ordering
linearization points
nullspace/gauge handling metadata
```

### 16.4 NFR export

Basalt NFR의 목적:

```text
high-rate local VIO가 축적한 visual-inertial information
-> marginalized keyframe pose 사이의 sparse nonlinear factors로 복구
-> global mapper에 전달
```

NFR는 단순 relative pose 값만 넘기는 것보다 local VIO의 information/covariance를 더 잘 보존한다.

global layer가 받는 factor:

```text
relative pose constraints
roll/pitch / gravity constraints
factor information matrices
keyframe pose linearization metadata
```

### 16.5 Stable NFR only

동적 객체로 오염된 window에서 factor를 export하면 global map도 오염된다.

최종 invariant:

```text
ATLS complete
-> BCC pass
-> stable commit
-> marginalization
-> NFR export
```

### 16.6 2020 multi-camera Basalt-SLAM thesis에서 사용하는 부분

채택:

- 빠른 optical-flow VIO와 저주기 descriptor mapper 분리.
- VIO marginalization information을 NFR factor로 mapper에 전달.
- loop closure와 global optimization을 local tracker와 다른 thread로 수행.

채택하지 않음:

```text
camera마다 Tracker + Mapper + LoopCloser를 독립 실행
-> 여러 map을 loop closure로 merge
```

우리 rig는 하나의 rigid body이므로 pose state와 local map은 하나다.

---

## 17. Rig-Level Keyframe Selection

keyframe은 camera별 image가 아니라 one rig pose node다.

```text
RigKeyframe k:
  one body pose T_WB_k
  up to 6 camera images / observations
  selected descriptor views
  local landmarks
  NFR connection metadata
```

### 17.1 Keyframe score

다음 신호를 결합한다.

\[
S_{\mathrm{KF}}
=
\lambda_p S_{\mathrm{weighted\;parallax}}
+
\lambda_I S_{\mathrm{information\;drop}}
+
\lambda_n S_{\mathrm{new\;landmarks}}
+
\lambda_h S_{\mathrm{tracking\;health}}
+
\lambda_t S_{\mathrm{max\;interval}}.
\]

### 17.2 Information criterion

body pose covariance/information:

\[
E_k=\log\det(\boldsymbol\Lambda_{\mathrm{pose},k}+\epsilon\mathbf I).
\]

이전 keyframe 대비 information이 충분히 감소하거나 viewpoint가 충분히 변할 때 keyframe을 생성한다.

### 17.3 Required conditions

```text
minimum frame/time interval
또는 emergency tracking-health drop

그리고 다음 중 일부:
  static-weighted rig angular parallax
  body translation / rotation
  신규 triangulation 가능한 track
  information reduction
  maximum keyframe interval
```

### 17.4 One weak camera does not force a keyframe

잘못된 규칙:

```text
C3 feature 수가 적음
-> 6-camera bundle을 매 frame keyframe
```

최종 규칙:

```text
rig 전체 pose information과 static-weighted geometry가 기준
```

단, 특정 azimuth가 장시간 map에 저장되지 않으면 loop closure coverage가 약해질 수 있으므로 global mapper는 descriptor-view coverage 정책을 별도로 둔다.

### 17.5 Keyframe image storage

모든 rig keyframe에서 6개 full-resolution image를 영구 저장할 필요는 없다.

```text
local mapping:
  required patches / observations

global place recognition:
  selected DescriptorView records

DescriptorView minimum fields:
  camera_id
  exposure_time
  keypoint pixel coordinates
  descriptors
  static confidence
  camera calibration/extrinsic version
  rig keyframe pose ID
  optional compressed image or verification patch

Verification-capable DescriptorView additionally requires:
  associated local/global landmark ID
  landmark 3D position or map point reference
  landmark covariance / source map version

coverage policy:
  front/rear/left/right azimuth가 장기적으로 균형 있게 DB에 들어가도록 선택
```

pose node는 하나지만 descriptor views는 여러 개일 수 있다. full-resolution image를 모두 저장할 필요는 없지만, descriptor vector만 저장해서는 generalized-camera PnP/RANSAC, loop reprojection 검증, global BA에 필요한 2D-3D provenance를 복구할 수 없다. landmark association이 없는 descriptor는 retrieval에는 쓸 수 있지만 2D-3D loop factor 생성에는 직접 쓰지 않는다.

---

## 18. Initialization and Reinitialization

### 18.1 Final initialization path

```text
1. IMU stationary or low-motion initialization
   -> gravity
   -> initial gyro bias
   -> accel bias prior

2. six-camera temporal feature tracks
   -> camera별 relative visual constraints

3. one shared body-motion initialization
   -> multi-camera generalized geometry

4. visual-inertial alignment
   -> metric scale
   -> velocity
   -> gravity refinement
   -> biases

5. short joint multi-camera VI optimization

6. initialization consistency / observability check

7. RUNNING state
```

### 18.2 No stereo dependency

MAVIS의 first-frame stereo map initialization은 그대로 사용하지 않는다.

이유:

- 우리의 adjacent pair는 parallel stereo가 아니다.
- overlap이 15~20°로 좁다.
- physical baseline이 pair마다 다를 수 있다.
- 모든 pair가 first frame에서 충분한 triangulation을 제공한다는 보장이 없다.

same-time cross-camera depth는 조건이 좋을 때 initialization을 가속하는 보조 constraint다.

### 18.3 Stationary and dynamic initialization

기본:

```text
stationary IMU initialization
```

fallback:

```text
sufficient excitation가 있는 dynamic initialization
```

검사:

```text
gravity magnitude
bias plausibility
visual parallax
scale conditioning
Hessian singular values
multi-camera pose consistency
```

### 18.4 Initialization failure

```text
insufficient parallax:
  track 유지, initialization 연기

all cameras low texture:
  state 확정 금지

IMU not stationary and insufficient excitation:
  bias/gravity confidence 낮음 -> wait

cross-camera contradiction:
  suspect edge disable, temporal tracks로 계속 시도
```

### 18.5 Reinitialization

full reset 전에 다음 순서로 recovery한다.

```text
1. last stable snapshot + IMU propagation
2. active local map reprojection
3. new static features 확보
4. robust pose-only alignment
5. short window rebuild
6. BCC pass 후 RUNNING
```

재사용할 stable landmark가 없고 uncertainty가 임계치를 넘으면 full reinitialization을 명시한다.

---

## 19. Keyframe-Only Descriptor Loop Closure and Global SLAM

### 19.1 Descriptor role separation

```text
Local VIO at camera rate:
  descriptor 없음
  patch tracking + geometry

Global SLAM at keyframe rate:
  descriptor 사용
  place recognition / relocalization / loop closure
```

“descriptor-free VIO”와 “descriptor-based loop closure”는 주기와 목적이 다른 두 layer이므로 모순이 아니다.

### 19.2 Descriptor extraction policy

```text
input:
  stable rig keyframe only

camera selection:
  texture quality
  azimuth coverage
  static landmark ratio
  expected place-recognition value

execution:
  asynchronous low-priority thread
  local VIO critical path와 분리
```

매 frame `6 cameras x descriptors`를 계산하지 않는다.

### 19.3 Intra-camera and inter-camera loops

```text
intra-camera:
  과거 C0 view <-> 현재 C0 view

inter-camera:
  과거 C0 view <-> 현재 C3 view
  U-turn / reverse traversal에서 중요
```

MAVIS의 multi-camera loop closure 개념을 채택한다.

### 19.4 Place retrieval and verification

candidate retrieval:

```text
DBoW2/DBoW3 + ORB
또는 learned global descriptor
```

retrieval 결과만으로 loop를 확정하지 않는다.

geometric verification:

```text
verification-capable DescriptorView가 충분:
  2D-3D generalized-camera PnP / RANSAC
  또는 camera-specific PnP 후 rig transform consistency

landmark association이 부족:
  generalized 2D-2D epipolar / relative-pose verification
  또는 camera-specific essential / homography verification
  이후 local map projection으로 2D-3D support를 추가 확인

static-confidence feature 우선
pose / scale / gravity consistency
```

global BA에 reprojection factor로 넣는 loop measurement는 2D-3D provenance가 있는 match만 사용한다. 2D-2D verification만 통과한 후보는 place hypothesis로 유지하거나, metric scale이 관측된 경우에만 relative-pose loop로 승격한다. camera-specific essential / homography만으로 얻은 constraint는 일반적으로 translation scale이 없으므로 full `SE(3)` loop factor가 아니라 rotation-only, translation-direction, up-to-scale, 또는 planar-hypothesis factor로 제한한다. map point association이 확인되기 전에는 landmark reprojection factor로 승격하지 않는다.

### 19.5 Dynamic and temporarily static loop features

loop DB에 넣는 landmark/descriptor는:

```text
high static confidence
long-term multi-view consistency
semantic dynamic prior가 높지 않음
```

temporarily static 차량처럼 나중에 이동할 수 있는 object는 global map anchor로 신뢰하지 않는다.

### 19.6 Global optimization

권장 최종 구조:

```text
NFR factors
+ loop reprojection / metric relative-pose factors
+ optional map priors
-> global bundle adjustment or pose graph
```

NFR global BA는 VIO의 gravity/relative motion information을 보존한다.

factor provenance를 반드시 저장한다.

```text
local_vio_nfr:
  local sliding-window marginalization에서 복구된 factor

loop_detection:
  descriptor retrieval + geometric verification에서 새로 얻은 long-baseline factor

map_prior:
  외부 map / delayed occupancy / survey prior
```

같은 local VIO marginalization interval에서 나온 relative pose edge를 NFR와 별도 pose-graph edge로 동시에 추가하지 않는다. NFR는 local VIO information을 대표하고, loop factor는 독립적인 place-recognition measurement만 대표한다. global optimizer는 factor source interval과 provenance ID가 겹치면 duplicate factor로 reject하거나 하나만 유지한다.

2D-2D-only loop factor는 covariance와 gauge를 factor type에 맞춰 제한한다. scale covariance가 finite로 검증되지 않은 후보를 metric relative pose edge로 넣지 않는다.

### 19.7 Local smoothness and global correction

제어용 local pose:

```text
odom -> base_link
continuous
```

global loop correction:

```text
map -> odom
slow / filtered update or consistent transform update
```

local sliding window의 과거 state를 무리하게 즉시 점프시키지 않는다. global map correction과 low-latency odometry의 역할을 분리한다.

### 19.8 Relocalization

VIO lost 후:

```text
keyframe DB retrieval
-> multi-camera geometric verification
-> global pose hypothesis
-> current IMU gravity / orientation consistency
-> local window rebuild
```

relocalization descriptor가 local optical-flow frontend를 대체하지 않는다.

---

## 20. Runtime Concurrency and Scheduling

### 20.1 Thread graph

```text
Thread A: IMU ingest + compensation
Thread B: high-rate propagation
Thread C0~C5 or task pool: image pyramid / gradient / temporal tracking
Thread D: FrameBundle assembly / deadlines
Thread E: local-map projection + CCFT
Thread F: landmark management + feature budget
Thread G: local VI optimization + BCC/SSR + marginalization
Thread H: global descriptor / loop closure / NFR mapping
Thread I: logging / health / occupancy bridge
```

물리 thread를 9개로 고정할 필요는 없다. task graph의 dependency를 위처럼 분리하고 CPU core/GPU에 매핑한다.

### 20.2 Critical path

local output critical path:

```text
bundle ready
-> temporal tracking
-> selected CCFT
-> landmark/budget update
-> VI optimize
-> BCC
-> publish optimized state
```

critical path에서 제외:

```text
descriptor extraction
BoW retrieval
global BA
map serialization
heavy visualization
```

### 20.3 Parallel frontend

camera별 temporal tracking은 서로 다른 image memory를 쓰므로 병렬화하기 쉽다.

```text
per-camera task output:
  immutable track candidates

merge stage:
  one landmark manager가 global IDs / CCFT / budget 결정
```

여러 thread가 같은 landmark graph를 직접 수정하지 않는다.

### 20.4 GPU/VPI/CUDA use

GPU 후보:

```text
pyramid / gradient
FAST/AGAST
batch patch sampling
coarse optical flow / CCFT ROI alignment
```

CPU 후보:

```text
landmark graph
triangulation
feature selection
IMU preintegration
square-root optimization
marginalization
```

MCVIO가 보인 것처럼 embedded GPU frontend는 CPU resource와 latency를 줄일 수 있다. 다만 Basalt의 patch semantics와 numerical output을 보존하는지 검증한 뒤 교체한다.

### 20.5 Queue policy

```text
IMU queue:
  loss 최소화, timestamp order 보장

camera queue:
  bounded
  stale frame drop
  newest usable frame 우선

optimizer queue:
  bundle backlog를 무한 축적하지 않음
  overload 시 old unprocessed bundle을 건너뛰고 IMU interval은 정확히 통합
```

### 20.6 Determinism

```text
camera task completion order와 무관하게
  observation sort order
  landmark ID assignment
  feature selection tie-break
  linear system variable ordering
```

을 고정해 replay 결과를 재현 가능하게 한다.

---

## 21. Health Monitoring and Degraded Operation

### 21.1 Health levels

```text
NOMINAL:
  충분한 static visual information, BCC pass

DEGRADED_CAMERA:
  일부 camera/edge failure, remaining cameras로 VIO 유지

DEGRADED_VISUAL:
  rig 전체 visual information 부족, IMU 비중 증가

RECOVERING:
  stable snapshot/local map에서 window rebuild

IMU_ONLY_SHORT:
  짧은 visual outage, covariance 빠르게 증가

LOST:
  local state 신뢰 불가, relocalization/reinitialization 필요
```

### 21.2 Per-camera metrics

```text
active temporal tracks
mature static landmarks
median patch residual
forward-backward pass ratio
image entropy / saturation ratio
blur score
frame drop rate
timestamp lateness
```

### 21.3 Per-edge CCFT metrics

```text
candidate count
alignment success rate
cycle pass rate
median Mahalanobis residual
provisional->confirmed ratio
false-merge rollback count
```

### 21.4 Rig metrics

```text
B_track / B_opt fill ratio
azimuth/elevation coverage
pose information logdet
static-weight ratio
ATLS rejected ratio
bias update norm
BCC pass/fail
propagated covariance
optimization latency
```

### 21.5 Failure-specific behavior

#### One camera missing

```text
camera residual 제외
neighbor CCFT edges disable
remaining cameras + IMU로 계속
```

#### One overlap edge calibration suspected

```text
해당 CCFT edge만 disable
두 camera temporal tracking은 유지
```

#### One direction textureless

```text
그 camera quota를 잘못된 corner로 채우지 않음
다른 camera information이 shared body pose를 유지
```

#### Sudden exposure transition

```text
tracking q_o 감소
exposure affine initialization 갱신
새 seed detector branch 보조
다른 camera redundancy 활용
```

#### Severe motion blur

```text
IMU initial guess
coarse pyramid
blurred observation weight 감소
short IMU-only propagation 허용
```

#### Dynamic object dominates image

```text
s_l / ATLS weight 감소
rig의 다른 yaw camera static tracks 우선
BCC fail 시 SSR
```

#### All cameras occluded / dark

```text
IMU_ONLY_SHORT
covariance 증가
visual confidence 0으로 표시
장기 지속 시 LOST
```

### 21.6 No silent failure

output에 health와 covariance를 함께 내보낸다.

```cpp
struct VioOutput {
    NavState state;
    Matrix15 covariance;
    VioHealth health;
    uint32_t active_camera_mask;
    float static_visual_ratio;
    bool globally_localized;
};
```

pose 숫자만 계속 publish하면서 estimator가 IMU-only/lost 상태임을 숨기지 않는다.

---

## 22. Multi-Layer Outlier Rejection

Descriptor를 쓰지 않는다는 것은 검증을 생략한다는 뜻이 아니다. outlier는 여러 단계에서 역할을 나눠 제거한다.

### 22.1 Frontend patch gates

```text
patch texture / Hessian
image boundary
coarse-to-fine convergence
photometric residual
forward-backward check
exposure/gain plausibility
```

### 22.2 Geometry gates

```text
projection covariance / Mahalanobis gate
positive depth
camera visibility
triangulation angle
inverse-depth posterior consistency
generalized-camera motion consistency
```

### 22.3 Cross-camera gates

```text
adjacent edge only
depth-dependent overlap
camera cycle consistency
view-angle limit
provisional association confirmation
edge calibration health
```

### 22.4 Backend robustification

```text
landmark static confidence s_l
observation confidence q_o
ATLS truncation
BCC / SSR
```

### 22.5 Loop closure gates

```text
BoW/global descriptor retrieval
static-feature matching
generalized-camera geometric verification
gravity consistency
switchable/robust loop factor
global optimization residual check
```

### 22.6 Why robust kernel alone is insufficient

잘못된 cross-camera association을 모두 backend robust loss에 맡기면:

- inverse depth가 잘못 초기화될 수 있다.
- global landmark graph가 오염된다.
- dynamic feature가 SFS budget을 차지한다.
- bias가 잘못 변하기 전까지 residual이 충분히 작아질 수 있다.

따라서 frontend precision gate와 backend ATLS를 함께 사용하되, 같은 residual에 여러 robust kernel을 무분별하게 중첩하지 않는다.

### 22.7 Generalized-camera motion check

rig 전체 observation은 camera center가 다른 generalized camera ray다.

각 observation은 Plücker line 또는 `(ray origin, ray direction)`로 표현할 수 있다.

```text
ray origin:
  camera center in body frame

ray direction:
  unprojected bearing transformed to body frame
```

초기화·recovery·loop verification에서 generalized essential/PnP geometry를 사용하면 camera별 결과를 따로 풀어 평균하는 것보다 rig geometry를 정확히 반영한다.

---

## 23. Recommended C++ Data Model and Interfaces

### 23.1 Calibration

```cpp
struct CameraCalibration {
    CameraId id;
    CameraModel model;
    SE3d T_BC;
    bool global_shutter;
    double timestamp_to_body_offset_sec;       // camera timestamp clock -> IMU/body clock
    double nominal_timestamp_to_exposure_midpoint_sec; // fallback if metadata does not provide it
    double rolling_shutter_readout_sec;  // final global-shutter target: 0; not transport latency
    PhotometricCalibration photometric;
};

struct RigCalibration {
    std::array<CameraCalibration, 6> cameras;
    ImuCalibration imu;
    std::array<CameraOverlapEdge, 6> overlap_edges;
};
```

### 23.2 Tracking result

```cpp
struct TrackCandidate {
    LandmarkId landmark_id;
    FeatureObservation observation;
    TrackingSource source;  // temporal, projection, cross-camera
    float quality;
    bool provisional;
};
```

### 23.3 Landmark manager

```cpp
class RigLandmarkManager {
public:
    void ingestTemporalTracks(const std::vector<TrackCandidate>&);
    void ingestCrossCameraTracks(const std::vector<TrackCandidate>&);
    void updateTriangulation(const SlidingWindowStates&);
    void confirmOrRollbackAssociations();
    void rehostBeforeMarginalization(FrameId frame_id);
    LandmarkSnapshot snapshot() const;
    void restore(const LandmarkSnapshot&);
};
```

### 23.4 Budgeter

```cpp
struct FeatureBudgetConfig {
    int max_tracking_features;
    int max_optimization_landmarks;
    int max_cross_camera_candidates;
    std::array<int, 6> min_camera_quota;
    int azimuth_bins;
    int elevation_bins;
};

class RigFeatureBudgeter {
public:
    TrackingSelection selectTrackingSet(...);
    OptimizationSelection selectOptimizationSet(...);
};
```

### 23.5 Robust state manager

```cpp
class RobustStateManager {
public:
    void updateObservationConfidence(...);
    void updateStaticConfidence(...);
    void updateAtlsWeights(...);
    BiasConsistencyResult checkBiasConsistency(...);
};

class StableStateManager {
public:
    StableEstimatorSnapshot capture(...) const;
    void restore(const StableEstimatorSnapshot&, ...);
    void commit(...);
};
```

### 23.6 Estimator transaction

```cpp
EstimationResult processBundle(const FrameBundle& bundle) {
    auto snapshot = stable_state.capture(...);
    RetryPolicy retry_policy;
    TentativeGraph tentative_graph;
    EstimationResult result = EstimationResult::unstable();
    bool bcc_passed = false;

    for (int attempt = 0; attempt < max_reoptimization_attempts; ++attempt) {
        stable_state.restore(snapshot, ...);
        tentative_graph.clear();

        auto tracks = frontend.trackTemporal(bundle, retry_policy);
        auto projected = projector.projectLocalMap(bundle, retry_policy);
        auto xcam = ccft.trackAdjacentCameras(bundle, retry_policy);
        tentative_graph = landmarks.buildTentativeGraph(
            tracks, projected, xcam, retry_policy);
        robust.updateConfidence(tentative_graph, retry_policy);

        auto selected = budgeter.selectOptimizationSet(tentative_graph, ...);
        result = estimator.optimizeATLS(selected, ...);

        auto bcc = robust.checkBiasConsistency(result);
        if (bcc.passed) {
            bcc_passed = true;
            break;
        }

        retry_policy = RetryPolicy::fromFailureDiagnostics(
            bcc, result, tentative_graph);
    }

    if (!bcc_passed || !result.stable) {
        stable_state.restore(snapshot, ...);
        return enterRecoveryMode(...);
    }

    auto commit_txn = stable_state.beginStableCommit(snapshot);
    commit_txn.stageEstimatorResult(result);
    commit_txn.stageTentativeLandmarkGraph(tentative_graph);
    commit_txn.stageRehosting(...);
    commit_txn.stageMarginalization(...);
    commit_txn.stageNfrExport(...);

    if (!commit_txn.commitAtomically()) {
        stable_state.restore(snapshot, ...);
        return enterRecoveryMode(...);
    }

    return result;
}
```

retry에서는 restored graph를 직접 mutate하지 않는다. 실패 diagnostics로 `RetryPolicy`를 만들고, stable snapshot에서 raw bundle을 다시 처리해 tentative factors를 재생성한다. 실제 구현에서는 BCC 실패 시 무한 reoptimization을 막는 bounded retry가 필요하다.

stable commit transaction은 state, landmark graph, rehosting result, marginalization prior, NFR export source commit ID를 같은 commit ID로 publish한다. 중간 단계가 실패하면 snapshot으로 돌아가고 global mapper에는 어떤 NFR도 노출하지 않는다.

### 23.7 Output contract

```cpp
struct SparseLandmarkOutput {
    LandmarkId id;
    Vec3 position_world;
    Mat3 covariance_world;
    float static_confidence;
    std::vector<CameraObservationRef> recent_observations;
};

struct OccupancyBridgeOutput {
    TimeNs timestamp;
    SE3d T_WB;
    Matrix6 pose_covariance;
    Vec3 gravity_W;
    std::vector<SparseLandmarkOutput> landmarks;
    std::vector<RejectedTrackHint> dynamic_hints;
    std::vector<VisibilityRay> free_space_rays;
};
```

---

## 24. Configuration, Assertions and Architectural Invariants

### 24.1 Example configuration structure

```yaml
rig:
  num_cameras: 6
  topology: ring
  camera_order: [front, front_left, rear_left, rear, rear_right, front_right]
  global_shutter: true

time_model:
  fixed_timestamp_to_body_offsets: true
  timestamp_to_exposure_midpoint: metadata_or_per_camera_nominal
  optimize_time_offsets: false
  rolling_shutter_readout_sec: 0
  observation_time_mode: node_or_imu_interpolated

frontend:
  pyramid_levels: <calibrated>
  patch_size: <calibrated>
  temporal_tracking: direct_patch
  use_descriptors: false
  tracking_budget: <profiled>
  cross_camera_budget: <profiled>

cross_camera:
  edges: [[0,1], [1,2], [2,3], [3,4], [4,5], [5,0]]
  mature_projection: true
  immature_epipolar_search: true
  provisional_confirmation: true

backend:
  optimizer: basalt_square_root
  window_size: <profiled>
  optimization_budget: <profiled>
  imu_preintegration: exact_se23
  robust_loss: atls

stability:
  bias_consistency_check: true
  stable_state_recovery: true
  max_reoptimization_attempts: <bounded>

mapping:
  nfr: true
  keyframe_only_descriptors: true
  inter_camera_loop: true

occupancy_bridge:
  pose_gravity: true
  landmarks_covariance: true
  rejected_track_hints: true
  free_space_rays: true
```

### 24.2 Non-negotiable invariants

```text
I1. timestamp당 body navigation state는 하나다.
I2. camera pose는 body pose + calibrated extrinsic으로 계산한다.
I3. local VIO에 descriptor matching을 넣지 않는다.
I4. cross-camera search는 overlap graph edge에만 제한한다.
I5. overlap mask는 association proof가 아니라 candidate gate다.
I6. mature landmark는 feature별 depth/covariance를 사용한다.
I7. global landmark ID는 camera seam에서 유지된다.
I8. B_opt는 rig 전체에서 bounded된다.
I9. camera/azimuth coverage quota 없이 max-logdet만 쓰지 않는다.
I10. dynamic confidence를 반영한 뒤 feature selection을 수행한다.
I11. local visual backend robust loss는 ATLS가 주 책임이다.
I12. BCC pass 전 marginalization 금지.
I13. BCC pass 전 NFR export 금지.
I14. SSR은 state뿐 아니라 prior/linearization/landmarks/weights를 복구한다.
I15. exact IMU factor와 기존 approximate factor를 중복 추가하지 않는다.
I16. direct tracking 영상에 per-frame nonlinear histogram equalization을 적용하지 않는다.
I17. 한 camera drop이 전체 bundle drop을 뜻하지 않는다.
I18. rig keyframe pose node는 하나다.
I19. global loop correction은 map->odom에 반영하고 local odom continuity를 지킨다.
I20. current-frame Occupancy output을 독립 measurement처럼 즉시 VIO에 재투입하지 않는다.
I21. global shutter는 row-time residual이 없다는 뜻이며 transport delay 0을 뜻하지 않는다.
I22. fixed time-offset mode에서는 visual residual이 Delta t_i^clk 또는 midpoint correction을 optimize하지 않는다.
I23. 같은 local VIO marginalization information을 NFR와 별도 relative edge로 중복 추가하지 않는다.
I24. stable commit은 state/landmark/rehost/marginalization/NFR를 하나의 atomic transaction으로 publish한다.
```

### 24.3 Runtime assertions

```text
all camera IDs unique
all overlap edges valid ring neighbors
T_BC_i finite and proper rotation
camera timestamps monotonic
IMU interval covers visual factor interval
observation time either attaches to an active state or lies inside interpolation interval
fixed time-offset mode has no Delta t_i^clk or midpoint-correction optimization block
S_time contains observation-local jitter, not repeated fixed offset calibration prior
global_shutter implies rolling_shutter_readout_sec == 0
landmark host exists before optimization
selected observation camera frame exists
no duplicate observation (same landmark, frame, camera)
no marginalization while transaction unstable
stable commit transaction publishes one commit ID for state, landmarks, prior, and NFR
NFR export source commit ID == latest stable commit ID
global factor provenance has no duplicate local_vio interval
```

### 24.4 Parameter tuning rule

고정 숫자를 논문에서 그대로 복사하지 않는다.

```text
patch threshold:
  camera noise / exposure / resolution에 맞춤

feature budget:
  target hardware profile로 결정

ATLS range:
  residual distribution + IMU uncertainty로 adaptive

keyframe threshold:
  angular parallax / information metric으로 scale-independent하게 설정
```

논문 장치의 4 camera·36° overlap 설정을 6 camera·15~20° overlap에 그대로 적용하지 않는다.

---

## 25. Runtime and Compute Budget

### 25.1 Target rates

최종 목표는 특정 camera rate 하나에 hard-code하지 않고 입력 rate를 따라간다.

```text
optimized local VIO:
  camera FrameBundle rate
  deployment target >= 20 Hz

propagated odometry:
  IMU rate 또는 control-required rate

global descriptor / loop:
  keyframe rate, asynchronous
```

latency invariant:

```text
p95 local VIO critical-path latency
< one camera bundle period
```

backlog가 쌓여 오래된 pose를 정확히 계산하는 것보다 최신 pose를 bounded latency로 계산하는 것이 로봇 제어에 더 중요하다.

### 25.2 Compute scaling

naive:

\[
N_{\mathrm{features}}
\propto 6N_{\mathrm{per-camera}}.
\]

최종:

\[
N_{\mathrm{frontend}}
\le B_{\mathrm{track}},
\qquad
N_{\mathrm{backend}}
\le B_{\mathrm{opt}}.
\]

camera 수가 늘면서 필연적으로 증가하는 비용:

```text
image read / pyramid / gradient
camera별 최소 temporal tracking
```

bounded되는 비용:

```text
BA landmarks
CCFT candidates
loop descriptors
keyframe map observations
```

### 25.3 Compute priority

높은 우선순위:

```text
IMU ingest / propagation
temporal tracking
static mature landmarks
local VI optimization
BCC
```

중간:

```text
CCFT
new feature detection
triangulation
```

낮은 우선순위:

```text
descriptor extraction
loop retrieval
global BA
visualization
```

### 25.4 Load shedding order

```text
1. visualization / debug image 감소
2. loop descriptor frequency 감소
3. CCFT low-value candidate 감소
4. new immature seed 감소
5. B_opt 감소 (coverage quota 유지)
6. B_track 감소 (mature long track 보존)
```

IMU propagation과 stable-state checks를 먼저 끄지 않는다.

### 25.5 Performance telemetry

매 frame 기록:

```text
pyramid_ms
tracking_ms
ccft_ms
triangulation_ms
selection_ms
optimization_ms
bcc_ms
marginalization_ms
active_tracks
selected_landmarks
memory_usage
queue_depth
```

CPU total percentage만 보지 않고 camera별 / module별 latency를 분해한다.

---

## 26. Occupancy Network Integration

VIO와 Occupancy Network는 처음에는 명확한 one-way interface로 연결하고, 이후 통계적 상관을 통제할 수 있을 때 tight coupling을 추가한다.

### 26.1 VIO -> Occupancy mandatory outputs

```text
1. metric ego pose T_WB(t)
2. pose covariance
3. gravity direction
4. velocity / angular motion
5. camera pose at each exposure time
6. sparse landmark position / covariance / static confidence
7. rejected track / dynamic weak hints
8. camera-to-landmark visibility rays
```

### 26.2 A. Pose and gravity alignment

```text
pose:
  occupancy temporal memory ego-motion warp

gravity:
  occupancy voxel z-axis를 body가 아니라 gravity에 정렬
```

camera별 exposure pose를 제공하면 multi-camera image features를 같은 body time으로 align할 수 있다.

### 26.3 B. Sparse metric depth supervision

landmark를 source image에 재투영해 auxiliary depth supervision을 만든다.

```text
pixel
metric depth
landmark covariance
static confidence
```

loss weight:

\[
w_l^{\mathrm{depth}}
\propto
\frac{s_l}{\sigma_{z,l}^2+\epsilon}.
\]

corner/texture 위치에 편향되므로 dense GT를 대체하지 않고 보완한다.

### 26.4 C. Landmark prior input

두 방식:

```text
voxel prior:
  3D landmark를 occupancy coarse grid에 splat

landmark token:
  position + covariance + static confidence를 network token으로 입력
```

필수:

```text
training-time random dropout
VIO degraded 시 confidence gating
```

Occupancy가 VIO landmarks 없이도 동작해야 한다.

### 26.5 D. Rejected tracks as dynamic hints

```text
ATLS rejected / geometry-inconsistent track
-> image ray / pixel dynamic weak hint
-> Occupancy Flow Head auxiliary supervision or query priority
```

주의:

- outlier는 calibration error, blur, occlusion일 수도 있다.
- dynamic ground truth로 hard label하지 않는다.

### 26.6 E. Landmark visibility ray as free-space evidence

camera center에서 static landmark까지의 ray는 occlusion이 없었다는 sparse evidence다.

```text
camera center -> landmark 직전
  observed free evidence

landmark 주변
  occupied / surface evidence
```

unknown과 free를 구분하는 온라인 보조 신호다.

### 26.7 Future tight coupling: occupancy/SDF factor

static VIO landmark `p_l`과 Occupancy가 만든 static surface distance field `D(x)`를 연결할 수 있다.

\[
r_l^{\mathrm{occ}}
=
D(\mathbf T_{MW}\mathbf p_{W,l}).
\]

또는 local occupancy submap 사이 alignment factor:

\[
r_{k,j}^{\mathrm{submap}}
=
\operatorname{Align}
\left(
\mathcal M_k,
\mathbf T_{B_kB_j},
\mathcal M_j
\right).
\]

권장 우선순위:

```text
1. static occupancy/SDF surface factor
2. local submap alignment factor
3. renderable Gaussian photometric factor (선택)
```

Gaussian은 유일한 연결고리가 아니다. Occupancy/SDF가 planner geometry와 더 직접적으로 연결된다.

### 26.8 Avoiding circular evidence and double counting

같은 current images를 사용한 VIO와 Occupancy output은 독립 measurement가 아니다.

잘못된 구조:

```text
current image
-> VIO feature residual
-> Occupancy prediction
-> Occupancy residual을 같은 frame VIO에 독립 factor로 추가
```

이는 같은 image evidence를 두 번 세면서 cross-correlation을 무시한다.

안전 조건:

```text
delayed map factor:
  과거 여러 frame에서 안정화된 static occupancy submap 사용

temporal separation:
  current VIO observation과 다른 map epoch

uncertainty:
  Occupancy confidence / map covariance 포함

robust switch:
  network/map factor가 visual-IMU state를 강제로 끌지 않게 함

static-only:
  dynamic occupancy 제외
```

### 26.9 Map versioning

Occupancy factor는 생성에 사용한 pose graph/map version을 기록한다.

```cpp
struct OccupancyMapFactor {
    MapVersion map_version;
    TimeRange source_interval;
    FactorCovariance covariance;
    StaticSupportMask static_mask;
};
```

loop closure로 map이 크게 바뀌면 오래된 factor를 재선형화하거나 폐기한다.

### 26.10 Recommended integration sequence

```text
v1:
  pose / gravity / camera exposure pose
  sparse landmark depth supervision

v1.5:
  landmark prior input
  dynamic weak hints
  free-space rays

v2:
  delayed static occupancy/SDF factor
  submap alignment

optional:
  Gaussian renderable branch
```

---

## 27. Validation, Benchmarks and Ablation Plan

### 27.1 Public datasets

#### Newer College Multi-Cam

검증 가능:

```text
VILENS-MC-style CCFT
fixed feature budget
aggressive motion
lighting / narrow corridor
multi-camera robustness
```

차이:

```text
dataset rig:
  4 cameras
  front stereo + lateral cameras
  약 36° overlap

our rig:
  6 yaw-distributed monocular cameras
  15~20° adjacent overlap
```

따라서 CCFT module 검증에는 적합하지만 최종 geometry의 충분한 검증은 아니다.

#### Hilti multi-camera sequences

검증 가능:

```text
MAVIS/OpenMAVIS comparison
multi-camera frame handling
fast rotation
exact preintegration
intra/inter-camera loop closure
```

#### EuRoC / TUM-VI

검증 가능:

```text
Basalt baseline
IMU preintegration
square-root backend
NFR mapping
single/stereo regression test
```

6-camera ring 성능을 직접 증명하지는 않는다.

#### VIODE / dynamic datasets

검증 가능:

```text
ATLS
weighted parallax
BCC / SSR
dynamic object robustness
```

### 27.2 Own 6-camera dataset is mandatory

필수 시나리오:

```text
straight / rotation / combined motion
near-field and far-field texture
white wall on one or multiple cameras
bright-dark transition
motion blur
camera drop / delayed timestamp
people / cart / vehicle dynamic occlusion
abruptly moving large object
U-turn / reverse traversal
long route with few loops
loop-rich repeated corridor
```

GT:

```text
mocap indoor
surveyed fiducial / total station
LiDAR SLAM ground truth
high-grade INS outdoor
```

### 27.3 Accuracy metrics

```text
ATE RMSE
RPE translation / rotation
scale error
drift per distance / time
yaw drift
gravity alignment error
velocity error
bias error
NEES / covariance consistency
```

### 27.4 Robustness metrics

```text
tracking failure count
mean time between failures
reinitialization count / duration
IMU-only duration
camera-drop survival
all-camera low-texture survival
BCC trigger count
SSR recovery success rate
false recovery rate
```

### 27.5 Multi-camera frontend metrics

```text
CCFT candidate / success / confirmed ratio
false cross-camera merge rate
camera seam track extension length
duplicate landmark ratio
cross-camera triangulation depth variance
per-camera / per-sector coverage
```

### 27.6 Compute metrics

```text
p50 / p95 / max latency
CPU per module
GPU utilization
memory
queue backlog
B_track / B_opt
optimizer iterations
NFR/global mapping cost
```

### 27.7 Loop metrics

```text
place retrieval recall
geometric verification precision
intra-camera loop count
inter-camera loop count
false loop rate
post-loop global ATE
map->odom correction magnitude
```

### 27.8 Required ablations

| Ablation | 확인할 질문 |
|---|---|
| one camera vs 6-camera shared backend | wide-FoV redundancy가 failure를 얼마나 줄이는가 |
| temporal-only vs +CCFT | narrow overlap에서 track lifetime/duplicate 감소가 실제로 있는가 |
| nearest-corner CCFT vs direct patch CCFT | precision과 compute trade-off |
| fixed per-camera count vs rig B_track/B_opt | 계산량과 observability |
| naive max-logdet vs robust constrained SFS | dynamic/coverage collapse 방지 |
| mean depth prediction vs per-landmark/interval | translation motion에서 tracking error |
| approximate vs exact `SE_2(3)` preintegration | fast rotation / frame-drop interval 성능 |
| Huber vs ATLS | dynamic residual 영향 |
| ATLS only vs +BCC+SSR | abruptly dynamic divergence recovery |
| marginalize-before-BCC vs stable commit | prior contamination 영향 |
| local descriptor matching vs direct local tracking | compute / accuracy |
| intra-camera loops vs +inter-camera loops | U-turn/reverse route loop recall |
| no NFR vs NFR global mapping | gravity alignment / global consistency |
| no occupancy bridge vs one-way vs delayed factor | tight coupling 이득과 feedback instability |

### 27.9 Unit and numerical tests

```text
projection/unprojection round trip per camera model
T_AB convention test
cross-camera projection synthetic test
inverse-depth Jacobian finite difference
pose Jacobian finite difference
exact preintegration Jacobian finite difference
preintegration Monte-Carlo covariance
square-root marginalization nullspace test
rehosting state equivalence
SSR byte/state equivalence after restore
feature-selection deterministic replay
camera dropout replay
```

### 27.10 Acceptance criteria

최종 architecture가 성공하려면 평균 ATE 하나가 아니라 다음을 동시에 만족해야 한다.

```text
accuracy:
  single/stereo Basalt baseline 이상

robustness:
  one/two camera degradation에서 estimator 유지

bounded compute:
  camera 수 증가에도 B_opt와 latency bounded

dynamic safety:
  abruptly dynamic case에서 bias divergence 억제

global consistency:
  loop 후 map drift 감소, local odom continuity 유지

interface quality:
  Occupancy Network가 timestamp-consistent pose/gravity/covariance를 수신
```

---

## 28. Cross-Architecture Consistency and Contradiction Checks

이 section은 여러 논문에서 가져온 개념들이 실제로 같은 시스템 안에서 양립하는지 검증한다. 단순히 “섞을 수 있다”고 선언하지 않고, state·measurement·thread·frequency·commit boundary를 기준으로 모순 여부를 확인한다.

### 28.1 Descriptor-free local VIO vs descriptor-based loop closure

겉보기 모순:

```text
local VIO에서는 descriptor를 쓰지 않는다.
하지만 global SLAM에서는 ORB/learned descriptor를 쓴다.
```

모순이 아닌 이유:

```text
local VIO:
  목적 = frame-to-frame 저지연 state estimation
  주기 = camera rate
  association = small-motion / IMU-predicted local search

global SLAM:
  목적 = long-baseline place recognition
  주기 = keyframe rate
  association = viewpoint/lighting invariant retrieval
```

동일 frame에 동일 비용을 중복 지불하는 것이 아니다. Basalt NFR도 high-rate patch tracking과 low-rate distinctive keypoints를 분리한다.

결론:

```text
local descriptor 없음
+ keyframe-only descriptor 있음
= 의도된 two-rate architecture
```

### 28.2 Per-camera temporal trackers vs one tightly-coupled VIO

겉보기 모순:

```text
camera별로 optical flow를 독립 수행한다.
그런데 estimator는 하나라고 한다.
```

모순이 아닌 이유:

tracking thread가 분리되어도 residual은 동일한 body-state window와 동일 IMU state를 optimize한다.

\[
\mathbf r_{C_0},\ldots,\mathbf r_{C_5}
\rightarrow
\mathbf x_k.
\]

frontend의 병렬성은 state-space 분리를 의미하지 않는다.

결론:

```text
independent image processing
!= independent VIO state
```

### 28.3 CCFT가 없어도 tight coupling인데 왜 CCFT를 쓰는가

겉보기 중복:

```text
cross-camera matching 없이도 tight-coupled라면 CCFT가 불필요한가?
```

아니다.

```text
shared backend:
  body pose redundancy 제공

CCFT:
  landmark continuity / depth / graph compactness 제공
```

shared backend는 camera seam에서 동일 point가 두 landmark로 끊기는 것을 자동으로 해결하지 않는다. CCFT는 그 중복을 줄이고 long track을 만든다.

### 28.4 Narrow 15~20° overlap vs CCFT 중심 설계

잠재 모순:

```text
VILENS-MC는 더 큰 overlap 장치에서 검증되었다.
우리 overlap은 좁다.
```

해결:

- CCFT를 estimator 생존의 전제에서 보조 모듈로 낮춘다.
- temporal track + shared backend를 foundation으로 둔다.
- seam 근처의 mature landmark만 CCFT한다.
- precision을 recall보다 우선한다.

따라서 overlap 효과가 작아도 전체 VIO는 성립한다.

### 28.5 Duplicate seed suppression vs dual-camera observations

겉보기 모순:

```text
overlap에서 중복을 줄인다.
동시에 두 camera observation을 모두 넣는다.
```

대상이 다르다.

```text
중복 억제:
  같은 physical point에서 새 landmark를 두 개 seed하는 것

허용:
  이미 같은 global landmark임이 확인된 point를 두 camera가 관측하는 것
```

첫 번째는 graph redundancy, 두 번째는 useful measurement다.

### 28.6 VILENS-MC nearest-feature association vs Basalt direct patch

두 방식을 동시에 실행하지 않는다.

```text
가져오는 것:
  geometry-guided camera-to-camera projection이라는 원리

교체하는 것:
  nearest detected feature -> direct patch alignment
```

따라서 duplicate association module이 아니라 하나의 enhanced CCFT다.

### 28.7 MAVIS local-map descriptor matching vs descriptor-free local VIO

MAVIS는 projected local map point를 descriptor distance로 검증한다. 최종 구조는 local-map projection 개념만 가져오고 local verification을 direct patch로 교체한다.

```text
MAVIS에서 채택:
  visible local landmark를 current multi-camera rig 전체에 projection

MAVIS에서 교체:
  descriptor match -> covariance-aware patch alignment
```

loop closure에서는 MAVIS식 descriptor를 유지한다.

### 28.8 MAVIS histogram equalization vs direct photometric tracking

모순은 branch separation으로 해결한다.

```text
detector branch:
  optional equalization

tracking branch:
  linearized photometric image
```

동일 patch residual에 서로 다른 비선형 intensity transform을 섞지 않는다.

### 28.9 Basalt IMU factor vs MAVIS exact preintegration

둘을 동시에 넣지 않는다.

```text
Basalt:
  state / solver / marginalization interface 유지

MAVIS:
  preintegration implementation을 교체
```

동일 IMU measurement의 double counting을 방지한다.

### 28.10 ATLS vs Basalt square-root optimization

ATLS는 solver 대체물이 아니라 residual weight/robust objective다.

```text
ATLS:
  어떤 visual row를 얼마나 쓸지 결정

square-root solver:
  weighted linearized system을 수치적으로 푸는 방법
```

\[
\sqrt{w}\mathbf J,\sqrt{w}\mathbf r
\]

형태로 자연스럽게 결합된다.

### 28.11 ATLS vs frontend gates

중복 책임이 아니다.

```text
frontend gate:
  correspondence 자체가 물리적으로 타당한가

ATLS:
  남은 correspondence가 joint state optimization에서 dynamic/outlier인가
```

frontend는 obvious outlier를 제거하고, ATLS는 state-dependent residual을 robust하게 처리한다.

### 28.12 BCC/SSR vs square-root marginalization

잠재적 치명적 모순:

```text
marginalization은 정보를 비가역적으로 prior에 압축한다.
SSR은 이전 state로 되돌린다.
```

해결은 transaction boundary다.

```text
optimization tentative
-> BCC
-> pass일 때만 marginalization commit
```

fail이면 prior까지 snapshot으로 복구한다. 이 순서를 지키지 않으면 두 개념은 실제로 모순된다.

### 28.13 Exact preintegration vs BCC

BCC가 approximate IMU model을 쓰면 모순이다. 최종 구조는 BCC와 backend가 동일 exact preintegration residual, bias Jacobian, covariance를 공유한다.

### 28.14 Same pitch/yaw ring simplification vs arbitrary camera model

기구상 same pitch는 다음에만 사용한다.

```text
camera order
cylindrical grid
coarse overlap prior
```

optimization은 calibrated full `SE(3)`와 camera별 projection model을 사용한다. 따라서 조립 오차와 미래 hardware 변경에 대응한다.

### 28.15 Rig FrameBundle vs asynchronous observation time

bundle은 “같은 pose state node를 공유하는 처리 단위”이지 모든 exposure가 정확히 동일 시각이라는 선언이 아니다.

각 camera observation은 Section 1.3의 time contract를 따른다.

```text
synchronized global-shutter default:
  motion-induced S_time이 작으면 corresponding active state(s)에 직접 연결

asynchronous / delayed / high-motion case:
  active window 안에서 IMU interpolation으로 연결
```

따라서 partial synchronization과 rig bundle이 양립한다.

### 28.16 Same timestamp dual camera residual vs duplicate pose nodes

camera마다 pose state를 만들지 않는다.

```text
one reference state x_k
+ optional IMU interpolation to observation time
+ multiple camera residuals
```

이 구조가 state dimension을 억제하면서 timestamp correction을 반영한다.

### 28.17 Information selection vs camera minimum quota

pure max-logdet는 mathematically informative하지만 texture-rich 특정 방향으로 집중될 수 있다. quota를 추가하면 global optimum은 제약된 feasible set 안에서 찾는다.

```text
unconstrained optimum:
  최대 정보량만

constrained optimum:
  최소 coverage를 지키는 범위에서 최대 정보량
```

이는 정보이론을 훼손하는 heuristic이 아니라 실제 failure constraints를 포함한 최적화 문제다.

### 28.18 Static confidence vs observation confidence

하나의 weight로 합치면 다음이 구분되지 않는다.

```text
landmark가 dynamic
특정 camera observation만 blur/mismatch
```

`w=s_l q_o`로 factorization해 physical staticness와 measurement quality를 분리한다.

### 28.19 Dynamic semantic prior vs geometry-first ATLS

semantic segmentation이 잘못될 수 있으므로 hard mask가 아니다. semantic/Occupancy Flow는 `s_l` 초기 prior에만 약하게 반영하고, temporal geometry와 IMU consistency로 갱신한다.

### 28.20 Rehosting vs NFR/marginalization

rehosting은 local active landmark parameterization을 바꾸는 과정이고 NFR는 marginalized keyframe pose information을 global layer로 넘기는 과정이다.

순서:

```text
rehost surviving active landmarks
-> marginalize old local variables
-> recover/export pose factors
```

동일 landmark를 global mapper가 별도 descriptor map point로 표현할 수 있지만, local host parameterization과 global map ID를 혼동하지 않는다.

### 28.21 NFR vs loop pose graph/global BA

NFR와 loop constraint는 경쟁 factor가 아니다.

```text
NFR:
  local VIO가 축적한 relative/gravity information

loop:
  먼 시간 사이의 place-recognition constraint
```

둘을 global objective에 함께 넣을 수 있지만 factor provenance가 달라야 한다. 같은 local VIO marginalization interval에서 나온 relative pose 정보를 NFR와 별도 pose-graph edge로 중복 추가하지 않는다.

```text
allowed:
  local_vio_nfr + independent loop_detection factor

forbidden:
  local_vio_nfr + same interval에서 복사한 relative pose edge
```

### 28.22 Local odom continuity vs global correction

loop closure가 state를 정정해야 하지만 control pose가 점프하면 안 된다.

```text
local VIO:
  odom->base continuous

global mapper:
  map->odom correction
```

두 frame의 분리가 두 요구를 동시에 만족시킨다.

### 28.23 OpenMAVIS code reuse vs Basalt backbone

OpenMAVIS는 ORB-SLAM3 기반 재구현이고 GPL-3.0이다. Basalt는 BSD-3-Clause다.

최종 설계는:

```text
MAVIS paper의 수식/설계 개념을 Basalt에 독립 구현
OpenMAVIS backend를 직접 merge하지 않음
```

architecture mismatch와 license coupling을 동시에 피한다.

### 28.24 Occupancy feedback vs statistical double counting

current camera images가 VIO와 Occupancy 양쪽에 들어가므로 Occupancy output을 즉시 독립 residual로 재사용하면 correlated evidence를 두 번 센다.

해결:

```text
delayed stable submap
explicit uncertainty
static-only support
robust switch
map versioning
```

초기 one-way VIO->Occupancy는 안전하며, tight coupling은 위 조건을 만족할 때만 추가한다.

### 28.25 Global landmark continuity vs false merge risk

long track은 유용하지만 false merge는 여러 frame을 오염시킨다. provisional association, temporal confirmation, split rollback을 둬 continuity 이득과 precision을 동시에 지킨다.

### 28.26 Final consistency statement

최종 구조의 module dependency는 순환하지 않는다.

```text
sensor calibration
-> tracking / association
-> landmark confidence
-> feature selection
-> local optimization
-> BCC stable commit
-> marginalization / NFR
-> global mapping
-> map->odom / delayed optional occupancy factor
```

현재 frame의 tentative output이 그 same transaction의 prior로 다시 들어가는 path를 금지한다. 이 acyclic commit structure가 Basalt·VILENS-MC·MAVIS·DynaVINS++를 함께 쓸 때의 핵심 일관성 조건이다.

---

## 29. Explicitly Excluded Architectures

최종 버전에서 사용하지 않는다.

```text
1. camera별 Basalt VIO 6개를 돌리고 pose를 나중에 fuse
2. camera별 independent local map / loop closer 6개
3. 6-camera all-pairs descriptor matching at every frame
4. six images를 panorama로 stitching한 뒤 single-camera VIO
5. parent-child camera tree
6. fixed rectangular overlap box만으로 correspondence 결정
7. 모든 feature에 one average depth 적용
8. projected pixel의 nearest corner를 검증 없이 merge
9. camera별 fixed full feature count
10. 모든 tracked feature를 BA에 투입
11. dynamic confidence를 무시한 pure max-logdet
12. one weak camera가 전체 rig keyframe을 강제
13. per-camera pose state를 same timestamp마다 6개 생성
14. local visual factor에 Huber + GM + ATLS를 중첩
15. BCC 전에 marginalization
16. BCC 전에 NFR export
17. SSR에서 state vector만 복원하고 prior는 유지
18. Basalt IMU factor와 MAVIS exact factor 동시 사용
19. direct tracking image에 frame-wise histogram equalization
20. MAVIS stereo first-frame initialization을 필수 조건으로 사용
21. OpenMAVIS ORB-SLAM3 backend 직접 병합
22. semantic dynamic mask로 feature hard delete
23. runtime full extrinsic 6개를 unconstrained optimize
24. camera drop 시 전체 bundle 폐기
25. loop closure correction을 odom->base에 abrupt jump로 적용
26. current-frame Occupancy output을 independent VIO residual로 즉시 재사용
27. Gaussian을 VIO-Occupancy 연결의 필수 representation으로 강제
```

---

## 30. Implementation Notes and Delivery Order

아래 단계는 축소된 초기 architecture를 추천하는 것이 아니다. **최종 architecture는 앞 section 전체이며**, 다음은 의존성에 맞춰 그것을 구현·검증하는 순서다.

### Stage 1. Rig foundation and shared backend

```text
6-camera calibration / time model
FrameBundle + partial bundle
camera별 Basalt temporal tracking
one shared body-state backend
all-camera reprojection residual
square-root marginalization
```

acceptance:

```text
cross-camera matching 없이도 6-camera tight-coupled VIO 동작
one camera drop survival
single/stereo Basalt regression 통과
```

### Stage 2. Unified landmark graph and rig coverage

```text
global landmark ID
immature/mature state
inverse-depth covariance
cylindrical rig grid
seed ownership
rehosting
```

acceptance:

```text
host marginalization 후 long track 유지
camera/sector coverage telemetry
```

### Stage 3. Enhanced CCFT

```text
depth-dependent overlap graph
mature projection + direct patch alignment
immature epipolar curve search
provisional confirmation / rollback
```

acceptance:

```text
seam track extension 증가
false merge precision 목표 충족
temporal-only보다 duplicate landmark 감소
```

### Stage 4. Two-stage budget

```text
B_track
B_opt
rig constrained max-logdet
latency load controller
```

acceptance:

```text
camera 수가 증가해도 backend latency bounded
accuracy degradation 없이 B_opt 감소 구간 확인
```

### Stage 5. MAVIS sensor handling and exact IMU

```text
IMU intrinsic compensation
exact SE_2(3) preintegration
camera delay / global-shutter mid-exposure
frame drop degraded mode
```

acceptance:

```text
fast rotation / long integration improvement
finite-difference Jacobian
Monte-Carlo covariance
backend/BCC가 공유할 IMU residual interface 확정
```

### Stage 6. DynaVINS++ robustness

```text
s_l / q_o
weighted angular parallax
ATLS alternating optimization
BCC using the exact SE_2(3) backend IMU model
full stable snapshot / SSR
stable commit transaction
```

acceptance:

```text
VIODE / abruptly dynamic sequence에서 bias divergence 감소
marginalization prior rollback exactness test
BCC model consistency
```

### Stage 7. NFR global SLAM

```text
stable NFR export
keyframe-only descriptors
intra/inter-camera place recognition
geometric verification
global BA / map->odom
relocalization
```

acceptance:

```text
local odom continuity
loop-rich route global drift reduction
U-turn inter-camera loop 검출
```

### Stage 8. Occupancy integration

```text
pose / gravity / covariance
sparse depth supervision
landmark prior / dropout
dynamic hints
free-space rays
delayed occupancy/SDF factor
```

acceptance:

```text
one-way integration 안정성
double-counting 방지 ablation
tight factor가 실제 pose accuracy를 개선하는지 검증
```

### Implementation guardrails

```text
최적화 수식 변경마다 Jacobian finite-difference test
camera topology를 if(camera==left/right)로 hard-code하지 않음
module output을 immutable packet으로 전달
stable commit ID를 모든 exported factor에 기록
profiling 없이 fixed feature number를 논문에서 복사하지 않음
```

---

## 31. Current Recommended Architecture

현재 최종 추천을 한 블록으로 정리하면 다음과 같다.

```text
Sensor rig:
  6 hardware-synchronized monocular cameras + IMU
  same nominal pitch, different yaw
  adjacent overlap 15~20°
  full calibrated SE(3), not hard-coded yaw-only

Camera topology:
  ring graph
  edges = (0,1),(1,2),(2,3),(3,4),(4,5),(5,0)
  depth-dependent overlap masks

Time model:
  bundle reference time
  per-camera IMU offset
  global-shutter mid-exposure compensation
  no row-time residual in the final target
  partial bundle / frame drop support

Photometric input:
  linearized grayscale + vignetting correction
  affine brightness patch model
  detector-only contrast enhancement optional

Temporal frontend:
  Basalt sparse direct patch optical flow per camera
  IMU pose prediction
  mature landmark-specific inverse depth/covariance
  immature inverse-depth interval
  no global average depth

Local-map projection:
  mature static landmarks projected into visible cameras
  one or two visible cameras in normal ring operation

Cross-camera frontend:
  VILENS-MC CCFT concept
  adjacent edge only
  mature: geometry projection + covariance-aware direct patch refinement
  immature: epipolar-curve inverse-depth search
  forward/backward + Mahalanobis + provisional confirmation
  no nearest-corner-only association
  no descriptors

Landmark representation:
  rig-wide global landmark ID
  host frame-camera inverse depth
  inverse-depth covariance
  immature / mature lifecycle
  observation confidence q_o
  static confidence s_l
  rehosting before marginalization

Spatial distribution:
  body-frame azimuth/elevation cylindrical grid
  overlap new-seed owner camera 하나
  confirmed landmark dual-camera observation 허용

Feature budgets:
  B_track for frontend
  B_opt <= B_track for backend
  B_xcam for CCFT
  camera / azimuth / elevation / depth quotas
  robust rig-wide max-logdet selection

Dynamic robustness:
  DynaVINS++ static weights
  rig-level weighted angular parallax
  ATLS as local visual robust objective
  BCC for pose-bias consistency
  SSR with full state/prior/landmark/weight rollback

Local backend:
  one body state per bundle
  all-camera reprojection factors
  exact SE_2(3) IMU preintegration
  Basalt square-root optimization and marginalization
  BCC pass before stable commit

Marginalization / mapping:
  rehost surviving landmarks
  stable window only marginalize
  stable marginalization only NFR export

Keyframes:
  one rig pose node
  static-weighted angular parallax
  pose information reduction
  new landmark / health / max interval
  one weak camera alone cannot force keyframe

Global SLAM:
  descriptors only on stable rig keyframes
  selected cameras / coverage-aware descriptor budget
  intra-camera + inter-camera loop closure
  geometric verification
  NFR + global BA / pose graph
  continuous odom->base, corrected map->odom

Runtime:
  camera frontend parallel
  one landmark merge/budget stage
  one local optimizer transaction
  IMU-rate propagation
  loop/global mapping off critical path

Failure behavior:
  camera/edge-specific degradation
  short IMU-only with covariance growth
  recovery from last stable snapshot
  explicit LOST state, no silent confidence

Occupancy interface:
  pose / covariance / gravity / exposure poses
  static landmarks + covariance
  dynamic weak hints
  free-space rays
  future delayed static occupancy/SDF or submap factor
  Gaussian branch optional, not mandatory
```

가장 중요한 설계 원칙:

```text
6개의 카메라는 6개의 독립 VIO가 아니다.
하나의 generalized camera rig가 하나의 body state를 관측한다.

feature는 camera에 소속되는 것이 아니라 global landmark graph에 소속된다.
camera는 그 landmark를 관측하는 sensor view다.

local VIO의 association은 descriptor가 아니라
IMU + geometry + direct patch tracking으로 수행한다.

descriptor는 long-baseline place recognition이 필요한
stable rig keyframe global layer에서만 사용한다.

compute는 camera당 feature count가 아니라
rig-wide B_track / B_opt로 제한한다.

optimization 결과는 바로 prior로 굳히지 않는다.
ATLS 후 BCC를 통과한 state만 stable commit하고 marginalize한다.
```

---

## 32. Recommended Papers and Repositories

이 architecture를 구현할 때 모든 논문을 같은 깊이로 읽을 필요는 없다. module별로 본다.

### Core backbone

1. [Basalt repository](https://github.com/VladyslavUsenko/basalt)
   - camera model, calibration, patch tracker, VIO/mapping code 구조
   - GitHub는 mirror이며 upstream GitLab도 확인
   - BSD-3-Clause

2. [Visual-Inertial Mapping with Non-Linear Factor Recovery](https://arxiv.org/abs/1904.06504)
   - high-rate patch VIO와 low-rate descriptor mapper 분리
   - marginalization prior에서 nonlinear factors를 복구해 global BA로 전달

3. [Square Root Marginalization for Sliding-Window Bundle Adjustment](https://arxiv.org/abs/2109.02182)
   - Basalt square-root optimization/marginalization의 수치적 근거

### Multi-camera tracking and compute

1. [VILENS-MC: Balancing the Budget](https://arxiv.org/abs/2109.05975)
   - CCFT
   - fixed overall feature budget
   - informative feature selection
   - 주의: nearest-feature association과 camera별 SFS는 최종 구조에서 강화/변경

2. [Newer College Dataset Usage](https://ori-drs.github.io/newer-college-dataset/usage/)
   - VILENS-MC 평가와 multi-camera benchmark
   - dataset은 architecture module이 아님

3. [Toward Efficient and Robust Multiple Camera VIO](https://arxiv.org/abs/2109.12030)
   - camera별 temporal feature stream + shared backend
   - GPU/VPI frontend의 embedded compute 참고

4. [Redesigning SLAM for Arbitrary Multi-Camera Systems](https://arxiv.org/abs/2003.02014)
   - information-theoretic keyframe selection
   - adaptive initialization
   - 2~6 arbitrary camera setup

5. [OpenVINS](https://docs.openvins.com/)
   - arbitrary cameras, calibration/time offset, sparse KLT, bounded update 비교 baseline
   - final backend는 Basalt optimization이므로 OpenVINS EKF를 이식하지 않음

### Multi-camera SLAM and IMU

1. [MAVIS](https://arxiv.org/abs/2309.08142)
   - partially overlapped camera local-map projection
   - exact `SE_2(3)` IMU preintegration
   - IMU intrinsic compensation
   - frame drop / delay / mid-exposure
   - intra/inter-camera loop closure

2. [OpenMAVIS](https://github.com/MAVIS-SLAM/OpenMAVIS)
   - MAVIS 개념을 ORB-SLAM3에 재구현한 reference
   - 원본 MAVIS code가 아님
   - preprocessing / IMU intrinsic compensation 일부 미포함
   - GPL-3.0이므로 Basalt에 직접 code merge하지 않고 독립 구현

### Dynamic robustness

1. [DynaVINS++](https://arxiv.org/abs/2410.15373)
   - weighted parallax
   - ATLS
   - BCC
   - SSR

2. [DynaVINS](https://arxiv.org/abs/2208.11500)
   - dynamic / temporarily static feature에 대한 robust BA의 선행 연구

### Direct Basalt multi-camera implementation references

1. [Enhancing Visual-Inertial Odometry Robustness and Accuracy in Challenging Environments](https://doi.org/10.3390/robotics14060071)
   - Basalt source를 multi-camera로 확장하는 실제 코드 변경 관점
   - IMU feature prediction
   - adaptive detector 문제의식
   - parent-child / mean-depth / count-only threshold는 그대로 사용하지 않음

2. *Visual-Inertial Simultaneous Localization and Mapping with Multiple Cameras*, Pak Hong Chui, LMU Munich, 2020
   - Basalt-SLAM tracker/mapper/loop thread 구성
   - NFR factor 전달
   - multi-camera 부분은 independent map merging이므로 rigid rig local VIO에는 사용하지 않음

### Minimal reading order

```text
1. Basalt NFR paper + Basalt code
2. Square Root Marginalization
3. VILENS-MC
4. DynaVINS++
5. MAVIS
6. 2025 Basalt multi-camera extension
7. Redesigning SLAM for Arbitrary Multi-Camera Systems
8. MCVIO / OpenVINS as comparison
9. 2020 Basalt-SLAM thesis for mapper/NFR implementation details
```

---

## 33. References

1. V. Usenko, N. Demmel, D. Schubert, J. Stückler, D. Cremers, “Visual-Inertial Mapping with Non-Linear Factor Recovery,” *IEEE RA-L*, 2020. [arXiv](https://arxiv.org/abs/1904.06504)
2. N. Demmel, D. Schubert, C. Sommer, D. Cremers, V. Usenko, “Square Root Marginalization for Sliding-Window Bundle Adjustment,” *ICCV*, 2021. [arXiv](https://arxiv.org/abs/2109.02182)
3. L. Zhang, D. Wisth, M. Camurri, M. Fallon, “Balancing the Budget: Feature Selection and Tracking for Multi-Camera Visual-Inertial Odometry,” *IEEE RA-L*, 2022. [arXiv](https://arxiv.org/abs/2109.05975)
4. Y. Wang et al., “MAVIS: Multi-Camera Augmented Visual-Inertial SLAM using `SE_2(3)` Based Exact IMU Pre-integration,” *ICRA*, 2024. [arXiv](https://arxiv.org/abs/2309.08142)
5. S. Song, H. Lim, A. J. Lee, H. Myung, “DynaVINS++: Robust Visual-Inertial State Estimator in Dynamic Environments by Adaptive Truncated Least Squares and Stable State Recovery,” *IEEE RA-L*, 2024. [arXiv](https://arxiv.org/abs/2410.15373)
6. S. Song et al., “DynaVINS: A Visual-Inertial SLAM for Dynamic Environments,” *IEEE RA-L*, 2022. [arXiv](https://arxiv.org/abs/2208.11500)
7. A. Minervini, A. Carrio, G. Guglieri, “Enhancing Visual-Inertial Odometry Robustness and Accuracy in Challenging Environments,” *Robotics*, 2025. [DOI](https://doi.org/10.3390/robotics14060071)
8. J. Kuo, M. Muglikar, Z. Zhang, D. Scaramuzza, “Redesigning SLAM for Arbitrary Multi-Camera Systems,” *ICRA*, 2020. [arXiv](https://arxiv.org/abs/2003.02014)
9. Y. He, H. Yu, W. Yang, S. Scherer, “Toward Efficient and Robust Multiple Camera Visual-Inertial Odometry,” 2021. [arXiv](https://arxiv.org/abs/2109.12030)
10. P. Geneva, K. Eckenhoff, W. Lee, Y. Yang, G. Huang, “OpenVINS: A Research Platform for Visual-Inertial Estimation,” *ICRA*, 2020. [Documentation](https://docs.openvins.com/)
11. P. H. Chui, “Visual-Inertial Simultaneous Localization and Mapping with Multiple Cameras,” Master Thesis, LMU Munich, 2020.
12. [Basalt repository](https://github.com/VladyslavUsenko/basalt)
13. [OpenMAVIS repository](https://github.com/MAVIS-SLAM/OpenMAVIS)
14. [Newer College Dataset](https://ori-drs.github.io/newer-college-dataset/usage/)

---

## Appendix A. Final Module Dependency Graph

```text
Calibration / Timing
        |
        v
FrameBundle + IMU propagation
        |
        +-----------------------+
        |                       |
        v                       v
Temporal tracking       Local-map projection
        |                       |
        +-----------+-----------+
                    v
              Adjacent CCFT
                    |
                    v
             Landmark manager
                    |
                    v
         Static / observation confidence
                    |
                    v
        Rig-wide constrained feature budget
                    |
                    v
             ATLS VI optimization
                    |
                    v
                  BCC
             +------+------+
             |             |
            pass          fail
             |             |
       stable commit    full SSR
             |          + RetryPolicy
             |          + rebuild tentative graph
             |          + bounded retry / recovery
             |
             v
      rehost / marginalize
             |
             v
          NFR export
             |
             v
  keyframe loop / global mapping
             |
             v
       map->odom correction
```

---

## Appendix B. Final Design Decision Ledger

| Decision | Final choice | 이유 |
|---|---|---|
| estimator count | 1 | rigid rig는 body state 하나 |
| frontend association | direct patch | 6-camera descriptor cost 회피 |
| cross-camera scope | adjacent ring edges | 15~20° 실제 overlap만 활용 |
| cross-camera depth unknown | epipolar curve | average depth 오류 회피 |
| landmark identity | rig global ID | seam continuity |
| feature distribution | body cylindrical grid | camera boundary와 무관한 360° coverage |
| compute bound | B_track + B_opt | camera 수와 BA 크기 분리 |
| feature selection | robust constrained max-logdet | dynamic/coverage collapse 방지 |
| dynamic robust loss | ATLS | threshold 밖 large residual 영향 차단 |
| state safety | BCC + SSR | bias divergence와 prior contamination 방지 |
| IMU integration | exact `SE_2(3)` | fast rotation / long interval |
| marginalization | Basalt square-root | numerical stability |
| VIO->mapping | NFR | local VI information 보존 |
| loop descriptor | stable keyframe only | local latency와 long-baseline invariance 분리 |
| loop topology | intra + inter camera | U-turn/reverse traversal |
| global correction | map->odom | control odometry continuity |
| Occupancy coupling | one-way first, delayed robust factor later | correlated evidence / double counting 방지 |

---

## Appendix C. Final Pre-Merge Checklist

```text
[ ] 6 cameras all use full calibrated SE(3)
[ ] camera order and ring edges verified physically
[ ] all camera timestamp-to-body offsets and midpoint conventions measured
[ ] global-shutter row-time skew is zero, while transport delay is calibrated
[ ] fixed time-offset mode does not optimize Delta t_i^clk or midpoint correction
[ ] S_time contains per-frame jitter only, not fixed offset prior repeated per residual
[ ] direct node attach uses motion-induced S_time gate, not raw time only
[ ] tracking images are photometrically linearized
[ ] per-camera temporal tracker passes replay tests
[ ] one shared body-state optimizer confirmed
[ ] cross-camera projection finite-difference tested
[ ] provisional CCFT rollback works
[ ] BCC retry rebuilds tentative factors from raw bundle with RetryPolicy
[ ] landmark rehosting preserves reprojection cost
[ ] B_track/B_opt are globally bounded
[ ] static confidence precedes SFS
[ ] B_opt selection uses eta_pre/history without duplicating s_l/q_o or current ATLS auxiliary weights
[ ] ATLS row weights applied before square-root solve
[ ] BCC uses same exact IMU model as backend
[ ] stable commit transaction is atomic across state/landmarks/prior/NFR
[ ] SSR restores prior and linearization points
[ ] no marginalization before stable commit
[ ] no NFR export before stable commit
[ ] descriptors absent from local VIO critical path
[ ] loop DescriptorView stores pixel/descriptor/camera/time/landmark provenance
[ ] retrieval-only descriptors have 2D-2D fallback before 2D-3D loop factor creation
[ ] 2D-2D-only loop constraints are not promoted to metric SE(3) edges without scale observability
[ ] NFR and loop factors have provenance IDs and no duplicate local VIO interval
[ ] intra/inter-camera loop geometric verification enabled
[ ] camera drop does not drop whole bundle
[ ] health/covariance published with pose
[ ] Occupancy feedback does not reuse current image as independent evidence
```

---

## Appendix D. Source-Specific Evidence, Limits and Transfer Rules

이 architecture는 기존 논문의 module을 이름만 나열한 것이 아니라, 각 논문이 실제로 검증한 범위와 우리 rig로 옮길 때 생기는 경계를 구분한다.

### D.1 2025 Basalt multi-camera extension

해당 논문의 adaptive detector는 다음 형태다.

\[
r
=
\frac{N_{\mathrm{tracked}}}{N_{\mathrm{min}}},
\qquad
\tau=e^{-k(r-1)}.
\]

`tau`로 다음 camera의 detector threshold를 조절한다.

```text
앞 camera에서 overlap feature가 충분:
  다음 camera는 높은-quality feature만 적게 검출

앞 camera에서 부족:
  다음 camera detector가 더 많이 검출
```

문제:

```text
feature count != pose information
camera processing order에 의존
ring의 양쪽 neighbor를 대칭적으로 표현하지 못함
서로 반대 방향 camera 정보는 중복이 아님
```

따라서 최종 구조는 이 문제의식만 가져오고 다음으로 교체한다.

```text
count-only sequential threshold
-> rig cylindrical coverage + camera quota + robust information gain
```

논문의 IMU prediction도 average triangulated depth를 썼지만, 최종 구조는 translation parallax를 위해 landmark-specific depth 또는 interval을 사용한다.

### D.2 VILENS-MC transfer boundary

VILENS-MC가 직접 입증한 것:

```text
4-camera hardware-synchronized device
모든 camera를 simultaneous factor graph에 투입
cross-camera feature continuity
fixed overall feature budget
informative subset으로 backend cost 절감
challenging corridors / dark / aggressive motion
```

우리 rig와 다른 점:

```text
VILENS-MC:
  front stereo + lateral cameras
  더 큰 약 36° overlap

ours:
  six yaw-distributed monocular cameras
  adjacent 15~20° overlap
```

따라서 다음은 그대로 가정하지 않는다.

```text
같은 CCFT recall
같은 duplicate reduction ratio
같은 SFS budget number
같은 camera별 processing order
```

우리 architecture에서 CCFT는 narrow seam의 precision-first 보강이고, core robustness는 shared backend와 360° independent temporal tracks가 제공한다.

### D.3 MAVIS transfer boundary

MAVIS가 입증한 것:

```text
partially overlapped multi-camera VI-SLAM
local map point를 current multi-camera images에 projection
intra/inter-camera feature association
exact SE_2(3) preintegration
frame drop / camera delay / mid-exposure handling
intra/inter-camera loop closure
```

그대로 쓰지 않는 것:

```text
stereo first-frame initialization
descriptor-based local matching
tracking image histogram equalization
ORB-SLAM3-centered OpenMAVIS backend
```

OpenMAVIS는 original MAVIS code가 아니라 compact ORB-SLAM3 reimplementation이며, 일부 preprocessing과 IMU intrinsic compensation이 빠져 있다. 따라서 code transplantation보다 paper formulation을 Basalt convention에 맞춰 독립 구현한다.

### D.4 DynaVINS++ transfer boundary

DynaVINS++가 직접 다루는 것은 VINS 계열 sliding-window estimator다.

우리 architecture에서 새로 검증해야 하는 것:

```text
ATLS weight를 Basalt square-root linear system에 넣는 정확한 위치
multi-camera global landmark의 shared static confidence
rig angular parallax
cross-camera false association과 dynamic object residual의 구분
BCC threshold를 exact SE_2(3) preintegration과 맞추는 방법
SSR가 square-root prior와 rehosting graph까지 완전히 복구하는지
```

특히 `BCC -> marginalization` 순서는 DynaVINS++ 개념을 Basalt prior에 맞춰 확장한 설계 결정이다. 논문 이름만으로 자동 보장되는 부분이 아니며, rollback equivalence test가 필수다.

### D.5 2020 Basalt-SLAM thesis transfer boundary

직접 참고할 수 있는 것:

```text
optical-flow tracker는 high rate
mapper/loop closer는 lower rate
ORB/BoW는 global map association에 사용
marginalization information을 NFR factor로 전달
```

rig local estimator에 쓰지 않는 것:

```text
각 camera가 독립 Tracker/Mapper/LoopCloser를 보유
각 camera/sequence map을 loop closure로 merge
```

이 구조는 multi-agent/multi-session map merging에는 의미가 있지만, 하나의 rigid 6-camera rig에서 body state를 공동 최적화하는 구조보다 중복 계산과 정보 손실이 크다.

### D.6 MCVIO/OpenVINS/Kuo의 보조 논리

이 세 계열은 최종 backend를 제공하기보다 다음 논리를 검증한다.

```text
MCVIO:
  overlap이 없어도 camera별 tracks를 shared BA에 넣으면 multi-camera VIO가 성립
  GPU/VPI frontend로 embedded load 감소 가능

OpenVINS:
  independent camera tracking, arbitrary camera/time calibration,
  bounded feature update가 실제 구현 가능

Kuo et al.:
  camera arrangement에 종속되지 않은 initialization/keyframe metric이 가능
  2~6 camera setup에서 information-theoretic policy 사용 가능
```

따라서 우리 architecture의 foundation은 CCFT가 아니라 shared body estimator이며, CCFT는 overlap이 주는 추가 이득을 회수한다.

### D.7 Evidence matrix

| Architecture claim | 직접 근거 | 우리 rig에서 추가 검증 |
|---|---|---|
| one multi-camera factor graph | VILENS-MC, MCVIO, MAVIS | 6-camera Basalt state/Jacobian |
| cross-camera long track | VILENS-MC | 15~20° seam success/precision |
| descriptor-free local tracking | Basalt, VILENS-MC geometry concept | cross-camera direct patch photometry |
| fixed total budget | VILENS-MC | rig-wide constrained SFS |
| arbitrary 2~6 camera keyframe | Kuo et al. | body angular-parallax + dynamic weight |
| exact IMU integration | MAVIS | Basalt convention/Jacobian/BCC |
| ATLS dynamic rejection | DynaVINS++ | multi-camera residual and square-root solve |
| BCC/SSR divergence recovery | DynaVINS++ | prior/landmark/full transaction rollback |
| NFR global mapping | Basalt | stable-only NFR after BCC |
| inter-camera loop closure | MAVIS | selected-view descriptor policy |
| one-way Occupancy bridge | system design | downstream task accuracy |
| occupancy/SDF tight factor | proposed extension | correlation-safe ablation mandatory |

결론:

> 기존 연구가 각각 검증한 module은 충분한 근거를 갖지만, **6-camera·15~20° ring·Basalt square-root·ATLS/BCC/SSR·exact `SE_2(3)`를 모두 동시에 사용한 조합 자체는 별도 검증 대상**이다. 이 문서는 그 조합이 state와 factor 차원에서 모순되지 않도록 설계한 최종 architecture이며, Section 27의 ablation이 최종 실증 절차다.
