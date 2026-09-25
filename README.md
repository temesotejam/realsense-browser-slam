## Build 20260925.27 — absolute T265 map prior

Build 27 addresses two issues observed in the build-26 hardware log:

- STATIC map confirmation counters were being cleared on every frozen frame, so a 3-frame T265-assisted confirmation could never complete while stationary.
- LOST recovery could still accept a globally wrong keyframe when local RGB/T265 agreement looked plausible.

Changes:

- pending map confirmations now persist across STATIC frames;
- the first keyframe's T265 snapshot defines a session-local origin-relative pose prior;
- every map candidate is compared against that absolute T265 map prior;
- very large recovery corrections require RGB identity, Depth/T265 agreement, keyframe proximity, and tight absolute T265-map consistency;
- large T265-assisted normal map locks also require absolute map consistency;
- diagnostics expose the accepted anchor's absolute T265-map error.

The persistent RGB-D map remains the authoritative coordinate system; T265 remains a prior/consistency source rather than the final map pose.

## Build 20260925.26 — STATIC map relock

Build 26 fixes a structural issue found in the build-25 hardware log: while STATIC was latched, odometry was frozen correctly, but persistent-map matching was also disabled. That meant the camera could return to a known place, stop, and remain frozen at the last odometry pose without another opportunity to relocalize.

Changes:

- STATIC still freezes frame-to-frame pose integration;
- STATIC still blocks new keyframes;
- persistent-map matching now continues at low rate while STATIC;
- accepted STATIC map matches can update the pose and trusted anchor;
- diagnostics expose STATIC map checks / locks;
- build-25 cumulative T265 motion gating and duplicate-keyframe suppression remain active.

## Build 20260925.25 — cumulative T265 STATIC gate

Build 25 addresses the build-24 hardware log where the camera was physically stationary but RGB geometry alone repeatedly released the STATIC latch and created a duplicate start-location keyframe.

Changes:

- STATIC stores the T265 pose at latch time and measures cumulative translation/rotation from that pose;
- RGB translation, rotation and scale are treated as one RGB motion source, not three independent votes;
- ICP/RGB jitter cannot release STATIC while cumulative T265 still says the camera is stationary unless Depth geometry also corroborates motion;
- slow real motion can still release STATIC because cumulative T265 and Depth-from-latch grow over time;
- keyframe creation is suppressed when T265 says an existing keyframe is within 6 cm and 4 degrees;
- diagnostics expose T265 motion since STATIC and the independent motion-source votes.

The map-matching safeguards from build 24 remain unchanged.

## Build 20260925.24 — near-keyframe T265 map identity

Build 24 tightens map locking after the build-23 hardware log showed a large correction could be accepted against the wrong keyframe.

Changes:

- T265 keyframe-to-current distance/rotation is used to rank map candidates before ICP;
- the original trusted origin keyframe is forced into the candidate set when T265 says the camera is physically near it;
- T265 may initialize ICP, but it may no longer relax place identity by itself;
- large T265-assisted corrections require RGB identity, Depth agreement, T265/Depth consistency, and physical proximity to the candidate keyframe;
- T265-assisted large corrections require three consecutive confirmations instead of two;
- diagnostics expose the accepted anchor keyframe and its T265 proximity;
- T265 snapshots stored in saved map files are not reused after loading because the T265 world origin is session-local.

## Build 20260925.23 — T265-seeded map matching

Build 23 extends the T265 integration from short-term tracking into map verification:

- every newly-created keyframe stores the contemporaneous T265 pose snapshot;
- local keyframe ICP and global map-match ICP can initialize from the T265 keyframe-to-current relative pose instead of identity;
- large map corrections can be accepted with moderate RGB evidence when Depth ICP and T265 agree;
- the starting keyframe is still the trusted map origin;
- initial STATIC can snap back to the origin keyframe when T265 confirms the camera remained within 5 cm / 3 degrees.

T265 is still not used as the persistent map coordinate system. It supplies relative motion and a consistency check; RGB+Depth map locks remain authoritative for long-term coordinates.

## Build 20260925.22 — T265 relative-pose guard / bridge

Build 22 addresses the first build-21 hardware log:

- the first map keyframe is created immediately, before the initial STATIC latch, so the starting location is the trusted map origin;
- T265 relative XYZ is used only as a short-term frame-to-frame motion guard, not as the persistent map coordinate system;
- if Depth ICP translation disagrees strongly with T265 during a large turn, the tracker temporarily enters `T265_BRIDGE` instead of immediately entering LOST;
- no new keyframes are created during `T265_BRIDGE`;
- the large-rotation sanity gate is relaxed only for verified T265 bridge motion;
- Pause → Resume no longer resets the tracking session or forces map recovery.

Persistent RGB+Depth map locking remains authoritative for long-term absolute coordinates.


## Build 20260925.21 — D435 + T265 Sensor Hub fusion

The current build embeds the headless `realsense-web-viewer/sensor-hub.html` and automatically consumes:

- corrected 320×240 D435 Z16 Depth from the Sensor Hub,
- T265 6DoF pose over WebUSB,
- D435 RGB through a separate browser UVC stream for persistent-map feature matching.

T265 absolute XYZ is not used in this first fusion stage. Instead, relative T265 orientation between Depth frames is converted into the D435 optical basis and used as the primary odometry rotation. D435 projective ICP still estimates metric translation, while persistent RGB+Depth map locking remains the long-term absolute-coordinate authority.

The first map keyframe is now marked as the trusted origin immediately, fixing the build-20 case where a fresh session could create recent drifting keyframes before any globally trusted anchor existed.

# D435 Browser SLAM

Intel RealSense **D435を主対象**に、GitHub Pagesだけで6DoF位置姿勢を推定する実験プロジェクトです。

## 目的

欲しい出力は地図そのものではなく、D435の移動です。

- X / Y / Z [m]
- Roll / Pitch / Yaw [deg]
- Tracking quality
- ICP RMSE / inliers
- Loop closure / relocalization状態

PC側に以下は要求しません。

- RealSense SDK / librealsense
- Python
- ROS
- ローカルHTTPサーバー
- 常駐ネイティブアプリ

D435をUSB接続し、GitHub PagesをChrome/Edgeで開くことを前提にしています。

## v0.2 architecture

```text
D435
 ├─ Depth UVC ── getUserMedia
 │                  │
 │                  ▼
 │              WebGL2 R32F
 │                  │
 │          ┌───────┴────────┐
 │          │                │
 │      live view       320×240 depth
 │                           │
 │                           ▼
 │                     Web Worker
 │                           │
 │                  multi-scale ICP
 │                           │
 │                  local 6DoF tracking
 │                           │
 └─ RGB UVC (optional) ──────┤
                             │
                    keyframes / BRIEF-like
                    place recognition
                             │
                    loop candidate
                             │
                    depth ICP verification
                             │
                   trajectory correction
                             │
                    X Y Z / R P Y
```

### Tracking

- 640×480 Depth表示
- SLAM用DepthはGPUで320×240へ縮小
- Worker内でmulti-scale projective point-to-plane ICP
- constant-velocity initial guess
- UI threadとSLAM計算を分離
- Workerが処理中なら次フレームを捨てるback-pressure方式

### SLAM robustness

単純なframe-to-frame odometryだけではありません。

- keyframe保持
- Depth descriptorによる候補探索
- RGB入力が利用できる場合は軽量BRIEF-like descriptorによる場所候補探索
- loop候補をDepth ICPで幾何検証
- loop成立時に累積軌跡へdrift correction
- tracking lost時のkeyframe relocalization

現段階のloop correctionは、完全な非線形pose graph optimizerではなく、検出されたloop誤差を該当区間へ分散する軽量補正です。GitHub Pages単体での成立性を先に確認するための設計です。

## D435 calibration

初期値として、640×480向けのD430/D435 legacy intrinsics presetを使います。

```text
fx  = 381.902008056641
fy  = 381.902008056641
ppx = 318.229400634766
ppy = 239.944534301758
```

ブラウザだけではlibrealsenseからfactory intrinsicsを取得できないため、**実機固有値を入力できる場合は置き換えることを推奨**します。

Depth scaleの初期値は `0.001 m / Z16` です。これも実機設定に合わせて変更可能です。

## Performance strategy

30 fpsカメラ入力を全部SLAMへ入れません。

- Camera: 30 fps前後
- Depth preview: camera rate
- SLAM: 15 Hz target（変更可）
- loop search: keyframe追加時のみ
- RGB feature extraction: keyframe / relocalization時のみ

ページには以下を常時表示します。

- Camera FPS
- SLAM Hz
- Worker compute time
- dropped frames
- ICP inliers / RMSE
- keyframes
- loop closures
- relocalizations

**Browser benchmark**ボタンで、実機を動かす前にその端末上でsynthetic 320×240 ICPの処理時間を測れます。

## How to use

1. D435をUSB 3.xで接続。
2. GitHub PagesをDesktop Chrome/Edgeで開く。
3. `Refresh USB`。
4. `Depth input`にRealSense Depthを選択。
5. RGB候補が見える場合はRGBも選択。RGBがなくてもDepth-onlyで動作します。
6. `Start`。
7. 最初はD435を静止し、その後ゆっくり移動。
8. `X/Y/Z` と `Roll/Pitch/Yaw`、RMSE、inliersを確認。

## Important limitations

- D435にはIMUがないため、急激な回転や形状の乏しい場面ではDepth trackingが不安定になり得ます。
- RGB/Depthの厳密なhardware synchronizationは、ブラウザUVC経路ではRealSense SDKほど制御できません。そのためRGBは主にplace recognitionへ使い、metric trackingはDepthを主系統にしています。
- loop closureはv0.2では軽量実装です。ORB-SLAM3等と同等の成熟度を意味しません。
- 完全なpose-graph optimization / bundle adjustmentは次段階です。

## Development milestones

- [x] Browser-direct D435 Depth input
- [x] GPU display + GPU downsample
- [x] Worker multi-scale Depth ICP
- [x] 6DoF pose output
- [x] keyframes
- [x] RGB-assisted place candidate search
- [x] Depth-ICP loop verification
- [x] lightweight loop drift correction
- [x] relocalization path
- [x] browser-side benchmark / diagnostics
- [ ] D435実機のstationary drift検証
- [ ] 並進・回転の実測誤差評価
- [ ] exact per-device calibrationの入力手順確立
- [ ] loop false-positive耐性の実機調整
- [ ] full SE(3) pose graph optimizer
- [ ] optional WASM backend

## Related project

Depthをブラウザから直接取得できることの先行検証:

https://github.com/temesotejam/realsense-web-viewer
