
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
