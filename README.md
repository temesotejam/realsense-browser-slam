# RealSense Browser SLAM

Intel RealSense **D435 + T265** を使い、Desktop Chrome / Edge と GitHub Pages だけで動作するブラウザベースの6DoF自己位置推定・再ローカライズ実験です。

現在の基準版は **build 20260925.27** です。

- Latest: https://temesotejam.github.io/realsense-browser-slam/
- Fixed build 27: https://temesotejam.github.io/realsense-browser-slam/build-20260925-27.html?v=20260925.27
- Sensor Hub: https://temesotejam.github.io/realsense-web-viewer/sensor-hub.html

> このプロジェクトでは、T265だけを絶対座標系として使いません。  
> **T265 = 短時間の相対運動・姿勢・整合性確認**、**D435 RGB-D map = 長時間の絶対座標**という役割分担です。

---

## 目的

主な出力はカメラの6DoF位置姿勢です。

- X / Y / Z [m]
- Roll / Pitch / Yaw [deg]
- Tracking state
- ICP inliers / RMSE
- Keyframes
- Loop closure
- Relocalization
- T265 / Depth ICP consistency
- Persistent-map anchor

PC側には以下を要求しません。

- RealSense SDK / librealsense
- Python
- ROS
- ローカルHTTPサーバー
- 常駐ネイティブアプリ

D435とT265をUSB接続し、GitHub Pagesをブラウザで開く構成です。

---

## Current architecture

```text
D435
 ├─ Depth
 │    └─ realsense-web-viewer / Sensor Hub
 │         └─ 320x240 Z16 Depth
 │
 └─ RGB
      └─ Browser UVC
           └─ BRIEF-like visual features

T265
 └─ WebUSB
      └─ ~200 Hz 6DoF pose
           │
           ▼
     relative pose / rotation
     motion guard / ICP seed
     map consistency prior
           │
           ├───────────────┐
           ▼               ▼
     D435 Depth ICP   RGB place identity
           │               │
           └───────┬───────┘
                   ▼
            Persistent keyframes
                   │
            map matching / relock
                   │
                   ▼
             X Y Z / R P Y
```

### D435 Depth

Depthは `realsense-web-viewer` のHeadless Sensor Hubから受信します。

- Browser UVC
- WebGL2 `R32F/FLOAT`
- Z16相当へ復元
- 320×240でSLAMへ配信
- 現在の物理向き補正はSensor Hub側で完了

SLAM側ではSensor HubのDepth配列をそのまま使用し、追加の上下左右反転は行いません。

### D435 RGB

RGBはSLAMページ側でD435のRGB UVCを自動取得します。

主用途は、

- 場所候補探索
- Keyframe identity
- Loop closure確認
- Relocalization確認

です。

metricな短時間移動量の主系統はDepth / T265で、RGB単独で位置を決めません。

### T265

T265は `realsense-web-viewer` のWebUSB経路から取得します。

実機確認済みの経路:

```text
03E7:2150 boot device
      ↓
firmware boot
      ↓
8087:0B37 runtime
      ↓
TM2 WebUSB protocol
      ↓
~200 Hz pose
```

T265から使用する主な値:

- position
- quaternion
- velocity
- angular velocity
- tracker confidence
- mapper confidence

T265座標系はD435 optical frameへ変換してから使用します。

---

## Sensor fusion policy

現在の基本方針は次の通りです。

### Short-term motion

```text
Rotation
  → T265を主

Translation
  → D435 Depth ICPを主
  → ICPが不自然な場合はT265相対移動をguard / bridgeに使用
```

### Long-term absolute coordinates

```text
Persistent RGB-D map
  → 絶対座標の基準

T265
  → map ICPの初期値
  → 候補Keyframeとの近接確認
  → 原点基準のabsolute-map consistency check
```

T265のworld座標そのものをPersistent Map座標として採用しない理由は、T265のworld originがセッション依存であり、長時間ドリフトも存在するためです。

---

## Tracking states

主な状態は以下です。

### `INITIALIZED`

セッション開始直後です。

最初のDepthフレームで最初のKeyframeを作成し、これをTrusted originとして扱います。

### `TRACKING`

通常の追跡状態です。

- T265 relative pose
- D435 Depth ICP
- local keyframe anchor

を使用してPoseを更新します。

### `T265_BRIDGE`

Depth ICPの並進がT265相対移動と大きく矛盾した場合の一時的な橋渡し状態です。

- T265 relative poseで追跡継続
- 新規Keyframeは作らない
- Depth追跡が戻れば `TRACKING` へ復帰

大きな旋回で即LOSTすることを防ぐための状態です。

### `STATIC`

静止判定が成立した状態です。

STATIC中は、

- frame-to-frame Pose積分: **停止**
- 新規Keyframe作成: **停止**
- Persistent Map照合: **継続**

します。

STATIC解除には、T265のSTATIC開始時からの累積移動量を利用します。RGB translation / rotation / scaleはまとめて「RGB 1センサ」として扱い、RGBだけの揺らぎでSTATICが解除されないようにしています。

### `MAP_CONFIRM` / `RECOVERY_CONFIRM`

大きなMap補正を即採用せず、複数回連続で確認している状態です。

T265-assistedな大補正では3回確認を要求する場合があります。

### `MAP_LOCKED`

既存Map Keyframeとの照合が成立し、Persistent Map座標へPoseを補正した状態です。

### `LOST`

通常追跡が成立しない状態です。

Persistent Mapからの再ローカライズを試みます。

### `RELOCALIZED`

LOST後に既存Map Keyframeとの照合が成立した状態です。

---

## Map matching safeguards

誤った場所への大補正を防ぐため、現在は複数条件を組み合わせています。

- RGB identity
- Depth ICP
- forward / reverse ICP consistency
- T265 keyframe-to-current proximity
- T265 / Depth motion consistency
- first keyframe origin prior
- origin-relative absolute T265 map prior
- multi-frame confirmation

特に大きなrelocalization correctionでは、単に「あるKeyframeとのT265相対Poseが小さい」だけでは採用しません。

build 27では、最初のKeyframeのT265 snapshotをセッション内の基準として、

```text
origin T265
    ↓
current T265
    ↓
predicted map pose
```

を作り、Map候補の絶対Poseとの整合性も確認します。

---

## STATIC logic

静止時のDepth ICPは数mm単位で見かけ上動くことがあります。そのため、単純にICPだけでSTATIC判定しません。

現在は以下を組み合わせます。

- T265 cumulative translation / rotation
- Depth geometry change
- ICP motion
- RGB geometry

STATICに入った時点のT265 poseを保存し、その時点からの累積移動量を測定します。

重複Keyframeを防ぐため、T265上で既存Keyframeから概ね **6 cm / 4°以内**の場合は新規Keyframeを抑制します。

---

## Browser Sensor Hub

このSLAMは別リポジトリのSensor Hubを利用します。

Repository:

https://github.com/temesotejam/realsense-web-viewer

Sensor Hub:

https://temesotejam.github.io/realsense-web-viewer/sensor-hub.html

SLAMページ内にSensor Hubをiframeとして埋め込み、同一originの `BroadcastChannel` を使って通信します。

```text
BroadcastChannel("realsense-sensor-api-v1")
```

主な受信イベント:

- `d435_depth`
- `d435_status`
- `t265_pose`
- `t265_status`
- `sensor_api_status`

D435 Depthはブラウザのbackground throttlingを避けるため、Sensor Hubを同じforeground top-level page内に埋め込んで動作させます。

---

## How to use

### 1. Hardware

- Intel RealSense D435
- Intel RealSense T265
- USB 3.x接続推奨

D435とT265は、できるだけ物理的な向きを揃えて固定してください。

現状は両カメラ間の厳密なSE(3) extrinsic calibrationをまだ使用していないため、特に並進には取付位置差の影響が残ります。

### 2. Open

Desktop Chrome / Edgeで以下を開きます。

https://temesotejam.github.io/realsense-browser-slam/

### 3. Permission

ブラウザの仕様上、初回だけは以下の許可が必要になる場合があります。

- Camera permission
- WebUSB / T265 authorization

一度許可済みであれば、通常はSLAMページを開くだけでSensor HubとRGB取得が自動開始します。

### 4. Initial static period

起動直後は数秒間カメラを静止させます。

正常時の目安:

```text
Keyframes = 1
state = STATIC
pose ≈ [0, 0, 0]
rpy  ≈ [0, 0, 0]
```

### 5. Move and return

実験では、

```text
初期位置で静止
  ↓
移動 / 旋回
  ↓
元の位置・向き付近へ戻る
  ↓
数秒静止
```

を推奨します。

戻ったときに、

- `MAP_CONFIRM`
- `MAP_LOCKED`
- `RELOCALIZED`
- `Last map anchor KF`
- `Anchor T265 proximity`
- `T265 absolute-map error`

を確認します。

---

## Current validation status

### Confirmed

- D435 Depthのブラウザ直接取得
- D435 RGBのブラウザ直接取得
- T265 WebUSB direct pose取得
- T265 raw pose 約200 Hz
- Sensor Hub経由のD435 Depth連続配信
- D435 Depth物理向き補正
- Headless Sensor Hub自動起動
- STATIC時のPose安定化
- STATIC中の誤解除抑制
- T265_BRIDGEによる大旋回時の追跡継続
- Persistent keyframe map
- RGB + DepthによるMap照合
- LOST後のrelocalization
- STATIC中のMap relock

### Current experimental baseline

**build 20260925.27**

build 27では初期STATIC動作までは実機確認済みです。大きな移動を複数回繰り返した場合のabsolute T265 map priorによる誤ロック抑制は、引き続き実機評価対象です。

---

## D435 calibration

初期値として640×480向けのD430/D435 legacy intrinsics presetを使います。

```text
fx  = 381.902008056641
fy  = 381.902008056641
ppx = 318.229400634766
ppy = 239.944534301758
```

SLAM内部の320×240では0.5倍した値を使用します。

```text
fx = 190.951
fy = 190.951
cx = 159.114
cy = 119.972
```

Depth scale初期値:

```text
0.001 m / Z16
```

ブラウザだけではlibrealsenseからfactory intrinsicsを直接取得しないため、実機固有の値が分かる場合は置き換えることを推奨します。

---

## Performance

現在の実機ログでは、おおむね以下の範囲で動作しています。

- D435 Depth API: 約8 Hz
- SLAM: 約7–8 Hz
- T265 pose: Sensor Hub側で約200 Hz
- Worker compute: 通常 約5–30 ms
- Map search時: 数十msまで増加する場合あり

Worker処理中に次フレームが来た場合はback-pressureとしてフレームを捨てます。

---

## Diagnostics

画面では主に以下を確認できます。

### Sensor

- Hub API
- D435 Depth
- D435 RGB
- T265 Pose
- T265 confidence
- Depth orientation

### Tracking

- Tracking state
- ICP inliers
- ICP RMSE
- Camera FPS
- SLAM Hz
- Worker compute time
- dropped frames

### T265 fusion

- T265 fusion
- T265 Δ pose
- T265 / ICP disagreement
- T265 retry / bridge
- T265 since STATIC
- Motion sources

### Map

- Keyframes
- Map mode
- Loop closures
- Relocalizations
- Last map anchor KF
- Anchor T265 proximity
- T265 absolute-map error
- STATIC map relock

---

## Important limitations

### 1. D435 / T265 extrinsics

D435とT265の厳密な剛体変換

```text
T_T265_D435
```

はまだ正式にキャリブレーションしていません。

現在は物理取付方向を揃え、T265 translationにはある程度のlever-arm許容値を持たせています。

### 2. Browser RGB / Depth synchronization

RGBとDepthの厳密なhardware synchronizationは、librealsenseほど制御できません。

そのため、

- Depth / T265 = metric tracking
- RGB = place identity

という使い分けにしています。

### 3. T265 world origin

T265のworld originはセッション依存です。

そのため保存Mapに含まれる過去セッションのT265 snapshotは、Mapを再ロードした別セッションで初期値として再利用しません。

### 4. Map optimization

現在のMap correctionは軽量実装です。

ORB-SLAM3等のような完全な、

- pose graph optimization
- bundle adjustment
- tightly coupled visual-inertial optimization

ではありません。

### 5. Browser-only experimental system

このプロジェクトはGitHub Pages単体でRealSense SLAMがどこまで成立するかを検証する実験実装です。安全用途・計測保証用途を目的としたものではありません。

---

## Development history

主要なbuildだけを記載します。

| Build | Main change |
|---|---|
| 21 | D435 + T265 Sensor Hub統合 |
| 22 | T265 relative-pose guard / `T265_BRIDGE` |
| 23 | T265-seeded map matching |
| 24 | near-keyframe T265 map identity |
| 25 | cumulative T265 STATIC gate / duplicate keyframe抑制 |
| 26 | STATIC中のpersistent-map relock |
| **27** | **origin-relative absolute T265 map prior** |

build 20以前では、D435単体のDepth ICP / RGB persistent mapを中心に開発しました。STATIC driftは大きく改善しましたが、大旋回時のDepth ICPが弱かったため、build 21以降でT265を統合しています。

---

## Next candidates

現時点で優先度が高い候補です。

- build 27の複数往復実機評価
- D435↔T265 extrinsic calibration
- keyframe生成条件の追加評価
- absolute T265 priorの閾値調整
- saved map再読込時の長期relocalization評価
- full SE(3) pose graph optimization
- optional WASM backend

---

## Related project

### RealSense Web Viewer / Sensor Hub

https://github.com/temesotejam/realsense-web-viewer

https://temesotejam.github.io/realsense-web-viewer/

D435 Depthのブラウザ直接取得と、T265 WebUSB direct pose取得を担当します。
