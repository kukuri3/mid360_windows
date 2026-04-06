# FAST-LIO2 組み込みプラン

## 概要
FAST-LIO2のC++コアをROS依存なしで抽出し、pybind11経由でPythonから呼び出す。
既存のMID-360デモGUIにSLAM機能を統合する。

## アーキテクチャ

```
┌─────────────────────────────────────┐
│  tkinter Control Panel              │
│  ┌─────────┬──────────┬───────────┐ │
│  │ Connect │ Mode:    │ Save Map  │ │
│  │ Scan    │ ○ View   │ Load Map  │ │
│  │         │ ○ Odom   │ Export    │ │
│  │         │ ○ SLAM   │ Reset     │ │
│  └─────────┴──────────┴───────────┘ │
└─────────────────────────────────────┘
         │                    │
         ▼                    ▼
┌─────────────────┐  ┌──────────────────┐
│  Open3D Viewer  │  │  Python Layer    │
│  - Point Cloud  │  │  - main.py       │
│  - Trajectory   │  │  - slam_engine.py│
│  - Map          │  │                  │
└─────────────────┘  └───────┬──────────┘
                             │ pybind11
                     ┌───────▼──────────┐
                     │  C++ Library     │
                     │  fastlio_core.dll│
                     │  - iEKF          │
                     │  - ikd-Tree      │
                     │  - IMU preint    │
                     └──────────────────┘
```

## 動作モード（GUI切替）

| モード | 機能 |
|--------|------|
| **View** | 現行と同じ。リアルタイム点群表示のみ |
| **Odometry** | IMU融合による姿勢・移動量推定。軌跡表示。点群は現フレームのみ |
| **SLAM** | Odometry + グローバルマップ蓄積。3D地図構築・保存 |

## 実装ステップ

### Phase 1: C++コア抽出 & ビルド環境
1. FAST-LIO2リポジトリからROS依存を除去
   - `laserMapping.cpp` → 純粋なC++クラスに分離
   - `IMU_Processing.hpp` → そのまま使用可能（ROS依存少）
   - `ikd-Tree/` → 完全にスタンドアロン
   - PCLへの依存は Eigen + nanoflann に置換 or 最小限のPCL利用
2. CMakeLists.txt作成（ROS不要、Windows対応）
   - 依存: Eigen3, pybind11
   - 出力: `fastlio_core.pyd` (Python拡張モジュール)
3. 必要なサードパーティ:
   - **Eigen3**: 行列演算（ヘッダオンリー）
   - **pybind11**: Python binding
   - **Sophus**: SO3/SE3 Lie群（ヘッダオンリー）

### Phase 2: pybind11 Pythonバインディング
```
fastlio_core モジュール API:
  class FastLIO:
      def __init__(config: dict)
      def process_imu(acc: np.array, gyro: np.array, timestamp: float)
      def process_points(xyz: np.ndarray, timestamp: float) -> dict
          # returns: {
          #   "pose": np.array(4x4),      # 現在のSE3姿勢
          #   "trajectory": np.array(Nx7), # [t,x,y,z,qx,qy,qz,qw]
          #   "map_points": np.array(Mx3), # グローバルマップ（SLAMモード時）
          #   "state": dict                # EKFの状態量
          # }
      def reset()
      def get_map() -> np.ndarray          # 現在のマップ全体
      def save_map(path: str)              # PLY/PCD保存
```

### Phase 3: IMU受信追加
- `mid360/imu_receiver.py` 新規作成
  - ポート56401でIMUデータ受信（別スレッド）
  - data_type=0: gyro_xyz(float x3) + acc_xyz(float x3) = 24B/sample
  - タイムスタンプ付きリングバッファ

### Phase 4: SLAMエンジン統合
- `mid360/slam_engine.py` 新規作成
  - `fastlio_core.FastLIO` をラップ
  - IMU受信スレッドと点群受信スレッドからデータを受け取り
  - タイムスタンプ同期（IMUは高レート200Hz、点群は10Hz）
  - モード切替（View / Odom / SLAM）

### Phase 5: GUI拡張
- **モード切替**: ラジオボタン（View / Odometry / SLAM）
- **軌跡表示**: Open3Dに LineSet で移動軌跡を描画
- **マップ操作**:
  - 「Save Map」ボタン → PLY/PCD形式でエクスポート
  - 「Reset」ボタン → マップ・軌跡クリア
- **状態表示**: 現在のpose (x,y,z,roll,pitch,yaw), 速度

### Phase 6: ビューア拡張
- `mid360/viewer.py` に追加:
  - 軌跡の LineSet 描画（赤線）
  - 現在位置のマーカー（座標フレーム）
  - SLAMモード時：グローバルマップ点群 + 現フレーム点群を色分け

## ファイル構成（追加分）

```
mid360_windows/
├── cpp/                          # C++ソース
│   ├── CMakeLists.txt
│   ├── include/
│   │   ├── fast_lio.h            # メインクラス
│   │   ├── imu_processing.h      # IMU前処理
│   │   ├── state.h               # EKF状態量
│   │   └── ikd_tree/             # ikd-Tree (FAST-LIO2から)
│   ├── src/
│   │   ├── fast_lio.cpp
│   │   ├── imu_processing.cpp
│   │   └── ikd_tree.cpp
│   └── bindings/
│       └── pybind_fastlio.cpp    # pybind11バインディング
├── mid360/
│   ├── imu_receiver.py           # 新規: IMUデータ受信
│   └── slam_engine.py            # 新規: SLAMエンジン統合
└── build_cpp.py                  # C++ビルドスクリプト
```

## ビルド手順（Windows）

```bash
# 前提: Visual Studio 2019/2022 (C++ワークロード), CMake, Python 3.10+
pip install pybind11 numpy

# Eigen3をvcpkgまたは手動インストール
vcpkg install eigen3:x64-windows

# ビルド
cd cpp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . --config Release

# fastlio_core.pyd が mid360_windows/ にコピーされる
```

## リスクと対策

| リスク | 対策 |
|--------|------|
| FAST-LIO2のPCL依存が深い | Eigen直接操作に書き換え、PCL不要化 |
| Windowsでのビルド問題 | vcpkg + CMakeで依存管理 |
| リアルタイム性能 | C++コアは別スレッド、GILリリース付きpybind11 |
| IMUと点群の同期ずれ | タイムスタンプベースの補間（FAST-LIO2のforward/backward propagation） |

## 段階的デリバリー

1. **Phase 1-2** (C++抽出+binding): 最も工数が大きい。先行着手
2. **Phase 3** (IMU受信): 現GUIに先に統合可能。IMU生データ表示
3. **Phase 4-6** (SLAM統合+GUI): Phase 1-2完了後
