# MID-360 Windows デモ GUI 実装プラン

## 概要
Livox MID-360 LiDARセンサーからUDP経由で点群データを受信し、3Dリアルタイム表示するWindowsデモアプリ（Python）

## 技術スタック
- **3D表示**: Open3D（別ウィンドウ）
- **GUI**: tkinter（操作パネル）
- **通信**: socket（UDP直接通信、Livox SDK2プロトコル準拠）
- **ROS依存**: なし

## MID-360 通信プロトコル（SDK2）
| 機能 | UDPポート |
|---|---|
| デバイス検出 | 56000 |
| コマンド | 56100 |
| プッシュメッセージ | 56200 |
| 点群データ | 56300 |
| IMUデータ | 56400 |

## ファイル構成
```
mid360_windows/
├── requirements.txt          # 依存パッケージ（open3d）
├── main.py                   # エントリーポイント
├── mid360/
│   ├── __init__.py
│   ├── protocol.py           # SDK2プロトコル定義（パケット構造体、定数）
│   ├── connection.py         # デバイス検出・接続・切断・コマンド送受信
│   ├── pointcloud_receiver.py # 点群UDPデータ受信・パース（スレッド）
│   └── viewer.py             # Open3D 3Dビューア（非ブロッキング更新）
└── gui/
    ├── __init__.py
    └── control_panel.py      # tkinter操作パネル（接続/切断、ステータス）
```

## 実装ステップ

### Step 1: プロトコル定義 (`mid360/protocol.py`)
- SDK2パケットヘッダー構造体（magic, version, length, cmd_type, etc.）
- 点群データフレームのパース関数
- 各コマンドID定数（Handshake, Heartbeat, SamplingStart/Stop, etc.）
- 点群データ形式: x,y,z (int32, mm単位) + reflectivity (uint8) + tag (uint8)

### Step 2: デバイス接続管理 (`mid360/connection.py`)
- UDP broadcast送信でデバイス検出（ポート56000）
- Handshakeコマンド送信（ポート56100）
- Heartbeat定期送信（スレッド）
- サンプリング開始/停止コマンド
- 接続状態管理

### Step 3: 点群受信 (`mid360/pointcloud_receiver.py`)
- UDPポート56300でリッスン（受信スレッド）
- SDK2フレームをパースしてNumPy配列（Nx3 float）に変換
- スレッドセーフなバッファで最新フレームを保持

### Step 4: 3Dビューア (`mid360/viewer.py`)
- Open3Dの非ブロッキングビジュアライザ使用
- 定期的にバッファから最新点群を取得して描画更新
- 座標軸表示、視点操作はOpen3D標準機能を利用

### Step 5: GUI操作パネル (`gui/control_panel.py`)
- tkinterウィンドウ
  - センサーIPアドレス入力欄（デフォルト: 192.168.1.1xx）
  - 「接続」「切断」ボタン
  - 接続ステータス表示（ラベル）
  - 受信点数/FPS表示
- Open3Dビューアの起動/停止制御

### Step 6: メインスクリプト (`main.py`)
- 各コンポーネントの初期化・統合
- tkinterメインループとOpen3D更新の共存（tkinter.after()で定期更新）

## 注意事項
- MID-360のデフォルトIPは `192.168.1.1xx`（xxはSN末尾2桁）
- PCのNICは同じサブネット（192.168.1.x）に設定が必要
- Open3Dのビジュアライザは `create_window` + `poll_events` で非ブロッキング動作させる
