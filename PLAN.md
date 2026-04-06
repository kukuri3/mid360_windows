# MID-360 Windows デモ GUI

Livox MID-360 LiDARセンサーからUDP経由で点群データを受信し、3Dリアルタイム表示するWindowsデモアプリ（Python）

## 技術スタック
- **3D表示**: Open3D（別ウィンドウ）
- **GUI**: tkinter（操作パネル）
- **通信**: socket（UDP直接通信、Livox SDK2プロトコル準拠）
- **ROS依存**: なし
