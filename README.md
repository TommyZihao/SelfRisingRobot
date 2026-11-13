# SelfRisingRobot

2軸サーボの起き上がりロボット "poco" です。

MuJoCo上で強化学習した起き上がり動作を、M5Atom搭載の実機ロボットで動かします。

詳細記事:

https://homemadegarbage.com/rl13

## Contents

- `3Dmodel/` - 実機ロボット用の3Dプリントモデル
- `RL/` - MuJoCoモデル、強化学習環境、学習済みPPOモデル
- `Arduino/` - M5Atom用スケッチとエクスポート済みポリシーネットワーク

各フォルダ内の詳しい使い方は、それぞれのREADMEを参照してください。

## 3D Model

`3Dmodel/` には実機製作用のSTLファイルを置いています。

- `footP.stl`
- `arm1P.stl`
- `arm2P.stl`
- `armhornP.stl`

## Reinforcement Learning

`RL/` にはシミュレーションと学習済みモデルを置いています。

主な内容:

- MuJoCoモデル
- Gymnasium環境
- Stable-Baselines3 PPOの学習済みモデル
- 再生・評価スクリプト

## Arduino

`Arduino/` には実機制御用のM5Atomスケッチを置いています。

主な内容:

- `robo03.ino`
- `policy_network.h`

`policy_network.h` は学習済みポリシーをCヘッダ化したもので、M5Atom上で起き上がり動作を実行するために使います。

## RoBoBo ATOM S3R 4.1.2-PCB

[同济子豪兄的4.1.2固件、模型、实验报告与视频](zihao_s3r_v412_pcb/README.md)：优化右侧翻和大幅前进/后退，包含电脑端MuJoCo姿态查看器。仿真结果与真机验证范围详见说明。
