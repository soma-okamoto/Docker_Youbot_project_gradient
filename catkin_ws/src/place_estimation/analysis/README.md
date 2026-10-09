# 予備実験データ収集・解析

真値を使わず、システム内部の再現性、整合度、学習収束を確認するための
予備実験用ツールです。ここで出力される値はConfigの候補であり、
本実験用の値を自動的に確定するものではありません。

## 1. 記録

全体パイプラインとrosbag記録を同時に開始します。

```bash
roslaunch place_estimation preliminary_experiment.launch \
  output_bag:=/保存先/preliminary_01
```

端末に`Recording to '...bag'`と表示されたことを確認してから、最初の
Place操作を開始してください。開始直後に入力すると、rosbagの購読接続前の
1回だけのメッセージが記録されない可能性があります。

人の操作に合わせて任意回数のPlace試行を行い、終了時に`Ctrl-C`を押します。
試行数はlaunchやスクリプトで事前指定しません。

## 2. 解析

catkinワークスペースをビルド・sourceした後、次を実行します。

```bash
rosrun place_estimation preliminary_experiment_analyzer.py \
  /保存先/preliminary_01.bag \
  /保存先/preliminary_01_analysis
```

ソースツリーから直接実行する場合は次です。

```bash
python3 analysis/preliminary_experiment_analyzer.py \
  /保存先/preliminary_01.bag \
  /保存先/preliminary_01_analysis
```

## 3. 出力

```text
preliminary_01_analysis/
├── trials.csv
├── learning_history.csv
└── parameter_report.json
```

### `trials.csv`

試行IDごとに以下をまとめます。

- `P_current`、`P_tf`
- 事前位置・共分散
- `P_place`・`Sigma_place`
- 採用物理観測・共分散
- 観測分岐
- YOLO、Meta、センサ間のマハラノビス距離
- KL、平均位置差、累積平均、EMA
- 入力候補数・メッセージ数

### `learning_history.csv`

物理観測を採用した更新ごとに以下を保存します。

- operation ID
- 学習サンプル数
- `P_current`側バイアス・共分散
- `P_tf`側バイアス・共分散

### `parameter_report.json`

次を集計します。

- 距離・KL・平均位置差の平均、標準偏差、百分位点
- `P_current/P_tf - 採用物理観測`の平均と標本共分散
- 候補・採用観測の最大主軸標準偏差分布
- 学習共分散固有値と`min_variance`下限到達回数
- バイアス更新が安定した可能性のあるサンプル数
- ゲートや共分散上限のP95診断候補
- 試行数に対する主要トピックの記録数（`data_completeness`）

## 4. 解釈上の注意

- 真値を使わないため、絶対位置精度ではなく再現性・内部整合性の評価です。
- P95候補は現在のConfigと候補選択の影響を受けます。
- 予備実験用データで値を決定し、本実験前にConfigを固定してください。
- `min_variance`へ頻繁に到達する場合は、共分散下限や学習開始数を
  見直してください。
