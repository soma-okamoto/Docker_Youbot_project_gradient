# Legacy implementations

このディレクトリには、現在の通常パイプラインでは使用しない旧版を
比較・履歴参照のために保存しています。

## 保存している旧版

- `scripts/observation_fusion_node.py`
  - 初期の単純な観測融合実装です。
- `scripts/observation_fusion_node_multi_sensor.py`
  - 複数センサ対応の旧実装です。
- `config/observation_fusion.yaml`
  - 旧融合実装向けの設定です。

## 現行版

通常起動で使用する現行版は次です。

- `../scripts/observation_fusion_node_new.py`
- `../config/observation_fusion_multi_sensor.yaml`
- `../config/local_registration_thresholds.yaml`
- `../launch/place_estimation_pipeline.launch`

`legacy/`内のスクリプトは`CMakeLists.txt`のインストール対象ではなく、
通常のlaunchからも参照されません。旧版を直接起動しないでください。
