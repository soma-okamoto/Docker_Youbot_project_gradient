# 観測融合ノード 回帰テスト結果報告書

実行日時: 2026-09-25T15:52:59+09:00（日本時間）

## 結果

19件中19件成功、失敗0件、エラー0件、スキップ0件。unittestの実行時間は0.086秒。19件はテストメソッド数であり、入れ替え・不正値などのsubTestは別件として加算していない。

対象: `observation_fusion_node_new.py`。実装は現在の機能で一旦完成とし、本資料作成に伴う動作変更は行っていない。

## 現行仕様

| 条件 | 採用結果 |
|---|---|
| 両センサに有効候補があり、整合する組がある | 最も整合する組をCI融合する。 |
| 整合する組がなく、選択候補の不確かさが異なる | 共分散のtraceが小さい側を採用する。 |
| 片側だけに有効候補がある | その観測を採用する。 |
| 両側に有効候補がない | 事前分布を採用する。 |
| 競合し、選択候補の不確かさが同等 | 事前分布を採用する。 |

- 安定性は共分散が小さいほど高い。棄却には最大主軸標準偏差、競合時の比較にはXYZ分散の合計traceを使う。
- 距離判定は共分散を考慮したマハラノビス距離の二乗で、既定の閾値は7.815。固定の「何cm以内」という判定ではない。
- 不確かさが同等で競合する場合も事前へ戻す。この挙動を含む現在の実装を完成版として記録する。
- Metaの位置のみの入力は固定共分散を使用する。観測ごとの変動を反映するには共分散付き配列を送る。時間的な安定性の推定はこのノードでは行わない。

棄却の既定上限: `max_std_yolo = max_std_meta = 0.10` m。上限超過を棄却し、上限と等しい場合は許容する。共分散の単位は m²。

## テスト方法・環境

- Python: 3.8.10 / NumPy: 1.24.4
- OS: Linux-5.15.0-139-generic-x86_64-with-glibc2.29
- フレームワーク: Python標準のunittest、unittest.mock。
- 共通の事前位置: [5, 0, 0] m。事前共分散: 0.0025 I m²。通常の試験共分散はσ² I。
- テスト中に対象コード・テスト・設定ファイルが変化していないことをSHA-256で確認。

## 全19件の確認内容

| ID | 確認内容 | 入力・条件 | 確認した期待結果 | 結果 |
|---|---|---|---|---|
| T01 | 競合時の安定性による選択 | 位置差1 m。標準偏差2 cmと6 cmをYOLO・Metaで入れ替える。旧優先設定は逆側に指定。 | 両方向とも2 cm側の位置・共分散を採用。旧固定優先設定は影響しない。 | PASS |
| T02 | 整合する観測の融合 | 位置x=0 m／0.01 m、両方の標準偏差2 cm。 | both_valid。出力x=0.005 m、共分散は0.0004 I m²。 | PASS |
| T03 | 同等の不確かさで競合 | 位置差1 m、両方の標準偏差2 cm。 | 同等不確かさの専用status。事前位置[5,0,0] m・事前共分散を出力。 | PASS |
| T04 | 位置一致でも不安定候補を棄却 | 両者の位置は一致。片方の標準偏差20 cm、もう片方2 cm。両センサで入れ替え。 | 安定側のみ採用。不安定側の候補index=-1、score=0。 | PASS |
| T05 | 両方不安定なら事前を採用 | YOLOの標準偏差20 cm、Metaは30 cm。 | no_valid_observation。事前位置を採用し、両側index=-1。 | PASS |
| T06 | 単独観測が不安定 | YOLOのみ受信、標準偏差20 cm。 | no_valid_observation。 | PASS |
| T07 | 観測なし | YOLO・Metaとも候補なし。 | no_observation。 | PASS |
| T08 | 不正共分散と有効候補の混在 | NaN、Inf、ゼロ、負の対角成分、正の対角成分を持つ非PSD行列の各候補と、有効候補を混在。 | 有効なYOLO候補index=1を採用。不正候補のscore=0。 | PASS |
| T09 | 棄却候補の再採用防止 | 位置が完全一致する不安定候補と、1 cmずれた安定候補を混在。両センサで入れ替え。 | 安定候補同士で融合。棄却候補を選ばず、出力x=0.005 m。 | PASS |
| T10 | 相関を含む主軸分散の評価 | 対角分散は0.006,0.006,0.001 m²、XY相関成分は0.005 m²。最大固有値0.011 m²。 | 対角成分だけなら上限以下でも、最大主軸標準偏差が10 cmを超えるため棄却。 | PASS |
| T11 | 上限境界とセンサ別設定 | 上限10 cmに対して標準偏差10 cm。別ケースで上限YOLO=3 cm／Meta=7 cm、入力4 cm／6 cm。 | 上限と等しい観測は採用。個別上限ではYOLOを棄却しMetaを採用。 | PASS |
| T12 | 不正な上限設定の拒否 | max_std_yolo／max_std_metaそれぞれに0、負数、NaN、Infを指定。 | すべてValueErrorを発生させる。 | PASS |
| T13 | Metaの受信共分散を選択に使用 | Metaを12要素形式で受信。Metaの標準偏差1 cm、YOLOは6 cm、位置差1 m。 | Metaを採用。出力共分散はMetaが送った0.0001 I m²。 | PASS |
| T14 | 不正なYOLO共分散の棄却 | YOLO共分散がNaN。Metaは位置のみの有効入力。 | 固定共分散への置き換えでYOLOを復活させず、meta_only。 | PASS |
| T15 | 既存のMeta位置配列との互換性 | YOLO無効、MetaのXYZのみの配列を受信。 | meta_only。出力共分散が設定済みの固定Meta共分散と一致。 | PASS |
| T16 | 既存のMeta PoseStampedとの互換性 | YOLO無効、正しいframeのPoseStampedを受信。 | meta_onlyとなることを確認。 | PASS |
| T17 | Meta streamの候補・共分散対応 | NaN位置を含む入力と追加入力。候補上限2件、異なる共分散を付与。 | 有効位置と共分散の対応・件数を維持。早期確定せず、timeoutコールバックで確定し共分散リストを消去。 | PASS |
| T18 | 不正長・空のMeta配列 | 12要素設定に対し3要素を送信。その後、空配列を送信。 | 不正長では受信済みにしない。空配列ではno_valid_observation。 | PASS |
| T19 | Meta共分散入力設定の検証 | stride範囲外のindex、重複index、PoseStampedへの共分散index指定。 | 不正な3種類の設定でValueErrorを発生させる。 | PASS |

## 代表的な数値例

以下はテスト内で照合した期待値であり、独立した実機計測ログではない。位置・標準偏差の単位はm、共分散の単位はm²。Iは3×3単位行列。

| 条件 | 入力 | テストで照合した出力 | 対応 |
|---|---|---|---|
| 融合 | YOLO x=0、Meta x=0.01、両方σ=0.02 | x=0.005、Σ=0.0004 I | T02 |
| YOLOが安定 | 位置差1、YOLO σ=0.02、Meta σ=0.06 | YOLOを採用、Σ=0.0004 I | T01 |
| Metaが安定 | 位置差1、YOLO σ=0.06、Meta σ=0.02 | Metaを採用、Σ=0.0004 I | T01 |
| 両方不安定 | YOLO σ=0.20、Meta σ=0.30 | 事前位置[5,0,0]を採用 | T05 |
| 同等で競合 | 位置差1、両方σ=0.02 | 事前位置[5,0,0]、Σ=0.0025 I | T03 |

## 検証範囲と限界

- ROSのPublisher、Subscriber、ログ出力、および最終_publish処理を模擬している。ノードのPython処理と出力引数を検証した結果である。
- ROS master、実際のトピック通信、メッセージのシリアライズ、実タイマの発火・並行実行、センサ接続、実機動作は検証していない。stream試験はtimeoutコールバックを直接呼んでいる。
- 入力は合成データ。実環境の測位精度、共分散の推定精度、10 cmという暫定上限の妥当性は評価していない。
- テストはパラメータを模擬してノードを初期化する。実際のlaunch／YAMLを読み込む起動試験ではない。コードカバレッジ率・性能・長時間運用の測定は含まない。
- 全19件成功は、列挙した条件の期待結果との一致を示す。未試験条件や実機品質の保証を示すものではない。

## 再実行方法

```bash
python3 -B -m unittest discover \
  -s /home/dars/Docker_ws/Docker_Youbot_project_gradient/catkin_ws/src/place_estimation/tests \
  -p test_observation_fusion_stability.py -v
```

コマンドはテスト対象の実ファイルを読み込む。ROS環境は不要だがNumPyは必要。

## 対象バージョンと証跡

パッケージ: `/home/dars/Docker_ws/Docker_Youbot_project_gradient/catkin_ws/src/place_estimation`

Git HEAD: `9e34c96fb1e65b05618f2805269333b17e075e69`

HEADに未コミットの変更を含むため、試験対象の特定には下記のファイルハッシュを使用する。

| ファイル | SHA-256 |
|---|---|
| `scripts/observation_fusion_node_new.py` | `182cf12ec9795d4b434798c9e85f6cdb6f553b102eb249fe236db2993e32faa4` |
| `tests/test_observation_fusion_stability.py` | `5bf62703d5b34ee2c7fe88136492ead3df65401d718ca7b2b3b28ef201ea55f0` |
| `config/observation_fusion_multi_sensor.yaml` | `9c519a9eb2280119c6ca1b42856cc641e6f88b8a0c9337b09e4ce657e8cd3fb4` |

実行時のGit状態:

```text
 M config/observation_fusion_multi_sensor.yaml
 M scripts/observation_fusion_node_new.py
?? ../esaki_youbot_project_gradient/docs/
?? tests/
```

同梱証跡: `test_run.log`（unittest出力全文）、`test_results.json`（実行日時・環境・コマンド・ファイルハッシュ）。

## テストIDとメソッド名の対応

| ID | unittestメソッド名 |
|---|---|
| T01 | `test_mismatch_uses_more_stable_sensor_in_both_directions` |
| T02 | `test_consistent_observations_still_fuse` |
| T03 | `test_equal_uncertainty_conflict_uses_prior` |
| T04 | `test_large_covariance_rejected_even_when_positions_agree` |
| T05 | `test_both_unstable_use_prior` |
| T06 | `test_single_unstable_uses_prior` |
| T07 | `test_no_observation_uses_prior` |
| T08 | `test_invalid_covariance_does_not_hide_valid_candidate` |
| T09 | `test_rejected_candidate_cannot_reenter_pair_search` |
| T10 | `test_gate_uses_principal_variance_including_correlations` |
| T11 | `test_threshold_boundary_and_sensor_specific_configuration` |
| T12 | `test_invalid_threshold_configuration_is_rejected` |
| T13 | `test_meta_covariance_input_affects_actual_selection` |
| T14 | `test_invalid_yolo_input_covariance_is_not_replaced_by_fixed_covariance` |
| T15 | `test_legacy_meta_xyz_input_uses_fixed_covariance` |
| T16 | `test_legacy_meta_pose_input_uses_fixed_covariance` |
| T17 | `test_meta_stream_keeps_covariances_aligned_and_truncates_together` |
| T18 | `test_bad_or_empty_meta_covariance_messages` |
| T19 | `test_meta_covariance_configuration_is_validated` |

## 実行ログ

```text
test_bad_or_empty_meta_covariance_messages (test_observation_fusion_stability.StabilityTests) ... ok
test_both_unstable_use_prior (test_observation_fusion_stability.StabilityTests) ... ok
test_consistent_observations_still_fuse (test_observation_fusion_stability.StabilityTests) ... ok
test_equal_uncertainty_conflict_uses_prior (test_observation_fusion_stability.StabilityTests) ... ok
test_gate_uses_principal_variance_including_correlations (test_observation_fusion_stability.StabilityTests) ... ok
test_invalid_covariance_does_not_hide_valid_candidate (test_observation_fusion_stability.StabilityTests) ... ok
test_invalid_threshold_configuration_is_rejected (test_observation_fusion_stability.StabilityTests) ... ok
test_invalid_yolo_input_covariance_is_not_replaced_by_fixed_covariance (test_observation_fusion_stability.StabilityTests) ... ok
test_large_covariance_rejected_even_when_positions_agree (test_observation_fusion_stability.StabilityTests) ... ok
test_legacy_meta_pose_input_uses_fixed_covariance (test_observation_fusion_stability.StabilityTests) ... ok
test_legacy_meta_xyz_input_uses_fixed_covariance (test_observation_fusion_stability.StabilityTests) ... ok
test_meta_covariance_configuration_is_validated (test_observation_fusion_stability.StabilityTests) ... ok
test_meta_covariance_input_affects_actual_selection (test_observation_fusion_stability.StabilityTests) ... ok
test_meta_stream_keeps_covariances_aligned_and_truncates_together (test_observation_fusion_stability.StabilityTests) ... ok
test_mismatch_uses_more_stable_sensor_in_both_directions (test_observation_fusion_stability.StabilityTests) ... ok
test_no_observation_uses_prior (test_observation_fusion_stability.StabilityTests) ... ok
test_rejected_candidate_cannot_reenter_pair_search (test_observation_fusion_stability.StabilityTests) ... ok
test_single_unstable_uses_prior (test_observation_fusion_stability.StabilityTests) ... ok
test_threshold_boundary_and_sensor_specific_configuration (test_observation_fusion_stability.StabilityTests) ... ok

----------------------------------------------------------------------
Ran 19 tests in 0.086s

OK
```
