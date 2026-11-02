# ボトルIDの引き継ぎ（今回の変更範囲）

Unityの `P_cerrentPublisher.PublishCurrent(id, position)` は既に
`/P_current: std_msgs/Float32MultiArray [bottle_id,x,y,z]` を送っている。
この送信形式はそのまま使う。IDを同時刻に別トピックへ配信する必要はない。
既存のQR・Meta/RealSense検出・座標変換・CI/学習計算は変更しない。
新しいrequest_idや操作シーケンスIDは追加しない。

## ROS側の流れ

```
/P_current [bottle_id,x,y,z] + /P_tf
    → prior_distribution_node.py
    → /P_pred_with_id（ID＋事前位置＋共分散）
    → observation_fusion_node_new.py（既存の候補選択・融合）
    → /P_place_with_id（同じID＋補正位置＋共分散）
    → Unityで対応するボトルへ適用
```

同じIDのボトルを繰り返し操作できる。IDはBottleIdentityであり、YOLOクラスID、
候補index、内部operation_idではない。既存の内部operation_idと学習用Headerは維持する。

型は両トピックとも `place_estimation/BottleDistribution`。
正式定義は `msg/BottleDistribution.msg`:

```text
std_msgs/Header header
int32 bottle_id
geometry_msgs/Point position
float64[9] covariance
```

ID・位置・共分散が1メッセージにまとまるため、別々の到着順で対応付けない。
Headerと位置の座標系は従来の事前分布・結果と同じ。
covarianceはrow-major 3×3、位置はm、共分散はm²。
有効観測がない場合やtimeout、融合失敗による事前分布へのfallbackでも同じIDを返す。
新priorで前操作を確定する既存動作でも、前操作の結果には前操作のIDを付ける。

既存の `/P_pred`、`/Sigma_pred`、`/P_place`、`/Sigma_place` も従来の型で配信する。
可視化・評価の購読先を変更する必要はない。
ID有効時のfusionは `/P_pred_with_id` だけを事前入力として購読する。
同じ事前分布が新旧両トピックから二重入力されることはない。

`config/prior_distribution.yaml`:

```yaml
enable_bottle_id: true
pred_with_id_topic: /P_pred_with_id
```

`config/observation_fusion_multi_sensor.yaml`:

```yaml
enable_bottle_id: true
pred_with_id_topic: /P_pred_with_id
place_with_id_topic: /P_place_with_id
```

通常の既存launchがこの設定を読む。
両ノードを `enable_bottle_id: false` にすれば従来の入力経路へ戻る。
パラメータ未指定時は従来経路。独自launchで動かす場合は両ノードにtrueを設定する。
初回は通常のcatkin workspaceで `catkin_make` し、`devel/setup.bash` をsourceして
新しいメッセージ型を使える状態にする。

`/P_current` のIDはFloat32なので、Unity側でも整数0〜16,777,216の範囲に収める。
ROS側は負数・小数・NaN/InfなどのIDを拒否する（ID有効時）。

## Unity側Codexへ渡す変更指示

以下の内容でUnity側を変更してください。

1. `place_estimation/msg/BottleDistribution.msg` をROS#でC#クラスへ生成する。
   型名は `place_estimation/BottleDistribution`。新しい送信メッセージは不要。
2. `P_cerrentPublisher` と `/P_current` の `[id,x,y,z]` 配信を維持する。
   提示コードの同一クラスは2回貼られているだけなので、実ファイルでは定義を1つにする。
3. `CorrectedBottlePositionSubscriber` の購読先を `/P_place_with_id`、
   メッセージ型を生成したBottleDistributionに変更する。
   `/P_place` も同時に適用して二重更新しない。
4. `P_currentAndStringPub` の `ArmForBottle(currentId,currentBottle,objectList.transform)`
   呼出しは残せる。受信側のArmForBottleは、単一の「最後のボトル」を保存する代わりに
   ID→ボトルGameObject・ObjectList基準Transformの辞書へ登録する。
5. ReceiveMessageでは `message.bottle_id` と `message.position` を同じメッセージから
   取得してスレッド安全なキューへ入れる。UnityメインスレッドのUpdateで辞書を引き、
   一致したボトルだけに既存の位置適用処理を実行する。
   未登録ID・破棄済みオブジェクトは無視してログを出す。
6. 今回はID変更だけなので、既存の軸変換・ObjectListへの適用計算・QR処理を維持する。
   補正位置の取得箇所だけを `message.pose.position` から `message.position` へ変える。
   姿勢を推定するメッセージではないので、ボトルの回転を上書きしない。

受信処理の要点（実際の生成クラス名と既存実装に合わせる）:

```csharp
// ROS受信スレッド
protected override void ReceiveMessage(BottleDistribution message)
{
    pending.Enqueue(message); // ConcurrentQueue<BottleDistribution>
}

// Unityメインスレッド
private void Update()
{
    while (pending.TryDequeue(out var message))
    {
        if (!targets.TryGetValue(message.bottle_id, out var target))
            continue;
        // targetのボトルだけに、既存の座標変換・位置適用を実行する。
        // ROS位置は message.position.x/y/z。
    }
}
```

添付には受信側 `CorrectedBottlePositionSubscriber` の本体がないため、
このROSリポジトリでそのUnityファイルを直接変更したものではない。
既存の受信クラスに上記を組み込む。

## 検証と適用範囲

実行結果: 既存43件＋ID関連8件の計51単体テスト成功。
隔離catkin workspaceでBottleDistributionの生成・ビルド成功。
実ROSの2ノードでID `7 → 12 → 7` の引き継ぎ、fallbackの位置・共分散、
既存 `/P_place` の継続配信を確認した。テストにロボット・YOLO・QRは使用していない。

`tests/test_bottle_identity.py` は異なるID、同一IDの繰り返し、ID=0、
P_current/P_tfの前後到着、前操作の確定、timeout/fallback、観測選択後のID、
不正ID、旧結果トピックの継続を検証する。

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests
# ビルド済みworkspaceをsource後。独立localhost ROSマスターで2ノードを起動・終了。
PYTHONDONTWRITEBYTECODE=1 python3 tests/ros_bottle_identity_smoke_test.py
```

今回は**対象ボトルの識別**のみを追加した。
同一ボトルの古い結果と新しい結果の区別、遅延画像の別操作への混入防止は追加していない。
既存の順次操作を前提とした処理を維持する。
Unity実機での位置適用は未検証。Meta座標変換の調整は次の作業とする。
