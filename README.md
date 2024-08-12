# はじめに
本ソフトウェアはROS 2のrosbagファイルで保存した入出力とゲインスケジュールドパラメータのデータを用いてゲインスケジュールド制御を調整する自動調整ソフトである。ゲインスケジュールド制御調整用のパッケージ、サンプルのプラント・制御器のパッケージ、実行用スクリプトから構成されている。

参考文献：
Yahagi, S., & Suzuki, M. (2023). Controller parameter tuning for gain‐scheduled vehicle yaw‐rate control: Virtual reference feedback tuning approach. Electronics Letters, 59(6), e12764.

本ソフトにおけるqiita記事：人気だったら書くかも...

# 環境
OS:ubuntu 22.04
ROS version:ROS 2 humble
言語：C++、Python、shell scripts

# パッケージ構成
本ソフトにおける各パッケージの概要を本節で述べる。

### gs_controller_msgs
ゲイン調整用入出力信号のためのタイムスタンプ付きカスタムメッセージ。制御入力inputと出力応答outputと時変パラメータの変数を内包。

### time_sync
時刻同期用ノードのパッケージ。rosbagでプレイされた入力信号と出力信号を受信。受信した入出力を時刻同期させた上でgs_controller_msgsのメッセージとして出力。

### pid_tuner
ゲインスケジュールド制御調整用ノードのパッケージ。time_syncから受信した入出力データにてパラメータの自動調整を実行。調整後パラメータをターミナル上に表示。

### vehicle_sim(option)
例題用の制御対象ノードのパッケージ。ゲイン調整機能とは関係ないため，例題を実行しない場合は不要。

### gain_scheduled_control(option)
例題用の制御器ノードのパッケージ。ゲイン調整機能とは関係ないため，例題を実行しない場合は不要。

# ノード構成
本ソフトのノード構成を下記のグラフに示す。例題の関係上，制御対象の出力データのトピックが/vehicle_state，入力データのトピックが/steering，時変パラメータのトピックが/vehicle_velocityとなっている。/plant_infoが時刻同期済みの入出力信号におけるカスタムメッセージでのトピックとである。

![image](https://github.com/user-attachments/assets/a2f99119-950f-41d9-8c8c-743cd2624623)

# ゲインスケジュールド制御器
今回対象とする制御器は多項式ゲインスケジュールドによるPI制御である。制御則が
```math
u(t)=K_{P}(t)e(t)+K_{I} (t) \int_{0}^{t} e(\tau)d\tau 
```
```math
K_{P}(t)=w_{P0}+w_{P1}x(t)+w_{P2}x^2(t) 
```
```math
K_{I}(t)=w_{I0}+w_{I1}x(t)+w_{I2}x^2(t) 
```
となる。$`e(t)`$と$`u(t)`$が偏差と制御入力である。$`x(t)`$が制御対象の時変パラメータである。$`w_{Pi}{i=0,1,2}`$と$`w_{Ii}{i=0,1,2}`$が可調整パラメータとなる。 

本ツールでは一組の実験データのみを用いて可調整パラメータをオフラインで自動調整する。本ツールのアルゴリズムでは閉ループ応答を参照モデルの目標値応答に近づけるようなパラメータ調整を施す。参照モデルを一時遅れ系
```math
T_{d}=\frac{1}{T_{s}s+1} 
```
で設定している。$`T_{s}`$が時定数である。

# 例題の実行手順
1.ビルド
mainブランチをクローンして，下記のコマンドを実行する。

```
source build_setup.sh
```

2.初期入出力データの測定(rosbagの準備)
下記のコマンドを実行して例題用の入出力データ(rosbag)を生成しておく。

```
source run_yaw_rate_control_sim.sh
```

なお，本例題ではlaunchファイル(control_run.py)にて例題の制御器におけるパラメータを設定している。rosbagデータの入出力応答を変更する場合，launchファイル上のパラメータを変更して実行する必要がある。

3.ゲイン調整
シェルスクリプト(source run_PID_tuning.sh)内のrosbag名を該当ファイル名に変更する。launchファイル(pid_tuner.py)にて，参照モデル(time_const)の時定数とデータ数(読み込むデータの最大データ点数max_data_points)を設定する。下記のコマンドを実行してrosbagデータからパラメータをオフライン計算する。

```
source run_PID_tuning.sh
```

正常に実行ができた場合，下記のようにターミナル上にてゲインの調整結果が表示される。

![image](https://github.com/user-attachments/assets/7bde1a65-d527-40c0-9c18-2d37cb32b7d8)

4.ゲイン調整結果の確認
launchファイル(control_run.py)のパラメータを変更し，下記のコマンドを実行して制御器調整後の制御応答を確認する。

```
source run_yaw_rate_control_sim.sh
```

# 例題におけるゲイン調整結果例
本例題ではモーションプロファイルとして目標値と時変パラメータである車速が事前設定されている。

![image](https://github.com/user-attachments/assets/bc578462-3b73-45f4-b79f-e9b7064f4f98)

モーションプロファイル(上図：目標値、下図：時変パラメータ)

$`K_{P}=1.0`$，$`K_{I}=0.0`$の条件下にて初期入出力応答データを測定すると，下記のような制御応答を得られる。

![image](https://github.com/user-attachments/assets/38c2ee47-1f02-4f38-bb05-71d5e1c576be)

初期実験データ(青：出力、黄色：目標値、赤：参照モデル応答)

本条件下で得られたrosbagデータを用いてゲイン調整を実施することにより，制御器のパラメータを自動調整できる。ただし，最大データ点数は4500であり，参照モデルの時定数を0.5sと設定している。調整後パラメータを適用することにより，下記のような制御応答を得られる。

![image](https://github.com/user-attachments/assets/66449c59-e967-47fd-8bae-feb29e13bf29)
パラメータ調整後の閉ループ応答(青：出力、黄色：目標値、赤：参照モデル応答)

![image](https://github.com/user-attachments/assets/faca9877-244f-4fed-8acc-88e54323ec99)
PIゲインの応答(上図：Pゲイン、下図：Iゲイン)
