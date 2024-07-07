# NS302BT Digital Setting Circles
エンコーダによる導入支援を実現します。

星図アプリ： ステラナビゲータ、SkySafari　確認済。
マウント：　赤道儀に対応します、その他は未確認です。
インターフェース：bluetooth / USB (RS232C）
CPU : ESP32
ファイルシステムがあります。一度:@**format# して下さい。


コマンド例
:@*V#
　→NS302BT Digital Setting Circles Ver1.1#

:@*GVP#
　→NS302BT#

:@*GVN#
　→1.11#

:@*L#
　→素早くLED点滅

:@**format#               最初に一度実行してください。ファイルシステムを初期化します。
　→OK#

:@*REncoderP#　      　　赤経周回パルスを確認する。
　→10000#

:@*REncoderR#　　　      赤経エンコーダー逓倍を確認する。1　 2　 4逓倍　
　→2#                    2逓倍

:@**newfile#　　　　　　　現在の状態を再現する初期化ファイルNS302NEW.txtを生成（上書き）します。
　→OK#

:@**dir#　　　　　　　　　ディレクトを表示する。
　→/NS302NEW.txt 269 　　数字はファイルのバイト数
#

:@**TYP /NS302NEW.txt#　　ファイルの内容をタイプする
→
NS-302_NEWfile

BlueTooth ssid
:@*BTssid#

Serial1,2 mode NS302 or NS5000
:@*serial1NS302#
:@*serial2NS302#

NS302 Mode 0(equatorial)  1(altazimuth)  2(NEXUS)
:@*Mode0#

:@*REncoderP10000#
:@*REncoderR2#
:@*REncoderV0#

:@*DEncoderP10000#
:@*DEncoderR2#
:@*DEncoderV1##




*******参考操作*******
:@*REncoderP10000#　赤経周回パルスを登録する。
:@*REncoderR2#　　　エンコーダー分解能　1　 2　 4逓倍　を登録する
:@*REncoderV0#      エンコーダー回転方向、反転　 0　1　を登録する
:@*DEncoderP10000#　赤緯
:@*DEncoderR2#
:@*DEncoderV0#

以上の状態で、次のコマンドを順番に実行してパルステックエンコーダに変更します。
:@*REncoderP4320#
:@*REncoderR4#
:@*DEncoderP4320#
:@*DEncoderR4#
:@**newfile#    現在の状態を再現する初期化ファイルNS302NEW.txtを生成（上書き）

そして
:@**TYP /NS302NEW.txt#　を実行して内容を確認してください。
パラメータが変更されています。

NS-302_NEWfile

BlueTooth ssid
:@*BTssid#

Serial1,2 mode NS302 or NS5000
:@*serial1NS302#
:@*serial2NS302#

NS302 Mode 0(equatorial)  1(altazimuth)  2(NEXUS)
:@*Mode0#

:@*REncoderP4320#
:@*REncoderR4#
:@*REncoderV0#

:@*DEncoderP4320#
:@*DEncoderR4#
:@*DEncoderV1##




