# zmk-features-trackpad / トラックパッド機能群

Standalone ZMK/Zephyr module extracted from Travelers Pad at `b0ad83d`.
Recognizer, arbiter, broker and observer implementation is unchanged.
Requires `zmk-driver-iqs9150`, which exports the shared contact-frame header.
No keyboard wiring, chosen KSCAN or host key assignments are imposed.

Travelers Padの`b0ad83d`から切り出した独立ZMK/Zephyr module。
認識器・arbiter・broker・observerの実装は変更しない。
共有frameヘッダを提供する`zmk-driver-iqs9150`が必要。
キーボードの配線・chosen KSCAN・キー割当を上書きしない。

Add both projects to west. After defining a sensor labelled `iqs9150`, include
`<travelers-pad/iqs9150-pointer.dtsi>` for the basic pointer/scroll/tap-drag/inertia
profile. Set `CONFIG_ZMK_POINTING=y` and use an input queue of at least 64 messages
for the validated five-contact profile. Smooth scrolling is selected by the keyboard.

westで両projectを追加し、`iqs9150`ラベルのセンサを定義する。
基本profileは上記DTSをincludeする。CONFIG_ZMK_POINTINGを有効にし、
検証済み5点profileではinput queueを64以上にする。
高解像度スクロールの有効・無効はキーボード側で設定する。

Feature selection uses each node's `status` and existing CONFIG symbols. Tune
listener processors and thresholds from the keyboard overlay. Virtual-key gestures
need KSCAN composition and bindings supplied by the keyboard. The Travelers Pad
configuration repository remains the full integration example and regression suite.

各nodeのstatusと既存CONFIGで機能を選択する。倍率・閾値はキーボードoverlayで上書きする。
仮想キー操作はキーボード側のKSCAN統合とbindingが必要。
Travelers Pad設定repoに完全な結合例と回帰試験を残す。

The external grid module `zmk-input-matrix` is optional and is needed only by
configurations importing its grid definition, not by the basic profile.
Existing IQS9150 CONFIG dependencies remain; other ICs are not yet supported.

外部格子moduleは任意依存で、その格子定義をincludeする設定にだけ必要。
基本profileには不要。IQS9150 CONFIG依存は維持し、他IC対応は未実施。
