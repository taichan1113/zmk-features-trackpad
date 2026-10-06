[English](#english) | [日本語](#日本語)

# zmk-features-trackpad

## English

Reusable ZMK/Zephyr input functions for trackpad contact frames. The module
contains pointer and scroll processing, tap and drag recognition, Tip-Tap,
multi-finger gestures, edge functions, transform gestures, and contact-session
arbitration.

The implemented input contract currently targets the IQS9150 contact format.
Other sensors must provide compatible contact events before these functions
can consume them. Hardware validation has used an IQS9150EV02 on a Seeed XIAO
nRF52840 with ZMK v0.3. Other combinations are not verified.

### Included functions

| Function | Availability |
| --- | --- |
| Pointer and two-finger scroll | Basic profile |
| Tap, double-tap, and tap-drag | Basic profile; click-button mapping is fixed in the module |
| Scroll inertia | Optional node |
| Tip-Tap, three-to-five-finger gestures | Optional virtual KSCAN nodes; keyboard supplies KSCAN integration and bindings |
| Edge dial and edge continuation | Optional virtual KSCAN nodes and configuration |
| Pinch | Optional transform node |
| Rotation | Optional and disabled in the validated profile |

The module emits ordinary pointer, button, wheel, and key events through ZMK.
It does not present the sensor as an operating-system precision touchpad.

### Add to a ZMK workspace

Add the module to the west manifest for the keyboard build:

```yaml
manifest:
  projects:
    - name: zmk-features-trackpad
      url: https://github.com/taichan1113/zmk-features-trackpad.git
      revision: main
      path: module/zmk-features-trackpad
```

The build must also provide the contact input device and the shared contact
event definitions described in `include/travelers_pad/multitouch.h`. Those
definitions describe the interface; a compatible input driver must implement it.

### Basic pointer profile

After providing an input device labelled `iqs9150`, include:

```dts
#include <travelers-pad/iqs9150-pointer.dtsi>
```

Enable pointing and use an input queue of at least 64 messages for the
validated five-contact configuration:

```ini
CONFIG_ZMK_POINTING=y
CONFIG_INPUT_QUEUE_MAX_MSGS=64
CONFIG_ZMK_POINTING_SMOOTH_SCROLLING=y
```

The basic profile supplies pointer motion, two-finger scrolling, tap/drag,
and optional inertia. Keyboard overlays can tune the listener processors and
feature thresholds.

### Optional functions

Virtual-key functions need a keyboard-side KSCAN composition and key bindings.
Use the Devicetree bindings under `dts/bindings/` and the tested keyboard
integration as a reference. Each feature's node can be enabled or disabled
through its status and supported Kconfig options.

The optional grid integration is not included in the basic pointer profile.
Grid users need to provide a compatible virtual KSCAN source and layout.
Pinch and rotation are separately selectable; rotation is disabled in the
validated configuration. Their recognition thresholds may require tuning for
the sensor surface and finger placement.

### Troubleshooting

- If the host pointer does not move, verify the input device label and the
  `pointing-device` listener.
- If two-finger scroll is absent, confirm the feature node is enabled and the
  input queue is large enough.
- If virtual-key gestures do not appear, check the keyboard's KSCAN composition
  and bindings.
- If one function causes unexpected output, disable that optional node and
  test the basic pointer profile first.

### License

Original project contributions are licensed under the MIT License in
[`LICENSE`](LICENSE). Dependencies and input-device implementations retain
their own licensing terms. Review the terms for every component included in
a distributed firmware image.

## 日本語

トラックパッド接触フレームを処理する、再利用可能なZMK/Zephyr入力機能群です。
ポインタ・スクロール、タップ・ドラッグ、Tip-Tap、多指ジェスチャー、エッジ操作、
変換ジェスチャー、接触sessionの裁定機能を含みます。

現在の入力契約はIQS9150の接触形式を対象としています。他センサーを利用する場合は、
これらの機能が処理できる互換input eventを用意してください。
実機確認はZMK v0.3とSeeed XIAO nRF52840、IQS9150EV02で行っています。
他の組み合わせは未確認です。

### 含まれる機能

| 機能 | 利用条件 |
| --- | --- |
| ポインタと2本指スクロール | 基本profile |
| タップ、ダブルタップ、タップドラッグ | 基本profile。クリックボタンの割当はmodule内で固定 |
| スクロール慣性 | 任意node |
| Tip-Tap、3〜5本指ジェスチャー | 任意のvirtual KSCAN node。キーボード側でKSCAN統合とbindingが必要 |
| エッジダイヤルとエッジ継続 | 任意のvirtual KSCAN nodeと設定が必要 |
| ピンチ | 任意のtransform node |
| 回転 | 任意機能。確認済みprofileでは無効 |

ZMKを通じて通常のポインタ、ボタン、ホイール、キーイベントを出力します。
センサーをOSのprecision touchpadとして公開する機能ではありません。

### ZMK workspaceへの追加

キーボードのbuildで使うwest manifestにmoduleを追加します:

```yaml
manifest:
  projects:
    - name: zmk-features-trackpad
      url: https://github.com/taichan1113/zmk-features-trackpad.git
      revision: main
      path: module/zmk-features-trackpad
```

build環境には、接触input deviceと `include/travelers_pad/multitouch.h` に記載された共有接触event定義も必要です。
この定義はinterfaceを示すもので、互換input driverがそのeventを実装してください。

### 基本ポインタprofile

`iqs9150` というlabelのinput deviceを用意してから、次をincludeします:

```dts
#include <travelers-pad/iqs9150-pointer.dtsi>
```

pointingを有効にし、確認済み5点接触構成では64以上のinput queueを使用します:

```ini
CONFIG_ZMK_POINTING=y
CONFIG_INPUT_QUEUE_MAX_MSGS=64
CONFIG_ZMK_POINTING_SMOOTH_SCROLLING=y
```

基本profileはポインタ移動、2本指スクロール、タップ・ドラッグ、任意の慣性機能を提供します。
listener processorと各機能の閾値はキーボードoverlayで調整できます。

### 任意機能

virtual key機能には、キーボード側のKSCAN合成とkey bindingが必要です。
`dts/bindings/`内のDevicetree bindingと、動作確認済みキーボード統合を参考にしてください。
対応Kconfig optionと各nodeのstatusを使って機能を選択できます。

格子操作は基本ポインタprofileに含まれません。格子を利用する場合は、
互換virtual KSCAN sourceとlayoutを用意してください。
ピンチと回転は個別に選択できます。確認済み構成では回転を無効にしています。
センサー面や指の位置に応じて認識閾値の調整が必要な場合があります。

### トラブルシューティング

- ホストポインタが動かない場合は、input device labelと`pointing-device` listenerを確認してください。
- 2本指スクロールが動かない場合は、機能nodeとinput queueの大きさを確認してください。
- virtual keyジェスチャーが出ない場合は、キーボードのKSCAN合成とbindingを確認してください。
- 想定外の出力がある場合は、該当する任意nodeを無効にし、基本ポインタprofileから確認してください。

### ライセンス

本プロジェクト独自の成果は[`LICENSE`](LICENSE)に記載したMIT Licenseです。
依存先やinput device実装にはそれぞれのライセンス条件が適用されます。
配布ファームウェアに含まれるすべての構成要素の条件を確認してください。
