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

### Optional Tip-Tap modifier hold

Set `anchor-hold;` on the Tip-Tap node to hold local KSCAN row 0, column 2
after the first accepted Tip-Tap. Map that position to a modifier independently
from left/right columns 0/1. For Alt-Tab, use `&kp LALT` for the hold and
`&kp LS(TAB)` / `&kp TAB` for left/right. Without the property, no held action
is emitted. Use consistent or transparent bindings across layers for release.
Anchor loss, reset, cancellation or more than two contacts ends the hold.
Normal end releases it after queued discrete keys finish; cancellation drops
pending actions and releases active keys. The bounded broker supports one such
hold at a time. The held modifier also affects other host key input.

### Stationary four-finger action

`four-finger-hold-immediate` optionally dispatches once through the arbiter
resolver as soon as the hold is confirmed while contacts remain down. Continued
holding and release do not repeat the action; a new contact session is required.
Without this property, the existing release-time behavior remains unchanged.

Set `four-finger-hold-ms` on the multi-finger gesture node to enable an optional
stationary four-finger action (default: 0, disabled). `four-finger-hold-slop`
sets the maximum displacement of each contact on either axis (default: 60
sensor units). Contact replacement, a count change before confirmation, or
excessive movement cancels the hold candidate. Existing competing ownership
is respected. The action uses local KSCAN row 2, column 2, and is emitted once
after all contacts lift. The keyboard decides its binding; it need not be a
Bluetooth operation. Map that extra physical position before enabling it.

The tap/drag node also accepts `disabled-layer` (default: 255, no filter).
On that layer it clears tracking and pending clicks while continuing to
acknowledge arbiter frames. This is useful when a keyboard uses the surface
for a non-mouse control layer. Output suppression and grid routing remain
keyboard integration responsibilities.

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

### 任意のTip-Tap modifier保持

Tip-Tap nodeに `anchor-hold;` を指定すると、最初の認可済みTip-Tap後にローカル
KSCAN行0列2を保持します。左右列0/1とは別にmodifierを割り当ててください。
Alt-Tabなら保持は `&kp LALT`、左右は `&kp LS(TAB)`／`&kp TAB` です。
未指定なら保持actionは出ません。解放のため、レイヤー間で同じbindingまたは
透過bindingを使用します。anchor喪失・reset・取消し・3本以上で終了し、通常終了は
受付済み単発キー完了後に解放します。取消しは未送信actionを破棄して押下済みキーを
解放します。有限brokerが同時に保持するanchor actionは1件です。
保持modifierは他のホストキー入力にも作用します。

### 4本指静止による操作

`four-finger-hold-immediate`を指定すると、指を置いたまま成立したframeのarbiter裁定後に
一度だけ発火します。保持継続・解放では繰り返さず、再操作には新しい接触sessionが必要。
未指定の場合は従来どおり解放時に発火します。

多指ジェスチャーnodeの`four-finger-hold-ms`で任意の4本指静止操作を有効にします。
既定値0は無効です。`four-finger-hold-slop`は各指のX/Y変位の許容値で、
既定値はセンサー座標60です。成立前の指の入れ替え、本数変化、過大な移動で
候補を取り消します。既存の競合ownerを尊重し、全指を離した後にローカルKSCANの
行2・列2へ一回だけ出力します。有効化する前にこの物理位置をmapしてください。
割り当てはキーボード側で決めるため、Bluetooth操作に限定しません。

tap/drag nodeの`disabled-layer`は既定値255（制限なし）です。指定レイヤー中は
追跡と遅延クリックを破棄しますが、arbiter frameのacknowledgeは継続します。
マウス以外の操作にパッドを使うレイヤー向けの設定です。出力抑止と格子の経路設定は
キーボード側の責務です。

### トラブルシューティング

- ホストポインタが動かない場合は、input device labelと`pointing-device` listenerを確認してください。
- 2本指スクロールが動かない場合は、機能nodeとinput queueの大きさを確認してください。
- virtual keyジェスチャーが出ない場合は、キーボードのKSCAN合成とbindingを確認してください。
- 想定外の出力がある場合は、該当する任意nodeを無効にし、基本ポインタprofileから確認してください。

### ライセンス

本プロジェクト独自の成果は[`LICENSE`](LICENSE)に記載したMIT Licenseです。
依存先やinput device実装にはそれぞれのライセンス条件が適用されます。
配布ファームウェアに含まれるすべての構成要素の条件を確認してください。
