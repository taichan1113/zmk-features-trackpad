/* Two-finger Tip-Tap recognizer / 2本指Tip-Tap認識器. */
#define DT_DRV_COMPAT zmk_travelers_pad_iqs9150_tip_tap_kscan
#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/drivers/kscan.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <travelers_pad/contact_arbiter.h>
#include <travelers_pad/contact_output_broker.h>
#include <travelers_pad/multitouch.h>

LOG_MODULE_REGISTER(travelers_pad_tip_tap, LOG_LEVEL_INF);

struct cfg { uint16_t max_ms, slop, separation; bool invert; };
struct data {
	const struct device *dev;
	kscan_callback_t callback; uint16_t x[TRAVELERS_PAD_MAX_CONTACTS], y[TRAVELERS_PAD_MAX_CONTACTS];
	int32_t id[TRAVELERS_PAD_MAX_CONTACTS]; bool valid[TRAVELERS_PAD_MAX_CONTACTS]; uint8_t slot, fingers, previous;
	uint8_t anchor, second; int32_t anchor_id, second_id; uint16_t ax, ay, sx, sy; int64_t second_start;
	uint32_t arbiter_frame_seq;
	uint32_t ownership_frame_seq;
	/* The anchor may remain on the pad while several temporary second contacts
	 * produce repeated Tip-Tap actions. A single arbiter proposal therefore
	 * represents that whole anchored interaction, rather than each repeat.
	 * anchorを維持したまま一時的な2本目の接触でTip-Tapを繰返せる。そのため
	 * arbiter proposalは各repeatではなく、anchorを含む一連の操作を表す。 */
	bool candidate, fired, reset, tip_owned, pending_action, pending_left;
	uint32_t pending_frame_seq;
};
static uint16_t absi(int value) { return value < 0 ? -value : value; }
static void reset(struct data *d) {
	d->candidate = false;
	d->fired = false;
	d->tip_owned = false;
	d->ownership_frame_seq = 0;
	d->pending_action = false;
	d->pending_frame_seq = 0;
	d->anchor_id = -1;
	d->second_id = -1;
}

static void tip_tap_resolve_action(void *context, const struct travelers_pad_owner_token *token,
				   uint32_t ownership_frame_seq)
{
	struct data *d = context;

	ARG_UNUSED(token);
	if (!d->pending_action || d->pending_frame_seq != ownership_frame_seq) return;
	d->pending_action = false;
	if (travelers_pad_output_behavior_primary(
			d->dev, d->callback, ownership_frame_seq, TRAVELERS_PAD_RECOGNIZER_TIP_TAP,
			TRAVELERS_PAD_OWNER_TIP_TAP, 0, d->pending_left ? 0 : 1, "tip-tap") == 0) {
		LOG_INF("Tip-Tap initial action dispatched by arbiter");
	}
}

static bool claim_tip_tap(struct data *d)
{
	enum travelers_pad_owner owner =
		travelers_pad_arbiter_owner_for_frame(d->arbiter_frame_seq);

	/* The first completed Tip-Tap establishes the ownership for its anchored
	 * interaction. A competing committed/provisional primary owner makes this
	 * candidate ineligible; TIP_TAP itself is allowed for its repeated actions.
	 * 最初に完了したTip-Tapがanchor操作全体の所有権を確立する。競合する
	 * committed/provisional primary ownerがあれば候補を無効にし、反復action
	 * のためTIP_TAP自身だけを許可する。 */
	if (owner != TRAVELERS_PAD_OWNER_NONE && owner != TRAVELERS_PAD_OWNER_TIP_TAP) {
		return false;
	}

	if (!d->tip_owned) {
		/* Submit the first completed action on its exact source frame. The
		 * resolver now owns the decision; later repeat actions retain this
		 * ownership frame and must pass the same token gate.
		 * 最初に完了したactionを、その正確なsource frameへ提出する。決定は
		 * resolverが行い、以後のrepeat actionはこのownership frameを保持して
		 * 同じtoken gateを通過する。 */
		if (!travelers_pad_arbiter_submit(&(struct travelers_pad_arbiter_proposal){
			.frame_seq = d->arbiter_frame_seq,
			.recognizer = TRAVELERS_PAD_RECOGNIZER_TIP_TAP,
			.intent = TRAVELERS_PAD_PROPOSAL_CLAIM,
			.requested_owner = TRAVELERS_PAD_OWNER_TIP_TAP,
			.capabilities = TRAVELERS_PAD_CAP_BEHAVIOR_ACTION,
			.contact_count = d->fingers,
			.evidence = 1,
		})) {
			LOG_WRN("Tip-Tap claim proposal rejected");
			return false;
		}
		d->ownership_frame_seq = d->arbiter_frame_seq;
		d->tip_owned = true;
	}

	return true;
}

static void process(const struct device *dev)
{
	struct data *d=dev->data; const struct cfg *c=dev->config; int64_t now=k_uptime_get();
	if (d->reset) { reset(d); d->reset=false; return; }
	if (d->fingers == 1 && d->previous == 0) {
		for (uint8_t i=0;i<TRAVELERS_PAD_MAX_CONTACTS;i++) if (d->valid[i]) { d->anchor=i; d->anchor_id=d->id[i]; d->ax=d->x[i]; d->ay=d->y[i]; break; }
	}
	if (d->fingers == 2 && d->previous < 2 && d->anchor_id >= 0) {
		/* The anchor stays down across a repeat. Re-arm only the temporary
		 * second contact; the session remains exclusively TIP_TAP-owned. */
		d->fired=false;
		for (uint8_t i=0;i<TRAVELERS_PAD_MAX_CONTACTS;i++) if (d->valid[i] && i!=d->anchor) { d->second=i; d->second_id=d->id[i]; d->sx=d->x[i]; d->sy=d->y[i]; d->second_start=now; d->candidate=true; break; }
	}
	if (d->candidate && d->fingers == 2 && (!d->valid[d->anchor] || d->id[d->anchor]!=d->anchor_id ||
		absi((int)d->x[d->anchor]-d->ax)>c->slop || absi((int)d->y[d->anchor]-d->ay)>c->slop ||
		absi((int)d->x[d->second]-d->sx)>c->slop || absi((int)d->y[d->second]-d->sy)>c->slop)) d->candidate=false;
	if (d->candidate && !d->fired && d->previous == 2 && d->fingers == 1 &&
	    d->valid[d->anchor] && d->id[d->anchor] == d->anchor_id &&
	    now - d->second_start <= c->max_ms &&
	    absi((int)d->sx - d->ax) >= c->separation && claim_tip_tap(d)) {
		bool left=d->sx<d->ax; if (c->invert) left=!left; d->fired=true;
		LOG_INF("Tip-Tap %s anchor=%d second=%d duration=%lld", left?"Left":"Right", d->anchor_id,d->second_id,now-d->second_start);
		if (d->ownership_frame_seq == d->arbiter_frame_seq) {
			d->pending_action=true; d->pending_left=left;
			d->pending_frame_seq=d->ownership_frame_seq;
		} else if (travelers_pad_output_behavior_primary(
				   dev, d->callback, d->ownership_frame_seq,
				   TRAVELERS_PAD_RECOGNIZER_TIP_TAP, TRAVELERS_PAD_OWNER_TIP_TAP, 0,
				   left ? 0 : 1, "tip-tap") != 0) {
			return;
		}
	}
	if (d->fingers == 0) {
		reset(d);
	}
	d->previous = d->fingers;
}
static int configure(const struct device *dev, kscan_callback_t cb) { if(!cb) return -EINVAL; ((struct data*)dev->data)->callback=cb; return 0; }
static const struct kscan_driver_api api={.config=configure};

static int tip_tap_init(const struct device *dev)
{
	((struct data *)dev->data)->dev = dev;
	(void)travelers_pad_arbiter_register(TRAVELERS_PAD_RECOGNIZER_TIP_TAP);
	(void)travelers_pad_arbiter_bind_resolve_handler(
		TRAVELERS_PAD_RECOGNIZER_TIP_TAP, tip_tap_resolve_action, dev->data);
	return 0;
}

#define INST(n) \
	static struct data data_##n={.anchor_id=-1,.second_id=-1}; \
	static const struct cfg cfg_##n={.max_ms=DT_INST_PROP_OR(n,second_contact_timeout_ms,180),.slop=DT_INST_PROP_OR(n,contact_slop,75),.separation=DT_INST_PROP_OR(n,minimum_horizontal_separation,160),.invert=DT_INST_PROP_OR(n,direction_inverted,false)}; \
	static void input_##n(struct input_event *e) { struct data *d=&data_##n; if(e->type==INPUT_EV_ABS) { if(e->code==INPUT_ABS_MT_SLOT)d->slot=e->value; else if(e->code==TRAVELERS_PAD_INPUT_ABS_MT_TRACKING_ID && d->slot<TRAVELERS_PAD_MAX_CONTACTS){d->valid[d->slot]=e->value>=0;d->id[d->slot]=e->value;} else if(e->code==TRAVELERS_PAD_INPUT_ABS_MT_POSITION_X && d->slot<TRAVELERS_PAD_MAX_CONTACTS)d->x[d->slot]=e->value; else if(e->code==TRAVELERS_PAD_INPUT_ABS_MT_POSITION_Y && d->slot<TRAVELERS_PAD_MAX_CONTACTS)d->y[d->slot]=e->value; else if(e->code==INPUT_ABS_Z)d->fingers=e->value; else if(e->code==INPUT_ABS_RZ && e->value)d->reset=true;} if(e->sync) { d->arbiter_frame_seq=travelers_pad_arbiter_next_frame(TRAVELERS_PAD_RECOGNIZER_TIP_TAP); process(DEVICE_DT_INST_GET(n)); travelers_pad_arbiter_observed(TRAVELERS_PAD_RECOGNIZER_TIP_TAP,d->arbiter_frame_seq); } } \
	INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_INST_PHANDLE(n,input_device)),input_##n); \
	DEVICE_DT_INST_DEFINE(n,tip_tap_init,NULL,&data_##n,&cfg_##n,POST_KERNEL,CONFIG_TRAVELERS_PAD_IQS9150_FUNCTION_INIT_PRIORITY,&api);
DT_INST_FOREACH_STATUS_OKAY(INST)
