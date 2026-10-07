/* Contact-count policy, separate from sensor decoding and ownership.
 * センサーdecode・所有権とは独立した接触本数の安定化方針。 */
#ifndef TRAVELERS_PAD_CONTACT_STABILITY_H_
#define TRAVELERS_PAD_CONTACT_STABILITY_H_
#include <stdbool.h>
#include <stdint.h>

struct travelers_pad_contact_stability {
	uint8_t accepted_mask, pending_mask;
	int64_t pending_since;
	bool pending;
};

static inline uint8_t travelers_pad_contact_count(uint8_t mask)
{
	uint8_t count = 0;
	for (; mask; mask >>= 1) count += mask & 1;
	return count;
}

static inline uint8_t travelers_pad_first_contact(uint8_t mask)
{
	for (uint8_t slot = 0; slot < 7; slot++) {
		if (mask & (1u << slot)) return slot;
	}
	return 7;
}

static inline uint8_t travelers_pad_stabilize_contacts(
	struct travelers_pad_contact_stability *state, uint8_t raw_mask,
	int64_t now, uint16_t confirm_ms)
{
	/* Only debounce adding a second contact to an existing single contact.
	 * Initial taps, release, and already accepted multi-contact streams pass.
	 * 継続中の1本指への2本目追加だけを確認。初回tap・release・確定済み多指は即時。 */
	if (confirm_ms && travelers_pad_contact_count(state->accepted_mask) == 1 &&
	    travelers_pad_contact_count(raw_mask) == 2) {
		if (!state->pending || state->pending_mask != raw_mask) {
			state->pending = true;
			state->pending_mask = raw_mask;
			state->pending_since = now;
		}
		if (now - state->pending_since < confirm_ms) {
			uint8_t surviving = state->accepted_mask & raw_mask;
			state->accepted_mask = surviving ? surviving :
				(1u << travelers_pad_first_contact(raw_mask));
			return state->accepted_mask;
		}
	}
	state->pending = false;
	state->accepted_mask = raw_mask;
	return raw_mask;
}
#endif
