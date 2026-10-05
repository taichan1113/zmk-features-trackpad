/* Shared immutable contact-session vocabulary.
 *
 * This header intentionally contains only normalized contact lifecycle data.
 * Ownership lives in contact_arbiter.h, so function modules cannot mutate
 * session state or claim ownership through this interface.
 *
 * 正規化済み接触lifecycle用の共通語彙。所有権はcontact_arbiter.hだけが持つため、
 * 機能moduleはこのinterface経由でsession状態や所有権を変更できない。 */
#ifndef TRAVELERS_PAD_CONTACT_SESSION_H_
#define TRAVELERS_PAD_CONTACT_SESSION_H_

#include <stdbool.h>
#include <stdint.h>

struct travelers_pad_session_snapshot {
	uint32_t session_id;
	uint32_t frame_seq;
	int64_t timestamp_ms;
	uint8_t contact_count;
	uint8_t peak_contact_count;
	bool cancelled;
};

#endif
