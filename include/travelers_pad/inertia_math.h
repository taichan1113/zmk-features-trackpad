/* Pure fixed-point inertia math, shared with host regression tests.
 * 実機とhost回帰試験で共有する固定小数点の慣性計算。 */
#ifndef TRAVELERS_PAD_INERTIA_MATH_H_
#define TRAVELERS_PAD_INERTIA_MATH_H_

#include <stdint.h>

#define TRAVELERS_PAD_INERTIA_ONE 65536

/* Velocity is sensor counts/ms in Q16, not counts/report.
 * 速度はreportごとの移動量ではなく、Q16のセンサーcount/ms。 */
static inline int32_t travelers_pad_inertia_velocity(int32_t previous, int32_t delta,
						    uint32_t elapsed_ms, uint32_t window_ms,
						    int fresh)
{
	if (elapsed_ms == 0) {
		elapsed_ms = 1;
	}
	int32_t instant = (int32_t)((int64_t)delta * TRAVELERS_PAD_INERTIA_ONE / elapsed_ms);
	if (fresh || (previous < 0 && instant > 0) || (previous > 0 && instant < 0) ||
	    window_ms == 0 || elapsed_ms >= window_ms) {
		return instant;
	}
	return previous + (int32_t)(((int64_t)instant - previous) * elapsed_ms / window_ms);
}

/* Approximate exp(-ln(2) * elapsed/half_life), with a Q24 1ms Pade factor
 * raised by exponentiation-by-squaring. Work is O(log elapsed), no floats.
 * 半減時間に基づく指数減衰を整数演算で近似。経過時間に対し対数時間で計算する。 */
static inline int32_t travelers_pad_inertia_decay(int32_t velocity, uint32_t elapsed_ms,
						 uint32_t half_life_ms)
{
	if (half_life_ms == 0) {
		return 0;
	}
	const int64_t one = 1LL << 24;
	int64_t scaled_time = (int64_t)half_life_ms * 2000;
	int64_t factor = (scaled_time - 693) * one / (scaled_time + 693);
	int64_t product = one;
	while (elapsed_ms != 0) {
		if (elapsed_ms & 1U) {
			product = product * factor / one;
		}
		factor = factor * factor / one;
		elapsed_ms >>= 1;
	}
	return (int32_t)((int64_t)velocity * product / one);
}

/* Integrate and retain sub-count travel instead of truncating every tick.
 * 各tickのcount未満の移動を切り捨てず次回へ保持する。 */
static inline int32_t travelers_pad_inertia_integrate(int32_t velocity, uint32_t elapsed_ms,
						     int64_t *remainder)
{
	int64_t travel = *remainder + (int64_t)velocity * elapsed_ms;
	int32_t delta = (int32_t)(travel / TRAVELERS_PAD_INERTIA_ONE);
	*remainder = travel - (int64_t)delta * TRAVELERS_PAD_INERTIA_ONE;
	return delta;
}

#endif
