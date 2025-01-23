#include <linux/module.h>
#include <linux/init.h>
#include <linux/timekeeping.h>
#include <net/tcp.h>


#define FIXED_POINT_SCALE 65536 // Scale for fixed-point arithmetic


MODULE_AUTHOR("MM");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Simple congestion control algorithm with cubic growth in slow start, exponencial fast recovery and linear congestion avoidance.");


static u64 start_time_ns; // Start time
static int recovery_mode = 0; // 0 = not in recovery, 1 = in fast recovery
static u32 recovery_point = 0; // Track the recovery start point
static u32 jBase = 1; // Track start point
static u32 snd_second_thresh = 200; // Define the second threshold
static u32 drep = 1; // recovery delta


static u32 expAdjust(u32 t, u32 k, u32 L)
{
	u32 scaled_exponent, adjustment;
	if (k == 0) {
		return 0;
	}
	u32 scaled_time = t / k;
	scaled_exponent = FIXED_POINT_SCALE >> scaled_time; // Approximate 2^(-t/k)
	adjustment = FIXED_POINT_SCALE - scaled_exponent; // Compute 1 - 2^(-t/k)
	return (L * adjustment) >> 16; // Scale back to integer
}

static void simple_init(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	tp->snd_cwnd = TCP_INIT_CWND; // Set initial congestion window size
	start_time_ns = ktime_get_ns(); // Initialize the start time to the current time
	recovery_mode = 0; // Start with no recovery mode
	recovery_point = 0; // Reset recovery point
}

static u32 simple_ssthresh(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);
	start_time_ns = ktime_get_ns(); // Reset start time on packet loss
	recovery_mode = 1; // Enter fast recovery mode
	recovery_point = tp->snd_nxt; // Set recovery point
	return tp->snd_cwnd / 2; // Reduce ssthresh to half of current cwnd
}

static void simple_cong_control(struct sock *sk, const struct rate_sample *rs)
{
	struct tcp_sock *tp = tcp_sk(sk);
	u64 current_time_ns;
	u32 time_in_ms;

	if (rs->losses > 0) {
		if (!recovery_mode || tp->snd_nxt > recovery_point) {
			// Enter recovery mode
			recovery_mode = 1;
			// Set recovery point to the next sequence number
			recovery_point = tp->snd_nxt;
			// Reduce threshold
			snd_second_thresh = tp->snd_cwnd;
			tp->snd_ssthresh = tp->snd_cwnd / 2;
			// Reduce cwnd
			u32 decrement = tp->snd_cwnd * 2 / 10;
			tp->snd_cwnd = tp->snd_cwnd - decrement;
			jBase = tp->snd_cwnd;
			// Reset start time
			start_time_ns = ktime_get_ns();
			// Set delta
			drep = decrement;
		}
	} else if (rs->acked_sacked) {
		current_time_ns = ktime_get_ns(); // Get the current time
		time_in_ms = (u32)((current_time_ns - start_time_ns) / 1000000); // Calculate elapsed time in ms

		if (recovery_mode) {
			// Fast recovery phase
			u32 increment = jBase + expAdjust(time_in_ms, 4, drep) + 1;
			tp->snd_cwnd = max(increment, 1U);
			if (tp->snd_cwnd >= snd_second_thresh) {
				recovery_mode = 0; // Exit recovery mode once above ssthresh
			}
		} else if (tp->snd_cwnd < tp->snd_ssthresh) {
			// slow start phase
			u32 increment = 2 * time_in_ms * time_in_ms * time_in_ms;
			tp->snd_cwnd += max(increment, 2U); // Ensure at least 2 is added
		} else {
			// congestion avoidance phase
			u32 increment = snd_second_thresh + 2 * time_in_ms;
			tp->snd_cwnd = max(increment, 1U);
		}
	}
}

static u32 simple_undo_cwnd(struct sock *sk)
{
	struct tcp_sock *tp = tcp_sk(sk);

	return max(tp->snd_cwnd, 2U); // Ensure cwnd does not drop below 2
}

// Define the congestion control operations
static struct tcp_congestion_ops simple_cong_ops = {
	.flags = TCP_CONG_NON_RESTRICTED,
	.name = "expolinear",
	.owner = THIS_MODULE,
	.init = simple_init,
	.ssthresh = simple_ssthresh,
	.cong_control = simple_cong_control,
	.undo_cwnd = simple_undo_cwnd,
};

static int __init simple_cong_init(void)
{
	pr_info("Expolinear CCA: Initializing\n");
	return tcp_register_congestion_control(&simple_cong_ops);
}

static void __exit simple_cong_exit(void)
{
	tcp_unregister_congestion_control(&simple_cong_ops);
	pr_info("Expolinear CCA: Unloaded\n");
}

module_init(simple_cong_init);
module_exit(simple_cong_exit);




