
//function fabsf is Defined in header <math.h>
/*
abs、fabs、fabsf三个函数都是用来求一个数的绝对值，区别如下：

1）int abs(int a);                     // 处理int类型的取绝对值

2）double fabs(double a);   //处理double类型的取绝对值

3）float fabsf(float a);           //处理float类型的取绝对值
————————————————
版权声明：本文为CSDN博主「叶晚zd」的原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/u013925378/article/details/84550977
*/

// Need further exploration of dt


//////////////code in datatypes.h/////////////////////
typedef struct {
	/*
	// Switching and drive
	mc_pwm_mode pwm_mode;
	mc_comm_mode comm_mode;
	mc_motor_type motor_type;
	mc_sensor_mode sensor_mode;
	*/
	// Limits
	float l_current_max;
	float l_current_min;
	float l_in_current_max;
	float l_in_current_min;
	float l_abs_current_max;
	float l_min_erpm;
	float l_max_erpm;
	float l_erpm_start;
	float l_max_erpm_fbrake;
	float l_max_erpm_fbrake_cc;
	float l_min_vin;
	float l_max_vin;
	float l_battery_cut_start;
	float l_battery_cut_end;
	bool l_slow_abs_current;
	float l_temp_fet_start;
	float l_temp_fet_end;
	float l_temp_motor_start;
	float l_temp_motor_end;
	float l_temp_accel_dec;
	float l_min_duty;
	float l_max_duty;
	float l_watt_max;
	float l_watt_min;
	float l_current_max_scale;
	float l_current_min_scale;
	// Overridden limits (Computed during runtime)
	float lo_current_max;
	float lo_current_min;
	float lo_in_current_max;
	float lo_in_current_min;
	float lo_current_motor_max_now;
	float lo_current_motor_min_now;
	// Sensorless (bldc)
	float sl_min_erpm;
	float sl_min_erpm_cycle_int_limit;
	float sl_max_fullbreak_current_dir_change;
	float sl_cycle_int_limit;
	float sl_phase_advance_at_br;
	float sl_cycle_int_rpm_br;
	float sl_bemf_coupling_k;
	// Hall sensor
	int8_t hall_table[8];
	float hall_sl_erpm;
	// FOC
	float foc_current_kp;
	float foc_current_ki;
	float foc_f_sw;
	float foc_dt_us;
	float foc_encoder_offset;
	bool foc_encoder_inverted;
	float foc_encoder_ratio;
	float foc_encoder_sin_offset;
	float foc_encoder_sin_gain;
	float foc_encoder_cos_offset;
	float foc_encoder_cos_gain;
	float foc_encoder_sincos_filter_constant;
	float foc_motor_l;
	float foc_motor_r;
	float foc_motor_flux_linkage;
	float foc_observer_gain;
	float foc_observer_gain_slow;
	float foc_pll_kp;
	float foc_pll_ki;
	float foc_duty_dowmramp_kp;
	float foc_duty_dowmramp_ki;
	float foc_openloop_rpm;
	float foc_sl_openloop_hyst;
	float foc_sl_openloop_time;
	float foc_sl_d_current_duty;
	float foc_sl_d_current_factor;
	//mc_foc_sensor_mode foc_sensor_mode;
	uint8_t foc_hall_table[8];
	float foc_sl_erpm;
	bool foc_sample_v0_v7;
	bool foc_sample_high_current;
	float foc_sat_comp;
	bool foc_temp_comp;
	float foc_temp_comp_base_temp;
	float foc_current_filter_const;
	//mc_foc_cc_decoupling_mode foc_cc_decoupling;
	//mc_foc_observer_type foc_observer_type;
	float foc_hfi_voltage_start;
	float foc_hfi_voltage_run;
	float foc_hfi_voltage_max;
	float foc_sl_erpm_hfi;
	uint16_t foc_hfi_start_samples;
	float foc_hfi_obs_ovr_sec;
	//foc_hfi_samples foc_hfi_samples;
	// GPDrive
	int gpd_buffer_notify_left;
	int gpd_buffer_interpol;
	float gpd_current_filter_const;
	float gpd_current_kp;
	float gpd_current_ki;
	// Speed PID
	float s_pid_kp;
	float s_pid_ki;
	float s_pid_kd;
	float s_pid_kd_filter;
	float s_pid_min_erpm;
	bool s_pid_allow_braking;
	// Pos PID
	float p_pid_kp;
	float p_pid_ki;
	float p_pid_kd;
	float p_pid_kd_filter;
	float p_pid_ang_div;
	// Current controller
	float cc_startup_boost_duty;
	float cc_min_current;
	float cc_gain;
	float cc_ramp_step_max;
	// Misc
	int32_t m_fault_stop_time_ms;
	float m_duty_ramp_step;
	float m_current_backoff_gain;
	uint32_t m_encoder_counts;
	//sensor_port_mode m_sensor_port_mode;
	bool m_invert_direction;
	//drv8301_oc_mode m_drv8301_oc_mode;
	int m_drv8301_oc_adj;
	float m_bldc_f_sw_min;
	float m_bldc_f_sw_max;
	float m_dc_f_sw;
	float m_ntc_motor_beta;
	//out_aux_mode m_out_aux_mode;
	//temp_sensor_type m_motor_temp_sens_type;
	float m_ptc_motor_coeff;
	// Setup info
	uint8_t si_motor_poles;
	float si_gear_ratio;
	float si_wheel_diameter;
	//BATTERY_TYPE si_battery_type;
	int si_battery_cells;
	float si_battery_ah;
} mc_configuration;

//////////////code in utils.h/////////////////////

#define UTILS_LP_FAST(value, sample, filter_constant)	(value -= (filter_constant) * ((value) - (sample)))

//////////////code in utils.c/////////////////////

float utils_angle_difference(float angle1, float angle2) {
//	utils_norm_angle(&angle1);
//	utils_norm_angle(&angle2);
//
//	if (fabsf(angle1 - angle2) > 180.0) {
//		if (angle1 < angle2) {
//			angle1 += 360.0;
//		} else {
//			angle2 += 360.0;
//		}
//	}
//
//	return angle1 - angle2;

	// Faster in most cases
	float difference = angle1 - angle2;
	while (difference < -180.0) difference += 2.0 * 180.0;
	while (difference > 180.0) difference -= 2.0 * 180.0;
	return difference;
}

int utils_truncate_number(float *number, float min, float max) {
	int did_trunc = 0;

	if (*number > max) {
		*number = max;
		did_trunc = 1;
	} else if (*number < min) {
		*number = min;
		did_trunc = 1;
	}

	return did_trunc;
}

int utils_truncate_number_int(int *number, int min, int max) {
	int did_trunc = 0;

	if (*number > max) {
		*number = max;
		did_trunc = 1;
	} else if (*number < min) {
		*number = min;
		did_trunc = 1;
	}

	return did_trunc;
}

int utils_truncate_number_abs(float *number, float max) {
	int did_trunc = 0;

	if (*number > max) {
		*number = max;
		did_trunc = 1;
	} else if (*number < -max) {
		*number = -max;
		did_trunc = 1;
	}

	return did_trunc;
}

/////////////////Code in mcpwm_foc.c//////////
typedef struct {
	float id_target;
	float iq_target;
	float max_duty;
	float duty_now;
	float phase;
	float i_alpha;
	float i_beta;
	float i_abs;
	float i_abs_filter;
	float i_bus;
	float v_bus;
	float v_alpha;
	float v_beta;
	float mod_d;
	float mod_q;
	float id;
	float iq;
	float id_filter;
	float iq_filter;
	float vd;
	float vq;
	float vd_int;
	float vq_int;
	float speed_rad_s;
	uint32_t svm_sector;
} motor_state_t;


static volatile motor_state_t m_motor_state;

// We can just regard speed_rad_s as insert point

float mcpwm_foc_get_rpm(void) {
	return m_motor_state.speed_rad_s / ((2.0 * M_PI) / 60.0);
//	return m_speed_est_fast / ((2.0 * M_PI) / 60.0);
}

/////////////////speed calculation, very deep in VESC, ignore now//////////////////////

static void pll_run(float phase, float dt, volatile float *phase_var,
		volatile float *speed_var) {
	UTILS_NAN_ZERO(*phase_var);
	float delta_theta = phase - *phase_var;
	utils_norm_angle_rad(&delta_theta);
	UTILS_NAN_ZERO(*speed_var);
	*phase_var += (*speed_var + m_conf->foc_pll_kp * delta_theta) * dt;
	utils_norm_angle_rad((float*)phase_var);
	*speed_var += m_conf->foc_pll_ki * delta_theta * dt;
}

static volatile float m_pll_phase;
static volatile float m_pll_speed;
/// Need further exploration of dt
pll_run(m_motor_state.phase, dt, &m_pll_phase, &m_pll_speed);
	m_motor_state.speed_rad_s = m_pll_speed;

//////////////////////////

static volatile mc_configuration *m_conf;

//////////////////////////pos control

static void run_pid_control_pos(float angle_now, float angle_set, float dt) {
	static float i_term = 0;
	static float prev_error = 0;
	float p_term;
	float d_term;
/*
	// PID is off. Return.
	if (m_control_mode != CONTROL_MODE_POS) {
		i_term = 0;
		prev_error = 0;
		return;
	}
*/
	// Compute parameters
	float error = utils_angle_difference(angle_set, angle_now);
/*
	if (encoder_is_configured()) {
		if (m_conf->foc_encoder_inverted) {
			error = -error;
		}
	}
*/
	p_term = error * m_conf->p_pid_kp;
	i_term += error * (m_conf->p_pid_ki * dt);

	// Average DT for the D term when the error does not change. This likely
	// happens at low speed when the position resolution is low and several
	// control iterations run without position updates.
	// TODO: Are there problems with this approach?
	static float dt_int = 0.0;
	dt_int += dt;
	if (error == prev_error) {
		d_term = 0.0;
	} else {
		d_term = (error - prev_error) * (m_conf->p_pid_kd / dt_int);
		dt_int = 0.0;
	}

	// Filter D
	static float d_filter = 0.0;
	UTILS_LP_FAST(d_filter, d_term, m_conf->p_pid_kd_filter);
	d_term = d_filter;


	// I-term wind-up protection
	utils_truncate_number_abs(&p_term, 1.0);
	utils_truncate_number_abs(&i_term, 1.0 - fabsf(p_term));

	// Store previous error
	prev_error = error;

	// Calculate output
	float output = p_term + i_term + d_term;
	utils_truncate_number(&output, -1.0, 1.0);
//lo_current_max 在pwm里是1？output从0到1，m_iq_set就是output
//So the logic of this part is: Only when there is an encoder and the encoder index is not found yet
// Rotate the motor with 40 % power until the encoder index is found.
// Otherwise, rotate the motor with output * lo_current_max
	if (encoder_is_configured()) {
		if (encoder_index_found()) {
			m_iq_set = output * m_conf->lo_current_max;
		} else {
			// Rotate the motor with 40 % power until the encoder index is found.
			m_iq_set = 0.4 * m_conf->lo_current_max;
		}
	} else {
		m_iq_set = output * m_conf->lo_current_max;
	}
}

//////////////////////////speed control


static volatile float m_speed_pid_set_rpm;

void mcpwm_foc_set_pid_speed(float rpm) {
	//m_control_mode = CONTROL_MODE_SPEED;
	m_speed_pid_set_rpm = rpm;
/*
	if (m_state != MC_STATE_RUNNING) {
		m_state = MC_STATE_RUNNING;
	}
*/
}

static void run_pid_control_speed(float dt) {
	static float i_term = 0.0;
	static float prev_error = 0.0;
	float p_term;
	float d_term;
/*
	// PID is off. Return.
	if (m_control_mode != CONTROL_MODE_SPEED) {
		i_term = 0.0;
		prev_error = 0.0;
		return;
	}
*/
	const float rpm = mcpwm_foc_get_rpm();
	float error = m_speed_pid_set_rpm - rpm;

	// Too low RPM set. Reset state and return.
	if (fabsf(m_speed_pid_set_rpm) < m_conf->s_pid_min_erpm) {
		i_term = 0.0;
		prev_error = error;
		return;
	}

	// Compute parameters
	p_term = error * m_conf->s_pid_kp * (1.0 / 20.0);
	i_term += error * (m_conf->s_pid_ki * dt) * (1.0 / 20.0);
	d_term = (error - prev_error) * (m_conf->s_pid_kd / dt) * (1.0 / 20.0);

	// Filter D
	static float d_filter = 0.0;
	UTILS_LP_FAST(d_filter, d_term, m_conf->s_pid_kd_filter);
	d_term = d_filter;

	// I-term wind-up protection
	utils_truncate_number(&i_term, -1.0, 1.0);

	// Store previous error
	prev_error = error;

	// Calculate output
	float output = p_term + i_term + d_term;
	utils_truncate_number(&output, -1.0, 1.0);

	// Optionally disable braking
	if (!m_conf->s_pid_allow_braking) {
		if (rpm > 20.0 && output < 0.0) {
			output = 0.0;
		}

		if (rpm < -20.0 && output > 0.0) {
			output = 0.0;
		}
	}

	m_iq_set = output * m_conf->lo_current_max;
}
