static void run_pid_control_pos(float angle_now, float angle_set, float dt) {
	static float i_term = 0;
	static float prev_error = 0;
	float p_term;
	float d_term;

	// PID is off. Return.
	if (m_control_mode != CONTROL_MODE_POS) {
		i_term = 0;
		prev_error = 0;
		return;
	}

	// Compute parameters
	float error = utils_angle_difference(angle_set, angle_now);

	if (encoder_is_configured()) {
		if (m_conf->foc_encoder_inverted) {
			error = -error;
		}
	}

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

static void run_pid_control_speed(float dt) {
	static float i_term = 0.0;
	static float prev_error = 0.0;
	float p_term;
	float d_term;

	// PID is off. Return.
	if (m_control_mode != CONTROL_MODE_SPEED) {
		i_term = 0.0;
		prev_error = 0.0;
		return;
	}

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
