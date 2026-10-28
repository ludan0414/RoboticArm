"""Serialize existing executor samples and commands without additional CPS IO."""


def hardware_sample(iface, stamp_ns):
    valid = {name: bool(getattr(iface, attr, False)) for name, attr in (
        ('acs','last_acs_ok'), ('tcp','last_tcp_ok'), ('current','last_cur_ok'),
        ('command','last_cmd_ok'), ('io','last_io_ok'))}
    return dict(schema='elfin_executor_sample/v1', stamp_ns=stamp_ns,
        sample_monotonic_sec=getattr(iface, 'last_sample_mono', None),
        sequence=getattr(iface, '_refresh_n', None), sample_ok=valid,
        refresh_sec=getattr(iface, 'last_refresh_dt_s', None),
        q_rad=list(iface.current_positions) if valid['acs'] else [],
        qd_rad_s=list(iface.current_velocities) if valid['acs'] and
        getattr(iface, 'vel_source', 'none') != 'none' else [],
        velocity_source=getattr(iface, 'vel_source', 'none'),
        current_a=list(iface.current_currents) if valid['current'] else [],
        command_q_rad=list(iface.command_positions) if valid['command'] else [],
        tcp_mm_deg=list(iface.current_tcp_mm_deg) if valid['tcp'] else [],
        vacuum=dict(DI0=iface.vacuum_di0, DO0=iface.vacuum_do0, DO1=iface.vacuum_do1)
        if valid['io'] else None)


def command_record(trajectory, goal_id, stamp_ns, *, servo_grid_deg=None,
                   servo_time=None, hold_count=None):
    return dict(schema='elfin_execution_command/v1', goal_id=goal_id, stamp_ns=stamp_ns,
        joint_names=list(trajectory.joint_names),
        points=[dict(time_sec=p.time_from_start.sec+p.time_from_start.nanosec*1e-9,
                     positions_rad=list(p.positions), velocities_rad_s=list(p.velocities),
                     accelerations_rad_s2=list(p.accelerations)) for p in trajectory.points],
        servo_grid_deg=servo_grid_deg, servo_time_sec=servo_time, hold_count=hold_count,
        note='Submitted command, not proof of actual execution; correlate goal result and measured samples.')
