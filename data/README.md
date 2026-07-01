# Flight Comparison Data

This folder keeps compact Sport Cub datasets used for measured-data comparison and optional regeneration of browser payloads.

Raw lab exports, ROS bags, Foxglove CSV dumps, and large intermediate products should stay outside the repo, for example under a local ignored `work/data/<dataset_id>/raw/` directory. Committed files should be compact NPZ artifacts that are small enough to review and clone.

The current compact array contract is `sysid.timeseries.ragged.v1`: segment arrays are padded to the longest segment and `valid_mask` marks real samples. Required arrays are `time_s`, `valid_mask`, `control_meas`, and `pose_meas`, plus companion name arrays, `dataset_id`, `split_name`, and `system_dof`.

For `system_dof=6`, `pose_meas` is fixed to `[x_e, y_n, z_u, q_w, q_x, q_y, q_z]`. `control_meas` is fixed to `[thrust, aileron, elevator, rudder]`.

Datasets may include `direct_state_meas` carrying a converter's canonical state estimate. Optional groups such as `accel_meas`, `gyro_meas`, `mag_meas`, and `onboard_pose_est` may be added with matching `*_names` arrays when a dataset has those channels. `onboard_pose_est` is always `[x_e, y_n, z_u, q_w, q_x, q_y, q_z]`.
