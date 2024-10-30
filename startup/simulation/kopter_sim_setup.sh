export UAV_NAMESPACE=duckorange
#export PIX_SYM=/dev/ttyUSB_px4:921600

#export ODOM_TOPIC=/$UAV_NAMESPACE/es_ekf/odom
export ODOM_TOPIC=/$UAV_NAMESPACE/odometry

export CONTROL_TYPE=pid_cascade_node_yawrate
export CONTROL_PARAMS=custom_config/sensor_fusion_position_control.params.yaml
#export RC_MAPPING=
#export CARROT_PARAMS=custom_config/carrot_config_sf.yaml
