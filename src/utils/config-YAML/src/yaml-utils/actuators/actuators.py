import yaml


def load_actuators(config_data):
    return config_data.get("actuators", {})


def get_motor_configs(config_data):
    return config_data.get("actuators", {}).get("motors", [])


def get_servo_configs(config_data):
    return config_data.get("actuators", {}).get("servos", [])
