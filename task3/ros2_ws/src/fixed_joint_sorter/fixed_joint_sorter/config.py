from pathlib import Path

import yaml


class ConfigError(ValueError):
    pass


def load_config(path):
    file_path = Path(path).expanduser()
    if not file_path.is_file():
        raise ConfigError("找不到配置文件: {}".format(file_path))
    with file_path.open("r", encoding="utf-8") as handle:
        cfg = yaml.safe_load(handle)
    if not isinstance(cfg, dict):
        raise ConfigError("配置文件格式错误")
    return cfg


def _angles(value, label, cfg, allow_null):
    if value is None and allow_null:
        return
    if not isinstance(value, list) or len(value) != 6:
        raise ConfigError("{} 必须是 6 个关节角".format(label))
    lower = cfg["robot"]["joint_min_deg"]
    upper = cfg["robot"]["joint_max_deg"]
    for index, angle in enumerate(value):
        if angle is None or not lower[index] <= float(angle) <= upper[index]:
            raise ConfigError("{} 的 J{}={} 超出 [{}, {}]".format(label, index + 1, angle, lower[index], upper[index]))


def validate_config(cfg, live=False):
    for key in ("robot", "vision", "class_routes", "inspection_home_angles", "stations", "bins"):
        if key not in cfg:
            raise ConfigError("缺少配置: {}".format(key))
    if len(cfg["stations"]) != 6:
        raise ConfigError("stations 必须正好有 6 个工位")
    if set(cfg["class_routes"].values()) - set(cfg["bins"]):
        raise ConfigError("class_routes 指向了不存在的放置区")
    for index, station in enumerate(cfg["stations"]):
        for key in ("approach_angles", "pick_angles", "retreat_angles"):
            _angles(station.get(key), "stations[{}].{}".format(index, key), cfg, not live)
    _angles(cfg.get("inspection_home_angles"), "inspection_home_angles", cfg, not live)
    for bin_name, bin_cfg in cfg["bins"].items():
        if bin_cfg.get("expected_count") != 3:
            raise ConfigError("{}.expected_count 必须是 3".format(bin_name))
        drop_point = bin_cfg.get("drop_point")
        for key in ("approach_angles", "place_angles", "retreat_angles"):
            _angles(drop_point.get(key), "{}.drop_point.{}".format(bin_name, key), cfg, not live)
