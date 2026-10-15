"""系统级机器人模型配置的读取与解析（F174）。

配置文件：``config/robot_model.yaml``（本包，仓内默认）与 ``~/.a3/robot_model.yaml``
（本机覆盖，字段级 merge）。所有消费方通过本模块解析，不要各自写死描述包名。

用法（Python）::

    from a3_robot_model import robot_model
    cfg = robot_model.load()
    share = robot_model.description_share(cfg)      # .../share/a3_description_6j
    urdf = robot_model.urdf_path(cfg)               # .../urdf/el_a3.urdf

用法（shell / 排障）::

    python3 -m a3_robot_model.robot_model                # 打印解析后的完整配置
    python3 -m a3_robot_model.robot_model --get urdf_static
    python3 -m a3_robot_model.robot_model --path          # 实际生效的配置文件
"""

import argparse
import json
import os
import sys
from typing import Dict, Iterable, List, Optional

import yaml

CONFIG_RELPATH = "config/robot_model.yaml"
USER_CONFIG_PATH = os.path.expanduser("~/.a3/robot_model.yaml")

# 兜底默认值：仅在配置文件缺失（例如包未安装）时生效，正常走 robot_model.yaml。
DEFAULT_CONFIG: Dict[str, object] = {
    "description_package": "a3_description_6j",
    "urdf_xacro": "urdf/el_a3.urdf.xacro",
    "urdf_static": "urdf/el_a3.urdf",
    "controllers": "config/el_a3_controllers.yaml",
    "joint_names": [
        "L1_joint", "L2_joint", "L3_joint", "L4_joint", "L5_joint", "L6_joint",
    ],
    "include_gripper": False,
    "srdf": "el_a3_6j.srdf",
    "motor_map": "motor_map_6j.yaml",
}


def _candidate_package_configs() -> List[str]:
    """本包 config/robot_model.yaml 的候选路径（已安装 share 优先，源码树兜底）。"""
    paths: List[str] = []
    try:
        from ament_index_python.packages import get_package_share_directory

        paths.append(
            os.path.join(get_package_share_directory("a3_robot_model"), CONFIG_RELPATH)
        )
    except Exception:  # noqa: BLE001  (包未 source / 未安装)
        pass
    paths.append(
        os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), CONFIG_RELPATH
        )
    )
    return paths


def config_file_paths() -> List[str]:
    """实际生效的配置文件列表：包内默认 + 本机覆盖（存在才列出），按覆盖顺序。"""
    found: List[str] = []
    for path in _candidate_package_configs():
        if os.path.isfile(path):
            found.append(path)
            break
    if os.path.isfile(USER_CONFIG_PATH):
        found.append(USER_CONFIG_PATH)
    return found


def _read_yaml(path: str) -> Dict[str, object]:
    try:
        with open(path, "r", encoding="utf-8") as fh:
            doc = yaml.safe_load(fh) or {}
    except (OSError, yaml.YAMLError):
        return {}
    if not isinstance(doc, dict):
        return {}
    section = doc.get("robot_model")
    return section if isinstance(section, dict) else doc


def load(overrides: Optional[Dict[str, object]] = None) -> Dict[str, object]:
    """解析配置：包内默认 ← ~/.a3 覆盖 ← overrides 显式覆盖。"""
    cfg: Dict[str, object] = dict(DEFAULT_CONFIG)
    for path in config_file_paths():
        cfg.update(_read_yaml(path))
    if overrides:
        cfg.update({k: v for k, v in overrides.items() if v is not None})
    cfg["joint_names"] = [str(n) for n in cfg.get("joint_names") or []]
    cfg["include_gripper"] = _as_bool(cfg.get("include_gripper"))
    return cfg


def _as_bool(value: object) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() not in ("false", "0", "no", "off", "")


def description_share(cfg: Optional[Dict[str, object]] = None) -> str:
    """描述包（URDF 宿主包）的 share 目录。"""
    from ament_index_python.packages import get_package_share_directory

    cfg = cfg or load()
    return get_package_share_directory(str(cfg["description_package"]))


def xacro_path(cfg: Optional[Dict[str, object]] = None) -> str:
    cfg = cfg or load()
    return os.path.join(description_share(cfg), str(cfg["urdf_xacro"]))


def urdf_path(cfg: Optional[Dict[str, object]] = None) -> str:
    """静态 URDF 绝对路径（Pinocchio / 关节限位读取用）。"""
    cfg = cfg or load()
    return os.path.join(description_share(cfg), str(cfg["urdf_static"]))


def controllers_path(cfg: Optional[Dict[str, object]] = None) -> str:
    cfg = cfg or load()
    return os.path.join(description_share(cfg), str(cfg["controllers"]))


def joint_names(cfg: Optional[Dict[str, object]] = None) -> List[str]:
    cfg = cfg or load()
    return list(cfg["joint_names"])


def include_gripper(cfg: Optional[Dict[str, object]] = None) -> bool:
    cfg = cfg or load()
    return bool(cfg["include_gripper"])


def _split_pkg_path(value: str) -> Iterable[str]:
    return str(value).split("/", 1)


def _cli(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="A3 系统级机器人模型配置（F174）：打印/查询解析后的配置"
    )
    parser.add_argument("--get", metavar="KEY", help="只打印某个字段（列表/字典输出 JSON）")
    parser.add_argument("--path", action="store_true", help="打印实际生效的配置文件路径")
    parser.add_argument(
        "--resolved", action="store_true",
        help="额外解析 description_package 为绝对路径（urdf_static/xacro/controllers）",
    )
    args = parser.parse_args(argv)

    if args.path:
        for path in config_file_paths():
            print(path)
        return 0

    cfg = load()
    if args.get:
        if args.get not in cfg:
            print(f"unknown key: {args.get}", file=sys.stderr)
            return 2
        value = cfg[args.get]
        if isinstance(value, bool):          # shell 友好：true / false
            print("true" if value else "false")
        elif isinstance(value, (list, dict)):
            print(json.dumps(value, ensure_ascii=False))
        else:
            print(value)
        return 0

    out = dict(cfg)
    if args.resolved:
        try:
            out["urdf_static_abs"] = urdf_path(cfg)
            out["urdf_xacro_abs"] = xacro_path(cfg)
            out["controllers_abs"] = controllers_path(cfg)
        except Exception as exc:  # noqa: BLE001
            print(f"cannot resolve description package: {exc}", file=sys.stderr)
            return 1
    print(yaml.safe_dump({"robot_model": out}, allow_unicode=True, sort_keys=False))
    return 0


if __name__ == "__main__":
    sys.exit(_cli())
