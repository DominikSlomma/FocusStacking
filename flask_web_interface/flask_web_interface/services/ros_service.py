# services/ros_service.py
import os, yaml, uuid, multiprocessing as mp
from typing import Dict, Optional

from launch import LaunchService, LaunchDescription
from launch_ros.actions import Node as LaunchNode
from launch.actions import RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit

# services/ros_service.py (oben ergänzen)
import threading
import rclpy
from rclpy.node import Node as RclpyNode
# services/ros_service.py (oben ergänzen)
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from flask_web_interface.services.project_service import proj_yaml, _read_yaml_or_empty, _atomic_write_yaml, _set_status, set_status_at_yaml

_rclpy_ready = False
_graph_node: RclpyNode | None = None
_spin_thread: threading.Thread | None = None

_progress_lock = threading.Lock()
_progress_vals: dict[str, float] = {}
_progress_subs: dict[str, object] = {}   # slug -> Subscription

def _ensure_graph():
    """Einmalig rclpy starten und eine Graph-Node im Hintergrund spinnen."""
    global _rclpy_ready, _graph_node, _spin_thread
    if _rclpy_ready:
        return
    rclpy.init(args=None)
    _graph_node = rclpy.create_node("web_graph_probe")
    _spin_thread = threading.Thread(target=rclpy.spin, args=(_graph_node,), daemon=True)
    _spin_thread.start()
    _rclpy_ready = True

def check_node_exists(full_name: str) -> bool:
    """
    Prüft, ob eine Node mit diesem Namen im ROS-Graph existiert.
    Akzeptiert 'name', '/name' oder '/ns/name'.
    """
    _ensure_graph()
    full_name = (full_name or "").strip()
    if not full_name:
        return False
    pairs = _graph_node.get_node_names_and_namespaces()  # List[Tuple[name, ns]]
    for name, ns in pairs:
        fqn = f"{ns}/{name}".replace("//", "/") if ns else f"/{name}"
        if full_name in {name, f"/{name}", fqn}:
            return True
    return False

# ---- Jobs Registry (im Flask-Prozess) ----
_JOBS: Dict[str, dict] = {}   # job_id -> {"proc": Process, "queue": mp.Queue, "project": str, "last_rc": Optional[int]}

def _worker_launch(project: str, params: dict, images: list,
                   out_depth: str, out_sharp: str, rc_queue: mp.Queue):
    from launch import LaunchService, LaunchDescription
    from launch_ros.actions import Node as LaunchNode
    from launch.actions import RegisterEventHandler, OpaqueFunction
    from launch.event_handlers import OnProcessExit

    node = LaunchNode(
        package="fs_backend",
        executable="cpu_node",
        name=f"cpu_node_{project}",   # <— HIER: 'project' statt 'project_slug'
        namespace=f"/{project}",
        output="screen",
        parameters=[{
            "image_dir": images,                           # vector<string>
            "kernel_size_laplacian": params.get("LaplaceFilterSize", 5),
            "num_pyr_lvl": params.get("NumberofImagesinPyramid", 1),
            "sharpness_patch_size_x": params.get("PatchSizeX", 5),
            "sharpness_patch_size_y": params.get("PatchSizeY", 5),
            "img_resize": float(params.get("ImgResize", 1.0)),
            "z_spacing": float(params.get("z_spacing", 1.0)),
            "output_path_sharpness": out_sharp,
            "output_path_depth": out_depth,
            # optional – nur wenn im C++ deklariert:
            "project_id": project,
        }],
        arguments=["--ros-args", "--log-level", "info"],
    )

    def _report_exit(context):
        # ProcessExited-Event einsammeln
        ev = getattr(context.locals, "event", None)
        rc = None
        if ev is not None:
            rc = getattr(ev, "returncode", None)
            if rc is None:
                rc = getattr(ev, "exit_code", None)
        if rc is None:
            rc = -1  # Fallback, falls API anders ist
        try:
            rc = int(rc)
        except Exception:
            rc = -1
        rc_queue.put(rc)
        return []  # OpaqueFunction muss eine Sequenz von Actions zurückgeben


    ld = LaunchDescription([
        node,
        RegisterEventHandler(
            OnProcessExit(target_action=node, on_exit=[OpaqueFunction(function=_report_exit)])
        ),
    ])

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()


def start_ros_launch_from_cfg(cfg: dict) -> dict:
    proj = (cfg.get("project") or {})
    slug = proj.get("slug") or proj.get("name")
    params  = cfg.get("params", {}) or {}
    paths   = cfg.get("paths", {}) or {}
    images   = paths.get("images") or []
    out_depth = paths.get("output_depth")
    out_sharp = paths.get("output_sharp")

    # YAML-Pfad im App-Kontext bestimmen
    yaml_path = proj_yaml(slug)

    # Status sofort auf Running (pfadbasierend, kein current_app nötig)
    set_status_at_yaml(yaml_path, "Running")

    job_id = uuid.uuid4().hex[:10]
    q = mp.Queue(maxsize=1)
    p = mp.Process(
        target=_worker_launch,
        args=(slug, params, images, out_depth, out_sharp, q),
        daemon=True,
    )
    _JOBS[job_id] = {
        "proc": p, "queue": q, "project": slug,
        "yaml_path": yaml_path, "last_rc": None
    }

    print(f"[launch] starting fs_backend cpu_node for slug={slug}, job_id={job_id}")
    p.start()

    # Monitor-Thread: nach Exit Done/Failed setzen – OHNE App-Kontext
    def _monitor():
        try:
            rc = q.get()
        except Exception:
            rc = -1

        print("RC val")
        print(rc)
        _JOBS[job_id]["last_rc"] = rc
        status = "Done" if rc == 0 else "Failed"
        set_status_at_yaml(yaml_path, status, extra={"last_rc": rc})

    threading.Thread(target=_monitor, daemon=True).start()
    return {"ok": True, "job_id": job_id, "project": slug}


def poll_job(job_id: str) -> Optional[int]:
    """Nicht blockierend den Exitcode abholen; None = läuft noch."""
    meta = _JOBS.get(job_id)
    if not meta:
        return None
    q = meta["queue"]
    if meta["last_rc"] is not None:
        return meta["last_rc"]
    try:
        rc = q.get_nowait()
        meta["last_rc"] = rc
        return rc
    except Exception:
        return None

def stop_job(job_id: str) -> bool:
    meta = _JOBS.get(job_id)
    if not meta:
        return False
    try:
        if meta["proc"].is_alive():
            meta["proc"].terminate()
            meta["proc"].join(timeout=2.0)
        set_status_at_yaml(meta["yaml_path"], "Stopped")
    finally:
        _JOBS.pop(job_id, None)
    return True


def list_jobs() -> Dict[str, dict]:
    out = {}
    for jid, m in _JOBS.items():
        out[jid] = {
            "project": m["project"],
            "alive": m["proc"].is_alive(),
            "rc": m["last_rc"],
            "pid": m["proc"].pid,
        }
    return out


def _ensure_progress_subscription(slug: str):
    """Einmalig auf /<slug>/progress subscriben (transient_local)."""
    if not slug:
        return
    _ensure_graph()  # stellt _graph_node + Spin-Thread sicher
    if slug in _progress_subs:
        return

    qos = QoSProfile(depth=1)
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    qos.durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL  # "latched"-ähnlich

    topic = f"/{slug}/progress"

    def _cb(msg: Float32, s=slug):
        with _progress_lock:
            _progress_vals[s] = float(msg.data)

    sub = _graph_node.create_subscription(Float32, topic, _cb, qos)
    _progress_subs[slug] = sub  # Referenz halten, sonst wird sie GC'd

def ensure_progress_subscriptions(slugs: list[str]):
    for s in slugs:
        _ensure_progress_subscription(s)

def read_progress(slugs: list[str]) -> dict[str, float | None]:
    with _progress_lock:
        return {s: _progress_vals.get(s) for s in slugs}

# # (Optional) aufräumen, wenn du willst:
# def drop_progress_subscriptions(keep_slugs: set[str]):
#     for s, sub in list(_progress_subs.items()):
#         if s not in keep_slugs:
#             try:
#                 _graph_node.destroy_subscription(sub)
#             except Exception:
#                 pass
#             _progress_subs.pop(s, None)
#             with _progress_lock:
#                 _progress_vals.pop(s, None)
