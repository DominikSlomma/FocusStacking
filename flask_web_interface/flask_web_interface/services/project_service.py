# services/project_service.py
import os, re, yaml, tempfile, shutil
from datetime import datetime, timezone, timedelta
from werkzeug.utils import secure_filename
from flask import current_app

# ---------- Low-level IO ----------
def _atomic_write_yaml(path: str, data: dict):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    try: shutil.copyfile(path, path + ".bak")
    except FileNotFoundError: pass
    fd, tmp = tempfile.mkstemp(prefix=".yaml_tmp_", dir=os.path.dirname(path) or ".")
    try:
        with os.fdopen(fd, "w") as f:
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.remove(tmp)

def _read_yaml_or_empty(path: str) -> dict:
    try:
        with open(path, "r") as f:
            return yaml.safe_load(f) or {}
    except FileNotFoundError:
        return {}

# ---------- Helpers ----------
def _iso_now():
    # Sydney +10 (vereinfachend, DST ggf. später)
    return datetime.now(timezone(timedelta(hours=10))).isoformat()

def slugify(text: str) -> str:
    s = text.strip().lower()
    s = re.sub(r"[^a-z0-9._-]+", "-", s)
    s = re.sub(r"-{2,}", "-", s).strip("-")
    return s or "project"

# ---------- Paths ----------
def projects_base() -> str:
    return current_app.config["PROJECTS_BASE"]

def index_file() -> str:
    return current_app.config["INDEX_FILE"]

def proj_dir(slug: str) -> str:
    return os.path.join(projects_base(), slug)

def proj_yaml(slug: str) -> str:
    return os.path.join(proj_dir(slug), "project.yaml")

# ---------- Index ops ----------
def load_index():
    idx = _read_yaml_or_empty(index_file())
    idx.setdefault("projects", [])
    return idx

def save_index(idx: dict):
    _atomic_write_yaml(index_file(), idx)

def upsert_index_entry(name: str, slug: str, path: str):
    idx = load_index()
    for p in idx["projects"]:
        if p.get("slug") == slug or p.get("name") == name:
            p.update({"name": name, "slug": slug, "path": path})
            save_index(idx); return
    idx["projects"].append({"name": name, "slug": slug, "path": path})
    save_index(idx)

def remove_index_entry(slug_or_name: str):
    idx = load_index()
    idx["projects"] = [p for p in idx["projects"]
                       if p.get("slug") != slug_or_name and p.get("name") != slug_or_name]
    save_index(idx)

# ---------- High-level Project ops ----------
def create_project_yaml(name: str, images_fslist, params: dict, status="Init") -> str:
    assert name, "project name required"
    slug   = slugify(name)
    pdir   = proj_dir(slug)
    img_dir = os.path.join(pdir, "images")
    out_dir = os.path.join(pdir, "output")
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(out_dir, exist_ok=True)

    # Falls bereits vorhanden: bestehende YAML laden, um created_at zu erhalten
    existing = {}
    if os.path.exists(proj_yaml(slug)):
        existing = _read_yaml_or_empty(proj_yaml(slug))

    # Bilder speichern
    saved_images = []
    for i, file in enumerate(images_fslist or []):
        if file and getattr(file, "filename", ""):
            fname = f"{i:03d}_" + secure_filename(file.filename)
            fpath = os.path.join(img_dir, fname)
            file.save(fpath)
            saved_images.append(fpath.replace("\\", "/"))

    data = {
        "version": 1,
        "project": {
            "name": name,
            "slug": slug,
            "status": status,
            # nur setzen, wenn neu – sonst altes created_at behalten
            "created_at": (existing.get("project") or {}).get("created_at") or _iso_now(),
            "last_opened": (existing.get("project") or {}).get("last_opened"),
        },
        "paths": {
            "images": saved_images if saved_images else (existing.get("paths") or {}).get("images", []),
            "output_depth": os.path.join(out_dir, f"{slug}_depth.jpg").replace("\\", "/"),
            "output_sharp": os.path.join(out_dir, f"{slug}_sharpness.jpg").replace("\\", "/"),
            "output_exr":   os.path.join(out_dir, f"{slug}_depth.exr").replace("\\", "/"),
        },
        "params": {**(existing.get("params") or {}), **(params or {})},
    }

    _atomic_write_yaml(proj_yaml(slug), data)
    upsert_index_entry(name, slug, pdir.replace("\\", "/"))
    return slug  # praktisch, wenn du danach sofort damit arbeiten willst


def load_project(slug_or_name: str) -> dict | None:
    # by slug first
    p = proj_yaml(slug_or_name)
    if os.path.exists(p):
        return _read_yaml_or_empty(p)
    # else lookup in index by name
    idx = load_index()
    for e in idx["projects"]:
        if e.get("name") == slug_or_name or e.get("slug") == slug_or_name:
            return _read_yaml_or_empty(os.path.join(e["path"], "project.yaml"))
    return None

def set_last_opened(slug_or_name: str) -> bool:
    cfg = load_project(slug_or_name)
    if not cfg: return False
    slug = cfg["project"]["slug"]
    cfg["project"]["last_opened"] = _iso_now()
    _atomic_write_yaml(proj_yaml(slug), cfg)
    return True

def delete_project(slug_or_name: str):
    idx = load_index()
    match = next((p for p in idx["projects"]
                  if p.get("name")==slug_or_name or p.get("slug")==slug_or_name), None)
    if not match:
        return False, f"Projekt '{slug_or_name}' nicht gefunden."
    pdir = match.get("path")
    if pdir and os.path.isdir(pdir):
        shutil.rmtree(pdir)
    remove_index_entry(slug_or_name)
    return True, f"Projekt '{slug_or_name}' gelöscht."

def _set_status(slug: str, status: str, extra: dict | None = None):
    """Schreibt project.status = status in die YAML, atomar."""
    try:
        cfg = _read_yaml_or_empty(proj_yaml(slug))
        cfg.setdefault("project", {})["status"] = status
        if extra:
            cfg["project"].update(extra)
        _atomic_write_yaml(proj_yaml(slug), cfg)
    except Exception as e:
        print(f"[status] failed to update {slug}: {e}")

def set_status_at_yaml(yaml_path: str, status: str, extra: dict | None = None):
    cfg = _read_yaml_or_empty(yaml_path)
    cfg.setdefault("project", {})["status"] = status
    if extra:
        cfg["project"].update(extra)
    _atomic_write_yaml(yaml_path, cfg)

