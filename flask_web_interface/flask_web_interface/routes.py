# routes.py
from datetime import datetime
from flask import Blueprint, abort, jsonify, render_template, request, redirect, url_for, session, current_app, send_from_directory
import os, yaml, json, uuid 
from flask_web_interface.services.project_service import (
    create_project_yaml, load_project, set_last_opened,
    load_index, delete_project, _atomic_write_yaml, _read_yaml_or_empty, proj_yaml
)
from flask_web_interface.services.ros_service import (
    start_ros_launch_from_cfg, check_node_exists
)
from werkzeug.utils import secure_filename

from flask_web_interface.services.project_service import (
    proj_yaml, _read_yaml_or_empty, _atomic_write_yaml
)
from flask_web_interface.services.ros_service import start_ros_launch_from_cfg

from flask_web_interface.services.ros_service import (
    ensure_progress_subscriptions, read_progress
)

bp = Blueprint("web", __name__)

def _read_yaml_or_empty(path: str):
    try:
        with open(path, "r") as f:
            return yaml.safe_load(f) or {}
    except FileNotFoundError:
        return {}
    except Exception:
        return {}


@bp.route("/upload_temp", methods=["POST"])
def upload_temp():
    upload_id = request.form.get("upload_id") or uuid.uuid4().hex[:12]
    files = request.files.getlist("images")
    if not files:
        return jsonify({"error": "no files"}), 400

    # temp-Ziel: projects/_tmp/<upload_id>/images
    base = os.path.join(current_app.config["PROJECTS_BASE"], "_tmp", upload_id, "images")
    os.makedirs(base, exist_ok=True)

    saved = []
    for i, f in enumerate(files):
        if not f or not f.filename:
            continue
        fname = f"{i:03d}_" + secure_filename(f.filename)
        fpath = os.path.abspath(os.path.join(base, fname))
        f.save(fpath)
        # Pfad für weitere Verarbeitung (relativ ist ok; du kannst auch abs liefern)
        saved.append(fpath)

    return jsonify({"upload_id": upload_id, "paths": saved}), 200

@bp.route('/update_system')
def update_system():
    # ggf. Permission-Check hier
    #shutdown_server()
    # erreicht i.d.R. nicht mehr, weil shutdown_server den Prozess beendet
    return redirect(url_for('web.index'))

@bp.route('/image/<slug>/<path:filename>')
def serve_image(slug, filename):
    base = current_app.config.get("PROJECTS_BASE", "projects")
    if not os.path.isabs(base):
        base = os.path.abspath(base)   # <<< wichtig!
    directory = os.path.join(base, slug, "output")

    safe_name = secure_filename(os.path.basename(filename))
    full_path = os.path.join(directory, safe_name)
    print("SERVE:", full_path)

    if not os.path.isfile(full_path):
        abort(404)

    send = request.args.get('send', 'false').lower() == 'true'
    return send_from_directory(directory, safe_name, as_attachment=send)




def _unique_path(dst: str) -> str:
    if not os.path.exists(dst):
        return dst
    root, ext = os.path.splitext(dst)
    k = 1
    cand = f"{root}_{k}{ext}"
    while os.path.exists(cand):
        k += 1
        cand = f"{root}_{k}{ext}"
    return cand

@bp.route("/update_project", methods=["POST"])
def update_project():


    slug = (request.form.get("slug") or "").strip()
    if not slug:
        return redirect(url_for("web.index"))

    cfg = _read_yaml_or_empty(proj_yaml(slug))
    if not cfg:
        return redirect(url_for("web.index"))

    # Name (nur Anzeige; Slug/Pfad bleibt gleich)
    new_name = (request.form.get("projectName") or "").strip()
    if new_name:
        cfg.setdefault("project", {})["name"] = new_name

    # --- Parameter mergen ---
    params = cfg.get("params", {})

    def _getfloat(key, default=None):
        v = request.form.get(key, None)
        try:
            return float(v) if v not in (None, "",) else default
        except Exception:
            return default

    def _getint(key, default=None):
        v = request.form.get(key, None)
        try:
            return int(v) if v not in (None, "",) else default
        except Exception:
            return default

    cfg["params"] = {
        "ImgResize": _getfloat("ImgResize", params.get("ImgResize")),
        "LaplaceFilterSize": _getint("LaplaceFilterSize", params.get("LaplaceFilterSize")),
        "NumberofImagesinPyramid": _getint("NumberofImagesinPyramid", params.get("NumberofImagesinPyramid")),
        "PatchSizeX": _getint("PatchSizeX", params.get("PatchSizeX")),
        "PatchSizeY": _getint("PatchSizeY", params.get("PatchSizeY")),
        "z_spacing": _getfloat("z_spacing", params.get("z_spacing")),
    }

    # --- Bilder übernehmen ---
    images = list(cfg.get("paths", {}).get("images", []) or [])

    # 1) Pre-Uploads aus Hidden-Textarea verschieben
    uploaded_paths_json = request.form.get("uploaded_paths")
    if uploaded_paths_json:
        try:
            pre_paths = json.loads(uploaded_paths_json)
        except Exception:
            pre_paths = None
        if isinstance(pre_paths, list) and pre_paths:
            img_dir = os.path.join(current_app.config.get("PROJECTS_BASE", "projects"), slug, "images")
            os.makedirs(img_dir, exist_ok=True)
            for src in pre_paths:
                if not src or not os.path.exists(src):
                    continue
                basename = os.path.basename(src)
                dst = os.path.join(img_dir, basename)
                dst = _unique_path(dst)
                os.replace(src, dst)  # verschieben
                images.append(dst.replace("\\", "/"))

    # 2) Direkt mitgesendete Files (multipart)
    files = request.files.getlist("images")
    if files:
        img_dir = os.path.join(current_app.config.get("PROJECTS_BASE", "projects"), slug, "images")
        os.makedirs(img_dir, exist_ok=True)
        start_idx = len(images)
        for i, f in enumerate(files):
            if not f or not f.filename:
                continue
            fname = f"{start_idx + i:03d}_" + secure_filename(f.filename)
            dst = os.path.join(img_dir, fname)
            dst = _unique_path(dst)
            f.save(dst)
            images.append(dst.replace("\\", "/"))

    # Deduplizieren (Reihenfolge erhalten)
    images = list(dict.fromkeys(images))

    cfg.setdefault("paths", {})
    cfg["paths"]["images"] = images

    # Status
    action = request.form.get("action")
    if action == "update_run":
        cfg.setdefault("project", {})["status"] = "Running"
    else:
        cfg.setdefault("project", {})["status"] = "Not started"

    # Speichern
    _atomic_write_yaml(proj_yaml(slug), cfg)

    # ggf. starten
    if action == "update_run":
        start_ros_launch_from_cfg(cfg)

    # Zurück zur Projektseite (besserer UX als Index).
    # Du hast aktuell open_project nur als POST. Entweder:
    # A) Redirect zurück zum Index:
    # return redirect(url_for("web.index"), code=303)
    #
    # B) ODER: Wenn du eine GET-Route z.B. /project/<slug> hast:
    # return redirect(url_for("web.project_detail", slug=slug), code=303)
    #
    # C) ODER: POST open_project erneut (nicht so schön, aber möglich):
    #    Wir machen hier Version A für jetzt:
    return redirect(url_for("web.project_detail", slug=slug), code=303)

# routes.py
@bp.route("/project/<slug>", methods=["GET"])
def project_detail(slug):
    cfg = load_project(slug)
    if not cfg:
        return redirect(url_for("web.index"))

    # optional: last_opened setzen
    try:
        set_last_opened(slug)
    except Exception:
        pass

    # exist prüfen
    paths = cfg.get("paths", {}) or {}
    out_depth = paths.get("output_depth")
    if out_depth:
        out_dir = os.path.dirname(out_depth)
    else:
        base = current_app.config.get("PROJECTS_BASE", "projects")
        out_dir = os.path.join(base, slug, "output")

    exts = {".jpg", ".jpeg", ".png", ".exr"}
    exist = os.path.isdir(out_dir) and any(
        os.path.splitext(f)[1].lower() in exts for f in os.listdir(out_dir)
    )
    return render_template("includes/open_project.html", data=cfg, exist=exist)


@bp.errorhandler(404)
def page_not_found(e):
    return render_template('404.html'), 404

@bp.route("/create_project", methods=["GET", "POST"])
def create_project():
    if request.method != "POST":
        return redirect(url_for("web.new_project"))

    name = (request.form.get("projectName") or "").strip()
    if not name:
        return redirect(url_for("web.new_project"))

    # Parameter einlesen
    params = {
        "ImgResize": request.form.get("ImgResize", type=float),
        "LaplaceFilterSize": request.form.get("LaplaceFilterSize", type=int),
        "NumberofImagesinPyramid": request.form.get("NumberofImagesinPyramid", type=int),
        "PatchSizeX": request.form.get("PatchSizeX", type=int),
        "PatchSizeY": request.form.get("PatchSizeY", type=int),
        "z_spacing": request.form.get("z_spacing", type=float),
    }

    action = request.form.get("action")
    status = "Running" if action == "save_run" else "Not started"


    # Eingaben zu Bildern
    files = request.files.getlist("images")  # direkte Uploads im selben Submit
    uploaded_paths_json = request.form.get("uploaded_paths")  # pre-uploaded (AJAX)
    preuploaded_paths = None
    if uploaded_paths_json:
        try:
            preuploaded_paths = json.loads(uploaded_paths_json)
        except Exception:
            preuploaded_paths = None
    if not isinstance(preuploaded_paths, list):
        preuploaded_paths = None

    # Projekt anlegen – gib hier NUR files rein (falls vorhanden),
    # preuploaded verschieben wir im Anschluss.
    slug = create_project_yaml(
        name,
        files if files and not preuploaded_paths else None,
        params,
        status=status
    )
    # ^^^ ACHTUNG: create_project_yaml muss str zurückgeben!

    # Falls es pre-uploaded Dateien gab: in den Projekt-Ordner verschieben
    if preuploaded_paths:
        base = current_app.config.get("PROJECTS_BASE", "projects")
        img_dir = os.path.join(base, slug, "images")
        os.makedirs(img_dir, exist_ok=True)

        # Aktuelle cfg laden, um vorhandene images zu lesen & weiter zu nummerieren
        cfg = load_project(slug) or {}
        paths = cfg.get("paths", {}) or {}
        existing = list(paths.get("images", []) or [])
        count = len(existing)

        moved = []
        for i, src in enumerate(preuploaded_paths):
            if not src or not os.path.exists(src):
                continue
            # vorhandene Nummerierung weiterführen
            fname = f"{count + i:03d}_" + secure_filename(os.path.basename(src))
            dst = os.path.join(img_dir, fname)
            dst = _unique_path(dst)
            try:
                os.replace(src, dst)  # move
                moved.append(dst.replace("\\", "/"))
            except Exception as e:
                print("Move failed:", src, e)

        # YAML aktualisieren (dedupe + atomar speichern)
        new_images = list(dict.fromkeys(existing + moved))
        cfg.setdefault("paths", {})
        cfg["paths"]["images"] = new_images

        _atomic_write_yaml(proj_yaml(slug), cfg)

    # Optional: ROS starten
    if action == "save_run":
        cfg = load_project(slug)
        if cfg:
            start_ros_launch_from_cfg(cfg)

    # Zurück zum Dashboard (keine unnötigen Query-Params anfügen)
    return redirect(url_for("web.index"), code=303)


@bp.route("/open_project", methods=['POST'])
def open_project():
    ident = request.form.get("projectname")
    cfg = load_project(ident)
    if not cfg:
        return redirect(url_for("web.index"))

    # zuletzt geöffnet aktualisieren
    try:
        set_last_opened(ident)
    except Exception:
        pass

    # --- Aus cfg den Output-Ordner ermitteln ---
    paths = cfg.get("paths", {}) or {}
    out_depth = paths.get("output_depth")
    if out_depth:
        out_dir = os.path.dirname(out_depth)
    else:
        # Fallback, falls YAML noch keine Outputs enthält
        slug = (cfg.get("project") or {}).get("slug") or ident
        base = current_app.config.get("PROJECTS_BASE", "projects")
        out_dir = os.path.join(base, slug, "output")

    # exist = gibt es irgendeine sinnvolle Output-Datei?
    exts = {".jpg", ".jpeg", ".png", ".exr"}
    exists = os.path.isdir(out_dir) and any(
        os.path.splitext(f)[1].lower() in exts for f in os.listdir(out_dir)
    )
    return render_template("includes/open_project.html", data=cfg, exist=exists)


@bp.route('/delete', methods=['POST'])
def delete():
    ident = request.form.get("projectname")
    ok, msg = delete_project(ident)
    print(msg)
    
    return redirect(url_for("web.index"))

@bp.route('/new_project')
def new_project():
    return render_template('includes/new_project.html')

@bp.route('/logout')
def logout():
    session.pop('user', None)
    return redirect(url_for('web.login'))

@bp.route("/login", methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        if request.form['username'] == 'admin' and request.form['password'] == 'admin':
            session['user'] = 'admin'
            return redirect(url_for('web.index'))
        return render_template('accounts/login.html')
    return render_template('accounts/login.html')

@bp.app_context_processor
def inject_nav_projects():
    """Stellt der Sidebar in allen Templates eine leichte Projektliste bereit."""
    try:
        idx = load_index()
        projs = []
        for p in idx.get("projects", []):
            name = p.get("name") or p.get("slug") or "project"
            slug = p.get("slug") or name
            projs.append({"name": name, "slug": slug})
        # Sidebar alphabetisch nach Name
        projs.sort(key=lambda x: x["name"].lower())
        return {"nav_projects": projs}
    except Exception:
        # Fallback, Sidebar wird einfach leer angezeigt
        return {"nav_projects": []}


# @bp.route("/api/projects/status")
# def api_projects_status():
#     # Login-Schutz wie /index
#     if 'user' not in session:
#         return jsonify({"error": "unauthenticated"}), 401

#     idx = load_index()
#     items = []
#     for p in idx.get("projects", []):
#         slug = p.get("slug") or p.get("name")
#         if not slug:
#             continue
#         cfg = _read_yaml_or_empty(proj_yaml(slug))
#         proj = cfg.get("project") or {}
#         status = (proj.get("status") or "").strip()
#         last_rc = proj.get("last_rc")
#         items.append({"slug": slug, "status": status, "last_rc": last_rc})
#     return jsonify({"items": items})

@bp.route("/api/projects/status")
def api_projects_status():
    if 'user' not in session:
        return jsonify({"error": "unauthenticated"}), 401

    idx = load_index()
    items = []
    slugs_running = []

    for p in idx.get("projects", []):
        slug = p.get("slug") or p.get("name")
        if not slug:
            continue
        cfg = _read_yaml_or_empty(proj_yaml(slug))
        proj = cfg.get("project") or {}
        status = (proj.get("status") or "").strip()
        items.append({"slug": slug, "status": status})

        if status.lower() == "running":
            slugs_running.append(slug)

    # ROS-Subscriptions für Running-Projekte sicherstellen
    ensure_progress_subscriptions(slugs_running)
    prog = read_progress(slugs_running)

    # Progress injecten
    for it in items:
        it["progress"] = prog.get(it["slug"]) if it["status"].lower() == "running" else None

    return jsonify({"items": items})

@bp.route('/index')
@bp.route('/')
def index():
    if 'user' not in session:
        return redirect(url_for('web.login'))

    idx = load_index()
    projects = idx.get("projects", [])

    # Liste bauen
    items = []
    for p in projects:
        name = p.get("name") or p.get("slug") or "project"
        slug = p.get("slug") or name
        path = p.get("path") or os.path.join(current_app.config["PROJECTS_BASE"], slug)

        cfg = _read_yaml_or_empty(os.path.join(path, "project.yaml"))
        pj  = cfg.get("project") or {}
        st  = pj.get("status", "Init")
        created_at = pj.get("created_at")

        items.append({"name": name, "slug": slug, "path": path, "status": st, "created_at": created_at})

    # Node-Check
    for it in items:
        st = (it["status"] or "").lower()
        if st == "running":
            node = "/cpu_node_" + it["slug"]
            # if not check_node_exists(node):
            #     it["status"] = "Stopped"

    # Sortierung
    sortdir = request.args.get("sortdir")
    if sortdir and ":" in sortdir:
        sort_key, sort_dir = sortdir.split(":", 1)
    else:
        sort_key = request.args.get("sort", "name")
        sort_dir = request.args.get("dir", "asc")
    reverse = (sort_dir == "desc")

    def sort_value(p):
        if sort_key == "name":
            return (p["name"] or "").lower()
        if sort_key == "created":
            try:
                return datetime.fromisoformat(p.get("created_at") or "")
            except Exception:
                return datetime.min
        return (p["name"] or "").lower()

    items.sort(key=sort_value, reverse=reverse)

    # Nur Meta-Sachen in die Session, NICHT die Liste
    session["username"] = session.get('user')
    session["Project_Name"] = "Focus Stacking"

    # Sortierte Liste direkt ans Template geben
    return render_template('index.html', session=session, items=items, poll_status=True)
