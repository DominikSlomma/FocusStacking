#from flask import Flask, render_template, request, jsonify
from flask import Flask, render_template, request, redirect, url_for, session
import os
import yaml
import json
from werkzeug.utils import secure_filename
import subprocess  # Für subprocess.Popen(...)
import os          # Für os.environ.copy()
import yaml        # Für yaml.dump(...)
import rclpy
from rclpy.node import Node
from flask import send_from_directory
from ros2node.api import get_node_names

app = Flask(__name__)
app.secret_key = 'secret_key'

if 'ROS_DOMAIN_ID' not in os.environ:
    os.environ['ROS_DOMAIN_ID'] = '0'  # anpassen falls nötig
    
rclpy.init()



def jpg_files_exist(dir_path):
    if not os.path.isdir(dir_path):
        return False  # Ordner existiert nicht
    for filename in os.listdir(dir_path):
        if filename.lower().endswith('.jpg'):
            return True  # Mindestens eine JPG-Datei gefunden
    return False  # Keine JPG-Dateien vorhanden

def check_node_exists(node_name):
    exists = False
    return exists

def start_ros_launch(data):
    projectname = data["projectname"]
    
    # Kompakte einzeilige YAML-Liste (Flow Style)
    image_dir_yaml = yaml.dump(data['images'], default_flow_style=True).strip()

    cmd = [
        "ros2", "launch", "fs_backend", "cpu.launch.py",
        f"name:=cpu_node_{projectname}",
        f"image_dir:={image_dir_yaml}",
        f"kernel_size_laplacian:={data['LaplaceFilterSize']}",
        f"num_pyr_lvl:={data['NumberofImagesinPyramid']}",
        f"sharpness_patch_size_y:={data['PatchSizeY']}",
        f"sharpness_patch_size_x:={data['PatchSizeX']}",
        f"img_resize:={data['ImgResize']}",
        f"z_spacing:={data['z_spacing']}",
        f"output_path_sharpness:={data['output_path_sharpness']}",
        f"output_path_depth:={data['output_path_depth']}"
    ]
    
    subprocess.Popen(cmd, env=os.environ.copy())
    


def init_routes(app):


   @app.route('/image/<projectname>/<filename>')
   def serve_image(projectname, filename):
       send = request.args.get('send', 'false').lower() == 'true'
       directory = os.path.join("/home/ws/" + "projects", projectname, "output/")
       print(directory)
       print(filename)
       full_path = os.path.join(directory, filename)

       print(f"Full path exists? {os.path.isfile(full_path)}")
       return send_from_directory(directory, filename, as_attachment=send)
       
   @app.errorhandler(404)
   def page_not_found(e):
       return render_template('404.html'), 404

   def load_from_yaml(directory, filename):
       import os
       abs_path = os.path.abspath(os.path.join(directory, filename))

       if not os.path.exists(abs_path):
           os.makedirs(directory, exist_ok=True)
           with open(abs_path, 'w') as f:
               yaml.dump({}, f)  # leeres dict

       with open(abs_path, 'r') as file:
           data = yaml.safe_load(file)

           # 💡 automatische Umwandlung bei falschem Typ
           if not isinstance(data, dict):
               data = {}  # force fallback
               with open(abs_path, 'w') as f:
                   yaml.dump(data, f)

       return data



   def save_in_yaml(data, directory, filename):
       os.makedirs(directory, exist_ok=True)  # Stelle sicher, dass Ordner existiert
       abs_path = os.path.abspath(os.path.join(directory, filename))
       with open(abs_path, 'w') as file:
           yaml.dump(data, file, default_flow_style=False)



   def create_project_folder_with_yaml(data, directory, filename):
       # Ordner erstellen, falls er nicht existiert
       os.makedirs(directory, exist_ok=True)

       # Pfad zusammenbauen
       full_path = os.path.join(directory, filename)

       save_in_yaml(data, directory, filename)


   @app.route("/open_project", methods=['GET','POST'])
   def open_project():
       projectname = request.form.get("projectname")

       #print(projectname)
       data = {}
       data["projectname"] = projectname
       config_parameter = load_from_yaml(os.path.join("projects", projectname), "config.yaml")
       exists = False
       if jpg_files_exist(os.path.join("projects", projectname, "output")):
         exists = True
       return render_template('includes/open_project.html', data=config_parameter, exist=exists)



   @app.route("/create_project", methods=['GET', 'POST'])
   def create_project():
       if request.method == 'POST':
           data = {}
           project_overview = load_from_yaml("projects", "projectLists.yaml")

           projectname = str(request.form.get("projectName"))

           if not projectname:
               return redirect(url_for('new_project'))  # Absicherung gegen leere Namen

           project_overview[projectname] = "init"

           # Eingabewerte lesen
           data["projectname"] = projectname
           data["LaplaceFilterSize"] = request.form.get("LaplaceFilterSize", type=int)
           data["NumberofImagesinPyramid"] = request.form.get("NumberofImagesinPyramid", type=int)
           data["PatchSizeX"] = request.form.get("PatchSizeX", type=int)
           data["PatchSizeY"] = request.form.get("PatchSizeY", type=int)
           data["ImgResize"] = request.form.get("ImgResize", type=float)
           data["z_spacing"] = request.form.get("z_spacing", type=float)

           # Hochgeladene Bilder speichern
           image_files = request.files.getlist("images")
           image_paths = []

           upload_dir = os.path.join("projects", projectname, "images")
           os.makedirs(upload_dir, exist_ok=True)

           for i, file in enumerate(image_files):
               if file and file.filename:
                   filename = f"{i:03d}_" + secure_filename(file.filename)  # z. B. 000_bild1.png
                   filepath = os.path.join(upload_dir, filename)
                   file.save(filepath)
                   image_paths.append(filepath)

           data["images"] = image_paths  # optional für YAML-Export


           output_dir = os.path.join("projects", projectname, "output")
           os.makedirs(output_dir, exist_ok=True)
           
           data["output_path_sharpness"] = os.path.join(output_dir, "sharpness.jpg")
           data["output_path_depth"] = os.path.join(output_dir, "depth.jpg")
            
           print(data["output_path_sharpness"])
           print(data["output_path_depth"])
           # Status: Init, Running, Done
           data["status"] = "Init"

           
           action = request.form.get("action")
        
           if action == "save_run":
               print("dassdasd\n\n\n\n\n\n\n\n")
               start_ros_launch(data)
               data["status"] = "running"
               project_overview[projectname] = "running"
               
           # Projekt speichern
           save_in_yaml(project_overview, "projects", "projectLists.yaml")
           create_project_folder_with_yaml(data, "projects/" + projectname, "config.yaml")


           return redirect(url_for('index'))

       return redirect(url_for('new_project'))

   @app.route('/new_project')
   def new_project():  # <---- dieser Name ist entscheidend
       return render_template('includes/new_project.html')

   @app.route('/logout')
   def logout():
       session.pop('user', None)  # User aus Session entfernen
       return redirect(url_for('login'))

   @app.route("/login", methods=['GET', 'POST'])
   def login():
       if request.method == 'POST':
           username = request.form['username']
           password = request.form['password']
           # Beispiel: Test-Login

           if username == 'admin' and password == 'admin':

               session['user'] = username
               return redirect(url_for('index'))
           else:
               return render_template('accounts/login.html')
       return render_template('accounts/login.html')

   @app.route('/update_system')
   def update_system():
       return redirect(url_for('index'))

   @app.route('/index')
   @app.route('/')
   def index():
       if 'user' not in session:
           return redirect(url_for('login'))
       # Lade das YAML-Dokument
       project_overview = load_from_yaml("projects", "projectLists.yaml")
       names = list(project_overview.keys())
       session["projectnames"] = list(project_overview.keys())

       # Wandeln die YAML-Daten in JSON um, um sie im Template zu verwenden
       #return render_template('index.html', project_data=json.dumps(project_data))
       username = session.get('user')
       session["username"] = username
       session["Project_Name"] = "Focus Stacking"
       status = list(project_overview.values())
            
       
       
       #print(session["status"])
       
       for i, name in enumerate(names):
         if status[i] == "init":
            continue
         node_name = "/cpu_node_" + name
         res = check_node_exists(node_name)
         print(node_name)
         print(res)
         if not res:
            status[i] = "stopped"
            
       session["status"] = status
       
       
       
       
       return render_template('index.html', session=session)
