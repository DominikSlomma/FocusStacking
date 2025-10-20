# app.py
from flask import Flask
from flask_web_interface.routes import bp as web_bp

class Config:
    SECRET_KEY = "secret_key"
    PROJECTS_BASE = "projects"
    INDEX_FILE = "projects_index.yaml"
    # Optional: absolute Pfade für templates/static lieber hier zentral
    TEMPLATE_FOLDER = "/home/anonym/Schreibtisch/PhD/code/FocusStacking/flask_web_interface/flask_web_interface/templates"
    STATIC_FOLDER   = "/home/anonym/Schreibtisch/PhD/code/FocusStacking/flask_web_interface/flask_web_interface/static"
    ROS_DOMAIN_ID   = "0"  # überschreibbar via env

def create_app():
    app = Flask(__name__, template_folder=Config.TEMPLATE_FOLDER, static_folder=Config.STATIC_FOLDER)
    app.config.from_object(Config)

    # ROS Domain aus env übernehmen (falls gesetzt)
    import os
    app.config["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", app.config["ROS_DOMAIN_ID"])
    os.environ["ROS_DOMAIN_ID"] = app.config["ROS_DOMAIN_ID"]

    # Blueprints registrieren
    app.register_blueprint(web_bp)

    return app

def main():
    app = create_app()
    app.run(host="0.0.0.0", port=5000, debug=True)

if __name__ == "__main__":
    main()
