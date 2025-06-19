from flask import Flask
from flask_web_interface.routes import init_routes
import rclpy
import os

def create_app():
    #template_dir = os.environ.get('AMENT_PREFIX_PATH','') #os.path.join(os.path.dirname(__file__), 'templates')
    #static_dir = os.path.join(os.path.dirname(__file__), 'static')

    #first_ws = template_dir.split(':')[0] if template_dir else None
    #print("Dein Workspace:", first_ws)
    #print(template_dir)
    template_dir = "/home/ws/src/flask_web_interface/flask_web_interface/templates"
    static_dir = "/home/ws/src/flask_web_interface/flask_web_interface/static"
    app = Flask(__name__, template_folder=template_dir, static_folder=static_dir)
    #app = Flask(__name__)
    app.secret_key = 'secret_key'

    #define routes!
    init_routes(app)

    return app

def main():
    
    app = create_app()
    app.run(host='0.0.0.0', port=5000, debug=False)
    #rclpy.shutdown()
