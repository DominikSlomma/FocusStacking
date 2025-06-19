# FocusStacking









## related work (github links)
* [ (C++)](https://github.com/PetteriAimonen/focus-stack)
* [https://github.com/pwlnk/focus-stacking (C++)](https://github.com/pwlnk/focus-stacking)
* [https://github.com/cmcguinness/focusstack (Python)](https://github.com/cmcguinness/focusstack)
* [https://osf.io/j8kby (Python)](https://osf.io/j8kby)





# Docker commands
docker build -t focus_stacking:latest .

## For development
docker run -it -v /path_to_webserver/flask_web_interface/:/home/ws/src/flask_web_interface/ -v /path_to_backend/fs_backend/:/home/ws/src/fs_backend/ -p 5000:5000 focus_stacking:latest

## For release






