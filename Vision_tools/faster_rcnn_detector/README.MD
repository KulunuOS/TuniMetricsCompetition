set python virtual enviroment using requirements.txt
python3 -m venv env_name
source env_name/bin/activate
pip install -r requirements.txt

Model should be taken separately from OneDrive

To run, use command
python detect.py image_path1 image_path2 image_path3 ...
The code should generate images with bounding boxes and json containing bboxes and scores in the same folder
