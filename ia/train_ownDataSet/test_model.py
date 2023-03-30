import os 
from ultralytics import YOLO       

model_path = os.path.join('.', 'runs', 'detect', 'train7', 'weights', 'best.pt')
model = YOLO(model_path)  # load a pretrained YOLOv8n segmentation model

img_path = os.path.join('.', 'data', 'images', 'train', 'im1.jpeg')

model(img_path, save=True, save_txt=True) 