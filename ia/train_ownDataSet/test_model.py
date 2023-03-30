import os 
from ultralytics import YOLO       

model_path = os.path.join('.', 'runs', 'Segment', 'train', 'train_200epochs_98img_seg', 'weights', 'best.pt')

img_predict = os.path.join('.', 'data', 'image_7.jpeg')

model = YOLO(model_path)  # load a pretrained YOLOv8n segmentation model
img = model.predict(source=img_predict, show=True, save=True)