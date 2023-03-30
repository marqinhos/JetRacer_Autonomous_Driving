from ultralytics import YOLO

model = YOLO('yolov8n-seg.yaml')
#model = YOLO("yolov8n-seg.pt")
# Select file
model.train(data='config.yaml', epochs=50, patience=50, batch=20 ,imgsz=640) 