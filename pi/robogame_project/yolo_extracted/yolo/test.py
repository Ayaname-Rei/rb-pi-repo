from ultralytics import YOLO

model = YOLO("yolo26n.pt")

results = model("./assets/test_1.jpg", save=True)

print(results)