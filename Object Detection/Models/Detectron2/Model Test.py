import os
import cv2
import torch
import pandas as pd
import matplotlib.pyplot as plt
import time
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2 import model_zoo
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog

image_dir = "/home/ibrahim/ultralytics/robot_imagez"
output_dir = "./detectron2_outputsz"
os.makedirs(output_dir, exist_ok=True)

print("🚀 Starting model load + inference...")
start_time = time.time()

cfg = get_cfg()
cfg.MODEL.DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
cfg.merge_from_file(model_zoo.get_config_file("COCO-Detection/faster_rcnn_X_101_32x8d_FPN_3x.yaml"))
cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-Detection/faster_rcnn_X_101_32x8d_FPN_3x.yaml")
cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.25

predictor = DefaultPredictor(cfg)

results = []

for filename in sorted(os.listdir(image_dir)):
    if not filename.endswith((".jpg", ".png")):
        continue

    img_path = os.path.join(image_dir, filename)
    img = cv2.imread(img_path)

    outputs = predictor(img)
    boxes = outputs["instances"].pred_boxes.tensor.cpu().numpy()
    classes = outputs["instances"].pred_classes.cpu().numpy()
    scores = outputs["instances"].scores.cpu().numpy()

    for i in range(len(boxes)):
        x1, y1, x2, y2 = boxes[i]
        results.append({
            "image": filename,
            "class_id": int(classes[i]),
            "confidence": float(scores[i]),
            "bbox": [float(x1), float(y1), float(x2), float(y2)]
        })

    v = Visualizer(img[:, :, ::-1], MetadataCatalog.get(cfg.DATASETS.TRAIN[0]))
    out = v.draw_instance_predictions(outputs["instances"].to("cpu"))
    cv2.imwrite(os.path.join(output_dir, filename), out.get_image()[:, :, ::-1])

end_time = time.time()
total_time = end_time - start_time

df = pd.DataFrame(results)
df.to_csv("detectron2_results.csv", index=False)
print("✅ Detection results saved to detectron2_results.csv")
print(f"🕒 Total time (model load + inference for {len(df['image'].unique())} images): {total_time:.2f} seconds")

class_counts = df['class_id'].value_counts()
class_counts.plot(kind='bar', title='Class Distribution (Detectron2)')
plt.xlabel('Class ID')
plt.ylabel('Count')
plt.xticks(rotation=0)
plt.tight_layout()
plt.savefig("detectron2_class_distribution.png")
plt.close()

avg_conf = df.groupby('class_id')['confidence'].mean()
print("\n📈 Average confidence per class:\n", avg_conf)

plt.hist(df['confidence'], bins=20, edgecolor='black')
plt.title('Confidence Score Distribution (Detectron2)')
plt.xlabel('Confidence')
plt.ylabel('Frequency')
plt.grid(True)
plt.tight_layout()
plt.savefig("detectron2_confidence_histogram.png")
plt.close()

