from ultralytics import YOLO
import pandas as pd
import matplotlib.pyplot as plt
import time

start_time = time.time()

model = YOLO('yolo11x.pt')  

results = model('/home/ibrahim/ultralytics/robot_images', save=True)  

end_time = time.time()
total_time = end_time - start_time
print(f"\n🕒 Total inference time for {len(results)} images: {total_time:.2f} seconds\n")

data = []
for r in results:
    names = r.names  
    for box in r.boxes:
        class_id = int(box.cls)
        class_name = names[class_id]
        confidence = float(box.conf)
        bbox = box.xyxy.tolist()[0]  
        data.append({
            'image': r.path,
            'class_id': class_id,
            'class_name': class_name,
            'confidence': confidence,
            'bbox': bbox
        })

df = pd.DataFrame(data)

df.to_csv("yolo11_results.csv", index=False)

class_counts = df['class_name'].value_counts()
class_counts.plot(kind='bar', title='Class Distribution')
plt.xlabel('Object Class')
plt.ylabel('Count')
plt.xticks(rotation=45)
plt.tight_layout()
plt.show()

avg_conf_per_class = df.groupby('class_name')['confidence'].mean()
print("\n📈 Average confidence per class:\n", avg_conf_per_class)

plt.hist(df['confidence'], bins=20, edgecolor='black')
plt.title('Confidence Score Distribution')
plt.xlabel('Confidence')
plt.ylabel('Frequency')
plt.grid(True)
plt.show()
