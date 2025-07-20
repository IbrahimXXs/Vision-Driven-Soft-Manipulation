from PIL import Image
import timm
import torch
from pathlib import Path
import time
import matplotlib.pyplot as plt

img_path = Path("Dark.jpg")
if not img_path.exists():
    raise FileNotFoundError("Image 'Keyboard.png' not found in the current directory.")

test_img = Image.open(img_path).convert("RGB")

with open("imagenet1000_clsidx_to_labels.txt", "r") as f:
    label_dict_raw = f.read()
    image_net_labels = eval(label_dict_raw)

model_name = "hf_hub:timm/mobilenetv4_hybrid_large.ix_e600_r384_in1k"
model = timm.create_model(model_name, pretrained=True)
model.eval()

data_config = timm.data.resolve_data_config({}, model=model)
transform = timm.data.create_transform(**data_config)

def predict_inference_only(img):
    input_tensor = transform(img).unsqueeze(0)

    start_time = time.time()
    with torch.no_grad():
        output = model(input_tensor)
    end_time = time.time()

    inference_time = end_time - start_time

    top5_probabilities, top5_class_indices = torch.topk(output.softmax(dim=1), k=5)
    top5_probabilities = top5_probabilities[0] * 100
    top5_class_indices = top5_class_indices[0]

    predictions = []
    labels = []
    scores = []

    for idx, prob in zip(top5_class_indices, top5_probabilities):
        label = image_net_labels.get(idx.item(), f"Unknown class {idx.item()}")
        score = round(prob.item(), 2)
        predictions.append([label, score])
        labels.append(label)
        scores.append(score)

    print("Top-5 Predictions:")
    for pred in predictions:
        print(f"{pred[0]}: {pred[1]}%")

    print(f"\nInference time only: {inference_time:.4f} seconds")

    plt.figure(figsize=(10, 5))
    plt.barh(labels[::-1], scores[::-1])
    plt.xlabel('Confidence (%)')
    plt.title('Top-5 Predictions')
    plt.tight_layout()
    plt.savefig("prediction_graph.png")
    print("Prediction graph saved as 'prediction_graph.png'")


predict_inference_only(test_img)
