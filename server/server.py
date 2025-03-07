from flask import Flask, request, jsonify
from transformers import AutoModelForVision2Seq, AutoProcessor
from PIL import Image
import torch
import numpy as np
import io
import time

app = Flask(__name__)

processor = AutoProcessor.from_pretrained(
    "/home/isr-lab-4/openvla/models/drone3_cycle/runs/openvla-7b+drone_set3+b16+lr-0.0005+lora-r32+dropout-0.0/", 
    # "openvla/openvla-7b", 
    trust_remote_code=True
)
vla = AutoModelForVision2Seq.from_pretrained(
    "/home/isr-lab-4/openvla/models/drone3_cycle/finetuned_model/openvla-7b+drone_set3+b16+lr-0.0005+lora-r32+dropout-0.0/",
    # "openvla/openvla-7b", 
    attn_implementation="flash_attention_2",  # [Optional] Requires `flash_attn`
    torch_dtype=torch.bfloat16,
    low_cpu_mem_usage=True, 
    trust_remote_code=True
).to("cuda:0")

@app.route('/predict_action', methods=['POST'])
def predict_action():
    try:
        image_file = request.files['image']
        instruction = request.form['instruction']
        image = Image.open(io.BytesIO(image_file.read())).convert("RGB")
        # image.show()
        prompt = f"In: What action should the robot take to {instruction}?\nOut:"
        inputs = processor(prompt, image, return_tensors="pt").to("cuda:0", dtype=torch.bfloat16)
        time0 = time.perf_counter()
        action = vla.predict_action(**inputs, unnorm_key="drone_set3", do_sample=False)  #change unnorm key to (drone_set4) if u r using one gate (different positions), change it to drone_set5_two_gates for two gates and arched 
        time1 = time.perf_counter()
        print(f"Prediction time = {time1 - time0} sec")

        if isinstance(action, torch.Tensor):
            action = action.tolist()  
        elif isinstance(action, np.ndarray):
            action = action.tolist()  
        formatted_action = {
            "velocities": {
                "x": action[0],
                "y": action[1],
                "z": action[2]
            },
            "delta_yaw": action[5],
        }
        print(formatted_action)
        return jsonify(formatted_action)
        # return jsonify(action[:6])

    except Exception as e:
        print(f"Error: {str(e)}")
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
