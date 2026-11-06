"""HTTP server around SocialNav's released model, so ROS needs no GPU code.

POST /waypoints  {"images": [5 base64 JPEGs, oldest first],
                  "history": [5 [right, forward] m, oldest first, last is [0, 0]],
                  "goal": [right, forward] m, "step_scale": m per step}
              -> {"waypoints": [5 [right, forward] m], "seconds": s}
POST /describe   {"image": base64 JPEG} -> {"text": what the model says, "seconds": s}
GET  /health     -> {"model": type, "gpu_gb": memory in use}

Loading and calling the model follows SocialNav's utils/citywalker.py. The
training prompts are not public, so /waypoints sends only the placeholders the
model needs and /describe uses the prompt shown in the paper's Figure 3.
"""

import base64
import io
import json
import os
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import torch
import transformers
from PIL import Image
from transformers import AutoConfig, AutoProcessor

MODEL_PATH = os.environ.get('SOCIALNAV_MODEL', '/models/SocialNav-Qwen2-VL-SAFE-GRPO')
PORT = int(os.environ.get('SOCIALNAV_PORT', '8100'))
FLOW_STEPS = 5
# Pictures are shrunk to this width: GPU memory, and the robot's are small anyway.
IMAGE_WIDTH = int(os.environ.get('SOCIALNAV_IMAGE_WIDTH', '448'))
WAYPOINT_TOKENS = [f'<input_pos{i}>' for i in range(1, 6)] + ['<input_target>']
EXTRA_TOKENS = ['<flow_matching_policy>', '<time>']
BACKBONES = {
    'qwen2_vl': ('Qwen2VLForConditionalGeneration',
                 ['replace_qwen_2_with_mixed_modality_forward']),
    'qwen2_5_vl': ('Qwen2_5_VLForConditionalGeneration',
                   ['replace_qwen2_5_with_mixed_modality_forward', 'replace_qwen2_5_vision']),
}
ACTION_PROMPT = ''.join(WAYPOINT_TOKENS)
DESCRIBE_PROMPT = 'Based on the input, generate a chain of thought and navigation action decisions.'


def load_model():
    from src.train import monkey_patch_forward, monkey_patch_vision

    config = AutoConfig.from_pretrained(MODEL_PATH)
    class_name, patches = BACKBONES[config.model_type]
    for patch in patches:
        (getattr(monkey_patch_forward, patch, None) or getattr(monkey_patch_vision, patch))()
    model = getattr(transformers, class_name).from_pretrained(
        MODEL_PATH, dtype=torch.bfloat16, device_map='cuda:0', action_dim=2, action_chunk=5,
        flow_matching_policy=True, num_flow_steps=FLOW_STEPS, action_former=True,
        query_action_layer=4, sigma=0.0, ar_lambda_loss=1.0, min_value=-1.25, max_value=1.25,
        sde_mode='cps',
    )
    model.eval()
    processor = AutoProcessor.from_pretrained(MODEL_PATH)
    tokenizer = processor.tokenizer
    token_ids = {t: tokenizer.convert_tokens_to_ids(t) for t in WAYPOINT_TOKENS + EXTRA_TOKENS}
    return config.model_type, model, processor, token_ids


def decode(data):
    image = Image.open(io.BytesIO(base64.b64decode(data))).convert('RGB')
    height = max(28, round(image.height * IMAGE_WIDTH / image.width))
    return image.resize((IMAGE_WIDTH, height))


class Service:

    def __init__(self):
        self.model_type, self.model, self.processor, self.token_ids = load_model()

    def inputs(self, images, prompt):
        content = [{'type': 'image', 'image': image} for image in images]
        content.append({'type': 'text', 'text': prompt})
        text = self.processor.apply_chat_template(
            [{'role': 'user', 'content': content}], tokenize=False, add_generation_prompt=True)
        return text, images

    @torch.no_grad()
    def waypoints(self, request):
        scale = float(request['step_scale'])
        points = [*request['history'], request['goal']]
        if len(request['images']) != 5 or len(points) != 6 or scale <= 0:
            raise ValueError('need 5 images, 5 history points, a goal and step_scale > 0')
        text, images = self.inputs([decode(i) for i in request['images']], ACTION_PROMPT)
        inputs = self.processor(text=text + '<|im_end|>', images=images, padding=True,
                                return_tensors='pt').to('cuda:0')
        scaled = torch.tensor(points, dtype=torch.float32) / scale
        inputs['input_waypoints'] = scaled.unsqueeze(0).to('cuda:0')
        outputs = self.model(**inputs, train=False, train_branch='fm', num_samples=1,
                             special_token2id=self.token_ids)
        result = outputs[0].squeeze(0).detach().cpu().float() * scale
        return {'waypoints': result.tolist()}

    @torch.no_grad()
    def describe(self, request):
        text, images = self.inputs([decode(request['image'])], DESCRIBE_PROMPT)
        inputs = self.processor(text=text, images=images, padding=True,
                                return_tensors='pt').to('cuda:0')
        output = self.model.generate(**inputs, max_new_tokens=int(request.get('max_tokens', 150)),
                                     do_sample=False)
        new_tokens = output[0][inputs['input_ids'].shape[1]:]
        return {'text': self.processor.decode(new_tokens, skip_special_tokens=True)}


def make_handler(service):
    routes = {'/waypoints': service.waypoints, '/describe': service.describe}

    class Handler(BaseHTTPRequestHandler):

        def reply(self, code, data):
            payload = json.dumps(data).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            self.reply(200, {'model': service.model_type,
                             'gpu_gb': round(torch.cuda.memory_allocated() / 1e9, 2)})

        def do_POST(self):
            route = routes.get(self.path)
            if route is None:
                self.reply(404, {'error': 'unknown path'})
                return
            started = time.monotonic()
            try:
                request = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                self.reply(200, {**route(request), 'seconds': time.monotonic() - started})
            except (KeyError, TypeError, ValueError) as e:
                self.reply(400, {'error': str(e)})
            except torch.cuda.OutOfMemoryError:
                torch.cuda.empty_cache()
                self.reply(503, {'error': 'GPU out of memory'})

        def log_message(self, *args):
            pass

    return Handler


if __name__ == '__main__':
    service = Service()
    print(f'SocialNav {service.model_type} ready on port {PORT}, '
          f'{torch.cuda.memory_allocated() / 1e9:.2f} GB on the GPU', flush=True)
    # Localhost by default: this has no password, so reach it over an SSH tunnel.
    host = os.environ.get('SOCIALNAV_HOST', '127.0.0.1')
    HTTPServer((host, PORT), make_handler(service)).serve_forever()
