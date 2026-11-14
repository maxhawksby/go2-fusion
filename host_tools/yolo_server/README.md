# YOLO server

Runs YOLO on a GPU machine so `object_detection` on the robot laptop doesn't use its CPU. See the [Object detection](../../docs/wiki/object-detection.md) wiki page.

## On the GPU server (once)

```shell
python3 -m venv ~/yolo-venv
~/yolo-venv/bin/pip install -r requirements.txt
```

## Run

```shell
CUDA_VISIBLE_DEVICES=1 YOLO_MODEL=yolov8n.pt ~/yolo-venv/bin/python server.py
```

`CUDA_VISIBLE_DEVICES` picks the GPU. `YOLO_PORT` (default 8200) and `YOLO_HOST` (default `127.0.0.1`) set where it listens. It has no password, so keep it on localhost and reach it through SSH.

## On the laptop

```shell
ssh -N -L 8200:localhost:8200 <user>@<gpu-server>
```

Then start the detector with `YOLO_URL=http://host.docker.internal:8200/detect` in the container's environment (Windows Docker Desktop), or `-p inference_url:=...`. With WSL or Linux and host networking, use `http://localhost:8200/detect`.

Check it: `curl http://localhost:8200/health`.
