# GPU server for SocialNav (github.com/AMAP-EAI/SocialNav, Apache-2.0).
# Its own image: it needs CUDA PyTorch and SocialNav's patched transformers,
# which would clash with the robot image. See README.md.
FROM python:3.10-slim
RUN apt-get update && apt-get install -y --no-install-recommends git && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/AMAP-EAI/SocialNav.git /opt/SocialNav &&     git -C /opt/SocialNav checkout b958f0683521a024d14d183593750bddac7f75b5
RUN pip install --no-cache-dir torch==2.5.1+cu124 torchvision==0.20.1+cu124 --extra-index-url https://download.pytorch.org/whl/cu124
RUN pip install --no-cache-dir -e /opt/SocialNav/transformers &&     pip install --no-cache-dir accelerate==1.10.1 diffusers torchcfm qwen-vl-utils einops pillow modelscope
ENV PYTHONPATH=/opt/SocialNav HF_HOME=/models/hf
COPY server.py /opt/server.py
WORKDIR /opt/SocialNav
CMD ["python", "/opt/server.py"]
