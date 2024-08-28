# Sapiens 

Sapiens is a Meta's foundational model for human pose estimation. 

Human-centric vision tasks are: 
* image encoder 
* pose estimation 
* body part segmentation 
* depth estimation 
* surface normal estimation 

This Dockerfile enables plug&play start for the sapiens foundational models for following platforms: 
* PC with NVIDIA GPU (Ubuntu 20.04 local) 
* NVIDIA ORIN NX [TODO] 


### Lite 

Folder lite contains docker that builds environment only for the sapiens inference. 

Weights for the sapiens lite can be found [here](https://huggingface.co/facebook/sapiens/tree/main/sapiens_lite_host). 

### Full 

Folder full contains Dockerfile that builds environment for the training (fine-tuning) env for sapiens. 

Build full version by running: 
```
cd full 
docker build -t sapiens_img:full . 
```

Run container with: 
```
./first_run.sh
```

### How to install `mmcv`? 

Instructions how to properly build mmcv are [here](https://mmcv.readthedocs.io/en/latest/get_started/installation.html). 

Download weights simply entering file and finding button download for the chkpt you need. 
