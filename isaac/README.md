# NVIDIA ISAAC

Effort towards starting RL env for quadrotors. 

## [How to install NVIDIA container](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html)

## Rough set of steps: 

1. Get access to the [NVIDIA Isaac sim](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim)
2. Generate API key for the [ngc](https://docs.nvidia.com/ngc/gpu-cloud/ngc-user-guide/index.html#generating-api-key)
3. Login to the docker nvcr.io 
```
docker login nvcr.io 
```
4. Credentials are:  
```
username: $oauthtoken 
password: <YOUR_GENERATED_API_KEY>
```
When you type password in command line it wont show, therefore, just paste it once. 

5. Pull image 

6. Run it with provided command in the **How to install NVIDIA container**

## TODO: 

- [x] Register to the ngc
- [x] Generate API key
- [ ] Start pulling of the image 
- [ ] Create custom image that includes [NTNU aerial gym](https://ntnu-arl.github.io/aerial_gym_simulator/)
