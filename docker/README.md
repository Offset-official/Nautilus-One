# Process of loading the newly built docker image onto the Pi

## Building the image

Run the following command in this `directory`.
```bash
./build-pi.sh
```

## Tag the image
Be sure to check the image id  of the newly built image using `docker images`, then run this command

```bash
docker tag pi-ros-full ghcr.io/offset-official/pi-ros-full:latest
```

## Exporting the image
To create a portable tar file of the image.
```bash
docker save ghcr.io/offset-official/pi-ros-full:latest -o  pi-ros-full.tar
```

## Transfer the image
```bash
scp pi-ros-full.tar pi@192.168.2.2:~/docker_images
```
## Load the image
> Run this command inside the raspberry pi
```bash
docker load -i ~/docker_images/pi-ros-full.tar
```

## Run the container
> Must be run inside the raspberry pi!
```bash
docker run --rm --network host -it ghcr.io/offset-official/pi-ros-full
```
Be sure to setup the discovery server as well.

# Process of loading the newly built docker image onto the Nano

## Building the image
Run the following command in this `directory`.
```bash
./build-nano.sh
```

## Tag the image
Be sure to check the image id  of the newly built image using `docker images`, then run this command

```bash
docker tag nano-ros-full ghcr.io/offset-official/nano-ros-full:latest
```

## Exporting the image
To create a portable tar file of the image.
```bash
docker save ghcr.io/offset-official/nano-ros-full:latest -o  nano-ros-full.tar
```

## Transfer the image
```bash
scp nano-ros-full.tar nano@192.168.2.4:~/Documents
```
## Load the image
> Run this command inside the Jetson Nano
```bash
docker load -i ~/Documents/nano-ros-full.tar
```
## Run the nano container
```bash
sudo docker run --rm --network host -it --device=/dev/ttyACM0 --device=/dev/video0 --device=/dev/video2 ghcr.io/offset-official/nano-ros-full 
```
