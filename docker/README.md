# Process of loading the newly built docker image onto the Pi

> The packages which will be built onto the docker image are specified in the `pi_packages.list` file.

## Building the image

Run the following command in this `directory`.
```bash
./build-pi.sh
```

## Tag the image
Be sure to check the image id  of the newly built image using `docker images`, then run this command

```bash
docker tag {your-tag-here} ghcr.io/offset-official/pi-ros-full:latest
```

## Exporting the image
To create a portable tar file of the image.
```bash
docker save ghcr.io/offset-official/pi-ros-full:latest | gzip > pi-ros-full.tar.gz
```

## Transfer the image
```bash
scp pi-ros-full.tar.gz pi@192.168.2.2:~/docker_images
```
## Load the image
> Run this command inside the raspberry pi
```bash
docker load -i pi-ros.tar.gz
```

## Run the container
> Must be run inside the raspberry pi!
```bash
docker run --rm --network host -it ghcr.io/offset-official/pi-ros-full
```
Be sure to setup the discovery server as well.
