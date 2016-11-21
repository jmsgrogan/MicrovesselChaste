# How to build the docker container

The `microvessel-chaste` docker container is on the channel `jmsgrogan` on the Docker Hub. This directory contains the Dockerfile for setting up the container. Assuming Docker is set up, it can be built by doing:

```bash
[sudo] docker build -t microvessel-chaste .
```

If you are happy with the package it can be tagged and uploaded. Do:

```bash
[sudo] docker images
```

to find the image id. Then do:

```bash
[sudo] docker tag <image_id> jmsgrogan/microvessel-chaste:latest
docker login
docker push jmsgrogan/microvessel-chaste
```

## Preparing docker for the build

To set up docker for the build do:

```bash
sudo apt-get update
sudo apt-get install apt-transport-https ca-certificates
sudo apt-key adv --keyserver hkp://p80.pool.sks-keyservers.net:80 --recv-keys 58118E89F3A912897C070ADBF76221572C52609D
echo "deb https://apt.dockerproject.org/repo ubuntu-xenial main" | sudo tee /etc/apt/sources.list.d/docker.list
sudo apt-get update
sudo apt-get install linux-image-extra-$(uname -r) linux-image-extra-virtuals
sudo apt-get install docker-engine
sudo service docker start
```

## Working with the package

```bash
[sudo] docker run -it -p 8888:8888 microvessel-chaste
```

Then go to [htpp://localhost::8888](htpp://localhost::8888) in a web browser.

