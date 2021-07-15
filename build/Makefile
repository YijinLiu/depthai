default:
	docker build -t ${USER}/depthai:`date '+%Y-%m-%d'` \
		--build-arg NAME=${USER} --build-arg UID=$(shell id -u) --build-arg GID=$(shell id -g) .

# docker run -ti --privileged -v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw' \
#     -v /home/${USER}/projects:/home/${USER}/projects \
#     -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
#     --name "depthai" ${USER}/depthai:2021-07-13
