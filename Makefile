IMAGE_NAME := orca-auv-gazebo-simulation-image
CONTAINER_NAME := orca-auv-gazebo-simulation-container
WORKSPACE := orca_auv_gazebo_simulation_ws


all: network_certification build_container enter_container

build_container:
	@echo "Creating and starting a new container: $(CONTAINER_NAME)"
	docker run -dit \
		--name $(CONTAINER_NAME) \
		-v $(PWD)/$(WORKSPACE):/root/$(WORKSPACE) \
		-p 9002:9002 \
		-v $(PWD)/certs:/ign-certs:ro \
		$(IMAGE_NAME):latest

	docker exec $(CONTAINER_NAME) /bin/bash -i -c \
		"cd $(WORKSPACE) \
		&& colcon build --symlink-install \
		&& echo \"source /root/$(WORKSPACE)/install/setup.bash\" >> /etc/bash.bashrc"

build_image:
	docker build --pull -t $(IMAGE_NAME):latest .

enter_container:
	@echo "Executing a shell inside container: $(CONTAINER_NAME)"
	docker exec -it $(CONTAINER_NAME) /bin/bash

clean:
	docker rm -f $(CONTAINER_NAME) || true
	rm -rf certs
	rm -rf orca_auv_gazebo_simulation_ws/build
	rm -rf orca_auv_gazebo_simulation_ws/install
	rm -rf orca_auv_gazebo_simulation_ws/log

network_certification:
	bash -c "mkdir certs \
		&& cd certs \
		&& mkcert -install \
		&& mkcert localhost 127.0.0.1 ::1"
