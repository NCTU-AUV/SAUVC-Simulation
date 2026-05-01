IMAGE_NAME := orca-auv-gazebo-simulation-image
CONTAINER_NAME := orca-auv-gazebo-simulation-container
WORKSPACE := sim_ws
PWD := $(shell pwd)
WORLD ?= /root/$(WORKSPACE)/src/bringup/worlds/water_world.sdf
NAMESPACE ?= orca_auv
# Prefer Docker Compose v2 (docker compose) but fall back to v1 (docker-compose); allow override via env/CLI
COMPOSE ?= $(shell \
	if docker compose version >/dev/null 2>&1; then \
		printf "docker compose"; \
	elif docker-compose --version >/dev/null 2>&1; then \
		printf "docker-compose"; \
	else \
		printf ""; \
	fi)
ifeq ($(strip $(COMPOSE)),)
$(error Docker Compose not found: install Docker Compose v2 (docker compose) or v1 (docker-compose), or set COMPOSE to your compose binary)
endif

.PHONY: all compose_up compose_down compose_build compose_shell init launch compose_init compose_launch compose_clean network_certification clean

all: init launch

compose_up: network_certification
	$(COMPOSE) up -d --build

compose_down:
	$(COMPOSE) down

compose_build:
	$(COMPOSE) build --pull

compose_shell:
	$(COMPOSE) exec orca /bin/bash -lc "\
		source /opt/ros/humble/setup.bash; \
		if [ -f $(WORKSPACE)/install/setup.bash ]; then \
			source $(WORKSPACE)/install/setup.bash; \
		fi; \
		exec bash"

init: compose_up
	$(COMPOSE) exec orca /bin/bash -lc "\
		cd $(WORKSPACE) && \
		source /opt/ros/humble/setup.bash && \
		rosdep install --from-paths src --ignore-src -y && \
		colcon build --symlink-install && \
		echo \"source /root/$(WORKSPACE)/install/setup.bash\" >> /etc/bash.bashrc"

launch: compose_up
	$(COMPOSE) exec orca /bin/bash -lc "\
		set -e; \
		source /opt/ros/humble/setup.bash; \
		source /root/$(WORKSPACE)/install/setup.bash; \
		ign launch -v 4 /root/$(WORKSPACE)/websocket.ign & \
		WEBSOCKET_PID=\$$!; \
		ign gazebo -v 4 -s -r $(WORLD) & \
		GAZEBO_PID=\$$!; \
		ros2 launch bringup orca_ros_gz_bridge_launch.py namespace:=$(NAMESPACE) & \
		BRIDGE_PID=\$$!; \
		trap 'kill \$$WEBSOCKET_PID \$$GAZEBO_PID \$$BRIDGE_PID 2>/dev/null; wait 2>/dev/null' INT TERM EXIT; \
		wait -n \$$WEBSOCKET_PID \$$GAZEBO_PID \$$BRIDGE_PID"

compose_init: init

compose_launch: launch

compose_clean:
	$(COMPOSE) down -v

network_certification:
	mkdir -p certs
	cd certs && (mkcert -install || echo "mkcert -install failed; assuming CA already installed") && mkcert localhost 127.0.0.1 ::1

clean:
	-$(COMPOSE) down || true
	rm -rf certs
	rm -rf sim_ws/build
	rm -rf sim_ws/install
	rm -rf sim_ws/log
