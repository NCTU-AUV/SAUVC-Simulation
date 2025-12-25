IMAGE_NAME := orca-auv-gazebo-simulation-image
CONTAINER_NAME := orca-auv-gazebo-simulation-container
WORKSPACE := orca_auv_gazebo_simulation_ws
PWD := $(shell pwd)
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

.PHONY: all compose_up compose_down compose_build compose_shell compose_init compose_clean network_certification clean

all: compose_up

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

compose_init: compose_up
	$(COMPOSE) exec orca /bin/bash -lc "\
		cd $(WORKSPACE) && \
		rosdep install --from-paths src --ignore-src -y && \
		colcon build --symlink-install && \
		echo \"source /root/$(WORKSPACE)/install/setup.bash\" >> /etc/bash.bashrc"

compose_clean:
	$(COMPOSE) down -v

network_certification:
	mkdir -p certs
	cd certs && (mkcert -install || echo "mkcert -install failed; assuming CA already installed") && mkcert localhost 127.0.0.1 ::1

clean:
	-$(COMPOSE) down || true
	rm -rf certs
	rm -rf orca_auv_gazebo_simulation_ws/build
	rm -rf orca_auv_gazebo_simulation_ws/install
	rm -rf orca_auv_gazebo_simulation_ws/log
