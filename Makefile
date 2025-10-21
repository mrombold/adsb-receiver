# Makefile for stratux-aggregator

# Variables
BINARY_NAME=stratux-aggregator
SERVICE_NAME=stratux-aggregator.service
GO=go

# Default target
.PHONY: all
all: build

# Build the binary
.PHONY: build
build:
	@echo "Building $(BINARY_NAME)..."
	$(GO) build -o $(BINARY_NAME) ./cmd/stratux-aggregator
	@echo "Build complete!"

# Stop service, build, restart service
.PHONY: deploy
deploy:
	@echo "Stopping $(SERVICE_NAME)..."
	sudo systemctl stop $(SERVICE_NAME)
	@echo "Building $(BINARY_NAME)..."
	$(GO) build -o $(BINARY_NAME) ./cmd/stratux-aggregator
	@echo "Restarting $(SERVICE_NAME)..."
	sudo systemctl start $(SERVICE_NAME)
	@echo "Checking status..."
	@sleep 1
	sudo systemctl status $(SERVICE_NAME) --no-pager -l

# Just restart the service (useful if you manually built)
.PHONY: restart
restart:
	@echo "Restarting $(SERVICE_NAME)..."
	sudo systemctl restart $(SERVICE_NAME)
	@sleep 1
	sudo systemctl status $(SERVICE_NAME) --no-pager -l

# Stop the service
.PHONY: stop
stop:
	@echo "Stopping $(SERVICE_NAME)..."
	sudo systemctl stop $(SERVICE_NAME)

# Start the service
.PHONY: start
start:
	@echo "Starting $(SERVICE_NAME)..."
	sudo systemctl start $(SERVICE_NAME)
	@sleep 1
	sudo systemctl status $(SERVICE_NAME) --no-pager -l

# Check service status
.PHONY: status
status:
	sudo systemctl status $(SERVICE_NAME) --no-pager -l

# View logs
.PHONY: logs
logs:
	sudo journalctl -u $(SERVICE_NAME) -n 50 --no-pager

# Follow logs in real-time
.PHONY: logs-follow
logs-follow:
	sudo journalctl -u $(SERVICE_NAME) -f

# Clean build artifacts
.PHONY: clean
clean:
	@echo "Cleaning build artifacts..."
	rm -f $(BINARY_NAME)
	go clean

# Deep clean including caches
.PHONY: clean-all
clean-all: clean
	@echo "Cleaning all caches..."
	go clean -cache -modcache -testcache
	rm -rf ~/.cache/go-build

# Clean rebuild
.PHONY: rebuild-clean
rebuild-clean: clean-all
	@echo "Clean rebuild..."
	go build -v -o $(BINARY_NAME) ./cmd/stratux-aggregator
	@echo "✓ Clean build complete!"

# Clean deploy
.PHONY: deploy-clean
deploy-clean: clean-all
	@echo "Clean rebuild and deploy..."
	sudo systemctl stop $(SERVICE_NAME)
	go build -v -o $(BINARY_NAME) ./cmd/stratux-aggregator
	sudo systemctl start $(SERVICE_NAME)
	@sleep 1
	sudo systemctl status $(SERVICE_NAME) --no-pager -l

# Run locally without service (for testing)
.PHONY: run
run: build
	./$(BINARY_NAME)

# Full clean and rebuild
.PHONY: rebuild
rebuild: clean build

# Install/update the systemd service file
.PHONY: install-service
install-service:
	@echo "Installing service file..."
	sudo cp $(SERVICE_NAME) /etc/systemd/system/
	sudo systemctl daemon-reload
	sudo systemctl enable $(SERVICE_NAME)
	@echo "Service installed and enabled!"







# Show help
.PHONY: help
help:
	@echo "Available targets:"
	@echo "  make build          - Build the binary"
	@echo "  make deploy         - Stop service, build, restart service"
	@echo "  make restart        - Restart the service"
	@echo "  make start          - Start the service"
	@echo "  make stop           - Stop the service"
	@echo "  make status         - Show service status"
	@echo "  make logs           - Show recent logs"
	@echo "  make logs-follow    - Follow logs in real-time"
	@echo "  make run            - Build and run locally (no service)"
	@echo "  make clean          - Remove build artifacts"
	@echo "  make rebuild        - Clean and rebuild"
	@echo "  make install-service - Install systemd service file"