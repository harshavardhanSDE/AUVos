#!/bin/bash

# Set working directories
working_dir=$(pwd)
config_dir="$working_dir/config"
launch_dir="$working_dir/launch"

echo "Working Directory: $working_dir"
echo "Configuration Directory: $config_dir"
echo "Launch Directory: $launch_dir"
echo

# Function to detect Linux distro
detect_distro() {
    if [ -f /etc/os-release ]; then
        source /etc/os-release
        echo "$ID"
    else
        echo "unknown"
    fi
}

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to install system dependencies
install_dependencies() {
    distro=$(detect_distro)
    echo "Detected OS: $distro"
    
    # Update package list
    case "$distro" in
        ubuntu|debian)
            sudo apt update && sudo apt upgrade -y
            sudo apt install -y python3 python3-pip nodejs npm git curl docker docker-compose
            ;;
        fedora)
            sudo dnf install -y python3 python3-pip nodejs npm git curl docker docker-compose
            ;;
        arch)
            sudo pacman -Sy --noconfirm python python-pip nodejs npm git curl docker docker-compose
            ;;
        *)
            echo "Unsupported Linux distribution. Please install dependencies manually."
            exit 1
            ;;
    esac
}

# Function to set up Python environment
setup_python() {
    echo "Setting up Python environment..."
    python3 -m venv venv
    source venv/bin/activate
    pip install --upgrade pip
    pip install -r backend/requirements.txt
}

# Function to set up Node.js environment
setup_node() {
    echo "Setting up Node.js environment..."
    cd frontend || exit
    npm install
    cd ..
}

# Function to set up Docker
setup_docker() {
    echo "Setting up Docker..."
    sudo systemctl enable docker
    sudo systemctl start docker
    sudo usermod -aG docker $USER
    echo "Please log out and log back in for Docker group changes to take effect."
}

# Function to start backend service using Docker Compose
start_backend_service() {
    echo "Starting backend service using Docker Compose..."
    cd backend || exit
    docker-compose up -d
    cd ..
}

# Ensure the script is run with proper privileges
if [ "$EUID" -ne 0 ]; then
    echo "Warning: It's recommended to run this script as root (or using sudo) for installing system packages."
    echo "Continuing as a normal user for non-system dependencies..."
fi

# Install dependencies
install_dependencies

# Set up application environments
setup_python
setup_node
setup_docker

# Start backend service
start_backend_service

echo "✅ Setup complete! You can now run your application."
