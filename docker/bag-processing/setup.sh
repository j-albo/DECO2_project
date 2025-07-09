#!/bin/bash

# FAST-LIVO2 Setup Script
# Initial configuration script for FAST-LIVO2

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

print_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[!]${NC} $1"
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

echo "🚀 FAST-LIVO2 Setup - Initial Configuration"
echo "==========================================="
echo ""

# 1. Check Docker
print_step "Checking Docker..."
if ! command -v docker &> /dev/null; then
    print_error "Docker is not installed"
    echo "Install Docker from: https://docs.docker.com/get-docker/"
    exit 1
fi

if ! docker info &> /dev/null; then
    print_error "Docker is not running or you don't have permissions"
    echo "Run: sudo systemctl start docker"
    echo "Or add your user to docker group: sudo usermod -aG docker \$USER"
    exit 1
fi

print_success "Docker verified"

# 2. Create directory structure
print_step "Creating directory structure..."
mkdir -p bags data config logs

cat > .gitignore << 'EOF'
# FAST-LIVO2 Docker - Generated files
bags/*.bag
data/*
logs/*
config/*.yaml
!config/README.md
!bags/README.md
!data/README.md

# Docker
.dockerignore

# System files
.DS_Store
Thumbs.db
*.log
EOF

# Create informative READMEs
cat > bags/README.md << 'EOF'
# Datasets Directory

Place your .bag files for FAST-LIVO2 here.

## Official Datasets
Download datasets from: https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhengcr_connect_hku_hk/ErdFNQtjMxZOorYKDTtK4ugBkogXfq1OfDm90GECouuIQA?e=KngY9Z

## Recommended Structure:
```
bags/
├── CBD_Building_01.bag
├── calibration.yaml
└── other_datasets.bag
```
EOF

cat > config/README.md << 'EOF'
# Configuration Directory

Place custom configuration files for FAST-LIVO2 here.

## Typical Files:
- calibration.yaml: Calibration parameters
- custom_config.yaml: Custom configuration

## Usage:
```bash
./run_fast_livo2.sh -c calibration.yaml my_dataset.bag
```
EOF

cat > data/README.md << 'EOF'
# Data Directory

This directory is used for additional data and FAST-LIVO2 results.

Generated maps and other output files will be saved here.
EOF

print_success "Directory structure created"

# 3. Configure script permissions
print_step "Configuring script permissions..."
chmod +x docker/*.sh 2>/dev/null || true
chmod +x *.sh 2>/dev/null || true
print_success "Permissions configured"

# 4. Check required files
print_step "Checking required files..."
required_files=("docker/Dockerfile" "docker/build_image.sh" "docker/run_container.sh")
missing_files=()

for file in "${required_files[@]}"; do
    if [ ! -f "$file" ]; then
        missing_files+=("$file")
    fi
done

if [ ${#missing_files[@]} -gt 0 ]; then
    print_error "Missing files:"
    printf '  %s\n' "${missing_files[@]}"
    exit 1
fi

print_success "All required files are present"

# 5. Ask about building image
echo ""
read -p "Do you want to build the Docker image now? (y/N): " build_now
if [[ $build_now =~ ^[Yy]$ ]]; then
    print_step "Building Docker image (this will take 20-30 minutes)..."
    ./docker/build_image.sh
    print_success "Image built successfully"
else
    print_warning "Image not built. Run './docker/build_image.sh' when ready"
fi

# 6. Create example script
print_step "Creating example script..."
cat > usage_examples.sh << 'EOF'
#!/bin/bash

# FAST-LIVO2 Usage Examples

echo "🚀 FAST-LIVO2 Usage Examples"
echo ""

echo "1. Basic usage (interactive selector):"
echo "   ./run_fast_livo2.sh"
echo ""

echo "2. Usage with specific file:"
echo "   ./run_fast_livo2.sh my_dataset.bag"
echo ""

echo "3. Usage with custom configuration:"
echo "   ./run_fast_livo2.sh -c calibration.yaml my_dataset.bag"
echo ""

echo "4. Quick usage (first .bag found):"
echo "   ./quick_run.sh"
echo ""

echo "5. List available .bag files:"
echo "   ./run_fast_livo2.sh --list-bags"
echo ""

echo "6. Build image and run:"
echo "   ./run_fast_livo2.sh --build my_dataset.bag"
echo ""

echo "7. View complete help:"
echo "   ./run_fast_livo2.sh --help"
EOF

chmod +x usage_examples.sh
print_success "Example script created"

# 7. Final information
echo ""
print_success "Setup completed!"
echo ""
echo "📋 Next steps:"
echo "1. Place .bag files in the 'bags/' directory"
echo "2. (Optional) Place configuration files in 'config/'"
echo "3. Run one of these commands:"
echo "   - ./run_fast_livo2.sh          # Interactive selector"
echo "   - ./quick_run.sh               # Quick execution"
echo ""
echo "📁 Structure created:"
echo "   bags/    - .bag dataset files"
echo "   config/  - Configuration files"
echo "   data/    - Data and results"
echo "   logs/    - Log files"
echo ""

if [ ! -f "docker/Dockerfile" ]; then
    print_warning "Important! Make sure you have all Docker files in place."
fi

echo "🎉 Ready to use FAST-LIVO2!"
