#!/bin/bash
# Helper script for controlling slam_toolbox

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Function to print usage
usage() {
    echo -e "${GREEN}slam_toolbox Control Script${NC}"
    echo ""
    echo "Usage: $0 <command> [arguments]"
    echo ""
    echo "Commands:"
    echo "  pause                  - Pause mapping (switch to localization mode)"
    echo "  resume                 - Resume mapping"
    echo "  save <filename>        - Save map in posegraph format (can be reloaded)"
    echo "  save_ros <filename>    - Save map in ROS format (.pgm + .yaml)"
    echo "  load <filename>        - Load map and relocalize"
    echo "  clear                  - Clear the scan queue"
    echo "  list                   - List all slam_toolbox services"
    echo ""
    echo "Examples:"
    echo "  $0 pause"
    echo "  $0 save /home/user/maps/office_map"
    echo "  $0 load /home/user/maps/office_map"
    echo ""
}

# Check if ROS2 is sourced
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}Error: ros2 command not found. Please source your ROS2 workspace.${NC}"
    exit 1
fi

# Parse command
COMMAND=$1
shift || true

case $COMMAND in
    pause)
        echo -e "${YELLOW}Pausing mapping (switching to localization mode)...${NC}"
        ros2 service call /slam_toolbox/pause_new_measurements std_srvs/srv/SetBool "{data: true}"
        echo -e "${GREEN}✓ Mapping paused${NC}"
        ;;

    resume)
        echo -e "${YELLOW}Resuming mapping...${NC}"
        ros2 service call /slam_toolbox/pause_new_measurements std_srvs/srv/SetBool "{data: false}"
        echo -e "${GREEN}✓ Mapping resumed${NC}"
        ;;

    save)
        if [ -z "$1" ]; then
            echo -e "${RED}Error: Please provide a filename${NC}"
            echo "Usage: $0 save <filename>"
            exit 1
        fi
        echo -e "${YELLOW}Saving map to $1...${NC}"
        ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$1'}"
        echo -e "${GREEN}✓ Map saved to:${NC}"
        echo "  - $1.posegraph"
        echo "  - $1.data"
        ;;

    save_ros)
        if [ -z "$1" ]; then
            echo -e "${RED}Error: Please provide a filename${NC}"
            echo "Usage: $0 save_ros <filename>"
            exit 1
        fi
        echo -e "${YELLOW}Saving map in ROS format to $1...${NC}"
        ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$1'}}"
        echo -e "${GREEN}✓ Map saved to:${NC}"
        echo "  - $1.pgm"
        echo "  - $1.yaml"
        ;;

    load)
        if [ -z "$1" ]; then
            echo -e "${RED}Error: Please provide a filename${NC}"
            echo "Usage: $0 load <filename>"
            exit 1
        fi
        echo -e "${YELLOW}Loading map from $1 and relocalizing...${NC}"
        ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph "{filename: '$1', match_type: 2}"
        echo -e "${GREEN}✓ Map loaded${NC}"
        ;;

    clear)
        echo -e "${YELLOW}Clearing scan queue...${NC}"
        ros2 service call /slam_toolbox/clear_queue std_srvs/srv/Empty
        echo -e "${GREEN}✓ Queue cleared${NC}"
        ;;

    list)
        echo -e "${GREEN}Available slam_toolbox services:${NC}"
        ros2 service list | grep slam_toolbox
        ;;

    help|--help|-h)
        usage
        ;;

    *)
        echo -e "${RED}Error: Unknown command '$COMMAND'${NC}"
        echo ""
        usage
        exit 1
        ;;
esac
