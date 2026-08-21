#!/bin/bash

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}Autopilot NDT Tmuxinator Setup${NC}"
echo "================================"

# Check if tmux is installed
if ! command -v tmux &> /dev/null; then
    echo -e "${YELLOW}tmux is not installed. Installing...${NC}"
    sudo apt-get update
    sudo apt-get install -y tmux
fi

# Check if tmuxinator is installed
if ! command -v tmuxinator &> /dev/null; then
    echo -e "${YELLOW}tmuxinator is not installed. Installing...${NC}"

    # Check if ruby is installed
    if ! command -v ruby &> /dev/null; then
        echo -e "${YELLOW}Ruby is not installed. Installing Ruby...${NC}"
        sudo apt-get update
        sudo apt-get install -y ruby ruby-dev
    fi

    # Install tmuxinator gem
    sudo gem install tmuxinator

    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ tmuxinator installed successfully${NC}"
    else
        echo -e "${RED}✗ Failed to install tmuxinator${NC}"
        exit 1
    fi
else
    echo -e "${GREEN}✓ tmuxinator is already installed${NC}"
fi

# Create tmuxinator config directory if it doesn't exist
TMUXINATOR_DIR="$HOME/.config/tmuxinator"
if [ ! -d "$TMUXINATOR_DIR" ]; then
    echo -e "${YELLOW}Creating tmuxinator config directory...${NC}"
    mkdir -p "$TMUXINATOR_DIR"
fi

# Copy the config file
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
CONFIG_FILE="$SCRIPT_DIR/autopilot_ndt.yml"

if [ -f "$CONFIG_FILE" ]; then
    cp "$CONFIG_FILE" "$TMUXINATOR_DIR/autopilot_ndt.yml"
    echo -e "${GREEN}✓ Config copied to $TMUXINATOR_DIR/autopilot_ndt.yml${NC}"
else
    echo -e "${RED}✗ Config file not found: $CONFIG_FILE${NC}"
    exit 1
fi

# Create or update tmux config
TMUX_CONF="$HOME/.tmux.conf"
echo -e "${YELLOW}Setting up tmux configuration...${NC}"

# Backup existing config if it exists
if [ -f "$TMUX_CONF" ]; then
    cp "$TMUX_CONF" "$TMUX_CONF.backup.$(date +%Y%m%d_%H%M%S)"
    echo -e "${YELLOW}Backed up existing tmux config${NC}"
fi

# Create tmux config
cat > "$TMUX_CONF" << 'EOF'
# ============================
# Tmux Configuration
# ============================

# Remap prefix from 'C-b' to 'C-a'
unbind C-b
set-option -g prefix C-a
bind-key C-a send-prefix

# Terminal colors
set -g default-terminal "screen-256color"
set -g terminal-overrides "xterm-color256:smcup@:rmcup@"

# Mouse support (commented out by default)
# set -g mouse on

# ============================
# Navigation Shortcuts
# ============================

# Shift+arrow to switch windows (no prefix needed)
bind -n S-Left  previous-window
bind -n S-Right next-window

# Ctrl+arrow to switch panes (no prefix needed, doesn't conflict with shell)
bind -n C-Left  select-pane -L
bind -n C-Right select-pane -R
bind -n C-Up    select-pane -U
bind -n C-Down  select-pane -D

# ============================
# Useful Key Bindings
# ============================

# Remap kill button to ^K
bind-key k \
  split-window \; \
  setw synchronize-panes on \; \
  send-keys "sleep 1; pwd >> /tmp/tmux_restore_path.txt; tmux list-panes -s -F \"#{pane_pid} #{pane_current_command}\" | grep -v tmux | awk '{print \$1}' | while read in; do killp \$in; done" C-m exit C-m

# Toggle synchronize-panes with prefix + s
bind s \
    set synchronize-panes \;\
    display "Sync #{?synchronize-panes,ON,OFF}"

# Reload config with prefix + r
bind r source-file ~/.tmux.conf \; display "Config reloaded!"

# ============================
# Status Bar Configuration
# ============================

# Info on left (no session display)
set -g status-left ''

# Quiet mode
set-option -g visual-activity off
set-option -g visual-bell off
set-option -g visual-silence off
set-window-option -g monitor-activity off
set-option -g bell-action none

# The modes
setw -g clock-mode-colour colour135

# Status bar position
set -g status-position bottom

# Status bar colors
set -g status-bg colour234
set -g status-fg colour15

# Status bar content
set -g status-left "#[fg=colour15,bg=colour26] #S #[fg=colour103,bg=colour236,nobold,nounderscore,noitalics]"
set -g status-right "#[fg=colour239] #(echo $ROS_MASTER_URI) #[fg=colour239,bg=colour236,nobold,nounderscore,noitalics]#[fg=colour248,bg=colour239] %H:%M #[fg=colour15,bg=colour26] #H"

set -g status-right-length 50
set -g status-left-length 20

# Window status format
setw -g window-status-current-format "#[fg=colour236,bg=colour239,nobold,nounderscore,noitalics]#[fg=colour253,bg=colour239] #I |#[fg=colour253,bg=colour239] #W #[fg=colour239,bg=colour236,nobold,nounderscore,noitalics]"
setw -g window-status-format "#[fg=colour244,bg=colour236] #I |#[fg=colour244,bg=colour236] #W "

# ============================
# Copy Mode (Vim-style)
# ============================

# Use vim keybindings in copy mode
setw -g mode-keys vi

# Increase scrollback buffer size
set -g history-limit 50000
EOF

echo -e "${GREEN}✓ Tmux config created at $TMUX_CONF${NC}"

echo ""
echo -e "${GREEN}Setup complete!${NC}"
echo ""
echo "To start the session, run:"
echo -e "  ${YELLOW}tmuxinator start autopilot_ndt${NC}"
echo ""
echo "Or use the shorthand:"
echo -e "  ${YELLOW}mux start autopilot_ndt${NC}"
echo ""
echo "To stop the session:"
echo -e "  ${YELLOW}tmux kill-session -t autopilot_ndt${NC}"
echo ""
echo -e "${GREEN}Tmux Navigation Cheatsheet:${NC}"
echo "  Ctrl+Arrow keys     - Switch between panes (no prefix needed)"
echo "  Shift+Arrow keys    - Switch between windows (no prefix needed)"
echo "  Ctrl+A (prefix)     - Main prefix key (instead of Ctrl+B)"
echo "  Prefix + s          - Toggle synchronize-panes"
echo "  Prefix + k          - Kill panes"
echo "  Prefix + r          - Reload tmux config"
echo ""
echo "Status bar displays: Session | ROS_MASTER_URI | Time | Hostname"
