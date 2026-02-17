#!/bin/bash

SESSION="10_panes"

# 1. Kill the old session if it exists (Crucial for a clean start)
tmux kill-session -t $SESSION 2>/dev/null

# 2. Create a new session named "10_panes"
tmux new-session -d -s $SESSION

# 3. Split the window into 2 halves (Top and Bottom)
tmux split-window -v

# 4. Split the Top half into 5 vertical panes
tmux select-pane -t 0
tmux split-window -h
tmux split-window -h
tmux split-window -h
tmux split-window -h
tmux select-layout -t 0 tiled

# 5. Split the Bottom half into 5 vertical panes
tmux select-pane -t 5
tmux split-window -h
tmux split-window -h
tmux split-window -h
tmux split-window -h
tmux select-layout -t 5 tiled

# 6. Attach to the session so you can see it
tmux attach-session -t $SESSION
