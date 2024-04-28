
#!/bin/bash

# Open a new terminal window with three tabs
gnome-terminal --tab --title="Tab 1" --command="bash -c 'sshpass -p 000000 ssh orangepi@10.10.10.20'" \
               --tab --title="Tab 2" --command="bash -c 'sshpass -p 000000 ssh orangepi@10.10.10.20'" 
gnome-terminal -- bash -c " sshpass -p 000000 ssh orangepi@10.10.10.20;exec bash;"