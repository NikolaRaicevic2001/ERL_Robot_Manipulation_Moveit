sudo systemctl stop ssh
for sid in $(who -u | grep -v -e "2337" | awk '{print $6}'); do sudo kill $sid; done
