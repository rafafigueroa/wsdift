#!/bin/bash  
git add -A  
read -p "Commit description: " desc  
git commit -a -m "$desc"
git push
