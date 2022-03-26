#!/bin/bash

# Run from a build directory that contains the ./persistent executable or
# `cd` into one.

GREEN='\033[0;32m' # green
NC='\033[0m' # No Color

cd build_release

##################
# WALLS AND GAPS #
##################
declare -a pattern_files=("patterns_30_30.txt"
                          "frontier_only.txt")
count="0"
for map_file_path in ../maps/walls_and_gaps/*.map; do
    for pattern_file in "${pattern_files[@]}"; do
        pattern_file_path="../patterns/${pattern_file}"
        now=$(date +"%T")
        printf "${GREEN}Current time : $now${NC}\n"
        printf "\n"
        printf "map_file: ${map_file_path}\n"
        printf "pattern_file: ${pattern_file_path}\n"
        ./persistent "$map_file_path" "$pattern_file_path" &> /dev/null
        printf "${GREEN}Completed: ${map_file_path} with ${pattern_file_path}${NC}\n"
        ((count=count+1))
    done
done
printf "--------------------------------------------------------------------------------\n"
printf "${GREEN}DONE!${NC}\n"
