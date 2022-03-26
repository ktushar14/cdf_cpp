#!/bin/bash

# Run from a build directory that contains the ./persistent_nosim executable or
# `cd` into one.

GREEN='\033[0;32m' # green
NC='\033[0m' # No Color

cd build_release

./persistent_nosim ../maps/walls_and_gaps_500_500/1.map ../patterns_sampled/patternssampled_160_160.txt &> /dev/null &
./persistent_nosim ../maps/walls_and_gaps_500_500/1.map ../patterns/frontier_only.txt &> /dev/null &

./persistent_nosim ../maps/walls_and_gaps_500_500/2.map ../patterns_sampled/patternssampled_160_160.txt &> /dev/null &
./persistent_nosim ../maps/walls_and_gaps_500_500/2.map ../patterns/frontier_only.txt &> /dev/null

wait

printf "${GREEN} Completed 500 x 500 maps 1 & 2, 60 x 60 patterns & frontier_only.${NC}\n"

# ./persistent_nosim ../maps/walls_and_gaps_500_500/3.map ../patterns_sampled/patternssampled_160_160.txt &> /dev/null &
# ./persistent_nosim ../maps/walls_and_gaps_500_500/3.map ../patterns/frontier_only.txt &> /dev/null &

# ./persistent_nosim ../maps/walls_and_gaps_500_500/4.map ../patterns_sampled/patternssampled_160_160.txt &> /dev/null &
# ./persistent_nosim ../maps/walls_and_gaps_500_500/4.map ../patterns/frontier_only.txt &> /dev/null

# wait

# printf "${GREEN} Completed 500 x 500 maps 3 & 4, 60 x 60 patterns & frontier_only.${NC}\n"

# ./persistent_nosim ../maps/walls_and_gaps_500_500/5.map ../patterns_sampled/patternssampled_160_160.txt &> /dev/null &
# ./persistent_nosim ../maps/walls_and_gaps_500_500/5.map ../patterns/frontier_only.txt &> /dev/null &

# ./persistent_nosim ../maps/walls_and_gaps_500_500/6.map ../patterns_sampled/patternssampled_160_160.txt &> /dev/null &
# ./persistent_nosim ../maps/walls_and_gaps_500_500/6.map ../patterns/frontier_only.txt &> /dev/null

# printf "${GREEN} Completed 500 x 500 maps 5 & 6, 60 x 60 patterns & frontier_only.${NC}\n"

# wait

# printf "${GREEN}DONE!${NC}\n"
