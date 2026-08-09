#!/usr/bin/env bash
num_of_seeds=6

for seed in $(seq 4 $num_of_seeds); do
  echo random-32-32-20 with 600 agents, seed $seed
  ./mg.sh build/main -i ../lacam2/assets/random-32-32-20.600.scen -m ../lacam2/assets/random-32-32-20.map -N 600 -v 3 -t 60 -O 2 --refiner-num 1 --pibt-num 4 --recursive-rate 1 --seed $seed > rand.$seed.txt
done
