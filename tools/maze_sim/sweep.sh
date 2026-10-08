#!/bin/sh
# 探索のコストを変えながら、乱数の迷路でまとめて試す。
# 1行 = 1つの設定。moves(探索で動いた区画数 行き + 帰り)は少ないほど、
# optimal(真の最短を見つけた迷路の数)は多いほど、cost ratio(見つけた経路 / 真の最短)は 1 に近いほどよい。
#
# 使い方: sh tools/maze_sim/sweep.sh [迷路の数(既定300)]
set -e
cd "$(dirname "$0")"
N=${1:-300}
SIM=build/maze_sim

echo "--- reference ---"
$SIM --batch "$N" --algo adachi
$SIM --batch "$N"

echo "--- search turn cost (goal and back) ---"
for T in "0,0" "0,2" "1,4" "2,6" "4,10" "7,50"; do
    $SIM --batch "$N" --goal "1,$T,0" --back "1,$T,0"
done

echo "--- return: known-cell penalty (turn cost 0,2) ---"
for K in 0 1 2 4 8; do
    $SIM --batch "$N" --goal "1,0,2,0" --back "1,0,2,$K"
done

# 大会の迷路(sh tools/maze_sim/fetch_mazes.sh で取ってくる)があれば、全日本の迷路でも比べる
if ls mazes/alljapan-*.txt >/dev/null 2>&1; then
    echo "--- all-Japan contest mazes ---"
    $SIM --algo adachi mazes/alljapan-*.txt
    $SIM mazes/alljapan-*.txt
    $SIM --goal "1,0,2,0" --back "1,0,2,0" mazes/alljapan-*.txt
    $SIM --goal "1,0,2,0" --back "1,0,2,3" mazes/alljapan-*.txt
fi
