#!/bin/sh
# micromouseonline/mazefiles (https://github.com/micromouseonline/mazefiles) の
# classic (16x16) 迷路を tools/maze_sim/mazes/ に取ってくる。
# リポジトリにライセンスの記載がないため、取ってきたファイルは git に入れない (.gitignore)。
#
# 使い方: sh tools/maze_sim/fetch_mazes.sh
set -e
cd "$(dirname "$0")"
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

git clone --depth 1 -q https://github.com/micromouseonline/mazefiles.git "$TMP/mazefiles"
mkdir -p mazes
cp "$TMP"/mazefiles/classic/*.txt mazes/
# どの版を取ってきたかの記録(拡張子なしにして、迷路ファイルと区別する)
(cd "$TMP/mazefiles" && git log -1 --format='micromouseonline/mazefiles %H (%cd)') > mazes/_source

echo "fetched $(ls mazes/*.txt | wc -l) mazes into tools/maze_sim/mazes/"
