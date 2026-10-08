#ifndef INC_WALLMAP_H_
#define INC_WALLMAP_H_


#include "global.h"
#include "params.h"
#include "logic/maze/maze_types.h"

// 迷路の壁の地図。1区画1バイトで、BlueEyesのmaze_wallと同じビット配置。
//   下位4bit: 探索用。未知の壁は「ない」とみなす
//   上位4bit: 最短走行用。未知の壁は「ある」とみなす
//   各4bitは 北0x8 / 東0x4 / 南0x2 / 西0x1
// 見た壁は上下両方を同じ値にするので、「下位0・上位1」の壁だけが未知。
// 隣り合う区画の同じ壁は、常に両方の区画で揃えて持つ。
typedef struct {
    uint8_t cell[MAZE_SIZE][MAZE_SIZE]; // [y][x]
} WallMap;

// 壁の読み方
typedef enum {
    WALL_VIEW_SEARCH, // 未知の壁は「ない」(探索用。まだ見ていない道も通れるものとして計算する)
    WALL_VIEW_KNOWN,  // 未知の壁は「ある」(最短走行用。確かめた道だけを通る)
} WallView;

// 1区画で見えた壁(機体の向きから見た前・右・左)。後ろはセンサーで見ない。
typedef struct {
    bool front;
    bool right;
    bool left;
} WallObservation;

// 外周だけを既知の壁にし、内側の壁はすべて未知にする。
void WallMap_Init(WallMap *map);

// pのdの向きの壁を、見た結果としてexistsに決める(隣の区画の同じ壁も揃える)。
// 外周の壁は、existsがfalseでも消さない(センサーの誤りで迷路の外へ出ないように)。
void WallMap_SetWall(WallMap *map, MazePos p, Direction d, bool exists);

bool WallMap_HasWall(const WallMap *map, MazePos p, Direction d, WallView view);
bool WallMap_IsKnown(const WallMap *map, MazePos p, Direction d);

// pの4方向の壁がすべて分かっている(= その区画はもう見る必要がない)ならtrue。
bool WallMap_IsCellKnown(const WallMap *map, MazePos p);

// pに向きheadingでいるときに見えた前・右・左の壁を書き込む。
void WallMap_Observe(WallMap *map, MazePos p, Direction heading, WallObservation obs);

#endif
