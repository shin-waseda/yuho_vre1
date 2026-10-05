#ifndef INC_SDDUMP_H_
#define INC_SDDUMP_H_


#include "global.h"
#include "params.h"

// SDカードの未送信のCSVを、ボタンでまとめてUARTへ送るモード。
// 送ったファイルはSD上で sent/<元のパス> へ移す(消さない)ので、次回は新しいものだけ送る。
// PC側は tools/get_log.py で受け、logs/<元のパス> に保存する。
//
// 送信形式:
//   FILE_START\r\n
//   <path>\r\n
//   SIZE:<bytes>\r\n
//   <ファイルの中身そのまま>
//   FILE_END\r\n      (途中で読めなかったら FILE_ERROR\r\n。中身はSIZEぶん0で埋める)
//
// 電源を切るまで戻らない。
void SdDump_Run(void);

// SDカードのすべてのCSV(未送信 + sent/ の送信済み)を、ボタンでUARTへ送るモード。
// ファイルは移動しない。送信済みのものは sent/ を外した元のパスで送るので、
// PCでは元と同じ場所に保存される(get_log.pyは同じ中身のファイルがあれば保存しない)。
// 電源を切るまで戻らない。
void SdDumpAll_Run(void);

#endif
