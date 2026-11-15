/**
  ******************************************************************************
  * @file    board_test.h
  * @brief   电赛测试板2（F407版）板级自检程序
  * @note    独立初始化，不依赖 CubeMX 生成的 MX_xxx_Init()，
  *          当前空工程也能直接编译运行。
  ******************************************************************************
  */
#ifndef __BOARD_TEST_H
#define __BOARD_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* 在 main() 的 while(1) 之前调用，函数不返回 */
void BoardTest_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_TEST_H */
