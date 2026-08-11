#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

// 打开摄像头 /dev/video0, 分辨率 640x480
// 返回句柄 (intptr_t, 64-bit 安全), 失败返回 -1
intptr_t camera_open(const char *dev, int w, int h);

// 抓一帧, 返回 YUYV 缓冲区指针 (调用者不需释放)
unsigned char* camera_capture(intptr_t fd);

// HSV颜色识别 → 判定颜色 0-3
// 返回: 0=红 1=黄 2=绿 3=蓝, -1=未识别
int camera_detect_color(intptr_t fd);

// 采集一帧 → 颜色识别 + 返回 RGB 预览图 (带标注框)
//   rgb_out : 静态缓冲区指针 (320x240x3 bytes RGB, 下一次调用前有效)
//   w, h    : 输出实际宽高
//   cx, cy  : 输出目标质心坐标 (px, 无目标时=0)
//   area    : 输出轮廓面积 (px²)
//   box_w, box_h: 输出包围框宽高
// 返回: 0=红 1=黄 2=绿 3=蓝, -1=无目标
int camera_get_preview(intptr_t fd, const unsigned char **rgb_out,
                       int *w, int *h,
                       double *cx, double *cy, int *area,
                       int *box_w, int *box_h);

// 关闭摄像头
void camera_close(intptr_t fd);

//==========================================================================
// HSV 阈值运行时调节 (0=红 1=黄 2=绿 3=蓝)
//==========================================================================

// 获取某个颜色的当前 HSV 阈值 (6个值: H_min,H_max,S_min,S_max,V_min,V_max)
void camera_get_hsv_thresh(int color_idx, int *h_min, int *h_max,
                           int *s_min, int *s_max, int *v_min, int *v_max);

// 设置某个颜色的 HSV 阈值 (0-255)
void camera_set_hsv_thresh(int color_idx, int h_min, int h_max,
                           int s_min, int s_max, int v_min, int v_max);

// 获取/设置 红色第二HSV范围 (H 170-180)
void camera_get_red2_thresh(int *h_min, int *h_max,
                            int *s_min, int *s_max, int *v_min, int *v_max);
void camera_set_red2_thresh(int h_min, int h_max,
                            int s_min, int s_max, int v_min, int v_max);

// 保存/加载阈值到文件
int  camera_save_thresh(const char *path);
int  camera_load_thresh(const char *path);

#ifdef __cplusplus
}
#endif
