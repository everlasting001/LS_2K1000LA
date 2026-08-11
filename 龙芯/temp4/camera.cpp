//============================================================================
// camera.cpp — OpenCV USB摄像头 + HSV颜色识别 (v4: 可调阈值+保存)
//
// 新增:
//   4. camera_get_preview(): 采集+识别+标注 RGB 预览+质心/面积/包围框
//   5. 运行时 HSV 阈值调节 + 保存/加载
//============================================================================
#include "camera.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <cstdio>
#include <cstring>

// 四个颜色的HSV阈值 (0=红 1=黄 2=绿 3=紫) — 运行时可变
//   每色6值: H_min,H_max,S_min,S_max,V_min,V_max
static int g_hsv[4][6] = {
    {0, 10,  120, 255,  80, 255},   // 0=红色 (双范围, 第二组见 g_red2)
    {22, 32,  90, 255,  70, 255},   // 1=黄色
    {38, 75,  90, 255,  70, 255},   // 2=绿色
    {100, 130, 80, 255,  60, 255},  // 3=蓝色
};

// 红色第二HSV范围 (H 170-180, S/V 与主范围共享)
static int g_red2[6] = {170, 180, 120, 255, 80, 255};

// 标注颜色 (BGR) — 来自 ColorDetect 配色方案
static const int OVERLAY_BGR[4][3] = {
    {255, 60,  0},    // 红: QColor(0,60,255) → BGR
    {240, 220, 0},    // 黄: QColor(0,220,240) → BGR
    {40,  220, 40},   // 绿: QColor(40,220,40) → BGR
    {200, 80,  40},   // 蓝: QColor(40,80,200) → BGR
};
static const char* COLOR_NAMES[4] = {"RED", "YEL", "GRN", "BLU"};

// 预览图静态缓冲区 (320x240x3 = 230,400 bytes)
static unsigned char g_preview_rgb[320 * 240 * 3];

intptr_t camera_open(const char *dev, int w, int h) {
    int idx = -1;
    if (dev && sscanf(dev, "/dev/video%d", &idx) == 1) {
    } else {
        idx = 0;
    }
    cv::VideoCapture *cap = new cv::VideoCapture(idx, cv::CAP_V4L2);
    if (!cap->isOpened()) { cap->release(); delete cap; cap = new cv::VideoCapture(idx); }
    if (!cap->isOpened()) { delete cap; return -1; }

    cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap->set(cv::CAP_PROP_FRAME_WIDTH,  w);
    cap->set(cv::CAP_PROP_FRAME_HEIGHT, h);
    cap->set(cv::CAP_PROP_FPS, 30);
    cap->set(cv::CAP_PROP_BUFFERSIZE, 2);

    return (intptr_t)cap;
}

unsigned char* camera_capture(intptr_t) {
    return nullptr;
}

// ---- 内部: 对单帧做颜色检测, 返回最大色块 ----
static int detect_in_frame(cv::Mat &hsv, cv::Rect *outRect,
                           std::vector<std::vector<cv::Point>> *outContour, int *outArea) {
    int bestColor = -1, bestArea = 0;
    cv::Rect bestRect;
    std::vector<std::vector<cv::Point>> bestContour;

    for (int i = 0; i < 4; i++) {
        cv::Mat mask;
        if (i == 0) {
            // 红色: 双HSV范围取并集
            cv::Mat m1, m2;
            cv::inRange(hsv, cv::Scalar(g_hsv[0][0], g_hsv[0][2], g_hsv[0][4]),
                              cv::Scalar(g_hsv[0][1], g_hsv[0][3], g_hsv[0][5]), m1);
            cv::inRange(hsv, cv::Scalar(g_red2[0], g_red2[2], g_red2[4]),
                              cv::Scalar(g_red2[1], g_red2[3], g_red2[5]), m2);
            mask = m1 | m2;
        } else {
            cv::inRange(hsv, cv::Scalar(g_hsv[i][0], g_hsv[i][2], g_hsv[i][4]),
                              cv::Scalar(g_hsv[i][1], g_hsv[i][3], g_hsv[i][5]), mask);
        }

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (size_t j = 0; j < contours.size(); j++) {
            int a = cv::contourArea(contours[j]);
            if (a > bestArea && a > 80) {
                bestArea = a;
                bestColor = i;
                bestContour = contours;
                bestRect = cv::boundingRect(contours[j]);
            }
        }
    }
    if (outRect)    *outRect = bestRect;
    if (outContour) *outContour = bestContour;
    if (outArea)    *outArea = bestArea;
    return bestColor;
}

int camera_detect_color(intptr_t fd) {
    if (fd <= 0) return -1;
    cv::VideoCapture *cap = (cv::VideoCapture*)fd;
    cv::Mat frame;
    if (!cap->read(frame) || frame.empty()) return -1;
    cv::resize(frame, frame, cv::Size(320, 240));
    cv::Mat hsv; cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    return detect_in_frame(hsv, nullptr, nullptr, nullptr);
}

//============================================================================
// camera_get_preview
//============================================================================
int camera_get_preview(intptr_t fd, const unsigned char **rgb_out,
                       int *w, int *h,
                       double *cx, double *cy, int *area,
                       int *box_w, int *box_h) {
    if (fd <= 0) return -1;
    cv::VideoCapture *cap = (cv::VideoCapture*)fd;

    cv::Mat frame;
    if (!cap->read(frame) || frame.empty()) return -1;
    cv::resize(frame, frame, cv::Size(320, 240));

    cv::Mat hsv; cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::Rect bestRect; int bestArea;
    std::vector<std::vector<cv::Point>> bestContour;
    int bestColor = detect_in_frame(hsv, &bestRect, &bestContour, &bestArea);

    // 输出位置信息
    if (bestColor >= 0 && bestRect.width > 0) {
        *cx  = bestRect.x + bestRect.width  / 2.0;
        *cy  = bestRect.y + bestRect.height / 2.0;
        *area   = bestArea;
        *box_w  = bestRect.width;
        *box_h  = bestRect.height;
    } else {
        *cx = *cy = 0.0; *area = *box_w = *box_h = 0;
    }

    // 标注叠加
    if (bestColor >= 0 && bestRect.width > 0) {
        int r = OVERLAY_BGR[bestColor][2], g = OVERLAY_BGR[bestColor][1], b = OVERLAY_BGR[bestColor][0];
        cv::Scalar color(b, g, r);
        cv::rectangle(frame, bestRect, color, 2);
        int ccx = bestRect.x + bestRect.width / 2, ccy = bestRect.y + bestRect.height / 2;
        cv::line(frame, cv::Point(ccx - 8, ccy), cv::Point(ccx + 8, ccy), color, 1);
        cv::line(frame, cv::Point(ccx, ccy - 8), cv::Point(ccx, ccy + 8), color, 1);
        char label[32];
        snprintf(label, sizeof(label), "%s %d", COLOR_NAMES[bestColor], bestArea);
        cv::putText(frame, label, cv::Point(bestRect.x, bestRect.y - 8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
        if (!bestContour.empty())
            cv::drawContours(frame, bestContour, 0, color, 1);
    }

    // 画布参考十字
    cv::line(frame, cv::Point(160, 105), cv::Point(160, 135), cv::Scalar(128,128,128), 1);
    cv::line(frame, cv::Point(145, 120), cv::Point(175, 120), cv::Scalar(128,128,128), 1);

    // BGR → RGB
    int total = frame.rows * frame.cols * 3;
    if (total > (int)sizeof(g_preview_rgb)) total = sizeof(g_preview_rgb);
    unsigned char *dst = g_preview_rgb;
    for (int y = 0; y < frame.rows; y++) {
        const unsigned char *src = frame.ptr<unsigned char>(y);
        for (int x = 0; x < frame.cols; x++) {
            dst[0] = src[2]; dst[1] = src[1]; dst[2] = src[0];
            dst += 3; src += 3;
        }
    }
    *rgb_out = g_preview_rgb; *w = frame.cols; *h = frame.rows;
    return bestColor;
}

void camera_close(intptr_t fd) {
    if (fd <= 0) return;
    cv::VideoCapture *cap = (cv::VideoCapture*)fd;
    cap->release(); delete cap;
}

//==========================================================================
// HSV 阈值运行时调节
//==========================================================================

void camera_get_hsv_thresh(int ci, int *h_min, int *h_max,
                           int *s_min, int *s_max, int *v_min, int *v_max) {
    if (ci < 0 || ci > 3) return;
    if (h_min) *h_min = g_hsv[ci][0];
    if (h_max) *h_max = g_hsv[ci][1];
    if (s_min) *s_min = g_hsv[ci][2];
    if (s_max) *s_max = g_hsv[ci][3];
    if (v_min) *v_min = g_hsv[ci][4];
    if (v_max) *v_max = g_hsv[ci][5];
}

void camera_get_red2_thresh(int *h_min, int *h_max,
                            int *s_min, int *s_max, int *v_min, int *v_max) {
    if (h_min) *h_min = g_red2[0];
    if (h_max) *h_max = g_red2[1];
    if (s_min) *s_min = g_red2[2];
    if (s_max) *s_max = g_red2[3];
    if (v_min) *v_min = g_red2[4];
    if (v_max) *v_max = g_red2[5];
}

void camera_set_hsv_thresh(int ci, int h_min, int h_max,
                           int s_min, int s_max, int v_min, int v_max) {
    if (ci < 0 || ci > 3) return;
    g_hsv[ci][0] = h_min; g_hsv[ci][1] = h_max;
    g_hsv[ci][2] = s_min; g_hsv[ci][3] = s_max;
    g_hsv[ci][4] = v_min; g_hsv[ci][5] = v_max;
}

void camera_set_red2_thresh(int h_min, int h_max,
                            int s_min, int s_max, int v_min, int v_max) {
    g_red2[0] = h_min; g_red2[1] = h_max;
    g_red2[2] = s_min; g_red2[3] = s_max;
    g_red2[4] = v_min; g_red2[5] = v_max;
}

int camera_save_thresh(const char *path) {
    FILE *f = fopen(path, "w");
    if (!f) return -1;
    for (int i = 0; i < 4; i++)
        fprintf(f, "%d %d %d %d %d %d %d\n", i,
                g_hsv[i][0],g_hsv[i][1],g_hsv[i][2],g_hsv[i][3],g_hsv[i][4],g_hsv[i][5]);
    // 红色第二范围
    fprintf(f, "R2 %d %d %d %d %d %d\n",
            g_red2[0],g_red2[1],g_red2[2],g_red2[3],g_red2[4],g_red2[5]);
    fclose(f);
    return 0;
}

int camera_load_thresh(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    int ci, a,b,c,d,e,g;
    char tag[8];
    while (fscanf(f, "%7s", tag) == 1) {
        if (strcmp(tag, "R2") == 0) {
            if (fscanf(f, "%d %d %d %d %d %d", &a,&b,&c,&d,&e,&g) == 6)
                camera_set_red2_thresh(a,b,c,d,e,g);
        } else {
            ci = atoi(tag);
            if (ci >= 0 && ci <= 3 &&
                fscanf(f, "%d %d %d %d %d %d", &a,&b,&c,&d,&e,&g) == 6)
                camera_set_hsv_thresh(ci, a,b,c,d,e,g);
        }
    }
    fclose(f);
    return 0;
}
