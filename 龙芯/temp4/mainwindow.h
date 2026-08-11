#pragma once
#include <stdint.h>
#include <QMainWindow>
#include <QTimer>
#include <QLabel>
#include <QStackedWidget>
#include <QPushButton>
#include <QSpinBox>
#include <QSlider>
#include <QDateTime>
#include "i2c_worker.h"
namespace Ui { class MainWindow; }

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget *p=nullptr); ~MainWindow();
    enum Mode { SEL=0, REMOTE, TEACH, VISION, SAFETY };
private slots:
    void onPoll(); void toMode(Mode m);
    void j1c();void j1a();void j2u();void j2d();void j3c();void j3a();void j4f();void j4r();
    void s1p();void s1m();void s2p();void s2m();
    void stop();void em(int n);
    void ts(int s);void tv();void tsub();void tload();void tplay();
    void visDetect();
private:
    void b0();void b1();void b2();void b3();void b4();
    QWidget* mc(int i);
    QWidget* mj(const char*n,const char*l,void(MainWindow::*ls)(),const char*r,void(MainWindow::*rs)());
    QWidget* msv();

    Ui::MainWindow *ui; I2cWorker m_w;QTimer*m_t;Mode m_m=SEL;
    QLabel* m_pb[4],*m_cb[4],*m_tb[4],*m_vb[4];QLabel* m_s1b,*m_s2b;
    int m_sv1=127,m_sv2=80, m_sv2c=140,m_sv2o=80;bool m_fn=false;
    enum{SP_M1=100,SP_M2=300,SP_M3=100,SP_M4=200};
    struct WP {float a,c,d;quint8 e;float b;};WP m_wp[6];bool m_h[6];int m_ts;
    QLabel*m_wl[6][6],*m_tcur[6];float m_seg[5]={23,21,21,21,15};
    bool m_pl;int m_ps;qint64 m_pst;

    // 视觉 — 摄像头 + 实时预览 (融合 ColorDetect)
    intptr_t m_camfd=-1; int m_color=-1, m_detColor=-1;
    QLabel *m_cl,*m_cn;             // 预览图, 颜色结果
    QLabel *m_vx,*m_vy,*m_va,*m_vs;// X/Y/面积/尺寸
    QLabel *m_camSt;                // 摄像头状态标签
    double m_vxB[5], m_vyB[5];     // 坐标平滑缓冲 (5帧滑动窗)
    int    m_vbufIdx;

    // 安全分拣 (模式4)
    enum SortSt {SS_IDLE,
        SS_HOME_M2,SS_HOME_M1,SS_HOME_M3,SS_HOME_SV, // 回原点: 先升降! M2→M1→M3→SV1
        SS_DET_M1,SS_DET_M3,SS_DET_SV,               // 移到取料位: M1→M3→SV1
        SS_PICK,SS_LIFT,SS_PLACE,                   // Z轴下降/夹爪/Z轴升高
        SS_EJT_M1,SS_EJT_M3,SS_EJT_SV,             // 移到传送带: M1→M3→SV1
        SS_RET,SS_COOL,                              // Z轴放置/开爪
        SS_CAMADV,SS_CAMDET,                        // 推进CamDist/视觉检测
        SS_PLA_M1,SS_PLA_SV,                        // 回位: M1→SV1 + M4段位
        SS_EMTRIG,                                   // 推杆
        SS_RT2_M1,SS_RT2_M2,SS_RT2_M3,SS_RT2_WAIT}; // 回原点: M1→M2→M3
    SortSt m_ss=SS_IDLE; bool m_sr=false;  // m_sr=sortRunning
    qint64 m_sst; int m_sc[4]={0,0,0,0};   // step start time, color counts
    QLabel *m_sfSt,*m_sfCo,*m_sfTo,*m_sfCu,*m_sfEm;  // 安全指示灯
    QLabel *m_scLb[4];                     // 分拣计数标签
    void srStart(); void srStop(); void srTick();

    int jD(){return m_fn?1:5;}int jL(){return m_fn?1:1;}int jC(){return m_fn?1:5;}int svS(){return m_fn?2:10;}
    qint64 m_lj;bool cj(){qint64 n=QDateTime::currentMSecsSinceEpoch();if(n-m_lj<500)return false;m_lj=n;return true;}
};
