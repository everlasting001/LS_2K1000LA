//============================================================================
// mainwindow.cpp — temp3 final: 滑块马达行 + 示教双页(控/存)
//============================================================================
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "camera.h"
#include "kinematics.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QFrame>
#include <QFile>
#include <QDataStream>
#include <QMessageBox>
#include <QScrollArea>
#include <QTabWidget>
#include <QCoreApplication>
#include <QPixmap>
#include <cmath>
#include <cstring>

MainWindow::MainWindow(QWidget *p) : QMainWindow(p), ui(new Ui::MainWindow) {
    ui->setupUi(this); m_w.openI2C(); memset(m_h,0,sizeof(m_h)); memset(m_wp,0,sizeof(m_wp));
    memset(m_vxB,0,sizeof(m_vxB)); memset(m_vyB,0,sizeof(m_vyB)); m_vbufIdx=0;
    m_t=new QTimer(this); m_t->setInterval(100); connect(m_t,&QTimer::timeout,this,&MainWindow::onPoll); m_t->start();
    b0();b1();b2();b3();b4(); ui->modeStack->setCurrentIndex(0);
}
MainWindow::~MainWindow(){ if(m_camfd>=0) camera_close(m_camfd); delete ui; }
void MainWindow::toMode(Mode m){
    if((m_m==VISION||m_m==SAFETY)&&m!=VISION&&m!=SAFETY&&m_camfd>0){camera_close(m_camfd);m_camfd=-1;}
    if((m==VISION||m==SAFETY)&&m_camfd<=0){m_camfd=camera_open("/dev/video0",640,480);if(m_camfd<=0)m_camfd=camera_open("/dev/video2",640,480);}
    m_m=m; const char*tt[]={"SCARA","< 遥控 >","< 示教 >","< 视觉 >","< 安全 >"};ui->lbModeTitle->setText(tt[m]);ui->modeStack->setCurrentIndex(m);
    // 所有模式切入时先回原点 (WP0)
    if(m!=SEL&&m_h[0]){WP&wp=m_wp[0];RobotState c=m_w.poll();int16_t a=arm_deg_to_pulse(wp.a-c.m1_deg),b=lift_mm_to_pulse(wp.b-c.m2_mm),d=arm_deg_to_pulse(wp.c-c.m3_deg),e=conveyor_mm_to_pulse(wp.d-c.m4_mm);m_w.cmdBoth(b,d,a,e,SP_M1);m_w.cmdServo(wp.e,255);}
    if(m==TEACH)ts(0);
}

//=== 轮询 ===
void MainWindow::onPoll(){
    RobotState s=m_w.poll();if(m_m==SEL)return;
    if(m_m==SAFETY){srTick();}  // 安全分拣状态机
    if(m_m==VISION){
        if(m_camSt)m_camSt->setText(m_camfd>0?"摄像头: 已连接":"摄像头: 未连接");
        if(m_camfd>0){
            // 每200ms采集一帧 (timer 100ms × 每2次)
            static int vc=0;
            if(++vc%2==0){
                const unsigned char *rgb; int pw,ph; double cx,cy; int area,bw,bh;
                m_color = camera_get_preview(m_camfd, &rgb, &pw, &ph, &cx, &cy, &area, &bw, &bh);
                if(rgb){
                    // 始终更新预览画面 (无论是否识别到颜色)
                    QImage img(rgb, pw, ph, QImage::Format_RGB888);
                    m_cl->setPixmap(QPixmap::fromImage(img).scaled(m_cl->size(), Qt::KeepAspectRatio, Qt::FastTransformation));
                }
                if(m_color>=0){
                    // 坐标平滑
                    m_vxB[m_vbufIdx]=cx; m_vyB[m_vbufIdx]=cy;
                    m_vbufIdx=(m_vbufIdx+1)%5;
                    double sx=0,sy=0;
                    for(int i=0;i<5;i++){sx+=m_vxB[i];sy+=m_vyB[i];}
                    sx/=5.0; sy/=5.0;
                    const char*cn[]={"红色","黄色","绿色","蓝色"};
                    const char*clr[]={"#003cff","#00dcf0","#28dc28","#2850c8"};
                    m_cn->setText(QString("● %1").arg(cn[m_color]));
                    m_cn->setStyleSheet(QString("font-size:28px;font-weight:bold;color:%1;padding:8px;").arg(clr[m_color]));
                    m_vx->setText(QString::number(sx,'f',1));
                    m_vy->setText(QString::number(sy,'f',1));
                    m_va->setText(QString::number(area)+"px²");
                    m_vs->setText(QString("%1×%2").arg(bw).arg(bh));
                    m_cl->setStyleSheet(QString("border:2px solid %1;border-radius:4px;background:transparent;").arg(clr[m_color]));
                }else{
                    m_cn->setText("未识别");
                    m_cl->setStyleSheet("border:2px solid #30363d;border-radius:4px;background:#1a1a2e;");
                    m_vx->setText("-");m_vy->setText("-");m_va->setText("-");m_vs->setText("-");
                }
            }
        }else{m_cn->setText("摄像头未连接");m_cl->setStyleSheet("border:2px solid #30363d;border-radius:4px;background:#1a1a2e;");}
    }
    if(m_pl){
        qint64 el=QDateTime::currentMSecsSinceEpoch()-m_pst;
        auto go=[&](int step){m_pst=QDateTime::currentMSecsSinceEpoch();m_ps=step;};
        auto mvM1=[&](int wpi){WP&wp=m_wp[wpi];RobotState c=m_w.poll();int16_t aa=arm_deg_to_pulse(wp.a-c.m1_deg);m_w.cmdBase(aa,0,SP_M1);};
        auto mvM3=[&](int wpi){WP&wp=m_wp[wpi];RobotState c=m_w.poll();int16_t cc=arm_deg_to_pulse(wp.c-c.m3_deg);m_w.cmdArm(0,cc,SP_M3);};
        auto mvSV=[&](int wpi){WP&wp=m_wp[wpi];m_w.cmdServo(wp.e,255);};
        auto mvM2=[&](int wpi){WP&wp=m_wp[wpi];RobotState c=m_w.poll();int16_t p=lift_mm_to_pulse(wp.b-c.m2_mm);m_w.cmdArm(p,0,SP_M2);};
        // M4 相对距离: 永远正向, 不减去当前位置
        auto mvM4=[&](float mm){m_w.cmdBase(0,conveyor_mm_to_pulse(mm),SP_M4);};
        switch(m_ps){
        // Step0: 移到取料位 WP1 — M1→2s→M3→2s→SV
        case 0: if(el>=5000){go(1);mvM1(1);} break;  // 大臂
        case 1: if(el>=2000){go(2);mvM3(1);} break;  // 小臂
        case 2: if(el>=2000){go(3);mvSV(1);} break;  // 旋转舵机
        // Step1: Z轴下降 WP2
        case 3: if(el>=2000){go(4);mvM2(3);} break;
        // Step2: 夹爪闭合
        case 4: if(el>=4000){go(5);m_w.cmdServo(255,m_sv2c);} break;
        // Step3: Z轴升高
        case 5: if(el>=6000){go(6);mvM2(2);} break;
        // Step4: 移到传送带 WP4 — M1→2s→M3→2s→SV
        case 6: if(el>=2000){go(7);mvM1(4);} break;
        case 7: if(el>=2000){go(8);mvM3(4);} break;
        case 8: if(el>=2000){go(9);mvSV(4);} break;
        // Step5: Z轴放置 WP5
        case 9: if(el>=2000){go(10);mvM2(5);} break;
        // Step6: 开爪放下
        case 10: if(el>=4000){go(11);m_w.cmdServo(255,m_sv2o);} break;
        // Step7: 物料落稳→推进CamDist(相对距离!)
        case 11: if(el>=6000){go(12);mvM4(m_seg[4]*10.f);} break;
        // Step8: 结束
        case 12: if(el>=2000){m_pl=false;return;} break;
        } return;
    }
    struct MD{bool ok;float v;QString u;quint16 c;float t;};
    MD md[4]={{s.m1_ok,s.m1_deg,"deg",s.cur_m1_ma,s.temp_m1_c},{s.m2_ok,s.m2_mm,"mm",s.cur_m2_ma,s.temp_m2_c},{s.m3_ok,s.m3_deg,"deg",s.cur_m3_ma,s.temp_m3_c},{s.m4_ok,s.m4_mm,"mm",s.cur_m4_ma,s.temp_m4_c}};
    for(int i=0;i<4;i++)if(m_pb[i]){if(md[i].ok)m_pb[i]->setText(QString::number(md[i].v,'f',md[i].u=="mm"?2:1)+md[i].u);else m_pb[i]->setText("--");m_cb[i]->setText(QString::number(md[i].c)+"mA");m_tb[i]->setText(QString::number(md[i].t,'f',1)+"C");if(m_vb[i])m_vb[i]->setText(QString::number(s.voltage_mv)+"mV");}
    if(m_s1b)m_s1b->setText(QString::number(m_sv1));
    if(m_s2b)m_s2b->setText(QString::number(m_sv2));
}

//=== 组件 ===
QWidget* MainWindow::mc(int i){
    const char*nn[]={"M1 大臂","M2 升降","M3 小臂","M4 传送带"};
    QFrame*c=new QFrame;c->setObjectName("card");QVBoxLayout*l=new QVBoxLayout(c);l->setSpacing(0);l->setContentsMargins(4,2,4,2);
    l->addWidget(new QLabel(nn[i]));m_pb[i]=new QLabel("--");m_pb[i]->setObjectName("val");l->addWidget(m_pb[i]);
    QHBoxLayout*r1=new QHBoxLayout;r1->addWidget(new QLabel("电压:"));m_vb[i]=new QLabel("--");r1->addWidget(m_vb[i]);r1->addStretch();l->addLayout(r1);
    QHBoxLayout*r2=new QHBoxLayout;r2->addWidget(new QLabel("电流:"));m_cb[i]=new QLabel("--");r2->addWidget(m_cb[i]);r2->addStretch();
    r2->addWidget(new QLabel("温度:"));m_tb[i]=new QLabel("--");r2->addWidget(m_tb[i]);l->addLayout(r2);return c;
}

//=== 模式选择 ===
void MainWindow::b0(){QWidget*w=new QWidget;QString bg=QCoreApplication::applicationDirPath()+"/BackGround1.png";w->setStyleSheet("QWidget{border-image:url("+bg+") 0 0 0 0 stretch stretch;}");
    QVBoxLayout*l=new QVBoxLayout(w);l->setAlignment(Qt::AlignCenter);
    QLabel*t=new QLabel("SCARA 机械臂控制系统");t->setStyleSheet("font-size:26px;font-weight:bold;color:#1a73e8;background:transparent;");t->setAlignment(Qt::AlignCenter);l->addWidget(t);l->addSpacing(30);
    auto btn=[&](const char*tx,Mode m){QPushButton*b=new QPushButton(tx);b->setMinimumHeight(60);b->setMinimumWidth(400);b->setObjectName("primary");b->setStyleSheet("font-size:16px;");connect(b,&QPushButton::clicked,this,[this,m](){toMode(m);});l->addWidget(b);l->addSpacing(10);};
    btn("远程遥控模式",REMOTE);btn("示教/分拣参数",TEACH);btn("视觉识别调试",VISION);btn("安全分拣模式",SAFETY);l->addStretch();ui->modeStack->addWidget(w);}

//=== 遥控 ===
void MainWindow::b1(){QWidget*p=new QWidget;QHBoxLayout*hl=new QHBoxLayout(p);hl->setSpacing(4);hl->setContentsMargins(2,2,2,2);
    QWidget*lw=new QWidget;lw->setFixedWidth(155);QVBoxLayout*ll=new QVBoxLayout(lw);ll->setSpacing(1);ll->setContentsMargins(0,0,0,0);
    for(int i=0;i<4;i++)ll->addWidget(mc(i));
    auto mkSC=[&](const char*nm,QLabel*&lb,int df){QFrame*c=new QFrame;c->setObjectName("card");QHBoxLayout*l=new QHBoxLayout(c);l->setContentsMargins(4,2,4,2);l->addWidget(new QLabel(nm));l->addStretch();lb=new QLabel(QString::number(df));lb->setObjectName("val");l->addWidget(lb);return c;};
    ll->addWidget(mkSC("SV1",m_s1b,127));ll->addWidget(mkSC("SV2",m_s2b,80));ll->addStretch();
    QWidget*rw=new QWidget;QVBoxLayout*rl=new QVBoxLayout(rw);rl->setSpacing(2);rl->setContentsMargins(0,0,0,0);
    auto R=[this](const char*nm,QLabel*&pos,const char*un,int c1,int c2,auto f1,auto f2,auto f3,auto f4){
        QHBoxLayout*r=new QHBoxLayout;r->addWidget(new QLabel(nm));
        auto b=[&](QString t,auto cb){QPushButton*p=new QPushButton(t);p->setFixedWidth(42);connect(p,&QPushButton::clicked,this,cb);r->addWidget(p);};
        b("-"+QString::number(c1),f1);b("-"+QString::number(c2),f2);
        QSlider*s=new QSlider(Qt::Horizontal);s->setRange(0,100);s->setValue(50);s->setFixedHeight(32);r->addWidget(s,1);
        pos=new QLabel("--");pos->setObjectName("val");r->addWidget(pos);QLabel*u=new QLabel(un);u->setStyleSheet("font-size:12px;color:#888;min-width:28px;");r->addWidget(u);
        b("+"+QString::number(c2),f4);b("+"+QString::number(c1),f3);return r;
    };
    rl->addLayout(R("M1",m_pb[0],"deg",5,1,[this](){m_w.cmdBase(arm_deg_to_pulse(-5),0,SP_M1);},[this](){m_w.cmdBase(arm_deg_to_pulse(-1),0,SP_M1);},[this](){m_w.cmdBase(arm_deg_to_pulse(5),0,SP_M1);},[this](){m_w.cmdBase(arm_deg_to_pulse(1),0,SP_M1);}));
    rl->addLayout(R("M2",m_pb[1],"mm",5,1,[this](){m_w.cmdArm(lift_mm_to_pulse(-5),0,SP_M2);},[this](){m_w.cmdArm(lift_mm_to_pulse(-1),0,SP_M2);},[this](){m_w.cmdArm(lift_mm_to_pulse(5),0,SP_M2);},[this](){m_w.cmdArm(lift_mm_to_pulse(1),0,SP_M2);}));
    rl->addLayout(R("M3",m_pb[2],"deg",5,1,[this](){m_w.cmdArm(0,arm_deg_to_pulse(-5),SP_M3);},[this](){m_w.cmdArm(0,arm_deg_to_pulse(-1),SP_M3);},[this](){m_w.cmdArm(0,arm_deg_to_pulse(5),SP_M3);},[this](){m_w.cmdArm(0,arm_deg_to_pulse(1),SP_M3);}));
    rl->addLayout(R("M4",m_pb[3],"cm",5,1,[this](){m_w.cmdBase(0,conveyor_mm_to_pulse(-5*10.f),SP_M4);},[this](){m_w.cmdBase(0,conveyor_mm_to_pulse(-1*10.f),SP_M4);},[this](){m_w.cmdBase(0,conveyor_mm_to_pulse(5*10.f),SP_M4);},[this](){m_w.cmdBase(0,conveyor_mm_to_pulse(1*10.f),SP_M4);}));
    rl->addLayout(R("SV1",m_s1b,"deg",10,2,[this](){m_sv1-=10;if(m_sv1<0)m_sv1=0;m_w.cmdServo(m_sv1,255);},[this](){m_sv1-=2;if(m_sv1<0)m_sv1=0;m_w.cmdServo(m_sv1,255);},[this](){m_sv1+=10;if(m_sv1>270)m_sv1=270;m_w.cmdServo(m_sv1,255);},[this](){m_sv1+=2;if(m_sv1>270)m_sv1=270;m_w.cmdServo(m_sv1,255);}));
    rl->addLayout(R("SV2",m_s2b,"deg",10,2,[this](){m_sv2-=10;if(m_sv2<0)m_sv2=0;m_w.cmdServo(255,m_sv2);},[this](){m_sv2-=2;if(m_sv2<0)m_sv2=0;m_w.cmdServo(255,m_sv2);},[this](){m_sv2+=10;if(m_sv2>180)m_sv2=180;m_w.cmdServo(255,m_sv2);},[this](){m_sv2+=2;if(m_sv2>180)m_sv2=180;m_w.cmdServo(255,m_sv2);}));
    QHBoxLayout*r2=new QHBoxLayout;for(int i=1;i<=4;i++){QPushButton*b=new QPushButton(QString("EM%1").arg(i));b->setObjectName("warn");connect(b,&QPushButton::clicked,this,[this,i](){em(i);});r2->addWidget(b);}
    r2->addStretch();QPushButton*bs=new QPushButton("急停");bs->setObjectName("danger");connect(bs,&QPushButton::clicked,this,&MainWindow::stop);r2->addWidget(bs);
    QPushButton*bb=new QPushButton("返回");bb->setObjectName("back");connect(bb,&QPushButton::clicked,this,[this](){toMode(SEL);});r2->addWidget(bb);rl->addLayout(r2);
    rl->addSpacing(20);hl->addWidget(lw);hl->addWidget(rw,1);ui->modeStack->addWidget(p);}

//=== 示教 (页1=控制, 页2=保存) ===
void MainWindow::b2(){QWidget*p=new QWidget;QHBoxLayout*hl=new QHBoxLayout(p);hl->setSpacing(2);hl->setContentsMargins(2,2,2,2);
    QWidget*lw=new QWidget;lw->setFixedWidth(140);QVBoxLayout*ll=new QVBoxLayout(lw);ll->setSpacing(1);
    QPushButton*bp=new QPushButton("回放");bp->setObjectName("primary");connect(bp,&QPushButton::clicked,this,&MainWindow::tplay);ll->addWidget(bp);
    QPushButton*bb=new QPushButton("返回");bb->setObjectName("back");connect(bb,&QPushButton::clicked,this,[this](){toMode(SEL);});ll->addWidget(bb);
    for(int i=0;i<6;i++){QHBoxLayout*r=new QHBoxLayout;r->addWidget(new QLabel(QString("M%1:").arg(i+1)));m_tcur[i]=new QLabel("--");m_tcur[i]->setObjectName("val");r->addWidget(m_tcur[i]);ll->addLayout(r);}ll->addStretch();

    QWidget*rw=new QWidget;QVBoxLayout*rl=new QVBoxLayout(rw);rl->setSpacing(2);rl->setContentsMargins(0,0,0,0);
    QHBoxLayout*hp=new QHBoxLayout;QPushButton*p1=new QPushButton("控制");p1->setObjectName("primary");QPushButton*p2=new QPushButton("保存");
    QStackedWidget*tsw=new QStackedWidget;QWidget*tp1=new QWidget;QWidget*tp2=new QWidget;
    connect(p1,&QPushButton::clicked,this,[tsw](){tsw->setCurrentIndex(0);});connect(p2,&QPushButton::clicked,this,[tsw](){tsw->setCurrentIndex(1);});
    hp->addWidget(p1);hp->addWidget(p2);hp->addStretch();rl->addLayout(hp);
    // 页1: 传送带+马达行
    QVBoxLayout*tl1=new QVBoxLayout(tp1);tl1->setSpacing(2);tl1->setContentsMargins(0,0,0,0);
    QGroupBox*gs=new QGroupBox("传送带段距(cm)");QHBoxLayout*gsl=new QHBoxLayout(gs);gsl->setSpacing(1);gsl->setContentsMargins(2,2,2,2);
    auto mkSeg=[&](int i,const QString&nm){
        QHBoxLayout*r=new QHBoxLayout;r->setSpacing(0);
        QLabel*l=new QLabel(nm);l->setStyleSheet("font-size:10px;");r->addWidget(l);
        QLabel*lb=new QLabel(QString::number(m_seg[i]));lb->setObjectName("val");lb->setStyleSheet("font-size:12px;min-width:24px;");lb->setAlignment(Qt::AlignCenter);r->addWidget(lb);
        QPushButton*b1=new QPushButton("-");b1->setFixedSize(24,20);b1->setStyleSheet("font-size:12px;padding:0;");
        QPushButton*b2=new QPushButton("+");b2->setFixedSize(24,20);b2->setStyleSheet("font-size:12px;padding:0;");
        connect(b1,&QPushButton::clicked,this,[this,i,lb](){float&v=m_seg[i];v-=1;if(v<1)v=1;lb->setText(QString::number(v));});
        connect(b2,&QPushButton::clicked,this,[this,i,lb](){float&v=m_seg[i];v+=1;if(v>50)v=50;lb->setText(QString::number(v));});r->addWidget(b1);r->addWidget(b2);return r;};
    const char*segNames[]={"段1","段2","段3","段4","Cam"};
    for(int i=0;i<5;i++)gsl->addLayout(mkSeg(i,segNames[i]));
    tl1->addWidget(gs);
    auto R=[this](const char*nm,QLabel*&pos,const char*un,int c1,int c2,auto f1,auto f2,auto f3,auto f4){
        QHBoxLayout*r=new QHBoxLayout;r->addWidget(new QLabel(nm));
        auto b=[&](QString t,auto cb){QPushButton*p=new QPushButton(t);p->setFixedWidth(42);connect(p,&QPushButton::clicked,this,cb);r->addWidget(p);};
        b("-"+QString::number(c1),f1);b("-"+QString::number(c2),f2);
        QSlider*s=new QSlider(Qt::Horizontal);s->setRange(0,100);s->setValue(50);s->setFixedHeight(32);r->addWidget(s,1);
        pos=new QLabel("--");pos->setObjectName("val");r->addWidget(pos);QLabel*u=new QLabel(un);u->setStyleSheet("font-size:12px;color:#888;min-width:28px;");r->addWidget(u);
        b("+"+QString::number(c2),f4);b("+"+QString::number(c1),f3);return r;
    };
    QLabel *tpb[6];
    tl1->addLayout(R("M1",tpb[0],"deg",5,1,[this](){m_w.cmdBase(arm_deg_to_pulse(-5),0,SP_M1);},[this](){m_w.cmdBase(arm_deg_to_pulse(-1),0,SP_M1);},[this](){m_w.cmdBase(arm_deg_to_pulse(5),0,SP_M1);},[this](){m_w.cmdBase(arm_deg_to_pulse(1),0,SP_M1);}));
    tl1->addLayout(R("M2",tpb[1],"mm",5,1,[this](){m_w.cmdArm(lift_mm_to_pulse(-5),0,SP_M2);},[this](){m_w.cmdArm(lift_mm_to_pulse(-1),0,SP_M2);},[this](){m_w.cmdArm(lift_mm_to_pulse(5),0,SP_M2);},[this](){m_w.cmdArm(lift_mm_to_pulse(1),0,SP_M2);}));
    tl1->addLayout(R("M3",tpb[2],"deg",5,1,[this](){m_w.cmdArm(0,arm_deg_to_pulse(-5),SP_M3);},[this](){m_w.cmdArm(0,arm_deg_to_pulse(-1),SP_M3);},[this](){m_w.cmdArm(0,arm_deg_to_pulse(5),SP_M3);},[this](){m_w.cmdArm(0,arm_deg_to_pulse(1),SP_M3);}));
    tl1->addLayout(R("M4",tpb[3],"cm",5,1,[this](){m_w.cmdBase(0,conveyor_mm_to_pulse(-5*10.f),SP_M4);},[this](){m_w.cmdBase(0,conveyor_mm_to_pulse(-1*10.f),SP_M4);},[this](){m_w.cmdBase(0,conveyor_mm_to_pulse(5*10.f),SP_M4);},[this](){m_w.cmdBase(0,conveyor_mm_to_pulse(1*10.f),SP_M4);}));
    tl1->addLayout(R("SV1",tpb[4],"deg",10,2,[this](){m_sv1-=10;if(m_sv1<0)m_sv1=0;m_w.cmdServo(m_sv1,255);},[this](){m_sv1-=2;if(m_sv1<0)m_sv1=0;m_w.cmdServo(m_sv1,255);},[this](){m_sv1+=10;if(m_sv1>270)m_sv1=270;m_w.cmdServo(m_sv1,255);},[this](){m_sv1+=2;if(m_sv1>270)m_sv1=270;m_w.cmdServo(m_sv1,255);}));
    tl1->addLayout(R("SV2",tpb[5],"deg",10,2,[this](){m_sv2-=10;if(m_sv2<0)m_sv2=0;m_w.cmdServo(255,m_sv2);},[this](){m_sv2-=2;if(m_sv2<0)m_sv2=0;m_w.cmdServo(255,m_sv2);},[this](){m_sv2+=10;if(m_sv2>180)m_sv2=180;m_w.cmdServo(255,m_sv2);},[this](){m_sv2+=2;if(m_sv2>180)m_sv2=180;m_w.cmdServo(255,m_sv2);}));tl1->addStretch();tl1->addSpacing(15);tsw->addWidget(tp1);
    // 页2: 路径点+保存
    QVBoxLayout*tl2=new QVBoxLayout(tp2);tl2->setSpacing(2);tl2->setContentsMargins(0,0,0,0);
    QGroupBox*gw=new QGroupBox("路径点");QGridLayout*gwl=new QGridLayout(gw);gwl->setSpacing(1);gwl->setContentsMargins(2,2,2,2);
    const char*hd[]={"步骤","M1°","M3°","M4mm","SV1°","","M2mm",""};
    for(int i=0;i<8;i++){QLabel*h=new QLabel(hd[i]);h->setStyleSheet("font-size:9px;color:#888;");gwl->addWidget(h,0,i);}
    const char*sn[]={"①初始","②a物料","②b物料高","④a升离","④b传送带","⑤a放置高"};
    for(int s=0;s<6;s++){QPushButton*sel=new QPushButton(sn[s]);sel->setStyleSheet("font-size:8px;padding:1px;min-height:20px;");connect(sel,&QPushButton::clicked,this,[this,s](){ts(s);});gwl->addWidget(sel,s+1,0);
    for(int j=0;j<4;j++){m_wl[s][j]=new QLabel("--");m_wl[s][j]->setStyleSheet("font-size:9px;padding:1px;border:1px solid #ddd;min-width:48px;");gwl->addWidget(m_wl[s][j],s+1,j+1);}
    gwl->addWidget(new QLabel(""),s+1,5);m_wl[s][4]=new QLabel("--");m_wl[s][4]->setStyleSheet("font-size:9px;padding:1px;border:1px solid #ddd;min-width:52px;");gwl->addWidget(m_wl[s][4],s+1,6);
    m_wl[s][5]=new QLabel("-");m_wl[s][5]->setStyleSheet("font-size:8px;color:#888;");gwl->addWidget(m_wl[s][5],s+1,7);}tl2->addWidget(gw);
    QHBoxLayout*ha=new QHBoxLayout;QPushButton*bv=new QPushButton("保存当前步");bv->setObjectName("success");connect(bv,&QPushButton::clicked,this,&MainWindow::tv);ha->addWidget(bv);
    QPushButton*bt=new QPushButton("提交全部");bt->setObjectName("warn");connect(bt,&QPushButton::clicked,this,&MainWindow::tsub);ha->addWidget(bt);tl2->addLayout(ha);
    // 夹爪参数
    QGroupBox*gg=new QGroupBox("夹爪舵机 SV2");
    QVBoxLayout*ggl=new QVBoxLayout(gg);ggl->setSpacing(2);ggl->setContentsMargins(4,14,4,4);
    auto mkSv=[&](const char*nm,int&val,QLabel*&lb,int mx){
        QHBoxLayout*r=new QHBoxLayout;r->addWidget(new QLabel(nm));
        QSlider*s=new QSlider(Qt::Horizontal);s->setRange(0,mx);s->setValue(val);s->setFixedHeight(22);
        lb=new QLabel(QString::number(val));lb->setObjectName("val");lb->setFixedWidth(36);lb->setAlignment(Qt::AlignCenter);
        connect(s,&QSlider::valueChanged,gg,[&val,lb](int v){val=v;lb->setText(QString::number(v));});
        r->addWidget(s,1);r->addWidget(lb);return r;};
    QLabel *svLb1,*svLb2;
    ggl->addLayout(mkSv("闭合角",m_sv2c,svLb1,180));
    ggl->addLayout(mkSv("张开角",m_sv2o,svLb2,180));
    tl2->addWidget(gg);
    tl2->addStretch();tl2->addSpacing(15);tsw->addWidget(tp2);
    rl->addWidget(tsw);hl->addWidget(lw);hl->addWidget(rw,1);ui->modeStack->addWidget(p);tload();
    connect(m_t,&QTimer::timeout,this,[this](){if(m_m!=TEACH)return;RobotState s=m_w.poll();
        m_tcur[0]->setText(QString::number(s.m1_deg,'f',1));m_tcur[1]->setText(QString::number(s.m2_mm,'f',2));m_tcur[2]->setText(QString::number(s.m3_deg,'f',1));m_tcur[3]->setText(QString::number(s.m4_mm,'f',1));m_tcur[4]->setText(QString::number(m_sv1));m_tcur[5]->setText(QString::number(m_sv2));});}

//=== 视觉 (融合 ColorDetect + 可调HSV阈值) ===
// 红色第一范围 H:0~10°, 第二范围 H:170~180° — 因HSV色相环红色在两端
void MainWindow::b3(){QWidget*p=new QWidget;QHBoxLayout*hl=new QHBoxLayout(p);hl->setSpacing(4);hl->setContentsMargins(2,2,2,2);
    // 左侧: 实时预览
    QFrame*cf=new QFrame;cf->setObjectName("card");QVBoxLayout*cll=new QVBoxLayout(cf);cll->setContentsMargins(2,2,2,2);
    // 顶部状态栏: 摄像头 + 保存 + 返回
    QHBoxLayout*topBar=new QHBoxLayout;topBar->setSpacing(4);
    m_camSt=new QLabel("摄像头: --");topBar->addWidget(m_camSt);
    topBar->addStretch();
    QPushButton*bsv=new QPushButton("保存阈值");bsv->setObjectName("success");bsv->setMinimumHeight(28);bsv->setFixedWidth(80);
    connect(bsv,&QPushButton::clicked,this,[this](){QString thPath=QCoreApplication::applicationDirPath()+"/hsv_thresh.txt";camera_save_thresh(thPath.toUtf8().constData());});topBar->addWidget(bsv);
    QPushButton*bb=new QPushButton("返回");bb->setObjectName("back");bb->setMinimumHeight(28);bb->setFixedWidth(60);
    connect(bb,&QPushButton::clicked,this,[this](){toMode(SEL);});topBar->addWidget(bb);
    cll->addLayout(topBar);
    m_cl=new QLabel;m_cl->setAlignment(Qt::AlignCenter);m_cl->setMinimumSize(300,220);m_cl->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);m_cl->setStyleSheet("border:2px solid #30363d;border-radius:4px;background:#1a1a2e;");m_cl->setScaledContents(true);cll->addWidget(m_cl,1);hl->addWidget(cf,3);
    // 右侧: 信息+阈值
    QWidget*rw=new QWidget;QVBoxLayout*rll=new QVBoxLayout(rw);rll->setSpacing(2);rll->setContentsMargins(0,0,0,0);
    m_cn=new QLabel("等待检测...");m_cn->setStyleSheet("font-size:18px;font-weight:bold;color:#1a73e8;padding:4px;");m_cn->setAlignment(Qt::AlignCenter);rll->addWidget(m_cn);
    auto mkInfo=[&](const char*lb,QLabel*&val,const char*un){
        QHBoxLayout*r=new QHBoxLayout;r->setSpacing(2);r->addWidget(new QLabel(lb));val=new QLabel("-");val->setObjectName("val");val->setAlignment(Qt::AlignCenter);r->addWidget(val,1);if(un)r->addWidget(new QLabel(un));return r;};
    QGroupBox*gi=new QGroupBox("物块信息");QVBoxLayout*gil=new QVBoxLayout(gi);gil->setSpacing(1);gil->setContentsMargins(4,14,4,4);
    gil->addLayout(mkInfo("X:",m_vx,"px"));gil->addLayout(mkInfo("Y:",m_vy,"px"));gil->addLayout(mkInfo("面积:",m_va,""));gil->addLayout(mkInfo("尺寸:",m_vs,""));rll->addWidget(gi);
    // HSV 阈值调节 — 4 标签页
    QTabWidget*tw=new QTabWidget;tw->setStyleSheet("QTabWidget{font-size:10px;}QTabBar::tab{min-width:40px;padding:2px 4px;}");
    const char*tn[]={"红色","黄色","绿色","蓝色"};
    for(int ci=0;ci<4;ci++){
        QWidget*tab=new QWidget;QVBoxLayout*tl=new QVBoxLayout(tab);tl->setSpacing(0);tl->setContentsMargins(2,2,2,2);
        const char*pn[]={"H_min","H_max","S_min","S_max","V_min","V_max"};
        QSlider*sl[6];QLabel*vl[6];
        for(int j=0;j<6;j++){
            QHBoxLayout*r=new QHBoxLayout;r->setSpacing(2);r->addWidget(new QLabel(pn[j]));
            sl[j]=new QSlider(Qt::Horizontal);sl[j]->setRange(0,255);sl[j]->setFixedHeight(18);
            vl[j]=new QLabel("0");vl[j]->setFixedWidth(30);vl[j]->setAlignment(Qt::AlignRight|Qt::AlignVCenter);
            r->addWidget(sl[j],1);r->addWidget(vl[j]);
            QObject::connect(sl[j],&QSlider::valueChanged,tab,[vl,j](int v){vl[j]->setText(QString::number(v));});
            tl->addLayout(r);
        }
        // 红色额外第二范围
        if(ci==0){
            QLabel*r2l=new QLabel("红色第二范围 (H 170-180):");r2l->setStyleSheet("font-weight:bold;color:#d93025;margin-top:4px;");tl->addWidget(r2l);
            QSlider*sl2[6];QLabel*vl2[6];
            for(int j=0;j<6;j++){
                QHBoxLayout*r=new QHBoxLayout;r->setSpacing(2);r->addWidget(new QLabel(QString("R2_%1").arg(pn[j])));
                sl2[j]=new QSlider(Qt::Horizontal);sl2[j]->setRange(0,255);sl2[j]->setFixedHeight(18);
                vl2[j]=new QLabel("0");vl2[j]->setFixedWidth(30);vl2[j]->setAlignment(Qt::AlignRight|Qt::AlignVCenter);
                r->addWidget(sl2[j],1);r->addWidget(vl2[j]);
                QObject::connect(sl2[j],&QSlider::valueChanged,tab,[vl2,j](int v){vl2[j]->setText(QString::number(v));});
                tl->addLayout(r);
            }
            // 加载当前红色参数到滑块, valueChanged→立即写camera
            auto loadRed=[ci,sl,sl2](){
                int h1,h2,s1,s2,v1,v2;
                camera_get_hsv_thresh(ci,&h1,&h2,&s1,&s2,&v1,&v2);
                int vals[]={h1,h2,s1,s2,v1,v2};
                for(int j=0;j<6;j++){sl[j]->blockSignals(true);sl[j]->setValue(vals[j]);sl[j]->blockSignals(false);}
                camera_get_red2_thresh(&h1,&h2,&s1,&s2,&v1,&v2);
                int vals2[]={h1,h2,s1,s2,v1,v2};
                for(int j=0;j<6;j++){sl2[j]->blockSignals(true);sl2[j]->setValue(vals2[j]);sl2[j]->blockSignals(false);}
            };
            loadRed();
            for(int j=0;j<6;j++){
                QObject::connect(sl[j],&QSlider::valueChanged,tab,[sl,sl2](){
                    camera_set_hsv_thresh(0,sl[0]->value(),sl[1]->value(),sl[2]->value(),sl[3]->value(),sl[4]->value(),sl[5]->value());
                    camera_set_red2_thresh(sl2[0]->value(),sl2[1]->value(),sl2[2]->value(),sl2[3]->value(),sl2[4]->value(),sl2[5]->value());
                });
            }
        } else {
            auto load=[ci,sl](){
                int h1,h2,s1,s2,v1,v2;
                camera_get_hsv_thresh(ci,&h1,&h2,&s1,&s2,&v1,&v2);
                int vals[]={h1,h2,s1,s2,v1,v2};
                for(int j=0;j<6;j++){sl[j]->blockSignals(true);sl[j]->setValue(vals[j]);sl[j]->blockSignals(false);}
            };
            load();
            for(int j=0;j<6;j++){
                QObject::connect(sl[j],&QSlider::valueChanged,tab,[sl,ci](){
                    camera_set_hsv_thresh(ci,sl[0]->value(),sl[1]->value(),sl[2]->value(),sl[3]->value(),sl[4]->value(),sl[5]->value());
                });
            }
        }
        tw->addTab(tab,tn[ci]);
    }
    rll->addWidget(tw);
    hl->addWidget(rw,2);ui->modeStack->addWidget(p);
    // 启动时加载已保存阈值
    {QString p=QCoreApplication::applicationDirPath()+"/hsv_thresh.txt";camera_load_thresh(p.toUtf8().constData());}
}

// visDetect 保留但简化 (实时预览已覆盖)
void MainWindow::visDetect(){
    if(m_camfd>0){
        const unsigned char *rgb; int pw,ph; double cx,cy; int area,bw,bh;
        m_color=camera_get_preview(m_camfd,&rgb,&pw,&ph,&cx,&cy,&area,&bw,&bh);
        const char*cn[]={"红","黄","绿","蓝"};
        m_cn->setText(QString("● %1").arg(m_color>=0?cn[m_color]:"?"));
        if(m_color>=0 && rgb){
            QImage img(rgb,pw,ph,QImage::Format_RGB888);
            m_cl->setPixmap(QPixmap::fromImage(img).scaled(m_cl->size(),Qt::KeepAspectRatio,Qt::SmoothTransformation));
            m_vx->setText(QString::number(cx,'f',1));m_vy->setText(QString::number(cy,'f',1));
            m_va->setText(QString::number(area)+"px²");m_vs->setText(QString("%1×%2").arg(bw).arg(bh));
        }
    }
}

//=== 安全分拣 (模式4) ===
void MainWindow::b4(){QWidget*p=new QWidget;QHBoxLayout*hl=new QHBoxLayout(p);hl->setSpacing(4);hl->setContentsMargins(2,2,2,2);
    // 左侧: 实时预览
    QFrame*cf=new QFrame;cf->setObjectName("card");QVBoxLayout*cll=new QVBoxLayout(cf);cll->setContentsMargins(2,2,2,2);
    QHBoxLayout*tb=new QHBoxLayout;tb->setSpacing(4);
    QLabel*csl=new QLabel("摄像头: --");csl->setObjectName("sortCamSt");tb->addWidget(csl);
    // 启动时更新摄像头状态
    QTimer::singleShot(0,this,[csl,this](){csl->setText(m_camfd>0?"摄像头: 已连接":"摄像头: 未连接");});
    tb->addStretch();
    QPushButton*bs=new QPushButton("返回");bs->setObjectName("back");bs->setFixedWidth(60);bs->setMinimumHeight(28);
    connect(bs,&QPushButton::clicked,this,[this](){srStop();toMode(SEL);});tb->addWidget(bs);
    cll->addLayout(tb);
    QLabel*pv=new QLabel;pv->setObjectName("sortPreview");pv->setAlignment(Qt::AlignCenter);pv->setMinimumSize(300,220);pv->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);pv->setStyleSheet("border:2px solid #30363d;border-radius:4px;background:#1a1a2e;");pv->setScaledContents(true);cll->addWidget(pv,1);
    hl->addWidget(cf,3);
    // 右侧: 控制面板
    QWidget*rw=new QWidget;QVBoxLayout*rl=new QVBoxLayout(rw);rl->setSpacing(4);rl->setContentsMargins(0,0,0,0);
    // 状态
    m_sfCo=new QLabel("● 待机");m_sfCo->setStyleSheet("font-size:22px;font-weight:bold;color:#1a73e8;padding:4px;");m_sfCo->setAlignment(Qt::AlignCenter);rl->addWidget(m_sfCo);
    // 安全指示灯 (RichText 大号圆圈)
    auto si=[&](const char*n,QLabel*&lb){
        QHBoxLayout*r=new QHBoxLayout;r->setSpacing(6);
        QLabel*ln=new QLabel(n);ln->setStyleSheet("font-size:14px;font-weight:bold;");r->addWidget(ln);
        lb=new QLabel;lb->setTextFormat(Qt::RichText);lb->setText("<span style='color:#4caf50;font-size:18px;'>●</span>");r->addWidget(lb);r->addStretch();return r;};
    QGroupBox*gs=new QGroupBox("安全监控");QVBoxLayout*gsl=new QVBoxLayout(gs);gsl->setSpacing(3);gsl->setContentsMargins(6,18,6,6);
    gsl->addLayout(si("电机在线:",m_sfSt));gsl->addLayout(si("超时:",m_sfTo));gsl->addLayout(si("电流:",m_sfCu));gsl->addLayout(si("EM互斥:",m_sfEm));rl->addWidget(gs);
    // 分拣统计
    QGroupBox*gc=new QGroupBox("分拣计数");QHBoxLayout*gcl=new QHBoxLayout(gc);gcl->setSpacing(2);gcl->setContentsMargins(4,14,4,4);
    const char*cn[]={"红","黄","绿","蓝"};const char*cc[]={"#003cff","#00dcf0","#28dc28","#2850c8"};
    for(int i=0;i<4;i++){QLabel*l=new QLabel(QString(cn[i]));l->setStyleSheet(QString("color:%1;font-weight:bold;").arg(cc[i]));gcl->addWidget(l);m_scLb[i]=new QLabel("0");m_scLb[i]->setObjectName("val");gcl->addWidget(m_scLb[i]);}rl->addWidget(gc);
    // 控制按钮
    QHBoxLayout*hb=new QHBoxLayout;hb->setSpacing(4);
    QPushButton*bs1=new QPushButton("▶ 启动分拣");bs1->setObjectName("success");bs1->setMinimumHeight(44);
    connect(bs1,&QPushButton::clicked,this,[this](){srStart();});hb->addWidget(bs1,1);
    QPushButton*bs2=new QPushButton("⏹ 停止");bs2->setObjectName("danger");bs2->setMinimumHeight(44);
    connect(bs2,&QPushButton::clicked,this,[this](){srStop();});hb->addWidget(bs2);
    rl->addLayout(hb);rl->addStretch();hl->addWidget(rw,2);ui->modeStack->addWidget(p);
}

// 安全分拣 — 启动
void MainWindow::srStart(){
    if(m_camfd<=0){m_sfCo->setText("摄像头未连接!");return;}
    m_sr=true;
    // 先发回原点指令 (M1+M2+M3), 然后进入逐步回归
    if(m_h[0]){WP&wp=m_wp[0];RobotState c=m_w.poll();int16_t a=arm_deg_to_pulse(wp.a-c.m1_deg),b=lift_mm_to_pulse(wp.b-c.m2_mm),d=arm_deg_to_pulse(wp.c-c.m3_deg);m_w.cmdBoth(b,d,a,0,SP_M1);}
    m_ss=SS_HOME_M2; m_sst=QDateTime::currentMSecsSinceEpoch();
    m_sfCo->setText("● 回原点...");
    m_sfCo->setStyleSheet("font-size:22px;font-weight:bold;color:#1a73e8;padding:4px;");
}

// 安全分拣 — 停止
void MainWindow::srStop(){
    m_sr=false; m_ss=SS_IDLE; m_w.cmdEstop();
    m_sfCo->setText("● 待机");
    m_sfCo->setStyleSheet("font-size:22px;font-weight:bold;color:#1a73e8;padding:4px;");
}

// 安全分拣 — 每步 Tick (由 onPoll 调用)
void MainWindow::srTick(){
    RobotState s=m_w.poll();
    qint64 el=m_sr?QDateTime::currentMSecsSinceEpoch()-m_sst:0;
    bool mOk=s.m1_ok&&s.m2_ok&&s.m3_ok&&s.m4_ok;
    bool curOk=s.cur_m1_ma<300&&s.cur_m2_ma<300&&s.cur_m3_ma<300&&s.cur_m4_ma<300;
    auto okBad=[&](bool ok){return ok?QString("<span style='color:#4caf50;font-size:18px;'>●</span>"):QString("<span style='color:#f85149;font-size:18px;'>✗</span>");};

    // 更新安全指示灯 (始终更新, 不管是否运行)
    m_sfSt->setText(okBad(mOk));
    m_sfTo->setText(okBad(m_sr?el<8000:true));
    m_sfCu->setText(okBad(curOk));
    m_sfEm->setText(QString("<span style='color:#4caf50;font-size:18px;'>●</span>"));

    // 更新摄像头状态 + 预览 (始终运行)
    QLabel*st=findChild<QLabel*>("sortCamSt");
    if(st) st->setText(m_camfd>0?"摄像头: 已连接":"摄像头: 未连接");
    QLabel*pv=findChild<QLabel*>("sortPreview");
    if(pv){
        static int fc=0;
        if(++fc%2==0&&m_camfd>0){
            const unsigned char*rgb;int pw,ph;double cx,cy;int area,bw,bh;
            m_color=camera_get_preview(m_camfd,&rgb,&pw,&ph,&cx,&cy,&area,&bw,&bh);
            if(rgb){QImage img(rgb,pw,ph,QImage::Format_RGB888);pv->setPixmap(QPixmap::fromImage(img).scaled(pv->size(),Qt::KeepAspectRatio,Qt::FastTransformation));}
        }
    }

    if(!m_sr) return;
    auto next=[&](SortSt ns,qint64 to){m_ss=ns;m_sst=QDateTime::currentMSecsSinceEpoch();(void)to;};
    auto mvM1=[&](int wpi){WP&wp=m_wp[wpi];RobotState c=m_w.poll();int16_t aa=arm_deg_to_pulse(wp.a-c.m1_deg);m_w.cmdBase(aa,0,SP_M1);};
    auto mvM3=[&](int wpi){WP&wp=m_wp[wpi];RobotState c=m_w.poll();int16_t cc=arm_deg_to_pulse(wp.c-c.m3_deg);m_w.cmdArm(0,cc,SP_M3);};
    auto mvSV=[&](int wpi){WP&wp=m_wp[wpi];m_w.cmdServo(wp.e,255);};
    auto mvM2=[&](int wpi){WP&wp=m_wp[wpi];RobotState c=m_w.poll();int16_t p=lift_mm_to_pulse(wp.b-c.m2_mm);m_w.cmdArm(p,0,SP_M2);};
    auto mvM4f=[&](float mm){int32_t p=conveyor_mm_to_pulse(mm);if(p>16000){m_w.cmdBase(0,16000,SP_M4);int32_t rest=p-16000;QTimer::singleShot(500,this,[this,rest](){m_w.cmdBase(0,(int16_t)rest,SP_M4);});}else m_w.cmdBase(0,(int16_t)p,SP_M4);};
    const char*cn[]={"红","黄","绿","蓝"};

    switch(m_ss){
    //=== 回原点: 先升降! M2→2s→M1→2s→M3→2s→SV1 → 取料位 ===
    case SS_HOME_M2: if(el>=5000){m_sfCo->setText("● 回原点 M2↑");mvM2(0);next(SS_HOME_M1,2000);} break;
    case SS_HOME_M1: if(el>=2000){m_sfCo->setText("● 回原点 M1");mvM1(0);next(SS_HOME_M3,2000);} break;
    case SS_HOME_M3: if(el>=2000){m_sfCo->setText("● 回原点 M3");mvM3(0);next(SS_HOME_SV,2000);} break;
    case SS_HOME_SV: if(el>=2000){m_sfCo->setText("● 回原点 SV1");mvSV(0);next(SS_DET_M1,2000);} break;
    //=== Step0: 移到取料位 WP1 — M1→2s→M3→2s→SV1 ===
    case SS_DET_M1: if(el>=2000){m_sfCo->setText("● Step0 取料 M1");mvM1(1);next(SS_DET_M3,2000);} break;
    case SS_DET_M3: if(el>=2000){m_sfCo->setText("● Step0 取料 M3");mvM3(1);next(SS_DET_SV,2000);} break;
    case SS_DET_SV: if(el>=2000){m_sfCo->setText("● Step0 取料 SV1");mvSV(1);next(SS_PICK,2000);} break;
    //=== Step1: Z轴下降, Step2: 夹爪, Step3: Z轴升高 ===
    case SS_PICK: if(el>=2000){m_sfCo->setText("● Step1 Z下降");mvM2(3);next(SS_LIFT,4000);} break;
    case SS_LIFT: if(el>=4000){m_sfCo->setText("● Step2 夹爪闭合");m_w.cmdServo(255,m_sv2c);next(SS_PLACE,6000);} break;
    case SS_PLACE: if(el>=6000){m_sfCo->setText("● Step3 Z升高");mvM2(2);next(SS_EJT_M1,2000);} break;
    //=== Step4: 移到传送带 WP4 — M1→2s→M3→2s→SV1 ===
    case SS_EJT_M1: if(el>=2000){m_sfCo->setText("● Step4 传送带 M1");mvM1(4);next(SS_EJT_M3,2000);} break;
    case SS_EJT_M3: if(el>=2000){m_sfCo->setText("● Step4 传送带 M3");mvM3(4);next(SS_EJT_SV,2000);} break;
    case SS_EJT_SV: if(el>=2000){m_sfCo->setText("● Step4 传送带 SV1");mvSV(4);next(SS_RET,2000);} break;
    //=== Step5: Z轴放置, Step6: 开爪 ===
    case SS_RET: if(el>=2000){m_sfCo->setText("● Step5 Z放置");mvM2(5);next(SS_COOL,4000);} break;
    case SS_COOL: if(el>=4000){m_sfCo->setText("● Step6 开爪");m_w.cmdServo(255,m_sv2o);next(SS_CAMADV,6000);} break;
    //=== Step7: CamDist(相对!) → 视觉检测 ===
    case SS_CAMADV: if(el>=6000){m_sfCo->setText("● Step7 推进CamDist");mvM4f(m_seg[4]*10.f);next(SS_CAMDET,1000);} break;
    case SS_CAMDET:
        m_sfCo->setText("● 视觉检测中...");
        if(m_color>=0&&m_color<4){
            m_detColor=m_color;  // 锁定检测结果, 防止后续被预览覆盖
            m_sc[m_color]++; for(int i=0;i<4;i++) m_scLb[i]->setText(QString::number(m_sc[i]));
            m_sfCo->setText(QString("● %1 → %2段").arg(cn[m_color]).arg((int[]){2,3,4,1}[m_color]));
            next(SS_PLA_M1,0);
        }else if(el>=5000){m_sfCo->setText("● 未识别,重试");next(SS_CAMADV,0);}
        break;
    //=== Step8: 回WP0 + 段位推进 ===
    case SS_PLA_M1: if(el>=1000){m_sfCo->setText("● M1回原点");mvM1(0);next(SS_PLA_SV,2000);} break;
    case SS_PLA_SV: if(el>=2000){
        int sc[]={2,3,4,1};int sg=0;for(int i=0;i<sc[m_detColor];i++)sg+=m_seg[i];
        m_sfCo->setText(QString("● M4推进%1段").arg(sc[m_detColor]));
        mvM4f(sg*10.f);  // 相对距离!
        mvSV(0);
        next(SS_EMTRIG,5000);
    } break;
    //=== Step9: 推杆 + 回归 ===
    case SS_EMTRIG: if(el>=5000){
        if(m_detColor>=0&&m_detColor<4){int emCh[]={2,3,4,1};em(emCh[m_detColor]);}
        m_sfCo->setText("● 推杆分拣");
        next(SS_RT2_M1,2000);
    } break;
    case SS_RT2_M1: if(el>=2000){m_sfCo->setText("● 回归 M2↑先");mvM2(0);next(SS_RT2_M2,2000);} break;
    case SS_RT2_M2: if(el>=2000){m_sfCo->setText("● 回归 M1");mvM1(0);next(SS_RT2_M3,2000);} break;
    case SS_RT2_M3: if(el>=2000){m_sfCo->setText("● 回归 M3");mvM3(0);next(SS_RT2_WAIT,2000);} break;
    case SS_RT2_WAIT: if(el>=1000){next(SS_DET_M1,0);} break;  // 循环
    default: break;
    }
}

//=== 示教操作 ===
void MainWindow::ts(int s){m_ts=s;for(int i=0;i<6;i++)for(int j=0;j<6;j++)if(m_wl[i][j])m_wl[i][j]->setStyleSheet(i==s?"font-size:10px;padding:2px 4px;border:2px solid #1565c0;background:#e3f2fd;min-width:45px;":"font-size:10px;padding:2px 4px;border:1px solid #ddd;min-width:45px;");}
void MainWindow::tv(){RobotState s=m_w.poll();int st=m_ts;m_wp[st]={s.m1_deg,s.m3_deg,s.m4_mm,(quint8)m_sv1,s.m2_mm};m_h[st]=true;
    m_wl[st][0]->setText(QString::number(s.m1_deg,'f',1));m_wl[st][1]->setText(QString::number(s.m3_deg,'f',1));m_wl[st][2]->setText(QString::number(s.m4_mm,'f',1));m_wl[st][3]->setText(QString::number(m_sv1));m_wl[st][4]->setText(QString::number(s.m2_mm,'f',2));m_wl[st][5]->setText("OK");m_wl[st][5]->setStyleSheet("font-size:10px;color:#4caf50;font-weight:bold;");}
void MainWindow::tsub(){QString wpPath=QCoreApplication::applicationDirPath()+"/waypoints.bin";QFile f(wpPath);if(!f.open(QIODevice::WriteOnly)){QMessageBox::warning(this,"Error","Cannot write");return;}QDataStream ds(&f);for(int i=0;i<5;i++)ds<<m_seg[i];ds<<(qint32)m_sv2c<<(qint32)m_sv2o;for(int s=0;s<6;s++){ds<<m_wp[s].a<<m_wp[s].c<<m_wp[s].d<<m_wp[s].e<<m_wp[s].b<<m_h[s];}f.close();QMessageBox::information(this,"OK","Saved");}
void MainWindow::tload(){QString wpPath=QCoreApplication::applicationDirPath()+"/waypoints.bin";QFile f(wpPath);if(!f.open(QIODevice::ReadOnly))return;QDataStream ds(&f);for(int i=0;i<5;i++)ds>>m_seg[i];qint32 vc,vo;ds>>vc>>vo;m_sv2c=(int)vc;m_sv2o=(int)vo;for(int s=0;s<6;s++){bool v;ds>>m_wp[s].a>>m_wp[s].c>>m_wp[s].d>>m_wp[s].e>>m_wp[s].b>>v;m_h[s]=v;if(v){m_wl[s][0]->setText(QString::number(m_wp[s].a,'f',1));m_wl[s][1]->setText(QString::number(m_wp[s].c,'f',1));m_wl[s][2]->setText(QString::number(m_wp[s].d,'f',1));m_wl[s][3]->setText(QString::number(m_wp[s].e));m_wl[s][4]->setText(QString::number(m_wp[s].b,'f',2));m_wl[s][5]->setText("OK");m_wl[s][5]->setStyleSheet("font-size:10px;color:#4caf50;font-weight:bold;");}}f.close();}
void MainWindow::tplay(){if(m_pl){QMessageBox::information(this,"Busy","Running");return;}for(int s=0;s<6;s++)if(!m_h[s]){QMessageBox::warning(this,"Error",QString("Step %1 missing").arg(s));return;}m_pl=true;m_ps=0;m_pst=QDateTime::currentMSecsSinceEpoch();WP&wp=m_wp[0];RobotState c=m_w.poll();int16_t a=arm_deg_to_pulse(wp.a-c.m1_deg),b=lift_mm_to_pulse(wp.b-c.m2_mm),d=arm_deg_to_pulse(wp.c-c.m3_deg),e=conveyor_mm_to_pulse(wp.d-c.m4_mm);m_w.cmdBoth(b,d,a,e,SP_M1);m_w.cmdServo(wp.e,255);}

//=== 遥控操作 ===
void MainWindow::j1c(){if(!cj())return;m_w.cmdBase(arm_deg_to_pulse(5),0,SP_M1);}
void MainWindow::j1a(){if(!cj())return;m_w.cmdBase(arm_deg_to_pulse(-5),0,SP_M1);}
void MainWindow::j2u(){if(!cj())return;m_w.cmdArm(lift_mm_to_pulse(5),0,SP_M2);}
void MainWindow::j2d(){if(!cj())return;m_w.cmdArm(lift_mm_to_pulse(-5),0,SP_M2);}
void MainWindow::j3c(){if(!cj())return;m_w.cmdArm(0,arm_deg_to_pulse(5),SP_M3);}
void MainWindow::j3a(){if(!cj())return;m_w.cmdArm(0,arm_deg_to_pulse(-5),SP_M3);}
void MainWindow::j4f(){if(!cj())return;m_w.cmdBase(0,conveyor_mm_to_pulse(5*10.f),SP_M4);}
void MainWindow::j4r(){if(!cj())return;m_w.cmdBase(0,conveyor_mm_to_pulse(-5*10.f),SP_M4);}
void MainWindow::s1p(){m_sv1+=10;if(m_sv1>270)m_sv1=270;m_w.cmdServo(m_sv1,255);}
void MainWindow::s1m(){m_sv1-=10;if(m_sv1<0)m_sv1=0;m_w.cmdServo(m_sv1,255);}
void MainWindow::s2p(){m_sv2+=10;if(m_sv2>180)m_sv2=180;m_w.cmdServo(255,m_sv2);}
void MainWindow::s2m(){m_sv2-=10;if(m_sv2<0)m_sv2=0;m_w.cmdServo(255,m_sv2);}
void MainWindow::stop(){m_w.cmdEstop();}
void MainWindow::em(int n){m_w.cmdEm(n);QTimer::singleShot(200,this,[this](){m_w.cmdEmOff();});}
