#include <QApplication>
#include <QScreen>
#include "mainwindow.h"
int main(int argc, char *argv[]) {
    QApplication a(argc, argv);
    a.setStyleSheet(R"(
        *{font-family:"Noto Sans CJK SC","WenQuanYi Micro Hei",sans-serif;font-size:13px;color:#3c4043;}
        QMainWindow{background:#e8ecf1;}
        QGroupBox{font-weight:bold;font-size:12px;color:#5f6368;border:1px solid #dadce0;border-radius:8px;margin-top:14px;padding:14px 10px 6px 10px;background:white;}
        QGroupBox::title{subcontrol-origin:margin;left:10px;padding:0 6px;color:#1a73e8;}
        QPushButton{border:1px solid #dadce0;border-radius:8px;padding:8px 14px;background:white;color:#3c4043;min-height:36px;font-size:13px;}
        QPushButton:hover{background:#e8f0fe;border-color:#1a73e8;}
        QPushButton:pressed{background:#d2e3fc;}
        QPushButton#danger{background:#d93025;color:white;border:none;font-weight:bold;font-size:16px;min-width:72px;min-height:72px;border-radius:36px;}
        QPushButton#danger:hover{background:#c5221f;}
        QPushButton#primary{background:#1a73e8;color:white;border:none;font-weight:bold;}
        QPushButton#primary:hover{background:#1557b0;}
        QPushButton#success{background:#4caf50;color:white;border:none;font-weight:bold;}
        QPushButton#success:hover{background:#388e3c;}
        QPushButton#warn{background:#e37400;color:white;border:none;}
        QPushButton#warn:hover{background:#c95d00;}
        QPushButton#ok{background:#4caf50;color:white;border:none;font-weight:bold;border-radius:8px;}
        QPushButton#back{background:#f9ab00;color:white;border:none;font-weight:bold;border-radius:8px;}
        QPushButton#back:hover{background:#e69500;}
        QSlider::groove:vertical{width:56px;background:#e8eaed;border-radius:8px;border:1px solid #dadce0;}
        QSlider::handle:vertical{height:28px;margin:0-6px;background:qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0#1a73e8,stop:1#4fc3f7);border-radius:8px;}
        QSlider::sub-page:vertical{background:#a8d0f0;border-radius:8px;width:56px;}
        QSlider::groove:horizontal{height:32px;background:#e8eaed;border-radius:8px;border:1px solid #dadce0;}
        QSlider::handle:horizontal{width:24px;margin:-4px 0;background:#1a73e8;border-radius:6px;}
        QSlider::sub-page:horizontal{background:#a8d0f0;border-radius:8px;}
        QSlider::groove:horizontal:disabled{background:#e8eaed;}
        QSlider::handle:horizontal:disabled{background:#d0d5dd;}
        QSpinBox{padding:4px 10px;border:1px solid #dadce0;border-radius:8px;background:white;font-weight:bold;font-size:14px;}
        QFrame#card{background:white;border:1px solid #e8eaed;border-radius:8px;padding:8px;}
        QLabel#val{font-size:18px;font-weight:bold;color:#1a73e8;}
        QLabel#unit{font-size:11px;color:#80868b;}
        QScrollArea{border:none;background:transparent;}
    )");
    MainWindow w;
    QScreen *sc=QGuiApplication::primaryScreen();
    if(sc){QRect g=sc->availableGeometry();w.resize(g.width(),g.height());}
    else w.resize(1024,568);
    w.show();
    return a.exec();
}
