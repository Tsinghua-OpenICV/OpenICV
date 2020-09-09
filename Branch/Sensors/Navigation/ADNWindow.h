/*
*   Autonomous Driving Navigation Library
*   View Window Class
*   Li Bing, QYJY, 2018.7
*/

#ifndef ADNWINDOW_H
#define ADNWINDOW_H
#include <QApplication>
#include <QPainter>
#include <QWidget>
#include <QImage>

class ADNWidget : public QWidget
{
    Q_OBJECT

public:
    ADNWidget():QWidget(){
    QWidget::setWindowTitle("Autonomous Driving Navigation");
     QWidget::setWindowFlags(Qt::FramelessWindowHint);
     QWidget::move(0,0);
    };
    ~ADNWidget(){};

    void SetCanvas(uchar* buffer, int width, int height)
    {
    m_Canvas=QImage(buffer,width,height,QImage::Format_RGBA8888);
    resize(width,height);
    };

private:
    void paintEvent(QPaintEvent* e)
    {
        QPainter p;
    if(p.begin(this)){
//        p.setRenderHint(QPainter::Antialiasing);
        p.drawImage(0,0,m_Canvas);
        p.end();
       }
    };
    void closeEvent(QCloseEvent* e){ exit(0);};

private:
    QImage m_Canvas;
};
//View Window Class
class ADNWindow
{
public:
    // construct window
    // buffer:  canvas bits buffer
    // widht:   canvas width
    // height:  canvas height
    ADNWindow(char* buffer, int width, int height);
    ~ADNWindow();

    // show window
    void Show();

    // refresh window
    void Refresh();

private:
    QApplication *m_App;
    ADNWidget *m_Widget;
};


#endif // ADNWINDOW_H
