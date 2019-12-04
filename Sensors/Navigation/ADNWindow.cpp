/*
*   Autonomous Driving Navigation Library
*   View Window Class
*   Li Bing, QYJY, 2018.7
*/

#include "ADNWindow.h"
#include <QPainter>
#include <QApplication>


// ADNWidget::ADNWidget(QWidget *parent)
//     : QWidget(parent)
// {
//     setWindowTitle("Autonomous Driving Navigation");
//     setWindowFlags(Qt::FramelessWindowHint);
//     move(0,0);
// }

// ADNWidget::~ADNWidget()
// {

// }

// void ADNWidget::SetCanvas(uchar *buffer, int width, int height)
// {
//     m_Canvas=QImage(buffer,width,height,QImage::Format_RGBA8888);
//     resize(width,height);
// }

// void ADNWidget::paintEvent(QPaintEvent *e)
// {
//     QPainter p;
//     if(p.begin(this)){
// //        p.setRenderHint(QPainter::Antialiasing);
//         p.drawImage(0,0,m_Canvas);
//         p.end();
//     }
// }

// void ADNWidget::closeEvent(QCloseEvent *e)
// {
//     exit(0);
// }

ADNWindow::ADNWindow(char* buffer, int width, int height)
{
    int n;
    m_App=new QApplication(n,0);

 m_Widget ;//= new ADNWidget();
    //m_Widget->SetCanvas((uchar*)buffer,width,height);
//    m_Widget->move(400,0);
    Show();
}

ADNWindow::~ADNWindow()
{
   // delete m_Widget;
    delete m_App;
}

void ADNWindow::Show()
{
   // m_Widget->show();
    m_App->processEvents();
}

void ADNWindow::Refresh()
{
   // m_Widget->repaint();
    m_App->processEvents();
}
