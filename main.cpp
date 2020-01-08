
#include <QtGui>

#include "RMapViewer.h"

using namespace rtk;

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    // set random seed
    qsrand(QTime(0,0,0).secsTo(QTime::currentTime()));

    // create viewer obj
    RMapViewer *widget = new RMapViewer;

    QMainWindow mainWindow;
    mainWindow.setCentralWidget(widget);
    mainWindow.setGeometry(0, 0, 800, 800);
    mainWindow.show();

    // begin GUI loop
    return app.exec();
}
