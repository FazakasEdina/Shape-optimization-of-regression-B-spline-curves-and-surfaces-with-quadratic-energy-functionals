#include <QApplication>
#include "GUI/MainWindow.h"

using namespace cagd;

int main(int argc, char **argv)
{
    // it may happen that high dpi scaling is enabled by default;
    // if this is the case, you can delete the lines 8--10
//    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling, false);

    // if you have installed a different version of Qt, it may happen that
    // the application attribute Qt::AA_UseDesktopOpenGL is not recognized;
    // on Windows its existence is critical for our applications;
    // on Linux or Mac you can uncomment this line
    QApplication::setAttribute(Qt::AA_UseDesktopOpenGL, true);

    // creating an application object and setting one of its attributes
    QApplication app(argc, argv);

    // sets the style of the graphical user interface
    app.setStyle("fusion");

    // creating a main window object
    MainWindow mwnd;
    mwnd.showMaximized();

    // running the application
    return app.exec();
}
