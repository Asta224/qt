#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPropertyAnimation>
#include <QGraphicsOpacityEffect>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}

QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onLoginClicked();
    void onLoginFormFadeOutFinished();

private:
    Ui::MainWindow *ui;
    QGraphicsOpacityEffect *m_loginOpacityEffect;
    QPropertyAnimation *m_loginFadeOutAnimation;
    QPropertyAnimation *m_welcomeFadeInAnimation;
};
#endif // MAINWINDOW_H
