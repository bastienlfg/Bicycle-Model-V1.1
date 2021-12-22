//Written by Bastien LAFARGUE
//Last update 01/11/21 15:37 GMT+1

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }

extern double LF;
extern double LR;
extern double MASS;
extern double IZZ;
extern double CF;
extern double CR;
extern double AMPL;
extern double FREQ;
extern double VX;
extern QString ST_T;
extern QVector<QVector<double>> Global_vec;
extern size_t step;

QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:

    void on_pushButton_exec_clicked();


    void on_pushButton_clear_clicked();


    void on_pushButton_export_clicked();


private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
