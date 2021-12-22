//Written by Bastien LAFARGUE
//Last update 22/12/21 11:59 GMT+1

#include <iostream>
#include <vector>
#include <boost/numeric/odeint.hpp>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "resolving.h"
#include "equation.h"
#include "integ_meth.h"
#include "autoscale_graph.h"
#include "fstream"
#include <string>
#include <iomanip>
#include <ctime>

using namespace std;
using namespace boost::numeric::odeint;

int n=0;                                                                           //Number of run

double MASS;                                                                       //Mass (kg)
double IZZ;                                                                        //Yaw Moment of Inertia (kg.m²)
double LF;                                                                         //Lenght front axle->Cg (m)
double LR;                                                                         //Lenght rear axle->Cg  (m)
double CF;                                                                         //Front cornering stiffness (N/rad)
double CR;                                                                         //Rear cornering stiffness (N/rad)
double AMPL;                                                                       //Input amplitude (at 10s for ramp) (rad)
double FREQ;                                                                       //Input frequency (for sine wave only) (Hz)
double VX;                                                                         //Longitudinal velocity (m/s)
double kmin1=0;                                                                    //Minimum Yaw rate value
double kmin2=0;                                                                    //Minimum Lateral velocity value
double kmax1=0;                                                                    //Maximum Yaw rate value
double kmax2=0;                                                                    //Maximum Lateral velocity value

size_t step;                                                                       //Number of integration steps

QString ST_T;                                                                      //Input type : step, ramp, sine (/)
QString title;                                                                     //Title of the .csv export file

vector <double> lat_vel;                                                           //Lateral velocity initialisation for output
vector <double> yaw_rate;                                                          //Yaw rate initialisation for output
vector <double> timet;                                                             //Time initialisation for output

QVector<string> Param_ST;                                                          //Static parameters names
QVector<string> Param_DB;                                                          //Static parameters values
QVector<string> Titles_EX1;                                                        //Time, Yaw rate and Lat vel columns names
QVector<string> Titles_EX2;                                                        //Static column name
QVector<string> Titles_EX3;                                                        //Car parameters column name
QVector<string> Titles_EX;                                                         //Global columns names

QVector<QVector<double>> Global_vec;                                               //Concatenator table


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->comboBox->addItem("step");                                                 //Add "step steer" to combobox
    ui->comboBox->addItem("sine");                                                 //Add "sine steer" to combobox
    ui->comboBox->addItem("ramp");                                                 //Add "ramp steer" to combobox
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_exec_clicked() //RESOLVE EQUATION AND DISP GRAPH
{
   double kmin1p=kmin1;                                                            //Take the value of the current run (min yaw rate)
   double kmin2p=kmin2;                                                            //Take the value of the current run (min lat velocity)
   double kmax1p=kmax1;                                                            //Take the value of the current run (max yaw rate)
   double kmax2p=kmax2;                                                            //Take the value of the current run (max lat velocity)

   AMPL=ui->doubleSpinBox_7->value();                                              //Take value of spin box (steer amplitude)
   FREQ=ui->doubleSpinBox_8->value();                                              //Take value of spin box (steer frequency)
   VX=ui->doubleSpinBox_9->value();                                                //Take value of spin box (longitudinal velocity)
   ST_T=ui->comboBox->currentText();                                               //Take value of combo box (steering type)

   ui->label_39->setText(ST_T);                                                    //Disp the text (steering type)
   ui->label_44->setText(QString::number(AMPL));                                   //Disp the value (steering amplitude)
   ui->label_47->setText(QString::number(FREQ));                                   //Disp the value (steering frequency)

   ui->label_7->setText(QString::number(MASS));                                    //Disp the value (Mass)
   ui->label_8->setText(QString::number(IZZ));                                     //Disp the value (Izz)
   ui->label_9->setText(QString::number(LF));                                      //Disp the value (Lf)
   ui->label_10->setText(QString::number(LR));                                     //Disp the value (Lr)
   ui->label_11->setText(QString::number(CF));                                     //Disp the value (cf)
   ui->label_12->setText(QString::number(CR));                                     //Disp the value (cr)

   MASS=ui->doubleSpinBox->value();                                                //Take value of spin box (Mass)
   IZZ=ui->doubleSpinBox_2->value();                                               //Take value of spin box (Izz)
   LF=ui->doubleSpinBox_3->value();                                                //Take value of spin box (Lf)
   LR=ui->doubleSpinBox_4->value();                                                //Take value of spin box (Lr)
   CF=ui->doubleSpinBox_5->value();                                                //Take value of spin box (Cf)
   CR=ui->doubleSpinBox_6->value();                                                //Take value of spin box (Cr)

   Param_ST.push_back("Mass");                                                     //Add Static parameters column name (Mass)
   Param_ST.push_back("Izz");                                                      //Add Static parameters column name (Izz)
   Param_ST.push_back("Lf");                                                       //Add Static parameters column name (Lf)
   Param_ST.push_back("Lr");                                                       //Add Static parameters column name (Lr)
   Param_ST.push_back("cf");                                                       //Add Static parameters column name (cf)
   Param_ST.push_back("cr");                                                       //Add Static parameters column name (cr)
   Param_ST.push_back("Steering Type");                                            //Add Static parameters column name (steering type)
   Param_ST.push_back("Steering amplitude");                                       //Add Static parameters column name (steering amplitude)
   Param_ST.push_back("Steerting frequency");                                      //Add Static parameters column name (steering frequency)
   Param_ST.push_back("Longitudinal velocity");                                    //Add Static parameters column name (longitudinal velocity)

   Param_DB.push_back(to_string(MASS));                                            //Add Static parameters value (Mass)
   Param_DB.push_back(to_string(IZZ));                                             //Add Static parameters value (Izz)
   Param_DB.push_back(to_string(LF));                                              //Add Static parameters value (Lf)
   Param_DB.push_back(to_string(LR));                                              //Add Static parameters value (Lr)
   Param_DB.push_back(to_string(CF));                                              //Add Static parameters value (cf)
   Param_DB.push_back(to_string(CR));                                              //Add Static parameters value (cr)
   Param_DB.push_back(ST_T.toStdString());                                         //Add Static parameters name (steering type)
   Param_DB.push_back(to_string(AMPL));                                            //Add Static parameters value (steering amplitude)
   Param_DB.push_back(to_string(FREQ));                                            //Add Static parameters value (steering frequency)
   Param_DB.push_back(to_string(VX));                                              //Add Static parameters value (longitudinal velocity)

   //-----------------Integration-------------------

   integ Method;
   state_type x(2);
   Method.Euler(x);                                                                //Integration method : .Euler (Euler), .RK4 (Runge Khutta 4)
   tie(lat_vel, yaw_rate, timet, step) = resolution();                             //Get data from Resolving->Vector Resolution function

   //-------------Vectors construction--------------

     QVector<double> times1;                                                       //Transition vector initialisation (time)
     QVector<double> qVec1;                                                        //Transition vector initialisation (yaw rate)
     QVector<double> gVec1;                                                        //Transition vector initialisation (lateral velocity)

     for (size_t i = 0; i <= step; i++)                                            //Output loop
     {
     times1.push_back(timet[i]);                                                   //Transition vector construction (time)
     qVec1.push_back(yaw_rate[i]);                                                 //Transition vector construction (yaw rate)
     gVec1.push_back(lat_vel[i]);                                                  //Transition vector construction (lateral velocity)
     }

     Global_vec.push_back(times1);                                                 //Concatenator table construction
     Global_vec.push_back(qVec1);
     Global_vec.push_back(gVec1);

    //-------------Graph construction--------------

      double kmin1n = *std::min_element(qVec1.constBegin(), qVec1.constEnd());     //Min value yaw rate
      double kmax1n = *std::max_element(qVec1.constBegin(), qVec1.constEnd());     //Max value yaw rate
      double kmin2n = *std::min_element(gVec1.constBegin(), gVec1.constEnd());     //Min value lateral velocity
      double kmax2n = *std::max_element(gVec1.constBegin(), gVec1.constEnd());     //Max value lateral velocity

      tie(kmin1,kmax1,kmin2,kmax2)=graph(kmin1n,kmax1n,kmin2n,kmax2n);             //Get "k" values from autoscale program to set range

      if (kmin1p<kmin1)                                                            //Select values for graph autoscale
      {kmin1=kmin1p;}
      if (kmin2p<kmin2)
      {kmin2=kmin2p;}
      if (kmax1p>kmax1)
      {kmax1=kmax1p;}
      if (kmax2p>kmax2)
      {kmax2=kmax2p;}

      ui->customPlot->xAxis->setRange(0, 10);                                       //Range time axis of yaw rate
      ui->customPlot->yAxis->setRange(kmin1,kmax1);                                 //Range y axis yaw rate
      ui->customPlot->xAxis2->setRange(0, 10);                                      //Range time axis of lateral velocity
      ui->customPlot->yAxis2->setRange(kmin2,kmax2);                                //Range y axis lateral velocity

      string s = to_string(n+1);                                                    //Convert "n" in string
      QString numb = QString::fromStdString(s);                                     //Convert "n" in QString

      QPen Pen0;                                                                    //Graph 0 pen initialisation
      Pen0.setWidth(2);                                                             //Graph 0 pen width
      Pen0.setColor(QColor(40+50*n, 110+30*n, 255-50*n));                           //Graph 0 pen colour (changing every run)

      ui->customPlot->addGraph(ui->customPlot->xAxis, ui->customPlot->yAxis);       //Graph 0 creation
      ui->customPlot->graph(2*n)->setPen(Pen0);                                     //Grap 0 curve colour (blue)

      QPen Pen1;                                                                    //Graph 1 pen initialisation
      Pen1.setWidth(2);                                                             //Graph 1 pen width
      Pen1.setColor(QColor(255-50*n, 110-30*n, 40+50*n));                           //Graph 1 pen colour (changing every run)
      Pen1.setStyle(Qt::DotLine);                                                   //Graph 1 pen style (doted line)

      ui->customPlot->addGraph(ui->customPlot->xAxis2, ui->customPlot->yAxis2);     //Graph 1 creation
      ui->customPlot->graph(2*n+1)->setPen(Pen1);                                   //Graph 1 curve colour (red)

      ui->customPlot->graph(2*n)->setData(Global_vec[3*n],Global_vec[3*n+1]);       //Values used in graph 0
      ui->customPlot->graph(2*n+1)->setData(Global_vec[3*n],Global_vec[3*n+2]);     //Values used in graph 1

      ui->customPlot->xAxis2->setVisible(false);                                    //Hide or show graph 1 time axis
      ui->customPlot->yAxis2->setVisible(true);                                     //Hide or show graph 1 y axis

      ui->customPlot->xAxis->setLabel("Time (s)");                                  //Graph 0 time axis label
      ui->customPlot->yAxis->setLabel("Yaw Rate (rad/s)");                          //Graph 0 y axis label
      ui->customPlot->yAxis2->setLabel("Lateral velocity (m/s)");                   //Graph 1 y axis label

      ui->customPlot->graph(2*n)->setName("Yaw Rate vehicle "+numb);                //Graph 0 curve label
      ui->customPlot->graph(2*n+1)->setName("Lateral velocity vehicle "+numb);      //Graph 1 curve label

      ui->customPlot->legend->setVisible(true);                                     //Hide or show graphs legend

      QFont legendFont = font();                                                    //Legend font declaration
      legendFont.setPointSize(9);                                                   //Set legend font size
      ui->customPlot->legend->setFont(legendFont);                                  //Set legend font style
      ui->customPlot->legend->setBrush(QBrush(QColor(255,255,255,230)));            //Set legend font colour
      ui->customPlot->axisRect()->insetLayout()->setInsetAlignment\
              (0, Qt::AlignTop|Qt::AlignLeft);                                      //Set legend position on graph

      n=n+1;

      ui->customPlot->replot();                                                     //Update graph

     //------------------Prepare export-----------------------

      Titles_EX1.push_back("Time"+s);                                               //Create run-depedant columns for export
      Titles_EX1.push_back("Yaw_rate"+s);
      Titles_EX1.push_back("Lat_Velo"+s);
      Titles_EX3.push_back("Parameters"+s);
}

void MainWindow::on_pushButton_clear_clicked()    //CLEAR GRAPH AND DATAS

{
      ui->customPlot->clearPlottables();                                            //Double clear to erase en recreate an empty graph
      ui->customPlot->replot();
      ui->customPlot->clearPlottables();

      n=0;                                                                          //Clear run number

      Titles_EX1={};                                                                //Clear vectors and table
      Titles_EX3={};
      Titles_EX={};
      Global_vec={};

      kmin1=0;                                                                      //Clear autoscale values
      kmin2=0;
      kmax1=0;
      kmax2=0;
}

void MainWindow::on_pushButton_export_clicked()  //EXPORT DATAS IN .CSV
{
    title=ui->lineEdit->text();                                                     //Get text from lineEdit for export file title
    Titles_EX2={"static"};                                                          //Add column in export table

    ofstream myFile;
    myFile.open(title.toStdString()+".csv");                                        //Create export file in .csv

    Titles_EX=Titles_EX1+Titles_EX2+Titles_EX3;                                     //Concatenate columns titles for export table

    for(int j = 0; j <= n*3+n; ++j)
    {myFile << Titles_EX[j];                                                        //Add column names
    if(j != n*3-1+n+1 ) myFile << ",";                                              //Put "," after columns (except the last one)
    }
    myFile << "\n";
    for (size_t i = 0; i <= step; i++)
    {
    for(int j = 0; j <= n*3-1; ++j)
        {
         myFile << Global_vec[j][i];                                                //Add times, yaw rate and lat vel values
         if(j != n*3-1+n+1 ) myFile << ",";                                         //Put "," after columns (except the last one)
         if(i < 9 & j >= 3*(n-1)+2 ) myFile << Param_ST[i]<<",";                    //Add parameters names
         if(i < 9 & j >= 3*(n-1)+2 )
         for(int j = 0; j <= n-1; ++j)
         {
         myFile << Param_DB[i+10*(n-1)];                                            //Add parameters values
         if(i < 9 & j >= 3*(n-1)+2 )
         if(j != n-1) myFile << ",";}                                               //Put "," after columns (except the last one)
         }
         myFile << "\n";
     }
}



