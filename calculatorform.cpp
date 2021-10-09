/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "calculatorform.h"
#include <iostream>
#include <algorithm>
#include <QString>
#include <QLineEdit>
#include <fstream>

//! [0]
CalculatorForm::CalculatorForm(QWidget *parent)
    : QWidget(parent)
{
    ui.setupUi(this);

    connect(ui.CalculateTables_button, SIGNAL (released()),this, SLOT (handleButton()));
    ui.dtLine->setText(QString::fromLocal8Bit("0.0001"));
    ui.massLine->setText(QString::fromLocal8Bit("51"));
    ui.kLine->setText(QString::fromLocal8Bit("0.002421"));
    ui.velLine->setText(QString::fromLocal8Bit("840"));
    ui.MaxXLine->setText(QString::fromLocal8Bit("5"));
    ui.MinYLine->setText(QString::fromLocal8Bit("-5"));
    ui.MaxYLine->setText(QString::fromLocal8Bit("3"));
    ui.InitAngleStep->setText(QString::fromLocal8Bit("0.5"));
}
//! [0]

//! [1]
/*void CalculatorForm::on_inputSpinBox1_valueChanged(int value)
{
    ui.outputWidget->setText(QString::number(value + ui.inputSpinBox2->value()));
}*/
//! [1]

//! [2]
/*void CalculatorForm::on_inputSpinBox2_valueChanged(int value)
{
    ui.outputWidget->setText(QString::number(value + ui.inputSpinBox1->value()));
}*/
double get_number(QLineEdit *edit){
    bool ok;
    double res= edit->text().toDouble(&ok);
    if( !ok){
        std::cout<<"Wrong number, exit now"<<std::endl;
        exit(1);
    }
    return res;

}

double get_intnumber(QLineEdit *edit){
    bool ok;
    double res= edit->text().toInt(&ok);
    if( !ok){
        std::cout<<"Wrong number, exit now"<<std::endl;
        exit(1);
    }
    return res;

}
void CalculatorForm::handleButton(){
    double dt,m,k,v;
    double pi = 3.1415926535897932384626433832795;
    double g = 9.80665;
    double km;
    double Kabeltov = 182.5;
    double minimun = -40*Kabeltov;
    int i,j,i1,j1;
    std::vector<double> angles;angles.reserve(1000);
    std::vector<std::vector<double> > trajectoriesX;trajectoriesX.reserve(2000);
    std::vector<std::vector<double> > trajectoriesLen;trajectoriesLen.reserve(2000);
    std::vector<std::vector<double> > trajectoriesY;trajectoriesY.reserve(2000);
    std::vector<std::vector<double> > trajectoriesVX;trajectoriesVX.reserve(2000);
    std::vector<std::vector<double> > trajectoriesVY;trajectoriesVY.reserve(2000);
    dt = get_number(ui.dtLine);
    m = get_number(ui.massLine);
    k = get_number(ui.kLine);
    v = get_number(ui.velLine);

    std::cout<<dt<<" "<<m<<" "<<k<<" "<<v<<std::endl;
    if(m == 0){
        std::cout<<"mass is zero, exit now"<<std::endl;
        exit(1);
    }
    km = k/m;


    double step = get_number(ui.InitAngleStep);;
    double begin = -pi/2.0;
    double end = pi/2.0;
    double angle =0;
    step = step*pi/180.0;
  //  begin += step;
  //  end -= step;
    angle = begin;
    int max_progbar = (int)((end-begin)/step);
    ui.progressBar->setMinimum(0);
    ui.progressBar->setMaximum(max_progbar);
    ui.progressBar->setValue(0);
    ui.progressBar->setFormat("Расчет траекторий %p% ");

    while (angle<=end){
        angles.push_back(angle);
        double Vx = v*cos(angle);
        double Vy = v*sin(angle);
        double X,Y,Len;
        X=0;
        Y=0;
        Len=0;
        std::vector<double> trajX;trajX.reserve(10000000);
        std::vector<double> trajY;trajY.reserve(10000000);
        std::vector<double> trajVX;trajVX.reserve(10000000);
        std::vector<double> trajVY;trajVY.reserve(10000000);
        std::vector<double> trajLen;trajLen.reserve(10000000);
        while (Y>=minimun){
            double kor = sqrt(Vx*Vx+Vy*Vy);
            Vx = Vx-dt*(km*kor*Vx);
            Vy = Vy-dt*(g+km*kor*Vy);
            X=X+Vx*dt;
            Y=Y+Vy*dt;
            Len =Len+ dt*sqrt(Vx*Vx+Vy*Vy);
            trajX.push_back(X);
            trajY.push_back(Y);
            trajVX.push_back(Vx);
            trajVY.push_back(Vy);
            trajLen.push_back(Len);
            }

        trajectoriesX.push_back(trajX);
        trajectoriesY.push_back(trajY);
        trajectoriesVX.push_back(trajVX);
        trajectoriesVY.push_back(trajVY);
        trajectoriesLen.push_back(trajLen);
       // std::cout<<angle<<" "<<trajX.size()<<std::endl;

        angle +=step;
        ui.progressBar->setValue(ui.progressBar->value()+1);
        QCoreApplication::processEvents();
    }
    std::cout<<"Finished"<<std::endl;
    if(trajectoriesX.size() != trajectoriesY.size()){
        std::cout<<"Calculation failed, exit now"<<std::endl;
        exit(1);
    }
    if(trajectoriesVX.size() != trajectoriesVY.size()){
        std::cout<<"Calculation failed, exit now"<<std::endl;
        exit(1);
    }
    if(trajectoriesX.size() != trajectoriesVY.size()){
        std::cout<<"Calculation failed, exit now"<<std::endl;
        exit(1);
    }
    if(trajectoriesX.size() != angles.size()){
        std::cout<<"Calculation failed, exit now"<<std::endl;
        exit(1);
    }
    for(i=0;i<trajectoriesX.size();i++){
        if(trajectoriesX[i].size() != trajectoriesY[i].size()){
            std::cout<<"Calculation failed, exit now"<<std::endl;
            exit(1);
        }
    }

    for(i=0;i<trajectoriesX.size();i++){
        for(j=0;j<trajectoriesX[i].size();j++){
            trajectoriesX[i][j] /= Kabeltov;
            trajectoriesY[i][j] /= Kabeltov;
            trajectoriesLen[i][j] /= Kabeltov;
        }
    }
  /*  double max_Y=0;
    for(i=0;i<trajectoriesX.size();i++){
        for(j=0;j<trajectoriesX[i].size();j++){
            if(max_Y<trajectoriesX[i][j]){
                max_Y=trajectoriesX[i][j];
            }
 //           trajectoriesX[i][j] /= Kabeltov;
  //          trajectoriesY[i][j] /= Kabeltov;
        }
    }
    std::cout<<max_Y<<std::endl;*/

    int min_X=1;
    int max_X=get_intnumber(ui.MaxXLine);
    int max_Y=get_intnumber(ui.MaxYLine);
    int min_Y=get_intnumber(ui.MinYLine);;
//    int min_Y=std::min(-30,-max_Y);

    int zs_x = max_X -min_X +1;
    int zs_y = max_Y -min_Y +1;
    int **arr_num_traj = new int* [zs_x];
    int **arr_num_in_traj= new int*[zs_x];
    for(i=0;i<zs_x;i++){
        arr_num_traj[i]= new int[zs_y];
        arr_num_in_traj[i]= new int[zs_y];
    }
    max_progbar=zs_x*zs_y;
    ui.progressBar_2->setMinimum(0);
    ui.progressBar_2->setMaximum(max_progbar);
    ui.progressBar_2->setValue(0);
    ui.progressBar_2->setFormat("Поиск пересечений %p% ");

    int num_x_tmp,num_y_tmp;
    num_x_tmp=-1;
    num_y_tmp=-1;
    for(i1=0;i1<zs_x;i1++){
        for(j1=0;j1<zs_y;j1++){
            double X = min_X +i1;
            double Y = min_Y +j1;
            double angl_ = asin(Y/sqrt(X*X+Y*Y));
            double dist_min = 10000000;
           // std::cout<<"X "<<X<<" Y "<<Y<<std::endl;
            int num = trajectoriesX.size();
            //if(X==0 && Y<0) num = num/2;
            for(i=0;i<num;i++){

               // std::cout<<angles[i]<<" "<<angl_<<" "<<angles[i]-angl_<<std::endl;
                if((angles[i]-angl_<(pi/8)) && (angles[i] - angl_>0) )
                for(j=0;j<trajectoriesX[i].size();j++){
                    double vx = trajectoriesVX[i][j];
                    double vy = trajectoriesVY[i][j];

                    double angl__ = asin(vy/sqrt(vx*vx+vy*vy));
                    double dist=sqrt( (trajectoriesX[i][j]-X)*(trajectoriesX[i][j]-X) + (trajectoriesY[i][j]-Y)*(trajectoriesY[i][j]-Y) );
                    if((dist <dist_min)&&(angles[i]-angl__<3*pi/4)){
                        dist_min=dist;
                        num_x_tmp=i;
                        num_y_tmp=j;
                    }
                }
            }
            arr_num_traj[i1][j1]=num_x_tmp;
            arr_num_in_traj[i1][j1]=num_y_tmp;
            double vx = trajectoriesVX[arr_num_traj[i1][j1]][arr_num_in_traj[i1][j1]];
            double vy = trajectoriesVY[arr_num_traj[i1][j1]][arr_num_in_traj[i1][j1]];

            double angl = asin(vy/sqrt(vx*vx+vy*vy));
            if(angl_>angles[num_x_tmp]){
                std::cout<<"Wrong angle"<<std::endl;
             //   exit(1);
            }
            std::cout<<"X "<<X<<" Y "<<Y<<" angle "<<angles[num_x_tmp]*180/pi<< " our angle "<<angl_*180/pi<<" dist "<<dist_min<<" time "<<(num_y_tmp+1)*dt<<" final_angle "<<angl*180/pi<< std::endl;
            if(dist_min>0.5)
                {
                    std::cout<<"BIG dist!!!"<<std::endl;
                    std::cout<<"X "<<X<<" Y "<<Y<<" angle "<<angles[num_x_tmp]*180/pi<< " our angle "<<angl_*180/pi<<" dist "<<dist_min<<" time "<<(num_y_tmp+1)*dt<<" final_angle "<<angl*180/pi<< std::endl;
                 //   exit(1);
                    arr_num_traj[i1][j1]=-1;
                    arr_num_in_traj[i1][j1]=-1;

                }
            ui.progressBar_2->setValue(ui.progressBar_2->value()+1);
            QCoreApplication::processEvents();
            /*if(dist_min>0.1){
                std::cout<<"Wrong dist"<<std::endl;
                exit(1);
            }*/
        }
    }
    std::ofstream myfile;
    myfile.open ("init_angle.csv");
    myfile<<"Y\\X,";
    for(i1=0;i1<zs_x;i1++){
        myfile<<min_X+i1<<", ";
    }
    myfile<<std::endl;
    for(i1=0;i1<zs_y;i1++){
        myfile<<min_Y +i1<<", ";
        for(j1=0;j1<zs_x;j1++){
            if (arr_num_traj[j1][i1]>=0)
                myfile<<angles[arr_num_traj[j1][i1]]*180/pi;
            else
                myfile<<"###";
            if(j1 !=zs_x-1) myfile<<", ";
        }
        myfile<<std::endl;
    }
    myfile.close();

    myfile.open ("trajectoryLength.csv");
    myfile<<"Y\\X,";
    for(i1=0;i1<zs_x;i1++){
        myfile<<min_X+i1<<", ";
    }
    myfile<<std::endl;
    for(i1=0;i1<zs_y;i1++){
        myfile<<min_Y +i1<<", ";
        for(j1=0;j1<zs_x;j1++){
            if((arr_num_traj[j1][i1]>=0)&&(arr_num_in_traj[j1][i1]>=0))
                myfile<<trajectoriesLen[arr_num_traj[j1][i1]][arr_num_in_traj[j1][i1]];
            else
                myfile<<"###";
            if(j1 !=zs_x-1) myfile<<", ";
        }
        myfile<<std::endl;
    }
    myfile.close();

    myfile.open ("Final_angle.csv");
    myfile<<"Y\\X,";
    for(i1=0;i1<zs_x;i1++){
        myfile<<min_X+i1<<", ";
    }
    myfile<<std::endl;
    for(i1=0;i1<zs_y;i1++){
        myfile<<min_Y +i1<<", ";
        for(j1=0;j1<zs_x;j1++){
            if((arr_num_traj[j1][i1]>=0)&&(arr_num_in_traj[j1][i1]>=0))
            {
            double vx = trajectoriesVX[arr_num_traj[j1][i1]][arr_num_in_traj[j1][i1]];
            double vy = trajectoriesVY[arr_num_traj[j1][i1]][arr_num_in_traj[j1][i1]];

            double angl = asin(vy/sqrt(vx*vx+vy*vy));
            myfile<<angl*180/pi;}
            else
                myfile<<"###";
            if(j1 !=zs_x-1) myfile<<", ";
        }
        myfile<<std::endl;
    }
    myfile.close();

    myfile.open ("Final_velocity.csv");
    myfile<<"Y\\X,";
    for(i1=0;i1<zs_x;i1++){
        myfile<<min_X+i1<<", ";
    }
    myfile<<std::endl;
    for(i1=0;i1<zs_y;i1++){
        myfile<<min_Y +i1<<", ";
        for(j1=0;j1<zs_x;j1++){
            if((arr_num_traj[j1][i1]>=0)&&(arr_num_in_traj[j1][i1]>=0)){
            double vx = trajectoriesVX[arr_num_traj[j1][i1]][arr_num_in_traj[j1][i1]];
            double vy = trajectoriesVY[arr_num_traj[j1][i1]][arr_num_in_traj[j1][i1]];

            double V_mod = sqrt(vx*vx+vy*vy);
            myfile<<V_mod/v;}
            else
                myfile<<"###";
            if(j1 !=zs_x-1) myfile<<", ";
        }
        myfile<<std::endl;
    }
    myfile.close();


    myfile.open ("Final_angle_artillery.csv");
    myfile<<"Y\\X,";
    for(i1=0;i1<zs_x;i1++){
        myfile<<min_X+i1<<", ";
    }
    myfile<<std::endl;
    for(i1=0;i1<zs_y;i1++){
        myfile<<min_Y +i1<<", ";
        for(j1=0;j1<zs_x;j1++){
            if((arr_num_traj[j1][i1]>=0)&&(arr_num_in_traj[j1][i1]>=0)){
            double vx = trajectoriesVX[arr_num_traj[j1][i1]][arr_num_in_traj[j1][i1]];
            double vy = trajectoriesVY[arr_num_traj[j1][i1]][arr_num_in_traj[j1][i1]];

            double angl = asin(vy/sqrt(vx*vx+vy*vy));
            myfile<<-angl*180/pi;}
            else
                myfile<<"###";
            if(j1 !=zs_x-1) myfile<<", ";
        }
        myfile<<std::endl;
    }
    myfile.close();


    myfile.open ("Time.csv");
    myfile<<"Y\\X,";
    for(i1=0;i1<zs_x;i1++){
        myfile<<min_X+i1<<", ";
    }
    myfile<<std::endl;
    for(i1=0;i1<zs_y;i1++){
        myfile<<min_Y +i1<<", ";
        for(j1=0;j1<zs_x;j1++){
           // double vx = trajectoriesVX[arr_num_traj[j1][i1]][arr_num_in_traj[j1][i1]];
           // double vy = trajectoriesVY[arr_num_traj[j1][i1]][arr_num_in_traj[j1][i1]];

         //   double angl = asin(vy/sqrt(vx*vx+vy*vy));
            if(arr_num_in_traj[j1][i1]>=0)
            myfile<<(arr_num_in_traj[j1][i1]+1)*dt;
            else
                myfile<<"###";
            if(j1 !=zs_x-1) myfile<<", ";
        }
        myfile<<std::endl;
    }
    myfile.close();
    for(i=0;i<zs_x;i++){
        delete [] arr_num_traj[i];
        delete [] arr_num_in_traj[i];
    }
    delete [] arr_num_traj;
    delete [] arr_num_in_traj;
}
//! [2]
