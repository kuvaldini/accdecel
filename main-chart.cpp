/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the Qt Charts module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:GPL$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 3 or (at your option) any later version
** approved by the KDE Free Qt Foundation. The licenses are as published by
** the Free Software Foundation and appearing in the file LICENSE.GPL3
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QSplineSeries>
#include <QtCharts/QValueAxis>
#include <QtCharts/QCategoryAxis>
#include <QLineEdit>

QT_CHARTS_USE_NAMESPACE

#include <iostream>
#include <iomanip>
#include <functional>
#include "Motion.hpp"

using namespace std;
using namespace EnvisionTec;


int maino( QXYSeries *lineCalcEveryStep, QXYSeries *lineCalcVx )
{
    Motion  mo;
    mo.accel = 5000;
    mo.decel = 8000;
    mo.speedUpdatePeriod_ms = 25;
    mo.initialPosition = -0;
    mo.targetPosition = 2500;  //Motion::POSITION_MAX;  //10022222-80000;  //Motion::POSITION_MAX-1;  //-100000; //
    mo.wantedSpeedAfterAccel = 3000;
    mo.recalculate();

    /*auto  TarPosIsNotReached = mo.initialPosition < mo.targetPosition
            ? [](int pos, int tarpos)->bool{ return pos < tarpos; }
            : [](int pos, int tarpos)->bool{ return pos > tarpos; }
    ;
    auto  nextPos = mo.initialPosition < mo.targetPosition
            ? [](int &pos)->int&{ return ++pos; }
            : [](int &pos)->int&{ return --pos; }
    ;*/
    cout    <<"\n"
            <<"mo:{\n"
         //   <<"  dV_accel_: "<<mo.dV_accel_<<"\n"
         //   <<"  dV_decel_: "<<mo.dV_decel_<<"\n"
            <<"  Vc_: "<<mo.Vc_<<"\n"
            <<"  Xa_: "<<mo.Xa_<<"\n"
            <<"  Xc_: "<<mo.Xc_<<"\n"
            <<"  Xb_: "<<mo.Xb_<<"\n"
            <<"  Ta_: "<<mo.Ta_<<"\n"
            <<"  Tc_: "<<mo.Tc_<<"\n"
            <<"  Tb_: "<<mo.Tb_<<"\n"
            <<"}\n"
            <<endl;

    int pos=0;
    int prevpos = mo.initialPosition;
    cout<<"pos:"<<setw(7)<<pos<<"   "<<"dpos:"<<setw(6)<<pos-prevpos<<"  "<<"setStepFreq( "<<setw(6)<<"cspeed"<<" )"<<endl;
/*    for( ; TarPosIsNotReached(pos,mo.targetPosition); nextPos(pos) ){
        double cspeed = mo.updatePosition( pos );
        if( cspeed == 0 ){
            cout<<"pos:"<<setw(7)<<pos<<"   stopStepping()"<<endl;
        }else if( std::isfinite(cspeed) ){
            cout<<"pos:"<<setw(7)<<pos<<"   "<<"dpos:"<<setw(6)<<pos-prevpos<<"  "<<"setStepFreq( "<<setw(6)<<cspeed<<" )"<<endl;
//            *line << QPointF(pos,cspeed);
            prevpos = pos;
        }
        //cout <<pos<<" "<< *mo.iter_ <<" "<<endl;
    }*/

    /*// Do not use with POSITION_MAX and _MIN
    auto v = mo.calcEachPosition();
    for( auto const& ps : v ){
        *lineCalcEveryStep << QPointF( ps.pos, ps.spd );
    }*/

	cout <<"vecVx"<<endl;
    //auto vecXV = mo.calcPositionToSpeedPairs();
    for( auto const& posSpd : mo.positionToSpeedPairs ){
		cout <<"posSpd:{ pos:"<<setw(7)<<posSpd.pos<<", spd:"<<setw(7)<<posSpd.spd<<" }"<<endl;
		*lineCalcVx << QPointF( posSpd.pos, posSpd.spd );
    }
    cout <<"vec.size(): "<<mo.positionToSpeedPairs.size()<<endl;
    /*for( auto iter = vecVx.cbegin();  iter != vecVx.cend();  ++iter ){
		auto const& prev = *(iter);
		auto const& cur = *iter;
		cout <<"posSpd:{ pos:"<<setw(7)<<cur.pos<<", spd:"<<setw(7)<<cur.spd<<" }"<<endl;
		*lineCalcVx << QPointF( cur.pos, prev.spd ) << QPointF( cur.pos, cur.spd );
		++iter;
    }*/

    return 0;
}


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    //![1]
    QChart *chart = new QChart();
    chart->legend()->hide();
    chart->setTitle("Multiaxis chart example");
    //![1]

    //![2]
    QValueAxis *axisX = new QValueAxis;
	axisX->setTickCount(9);
	axisX->setMinorTickCount(2);
	axisX->setLabelFormat(QString("%d"));
    chart->addAxis(axisX, Qt::AlignBottom);
    //![2]

    //![3]
//    QSplineSeries *series = new QSplineSeries;
//   *series << QPointF(1, 5) << QPointF(3.5, 18) << QPointF(4.8, 7.5) << QPointF(10, 2.5);
//    chart->addSeries(series);

    QValueAxis *axisY = new QValueAxis;
//    axisY->setLinePenColor(series->pen().color());
//    axisY->setMax(+6200);
//    axisY->setMin(-4500);
	axisY->setTickCount(9);
	axisY->setMinorTickCount(4);
    chart->addAxis(axisY, Qt::AlignLeft);
//    series->attachAxis(axisX);
//    series->attachAxis(axisY);
    //![3]

    //![4]
 //   series = new QSplineSeries;
 //   *series << QPointF(1, 0.5) << QPointF(1.5, 4.5) << QPointF(2.4, 2.5) << QPointF(4.3, 12.5)
 //           << QPointF(5.2, 3.5) << QPointF(7.4, 16.5) << QPointF(8.3, 7.5) << QPointF(10, 17);
    //chart->addSeries(series);
	
    
    QXYSeries *lineCalcEveryStep = new QLineSeries; //QLineSeries;
    QXYSeries *lineCalcVx = new QLineSeries; //QScatterSeries;
    lineCalcVx->setPointsVisible(true);
    lineCalcVx->setPointLabelsVisible(true);
    maino(lineCalcEveryStep, lineCalcVx);
    //chart->addSeries(lineCalcEveryStep);
	chart->addSeries(lineCalcVx);
    lineCalcEveryStep->attachAxis(axisX);
    lineCalcEveryStep->attachAxis(axisY);
	lineCalcVx->attachAxis(axisX);
	lineCalcVx->attachAxis(axisY);


//    QCategoryAxis *axisY3 = new QCategoryAxis;
//    axisY3->append("Low", 5);
//    axisY3->append("Medium", 12);
//    axisY3->append("High", 17);
//    axisY3->setLinePenColor(series->pen().color());
//    axisY3->setGridLinePen((series->pen()));

    //chart->addAxis(axisY3, Qt::AlignRight);
//    series->attachAxis(axisX);
//    series->attachAxis(axisY3);
    //![4]

    //![5]
    QChartView *chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);
    //![5]

    //![6]
    QMainWindow window;
    window.setCentralWidget(chartView);
    window.resize(1200, 800);
    window.show();
    //![6]

//    auto ed = new QLineEdit();
    //window.s

    return a.exec();
}

