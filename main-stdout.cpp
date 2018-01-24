
#include <iostream>
#include <iomanip>
#include <functional>
#include "Motion.hpp"

using namespace std;
using namespace EnvisionTec;

int main()
{
    std::cout << "Hello, Motion!" << std::endl;

    Motion mo;

    mo.accel = 3000;
    mo.decel = 6000;
    mo.initialPosition = 0;
    mo.targetPosition = 51200;
    mo.wantedSpeedAfterAccel = 6000;

    mo.recalculate();

	cout    <<"\n"
            <<"mo:{\n"
            <<"  dV_accel_: "<<setw(7)<< mo.dV_accel_<<"\n"
            <<"  dV_decel_: "<<setw(7)<< mo.dV_decel_<<"\n"
            <<"  Vc_:       "<<setw(7)<< mo.Vc_<<"\n"
            <<"  Xa_:       "<<setw(7)<< mo.Xa_<<"\n"
            <<"  Xc_:       "<<setw(7)<< mo.Xc_<<"\n"
            <<"  Xb_:       "<<setw(7)<< mo.Xb_<<"\n"
            <<"  Ta_:       "<<setw(7)<< mo.Ta_<<"\n"
            <<"  Tc_:       "<<setw(7)<< mo.Tc_<<"\n"
            <<"  Tb_:       "<<setw(7)<< mo.Tb_<<"\n"
            <<"}\n"
            <<endl;

	cout <<"vecVx"<<endl;
	//auto vecXV = mo.calcPositionToSpeedPairs();
	for( auto const& posSpd : mo.positionToSpeedPairs ){
		cout <<"posSpd:{ pos:"<<setw(7)<<posSpd.pos<<", spd:"<<setw(7)<<posSpd.spd<<" }"<<endl;
	}
	cout <<"vec.size(): "<<mo.positionToSpeedPairs.size()<<endl;

    return 0;
}
