
#pragma once

#include <cassert>
#include <limits>
#include <vector>
#include <algorithm>
#include <functional>  // for std::function, ::less, ::greter
#include <cmath>  // for sqrt


namespace EnvisionTec {

	using std::abs;
	using std::sqrt;
	using std::round;
	using std::min;
	using std::max;
	
	
	/// Got from https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c#4609795
	template <typename T> 
	inline constexpr 
	int signum(T val) {
		return ( T(0) < val) - (val < T(0) );
	}

	
	/// Describes motion region for stepper motor. 
	/// Motion region consists of 3 parts: acceleration(1), constant speed(2), deceleration(3). 
	/// Graph of speed in time domain looks like trapeze.
	/// 2nd part could be removed if targetSpeed `Vc` is selected big enough. Then graph of speed in time domain looks like triangle.
	struct Motion
	{
	public:  // == TYPES ==
		using real = float;  //double
		struct PositionSpeedPair { 
			int   pos; 
			real  spd; 
		};
		using PositionSpeedVector = std::vector<PositionSpeedPair>;
		using posSpdIter = PositionSpeedVector::const_iterator;
		
	public:  // == STATIC CONST ==
		static constexpr real SpeedLimit_ = 160'000;
		static constexpr real SpeedLimit(){ constexpr real _ = 160'000; return _; }
		static constexpr real DefaultAccel = 8'000;
	
	public:  //protected:
		static constexpr real NaN_ = std::numeric_limits<real>::quiet_NaN();
		static constexpr int POSITION_MAX = std::numeric_limits<int>::max();
		static constexpr int POSITION_MIN = std::numeric_limits<int>::min();
		
	public:  // == CONFIGURATION VARIABLES ==  To syncronize results method `calculate()` should be called.
		/*const*/ unsigned speedUpdatePeriod_ms = 50;
		/*const*/ int initialPosition = 0;
		/*const*/ int targetPosition = 0;
		/*const*/ real accel = DefaultAccel;
		/*const*/ real decel = DefaultAccel;
		/*const*/ real wantedSpeedAfterAccel = 16000;  //SpeedLimit_;
		
	public://protected:  // == RESULT and STATUS VARIABLES == 
		int  X_;  /// Displacement, the sign shows movement direction
		real V0_ = 0;   /// Initial speed. Should always be 0. Behavior for other values UNIMPLEMENTED yet.
		real Vc_;  /// Highest speed will be reached in this motion.
		real Ta_;  /// Acceleration duration
		real Tc_;  /// Constant speed duration
		real Tb_;  /// Deceleration duration
		real Xa_ = NaN_;  /// Acceleration distance
		real Xc_ = NaN_;  /// Constant speed distance
		real Xb_ = NaN_;  /// Deceleration distance
	
	public://protected: //temp. MUST be 
		real dV_accel_;  // `speed_increment_`
		real dV_decel_;  // `speed_increment_`
		std::vector<int>  positionsAccel_;  // list of positions when speed should be updated
		std::vector<int>  positionsDecel_;  // list of positions when speed should be updated
		std::vector<int>::const_iterator  iter_;
		
		std::vector<PositionSpeedPair> positionToSpeedPairs;
		static constexpr size_t MinRecommendedTableCapacity = 200;
		static constexpr size_t MaxTableCapacity = 400;
		
	public://todo protected
		real currentSpeed_;
		
	public:
		posSpdIter iter;
		
	public:  // == CTOR ==
		Motion(){
			positionToSpeedPairs.reserve(MinRecommendedTableCapacity);  // I expect to accelerate/decelerate about 3 seconds, speed update period is 50ms. So max (3000+3000)/50 = 120 updates.
		}
			
	
	public:  // == GETTERS ==
		int const& getDisplacement() const{ return X_; }
		std::vector<int> const& getPositions(){ return positionsAccel_; }
		
		real const& getCurrentSpeed() const
		{ return currentSpeed_; }
		
		unsigned getCurrentSpeedUInt() const
		{  return unsigned(std::round(std::abs(currentSpeed_)));  }
		
		std::vector<PositionSpeedPair> const& getPositionToSpeedPairs() const
		{	return positionToSpeedPairs;	}
		
		posSpdIter const& getIter() const
		{  return iter;  }
		
	public:  // == CTORs ==
		//MotionRegion( unsigned speedUpdatePeriod_ms
		//		, int initialPosition
		//		, int targetPosition
		//		, real accel
		//		, real decel
		//		, real wantedSpeedAfterAccel 
		//)	: speedUpdatePeriod_ms(speedUpdatePeriod_ms)
		//	, initialPosition(initialPosition)
		//	, targetPosition(targetPosition)
		//	, accel(accel)
		//	, decel(decel)
		//	, wantedSpeedAfterAccel(wantedSpeedAfterAccel)
		//{
		//	X_ = targetPosition-initialPosition;
		//	bool ok = calculateSpeedAndDurations( X_, accel, decel, wantedSpeedAfterAccel );
		//	if( !ok )
		//		return;  //throw "invalid arguments"
		//	calculateAccelPositionsTable();  //calculateAccelerationTable(accel,initialSpeed,targetSpeed,initialPosition,targetPosition, positions_);
		//	calculateDecelPositionsTable();
		//	iter_ = positionsAccel_.cbegin();
		//}
	
	public:  // == METHODS ==	
		
		/// With endless motion, we expect no deceleration phase.
		bool isEndlessMotion(){
			return targetPosition == POSITION_MAX  ||  targetPosition == POSITION_MIN;
		}
		auto isEndless(){ return isEndlessMotion(); }
		
		bool recalculate()
		{
			if( isEndlessMotion() )  // with endless motion, we expect no deceleration phase.
				X_ = targetPosition;
			else  // normal calculation
				X_ = targetPosition - initialPosition;
			bool valid = validateAndFixInputs();
			if( !valid ){
				// Store failed results
				this->Vc_ =
				this->Ta_ = 
				this->Tb_ = 
				this->Tc_ = 
				this->currentSpeed_ = std::numeric_limits<real>::quiet_NaN();
				positionToSpeedPairs.clear();
				iter = positionToSpeedPairs.end();  //setDone();
				return false;  // nothing to do
			}
			dV_accel_ = accel * speedUpdatePeriod_ms / 1000;
			dV_decel_ = accel * speedUpdatePeriod_ms / 1000;
			bool success = isEndlessMotion()
					? calculateSpeedAndDurationsForEndlessMotion()
					: calculateSpeedAndDurations( X_, accel, decel, wantedSpeedAfterAccel );
			if( !success )
				return false;
			positionToSpeedPairs = calcPositionToSpeedPairs();
		#if DEBUG
			volatile size_t vec_pairs_size = positionToSpeedPairs.size();
		#endif
			iter = positionToSpeedPairs.cbegin();
			currentSpeed_ = dV_accel_;
			// optimize space consumption if it makes sense - vector has much unused space.
			if( (positionToSpeedPairs.capacity() > MinRecommendedTableCapacity )  &&
					( (positionToSpeedPairs.capacity() - positionToSpeedPairs.size() )*sizeof(PositionSpeedPair) > 1024*1024 ) 
			){ positionToSpeedPairs.shrink_to_fit(); }
			return true;
		}
		/*void old_recalculate(){
			//calculateAccelPositionsTable();
			//calculateDecelPositionsTable();
			//iter_ = positionsAccel_.cbegin();
		}*/
		
		///\return true for valid inputs
		bool validateAndFixInputs(){
			// Check inputs
			if( X_ == 0  ||  accel==0  ||  decel==0  ||  wantedSpeedAfterAccel==0 ){
				return false;  // nothing to do
			}
			// Fix signs, they have no matter, because `X_` shows motion direction. 
			accel = std::abs( accel );
			decel = std::abs( decel );
			wantedSpeedAfterAccel = std::abs( wantedSpeedAfterAccel );
			return true;
		}
		
		bool isValid(){
			return positionToSpeedPairs.size()>0; //!positionToSpeedPairs.empty();
		}
		
		/// Calculate motion parameters: targetSpeed and durations of acceleration, deceleration and middle constant speed motion.
		/// Assuming start speed V_0 is 0 and end speed is also 0.
		///\param displacement   The displacement. Motion direction it determined automatically. Could be any value, except numeric_limits<int>::min() or ::max().
		///\param accel          acceleration steps/microsteps per second. Sign does not matter. If 0 or nan or inf, DefaultAccel is used.
		///\param decel          deceleration steps/microsteps per second. Sign does not matter. If 0 or nan or inf, DefaultAccel is used.
		///\param targetSpeed    The target speed will be reached after acceleration. Shows desired value, it cannot be grater than `SpeedLimit` and `Vmax`. Fixed automatically. Sign does not matter. If 0 or nan or inf, DefaultAccel is used.
		bool calculateSpeedAndDurations( int displacement, real accel =DefaultAccel, real decel =DefaultAccel, real highSpeed =SpeedLimit() )
		{
			// if( !validateAndFixInputs() )  return false;
			// Short aliases
			int  const& X = displacement;
			auto const& a = accel;
			auto const& b = decel;
			// Calculate maximum (vertex) possible speed for current input: accel(a), decel(b), displacement
			real Vmax = sqrt( 2 * a * b * abs(X) / (a + b) );
			// Calculate target speed, the maximum speed will be reached in this motion
			real Vc = std::min( std::min(SpeedLimit(),highSpeed), Vmax );
			// Calculate acceleration and deceleration durations, and also constant speed duration
			real Ta = Vc / a;
			real Tb = Vc / b;
			// todo check wtf. wrong formula, wrong value
			real Tc = Vc < Vmax
					?  ( real(abs(X)) / Vc )-( Vc * (a + (-b)) / (2*a*(-b)) )
					:  0;
			int Xsign = displacement>=0 ? 1 : -1;
			real Xa = Vc * Vc / ( 2 * a );  //a * Ta*Ta / 2 * Xsign;
			//real Xc = Vc * Tc * Xsign;
			real Xb = Vc * Vc / ( 2 * b );  //Vc * Tb / 2;  //Tb * (Vc + (-b)*Tb/2) * Xsign;
			real Xc = real(abs(X)) - (Xa+Xb);
			assert( abs( (Xa+Xb+Xc) - abs(X) ) < 0.2 );
			// Store results. Basing on these values, accel and decel tables will be calculated
			  //this->X_ = displacement;
			this->Xa_ = Xa * Xsign;  //std::round( Xa ) * Xsign;
			this->Xc_ = Xc * Xsign;  //std::round( Xc ) * Xsign;
			this->Xb_ = Xb * Xsign;  //std::round( Xb ) * Xsign;
			this->Vc_ = Vc;
			this->Ta_ = Ta;
			this->Tb_ = Tb;
			this->Tc_ = Tc;
			return true;
		}
		
		/// Similar to calculateSpeedAndDurations()
		bool calculateSpeedAndDurationsForEndlessMotion(){
			// if( !validateAndFixInputs() )  return false;
			// Short aliases
			int  const& X = X_;
			auto const& a = accel;
			// Calculate target speed, the maximum speed will be reached in this motion
			real Vc = std::min( SpeedLimit(), wantedSpeedAfterAccel );
			// Calculate acceleration and deceleration durations, and also constant speed duration
			real Ta = Vc / a;
			real Tc = std::numeric_limits<real>::infinity();  // will run forever
			real Tb = 0;  // this time never never come
			int Xsign = signum(X);
			real Xa = Vc * Ta / 2;  //Vc * Vc / ( 2 * a );
			real Xc = std::numeric_limits<real>::infinity();
			real Xb = 0;
			//assert( abs( (Xa+Xb+Xc) - abs(X) ) < 0.2 );
			this->Xa_ = Xa * Xsign;  //std::round( Xa ) * Xsign;
			this->Xc_ = Xc * Xsign;  //std::round( Xc ) * Xsign;
			this->Xb_ = Xb * Xsign;  //std::round( Xb ) * Xsign;
			this->Vc_ = Vc;
			this->Ta_ = Ta;
			this->Tb_ = Tb;
			this->Tc_ = Tc;
			return true;
		}
		
		void calculateAccelPositionsTable(){
			this->positionsAccel_.clear();
			calculatePositionsTable( accel, V0_, Vc_, initialPosition, initialPosition+Xa_, dV_accel_, positionsAccel_, speedUpdatePeriod_ms);
		}
		
		void calculateDecelPositionsTable(){
			this->positionsDecel_.clear();
			calculatePositionsTable( decel, Vc_, V0_, initialPosition+Xa_+Xc_, initialPosition+Xa_+Xc_+Xb_, dV_decel_, positionsDecel_, speedUpdatePeriod_ms );
		}
		
		///\param accel  acceleration, sign has no reason, because it is determined from direction (`initialSpeed` and `targetSpeed`).
		/// `initialSpeed` and `targetSpeed` signs has no matter (because it shows direction), the sign is determined from direction (`initialPosition` and `targetPosition`).
		static void calculatePositionsTable( real accel,
				real initialSpeed, real targetSpeed, 
				int initialPosition, int targetPosition, 
				real& dV,                                         ///< \param[out] Speed delta (increment or decrement) which should be applied to current speed when another position in table was reached
				std::vector<int> & table,
				unsigned speedUpdatePeriod_ms =50 )
		{
			int displacement = targetPosition - initialPosition;
			if( displacement == 0 )
				return;
			// Lambda to check if target position still is not reached
			auto target_position_is_not_reached = displacement > 0
					? [](int pos, int tarpos)->bool{ return pos < tarpos; }
					: [](int pos, int tarpos)->bool{ return pos > tarpos; }
			;
			// Set speeds signs depending on displacement
			initialSpeed = displacement > 0  ?  std::abs(initialSpeed)  :  -std::abs(initialSpeed);
			targetSpeed  = displacement > 0  ?  std::abs(targetSpeed)   :  -std::abs(targetSpeed) ;
			// Lambda to check if target speed still is not reached
			auto target_speed_is_not_reached = initialSpeed < targetSpeed 
					? [](real const& spe, real const& tarspe)->bool{ return  spe < tarspe; } //std::less<int>{}
					: [](real const& spe, real const& tarspe)->bool{ return  spe > tarspe; } //std::greater<int>{};
			;
			accel = initialSpeed < targetSpeed  ?  std::abs(accel)  :  -std::abs(accel);
			// Calculate speed increment, in other terms delta V,or dV.
			auto const& dt = speedUpdatePeriod_ms;  // short alias
			dV = accel * dt / 1000;  // speed increment
			// Sanity check
			if( dV == 0 )
				return;  //table;  // nothing to do, no real acceleration
			// Filling acceleration table
			real speed = initialSpeed;   // v0 - initial speed
			int pos = initialPosition;  // x0 - initial position
			//table.push_back(pos+1);
			do{
				speed += dV;
				int dpos = std::round( speed * dt / 1000 );
				if( dpos != 0 ){
					pos += dpos;
					table.push_back(pos);
				}
			}while(     target_position_is_not_reached( pos, targetPosition )
					&&  target_speed_is_not_reached( speed, targetSpeed )
			);

			return ;
		}
		
		/// Calculate speed in every point
		[[deprecated]]  // only for testing algorithm on host machine
		std::vector<PositionSpeedPair> calcEachPosition()
		{
			std::vector<PositionSpeedPair> v;
			auto  nextPos = initialPosition < targetPosition
					? [](int &pos)->int&{ return ++pos; }
					: [](int &pos)->int&{ return --pos; }
			;
			// Lambda to check if target position still is not reached
			auto target_position_is_not_reached = X_ > 0
					? [](int pos, int tarpos)->bool{ return pos < tarpos; }
					: [](int pos, int tarpos)->bool{ return pos > tarpos; }
			;
			auto to_int = [](real const& x){ return int(round(x)); };
			int prePos;
			// acceleration
			prePos = initialPosition;
			for( int pos=0; target_position_is_not_reached(pos,Xa_);  nextPos(pos) ){
				real spe = std::sqrt( 2*abs(pos)*accel );
				v.push_back({ prePos+pos, spe }); //v.emplace_back( pos, spe );
			}
			// constant speed
			prePos += to_int(Xa_);
			for( int pos=0; target_position_is_not_reached(pos,Xc_);  nextPos(pos) ){
				real spe = Vc_;
				v.push_back({ prePos+pos, spe }); //v.emplace_back( Xa_+pos, spe );
			}
			// deceleration
			prePos += to_int(Xc_/*+Xb_*/);
			for( int pos=0; target_position_is_not_reached(pos,Xb_+1);  nextPos(pos) ){
				real spe = std::sqrt( 2*abs(Xb_-pos)*decel );
				v.push_back({ prePos+pos, spe });
				/*real spe = std::sqrt( 2*abs(pos)*decel );
				v.push_back({ prePos+pos, Vc_-spe });*/
			}
			return v;
		}
		
		/// This function generates vector of positions and speeds.
		/// Speed is lineary increased and position is calculated from speed.
		//std::vector<PositionSpeedPair> const& calcPositionToSpeedPairs()
		std::vector<PositionSpeedPair> calcPositionToSpeedPairs()
		{
			std::vector<PositionSpeedPair> vec;  //std::vector<PositionSpeedPair> & vec = positionToSpeedPairs;
			int dirSign = initialPosition < targetPosition ? 1 : -1;
			auto distanceAccel = [&a=accel](real v){ return v * v / (2 * a); };
			auto distanceDecel = [&b=decel](real v){ return v * v / (2 * b); };
			assert( abs( distanceAccel(Vc_) - abs(Xa_) ) < 0.2 );
			if( !isEndlessMotion() )
				assert( abs( distanceDecel(Vc_) - abs(Xb_) ) < 0.2 );
			
			auto to_int = [](real const& x){ return int(round(x)); };
			auto vec_push = [&,this](real const& x, real const& v){ 
				//assert( MaxTableCapacity > vec.capacity() );
				//if( vec.capacity() == vec.size()  &&   )
				//	return;
				vec.push_back({ initialPosition + to_int(x), v }); 
				assert( MaxTableCapacity >= vec.capacity() );
			};
			//auto const& dt = speedUpdatePeriod_ms;
			real dV = accel * speedUpdatePeriod_ms / 1000;
			auto const& Vc = Vc_;
			real v = dV;
			real x = 0;
		#if STORE_FIRST_0_ELEMENT
			vec_push(x,v);  //vec.push_back({ initialPosition + to_int(x), v });
		#endif
			v += dV;  // First [0] element has speed dV, so second [1] should have 2*dV.
			// acceleration stage
			while( v < Vc ){
				x = distanceAccel(v) * dirSign;  //v * v / (2 * accel);
				vec_push(x,v);  //vec.push_back({ initialPosition + to_int(x), v });  //
				v += dV;
			}
			// constant speed stage
			v = Vc;
			x = distanceAccel(v) * dirSign;
			assert( abs( x - Xa_ ) < 0.2 );
			vec_push(x,v);
			if( isEndlessMotion() ){
				vec_push(targetPosition,v);  // end point of constant speed stage, is first point of decel stage
			}
			else{  //if( !isEndlessMotion() ){
				if( x != Xa_ + Xc_ ){  // end point of constant speed stage, is first point of decel stage
					x = Xa_ + Xc_;
					vec_push(x,v);
				}
				// Deceleration stage
				dV = -decel * speedUpdatePeriod_ms / 1000;
				v = Vc+dV;
				while( v > 0 ){
					x = /*Xa_+Xc_+Xb_*/X_ - distanceDecel(v) * dirSign;  //v * v / (2 * accel);
					vec_push(x,v);
					v += dV;
				}
				v = 0;
				x = X_ - distanceDecel(v) * dirSign;
				assert( abs( x - X_ ) < 0.2 );
				vec_push(x,v);
			}
			return vec;
		}
		
		/// Additional check, useful if something was missed. If position accidentally jumped...
		posSpdIter const& advanceIter( int pos ){
			/*auto positionWasPassed =  X_>0 
					? [](int pos, int target){ return pos > target; }
					: [](int pos, int target){ return pos < target; };
			while( positionWasPassed(pos, iter->pos) )
				++iter;*/
			//auto iter_is_valid = [this](posSpdIter const& iter){ return iter != positionToSpeedPairs.end(); }
			if( X_>0 ){
				while( !isDone()  &&  pos > iter->pos )
					++iter;
			}else if( X_<0 ){
				while( !isDone()  &&  pos < iter->pos )
					++iter;
			}
			return iter;
		}
		
		
		///\return  normal value to update speed `setStepFreq(spd)`, 0 to stop immediately `stopStepping()`, NaN - do nothing continue with same speed.
		real updatePosition( int position )
		{
			//currentPosition_ = position;
			// Check if target position is reached and/or passed through (seems to be impossible but for reinsurance)
			if(    ( X_ > 0  &&  position >= targetPosition )
				|| ( X_ < 0  &&  position <= targetPosition )
			){//if( position == targetPosition ){
				return 0;  //stopStepping();
			}
	//		// Additional check, useful if something was missed. If position accidentally jumped...
	//		if( X_>0 ){
	//			while( position > *iter_ )
	//				++iter_;
	//		}else if( X_<0 ){
	//			while( position < *iter_ )
	//				++iter_;
	//		}
			real ret = NaN_;
			if( *iter_ == position ){
				++iter_;
				bool updated = incrementSpeed();
				if( updated ){
					ret = getCurrentSpeed();  //setStepFreq( getCurrentSpeed() );
				}
			}
			// If acceleration table passed, set iterator to deceleration table begin, and go with constant speed
			if( iter_ == positionsAccel_.cend() ){
				iter_ = positionsDecel_.cbegin();
//				assert( currentSpeed_ == Vc_ );
			}else if( iter_ == positionsDecel_.cend() ){
//				assert( currentSpeed_ == 0 );
			}
			return ret;  //do nothing - continue with same speed.
		}
		
		bool incrementSpeed(){
			auto const& dV = getSpeedIncrement();
			if( dV!=0  /*&&  speed!=Vc_*/ ){
				auto spd = currentSpeed_+dV; 
				int sign = spd>=0 ? 1 : -1;
				currentSpeed_ = std::min( std::abs(spd), Vc_ ) * sign;
				return true;
			}
			return false;
		}
		
		//int getSpeedIncrement() const{ return dV_accel_; }  // TODO depending on current action accel or decel
		/// Depending on where the motion is, accelerating or decelerating
		real getSpeedIncrement() const
		{ 
			// If iterator is in acceleration table
			if( iter_ >= positionsAccel_.cbegin()  &&  iter_ < positionsAccel_.cend() )
				return dV_accel_; 
			else if( iter_ >= positionsDecel_.cbegin()  &&  iter_ < positionsDecel_.cend() )
				return dV_decel_;
			else
				return 0;  //may be nan?
		}
		
		/// 
		bool isDone() const{
			return iter == positionToSpeedPairs.end();
			//return iter_ == positionsDecel_.cend();
			//assert( getCurrentSpeed() == 0 );
		}
		
		void setDone( /*bool done=true*/ ){
			iter = positionToSpeedPairs.cend();
		}
		
		/// positive(+1) -> upward, negative(-1) -> downward, zero(0) -> motion is done.
		int direction(){ return signum(X_); }   // isDone() ? 0 : signum(X_);
		int getDirection(){ return direction(); }
		
		bool isUpward(){ return X_ > 0; }
		bool isDownpward(){ return X_ < 0; }
		
		bool isOnStart(){
			return iter == positionToSpeedPairs.begin()  &&  iter != positionToSpeedPairs.end();  // && !positionToSpeedPairs.empty()
		}
	};
	
	
} //namespace EnvisionTec

