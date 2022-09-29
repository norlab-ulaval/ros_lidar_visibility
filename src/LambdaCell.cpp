#include "include/LambdaCell.hpp"

//default values overridden by param file
float LambdaCell::error_region_area = 1; //m^2
float LambdaCell::pm = 1; //.
float LambdaCell::ph = 1; //.
float LambdaCell::lambda_max = 1e5; 
float LambdaCell::lambda_unmeasured = -1; 
int LambdaCell::min_measure_nbr = 1; 
float LambdaCell::measure_duration = 1; 

LambdaCell::LambdaCell(): 
	hit(0), miss(0), hit_queue(), miss_queue(),
	C_normal(0), S_normal(0)
{
}

void LambdaCell::inc_hit(double const time){
	hit_queue.push(time);
	++hit;
	// update();
}

void LambdaCell::inc_miss(double const time){
	miss_queue.push(time);
	++miss;
	// update();
}

void LambdaCell::dec_hit(){
	hit_queue.pop();
	--hit;
	// update();
}

void LambdaCell::dec_miss(){
	miss_queue.pop();
	--miss;
	// update();
}

void LambdaCell::temporal_filter(double const now_time)
{
	while (!hit_queue.empty() && hit_queue.front() + measure_duration < now_time )
	{
		dec_hit();
	}

	while (!miss_queue.empty() && miss_queue.front() + measure_duration < now_time )
	{
		dec_miss();
	}
}

/*
void LambdaCell::update(){
	if (hit*hit + miss*miss > RADIUS_HIT_MISS*RADIUS_HIT_MISS)
	{
		hit  *= RADIUS_HIT_MISS/std::sqrt(hit*hit+miss*miss);
		miss *= RADIUS_HIT_MISS/std::sqrt(hit*hit+miss*miss);
	}
}
*/

void LambdaCell::update_normal(float n){
	C_normal += std::cos(n);
	S_normal += std::sin(n);
}

float LambdaCell::get_hit() const{
	return hit;
}
float LambdaCell::get_miss() const{
	return miss;
}

float LambdaCell::lambda() const{
	if(hit + miss < min_measure_nbr) {
		return lambda_unmeasured;
	}
	else if (miss ==0){
		return lambda_max;
	}
	else{
		return 1./error_region_area*std::log(1+(float)hit/(float)miss);
	}
}
float LambdaCell::lambda_up() const{
	float mu = hit*ph + miss*(1-pm);
	float sig2 = hit*ph*(1-ph) + miss*pm*(1-pm);

	float KU = std::min(mu + 1.96 * std::sqrt(sig2), (double)miss+hit);
	if(KU >= miss+hit)
		return lambda_max;

	float lU = 1./error_region_area * std::log(KU/(miss+hit-KU)+1.);

	return lU;
}
float LambdaCell::lambda_low() const{
	if(!is_measured())
		return lambda_unmeasured;

	float mu = hit*ph + miss*(1-pm);
	float sig2 = hit*ph*(1-ph) + miss*pm*(1-pm);

	float KL = std::max(mu - 0.96 * std::sqrt(sig2), 0.);
	if(KL <= 0.)
		return 0.;
	float lL = 1./error_region_area * std::log(KL/(miss+hit-KL)+1.);

	return lL;
}
float LambdaCell::normal() const{
	if(C_normal==0 and S_normal==0)
		return NAN;
	return std::atan2(S_normal, C_normal);
}
bool LambdaCell::is_measured() const{
	return !(hit==0 && miss==0);
}
