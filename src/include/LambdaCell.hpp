#pragma once
#include <array>
#include <limits>
#include <cmath>
#include <queue>

class LambdaCell
{
  public:
	LambdaCell();

	void inc_hit(double const time);
	void inc_miss(double const_time);
	void dec_hit();
	void dec_miss();
	// void update();
	void temporal_filter(double const now_time);

	void update_normal(float n);

	float get_hit() const;
	float get_miss() const;

	float lambda() const;
	float lambda_up() const;
	float lambda_low() const;

	float normal() const;

	bool is_measured() const;

    // Static getters / setters
	static void set_error_region_area(const float error_region_area_){ error_region_area=error_region_area_; }

	static float get_error_region_area(){ return error_region_area; }

	static void set_lambda_max(const float lambda_max_){ lambda_max=lambda_max_; }

	static float get_lambda_max(){ return lambda_max; }

	static void set_lambda_unmeasured(const float lambda_unmeasured_){ lambda_unmeasured=lambda_unmeasured_; }

	static float get_lambda_unmeasured(){ return lambda_unmeasured; }

	static void set_ph(const float ph_){ ph=ph_; }

	static float get_ph(){ return ph; }

	static void set_pm(const float pm_){ pm=pm_; }

	static float get_pm(){ return pm; }

	static void set_min_measure_nbr(const int min_measure_nbr_){ min_measure_nbr = min_measure_nbr_; }

	static int get_min_measure_nbr(){ return min_measure_nbr; }

	static void set_measure_duration(const float measure_duration_){ measure_duration = measure_duration_; }

	static float get_measure_duration(){ return measure_duration; }
  private:
	float hit,miss;
	std::queue <double> hit_queue;
	std::queue <double> miss_queue;
	float C_normal;
	float S_normal;
	static float error_region_area;
	static float ph;
	static float pm;
	static float lambda_max;
	static float lambda_unmeasured;
    static int min_measure_nbr;
    static float measure_duration;
};
