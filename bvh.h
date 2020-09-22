#ifndef BVH_H
#define BVH_H

#include "rtweekend.h"
#include "hittable_list.h"
#include <algorithm>

class bvh_node : public hittable
{
public:
	bvh_node();
	bvh_node(hittable_list& list, double time0, double time1) : bvh_node(list.objects, 0, list.objects.size(), time0, time1) {}
	bvh_node(std::vector<shared_ptr<hittable>>& objects, size_t start, size_t end, double time0, double time1);

	virtual bool hit(const ray& r, double tmin, double tmax, hit_record& rec) const override;
	virtual bool bounding_box(double t0, double t1, aabb& output_box) const override;

public:
	shared_ptr<hittable> left;
	shared_ptr<hittable> right;
	aabb box;
};

bool bvh_node::bounding_box(double t0, double t1, aabb& output_box) const
{
	output_box = box;
	return true;
}

bool bvh_node::hit(const ray& r, double t_min, double t_max, hit_record& rec) const
{
	if (!box.hit(r, t_min, t_max)) return false;

	bool hit_left = left->hit(r, t_min, t_max, rec);
	bool hit_right = right->hit(r, t_min, hit_left ? rec.t : t_max, rec);

	return hit_left || hit_right;
}

bvh_node::bvh_node(std::vector<shared_ptr<hittable>>& objects, size_t start, size_t end, double time0, double time1)
{
	int axis = random_int(0, 2);
	auto comparator = (axis == 0) ? box_x_compare : (axis == 1) ? box_y_compare : box_z_compare;

	size_t object_span = end - start;

	if (object_span == 1)
	{
		left = right = objects[start];
	}
	else if (object_span == 2)
	{
		if (comparator(objects[start], objects[start + 1]))
		{
			left = objects[start];
			right = objects[start + 1];
		}
		else
		{
			left = objects[start + 1];
			right = objects[start];
		}
	}
	else
	{
		std::sort(objects.begin() + start, objects.begin() + end, comparator);

		auto mid = start + object_span / 2;
		left  = make_shared<bvh_node>(objects, start, mid, time0, time1);
		right = make_shared<bvh_node>(objects, mid  , end, time0, time1);
	}


}

#endif