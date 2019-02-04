//
// Created by steve on 19-2-2.
//
#include <gtest/gtest.h>

#include <VisualOdometry/FeatureTrackServer.h>
#include "../../googletest/googletest/include/gtest/gtest.h"


class FeatureTrackServertest : public ::testing::Test {

public:

};

TEST_F(FeatureTrackServertest, test_reduce_vec) {
	std::vector<int> in_vec;
	std::vector<uchar> mask_vec;
	std::vector<int> ref_out_vec;
	for (int i = 0; i < 100; ++i) {
		in_vec.push_back(i);
		if (i % 2 == 0) {
			mask_vec.push_back(0);
			ref_out_vec.push_back(i);

		} else {
			mask_vec.push_back(1);
		}

	}

	reduceVector<int>(in_vec, mask_vec);
	bool flag = true;
	std::cout << "in_vec size:" << in_vec.size()
	          << "/n ref out vec size:" << ref_out_vec.size() << std::endl;
	ASSERT_TRUE(in_vec.size() == ref_out_vec.size());

}


