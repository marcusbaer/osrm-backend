#include "util/coordinate_tidy.hpp"

#include <boost/test/test_case_template.hpp>
#include <boost/test/unit_test.hpp>

#include <algorithm>
#include <iterator>
#include <vector>

BOOST_AUTO_TEST_SUITE(tidy_test)

using namespace osrm;
using namespace osrm::util;

BOOST_AUTO_TEST_CASE(empty_traces_test)
{
    tidy::Coordinates coords;
    tidy::Timestamps tss;

    auto result = tidy::tidy(coords, tss);

    BOOST_CHECK_EQUAL(result.tidied_coordinates.size(), 0);
    BOOST_CHECK_EQUAL(result.tidied_timestamps.size(), 0);
    BOOST_CHECK_EQUAL(result.original_to_tidied.size(), 0);
}

BOOST_AUTO_TEST_CASE(one_item_trace_test)
{
    tidy::Coordinates coords;
    coords.emplace_back(FloatLongitude{13.207993}, FloatLatitude{52.446379});

    tidy::Timestamps tss;
    tss.emplace_back(1477090402);

    auto result = tidy::tidy(coords, tss);

    BOOST_CHECK_EQUAL(result.tidied_coordinates.size(), 1);
    BOOST_CHECK_EQUAL(result.tidied_timestamps.size(), 1);
    BOOST_CHECK_EQUAL(result.original_to_tidied.size(), 1);

    BOOST_CHECK_EQUAL(result.tidied_coordinates[0], coords.front());
    BOOST_CHECK_EQUAL(result.tidied_timestamps[0], tss.front());
    BOOST_CHECK_EQUAL(result.original_to_tidied[0], 0);
}

BOOST_AUTO_TEST_CASE(two_item_trace_already_tidied_test)
{
    tidy::Coordinates coords;
    coords.emplace_back(FloatLongitude{13.207993}, FloatLatitude{52.446379});
    coords.emplace_back(FloatLongitude{13.231658}, FloatLatitude{52.465416});

    tidy::Timestamps tss;
    tss.emplace_back(1477090402);
    tss.emplace_back(1477090663);

    tidy::Thresholds thresholds;
    thresholds.distance_in_meters = 15;
    thresholds.duration_in_seconds = 5;

    auto result = tidy::tidy(coords, tss, thresholds);

    BOOST_CHECK_EQUAL(result.tidied_coordinates.size(), 2);
    BOOST_CHECK_EQUAL(result.tidied_timestamps.size(), 2);
    BOOST_CHECK_EQUAL(result.original_to_tidied.size(), 2);

    BOOST_CHECK_EQUAL(result.tidied_coordinates[0], coords[0]);
    BOOST_CHECK_EQUAL(result.tidied_timestamps[0], tss[0]);
    BOOST_CHECK_EQUAL(result.original_to_tidied[0], 0);

    BOOST_CHECK_EQUAL(result.tidied_coordinates[1], coords[1]);
    BOOST_CHECK_EQUAL(result.tidied_timestamps[1], tss[1]);
    BOOST_CHECK_EQUAL(result.original_to_tidied[1], 1);
}

BOOST_AUTO_TEST_CASE(two_item_trace_needs_tidiying_test)
{
    tidy::Coordinates coords;
    coords.emplace_back(FloatLongitude{13.207993}, FloatLatitude{52.446379});
    coords.emplace_back(FloatLongitude{13.231658}, FloatLatitude{52.465416});

    tidy::Timestamps tss;
    tss.emplace_back(1477090402);
    tss.emplace_back(1477090663);

    tidy::Thresholds thresholds;
    thresholds.distance_in_meters = 5000;
    thresholds.duration_in_seconds = 5 * 60;

    auto result = tidy::tidy(coords, tss, thresholds);

    BOOST_CHECK_EQUAL(result.tidied_coordinates.size(), 1);
    BOOST_CHECK_EQUAL(result.tidied_timestamps.size(), 1);
    BOOST_CHECK_EQUAL(result.original_to_tidied.size(), 2);

    BOOST_CHECK_EQUAL(result.tidied_coordinates[0], coords[0]);
    BOOST_CHECK_EQUAL(result.tidied_timestamps[0], tss[0]);
    BOOST_CHECK_EQUAL(result.original_to_tidied[0], 0);
    BOOST_CHECK_EQUAL(result.original_to_tidied[1], 0);
}

BOOST_AUTO_TEST_CASE(two_blobs_in_traces_needs_tidiying_test)
{
    tidy::Coordinates coords;

    coords.emplace_back(FloatLongitude{13.207993}, FloatLatitude{52.446379});
    coords.emplace_back(FloatLongitude{13.207994}, FloatLatitude{52.446380});
    coords.emplace_back(FloatLongitude{13.207995}, FloatLatitude{52.446381});

    coords.emplace_back(FloatLongitude{13.231658}, FloatLatitude{52.465416});
    coords.emplace_back(FloatLongitude{13.231659}, FloatLatitude{52.465417});
    coords.emplace_back(FloatLongitude{13.231660}, FloatLatitude{52.465417});

    tidy::Timestamps tss;

    tss.emplace_back(1477090402);
    tss.emplace_back(1477090403);
    tss.emplace_back(1477090404);

    tss.emplace_back(1477090661);
    tss.emplace_back(1477090662);
    tss.emplace_back(1477090663);

    BOOST_CHECK_EQUAL(coords.size(), tss.size());

    tidy::Thresholds thresholds;
    thresholds.distance_in_meters = 15;
    thresholds.duration_in_seconds = 5;

    auto result = tidy::tidy(coords, tss, thresholds);

    BOOST_CHECK_EQUAL(result.tidied_coordinates.size(), 2);
    BOOST_CHECK_EQUAL(result.tidied_timestamps.size(), 2);
    BOOST_CHECK_EQUAL(result.original_to_tidied.size(), coords.size());

    BOOST_CHECK_EQUAL(result.tidied_coordinates[0], coords[0]);
    BOOST_CHECK_EQUAL(result.tidied_timestamps[0], tss[0]);

    BOOST_CHECK_EQUAL(result.tidied_coordinates[1], coords[3]); // first after blob
    BOOST_CHECK_EQUAL(result.tidied_timestamps[1], tss[3]);     // first after blob

    auto valid = [](auto index) { return index == 0 || index == 1; };
    auto ok = std::all_of(begin(result.original_to_tidied), end(result.original_to_tidied), valid);
    BOOST_CHECK(ok);

    BOOST_CHECK(std::is_sorted(begin(result.original_to_tidied), end(result.original_to_tidied)));

    auto second_blob_start = std::partition_point(begin(result.original_to_tidied),
                                                  end(result.original_to_tidied),
                                                  [](auto index) { return index == 0; });

    BOOST_CHECK(begin(result.original_to_tidied) < second_blob_start);
    BOOST_CHECK(second_blob_start < end(result.original_to_tidied));
    BOOST_CHECK(second_blob_start == begin(result.original_to_tidied) + 3); // 3 in first blob
}

BOOST_AUTO_TEST_SUITE_END()
